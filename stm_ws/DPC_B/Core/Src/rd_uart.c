/* USER CODE BEGIN Header */

 /*
  * RD_UART.c
  *
  *  Created on: Aug 12, 2025
  *      Author: abc01
  */


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_uart.h"
#include <string.h>

/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/


/* Exported function prototypes -----------------------------------------------*/

RD_RET RD_UART_INIT(UART_Ring_t *uart_obj);
RD_RET RD_UART_IDLEHandler(UART_Ring_t *uart_obj);

RD_RET RD_UART_Transmit(UART_Ring_t *uart_obj);

/* Private user code ---------------------------------------------------------*/

RD_RET RD_UART_INIT(UART_Ring_t *uart_obj)
{
	if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK;

	// Hardware Initializing (Nuke Option)
	HAL_UART_Abort(uart_obj->huart);         // 진행 중인 모든 TX/RX 강제 중단
    HAL_UART_DeInit(uart_obj->huart);        // UART 레지스터 및 클럭 초기화
    if (HAL_UART_Init(uart_obj->huart) != HAL_OK) return RET_NOK; // UART 부팅

    // RX Initializing
	memset(uart_obj->rx_buffer, 0, RX_BUFFER_SIZE);
    uart_obj->head = 0;
    uart_obj->tail = 0;
    uart_obj->last_rx_tick = HAL_GetTick();
    uart_obj->rx_new = 0; //신규데이터 여부 0

    // TX Initializing
    memset(uart_obj->tx_buffer, 0, TX_BUFFER_SIZE);
    uart_obj->tx_length = 0;

    // Start Setting
    if (HAL_UART_Receive_DMA(uart_obj->huart, uart_obj->rx_buffer, RX_BUFFER_SIZE) != HAL_OK) return RET_NOK;

    // Error counter Initializing.
    uart_obj->rx_error_cnt = 0;
    uart_obj->tx_error_cnt = 0;
    uart_obj->is_fatal = 0;

    __HAL_UART_CLEAR_IDLEFLAG(uart_obj->huart);
    __HAL_UART_ENABLE_IT(uart_obj->huart, UART_IT_IDLE);
    return RET_OK;
}

/*=================================*/
/* stm32f4xx_it.c
  HAL_UART_IRQHandler(&huart2);
  if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE))
	  {
	  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
	  RD_UART_IDLEHandler(ECU_rs485.uart_obj);
  }
 * IDLE state 확인후 링버퍼 리 인덱싱 작업 */
/*=================================*/

RD_RET RD_UART_IDLEHandler(UART_Ring_t *uart_obj)
{
	if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK; //널 에러
	// Error Check
	if (uart_obj->huart->ErrorCode != HAL_UART_ERROR_NONE) {
		return RET_NOK;
	}
    //DMA현재값 추출
    uint16_t dma_tail = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart_obj->huart->hdmarx);

    //head tail 갱신
    uart_obj->head = uart_obj->tail;   // 이전 tail 기억
    uart_obj->tail = dma_tail;         // 현재 tail 갱신
    uart_obj->rx_length = (uart_obj->tail - uart_obj->head + RX_BUFFER_SIZE) % RX_BUFFER_SIZE; //길이정보 갱신
    if (uart_obj->rx_length == 0) return RET_WAIT;

    uart_obj->last_rx_tick = HAL_GetTick();
    //버퍼 리인덱싱 후 입력
    if (uart_obj->tail > uart_obj->head)
    {
    	memcpy(uart_obj->temp_buffer, (const uint8_t *)&uart_obj->rx_buffer[uart_obj->head], uart_obj->rx_length);
    }
    else
    {
    	uint16_t first_len = RX_BUFFER_SIZE - uart_obj->head;
    	memcpy(uart_obj->temp_buffer, (const uint8_t *)&uart_obj->rx_buffer[uart_obj->head], first_len);
    	memcpy(uart_obj->temp_buffer + first_len, (const uint8_t *)uart_obj->rx_buffer, uart_obj->tail);
    }
    uart_obj->rx_new = 1;
    uart_obj->rx_error_cnt = 0;

#ifdef RTOS_IS_AVAILABLE
    // RS485 Thread 깨우기
    if (uart_obj->huart->Instance == USART2) osThreadFlagsSet(rs485TaskHandle, 0x0001);
#endif

    return RET_OK;
}

RD_RET RD_UART_Transmit(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL || uart_obj->tx_length == 0) return RET_NOK;

    if(uart_obj->huart->gState != HAL_UART_STATE_READY) return RET_WAIT;

    if (HAL_UART_Transmit_DMA(uart_obj->huart, uart_obj->tx_buffer, uart_obj->tx_length) != HAL_OK)
    {
    	uart_obj->tx_error_cnt++;
        return RET_NOK;
    }
    uart_obj->tx_error_cnt = 0;
    return RET_OK;
}

RD_RET RD_UART_Checker(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK;

    // 3. 타임아웃 처리
    if (HAL_GetTick() - uart_obj->last_rx_tick > RX_TIMEOUT) {
    	uart_obj->rx_error_cnt++;
    }

	if (uart_obj->tx_error_cnt > ERROR_UART_MAX ||
		uart_obj->rx_error_cnt > ERROR_UART_MAX) {
		uart_obj->is_fatal = 1;
	}
    if ( uart_obj->is_fatal == 1 ) {
		if (RD_UART_INIT(uart_obj) != RET_OK) return RET_NOK;
    	return RET_WAIT;
    }
	return RET_OK;
	// is_fatal = 1 & RET_NOK인 경우 치명적 오류 초기화 필요
}

// ================RS485 Setting============================== //
#ifdef RS485_AVAILABLE
RD_RET RD_RS485_INIT(RS485_t *rs485_obj)
{
	if (rs485_obj == NULL ) return RET_NOK;

	rs485_obj->DIR.per_GPIO_Pin = RS485_EX_DIR_Pin;
	rs485_obj->DIR.per_GPIOx 	= RS485_EX_DIR_GPIO_Port;
	rs485_obj->tx_mode = 0;
	rs485_obj->last_tx_tick = HAL_GetTick();
	rs485_obj->error_cnt = 0;
	// RS485 Read Setting
	HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);

	if(RD_UART_INIT(rs485_obj->uart_obj) != RET_OK) return RET_NOK;
	return RET_OK;
}

RD_RET RD_RS485_Transmit(RS485_t *rs485_obj)
{
	if (rs485_obj == NULL) return RET_NOK;

	// RS485 Write Setting
    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_SET); // 1. 송신 모드로 전환 (DIR High)
    rs485_obj->tx_mode = 1;
    rs485_obj->last_tx_tick = HAL_GetTick();
    /*=================================*/
    /* stm32f4xx_it.c
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC))
	{
	  HAL_GPIO_WritePin(ECU_rs485.DIR.per_GPIOx, ECU_rs485.DIR.per_GPIO_Pin, GPIO_PIN_RESET);
	  ECU_rs485.tx_mode = 0;
	 }
	 // HAL_UART_IRQHandler 보다 위에 있어야함.
	 HAL_UART_IRQHandler(&huart2); // Flag_TC clear
     * TC(Transport Complete) Flag 확인 후 GPIO 내림 */
    /*=================================*/
    RD_RET state = RD_UART_Transmit(rs485_obj->uart_obj);
    if (state == RET_OK) {
    	rs485_obj->error_cnt = 0;
    	return state;
    } else {
    	HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
		rs485_obj->tx_mode = 0;
		if (state == RET_NOK) rs485_obj->error_cnt++;
    }
    return state;
}

RD_RET RD_RS485_Checker(RS485_t *rs485_obj) {
	if (rs485_obj == NULL) return RET_NOK;
	// 10 ms 이상 송신모드시 강제 수신모드 전환
	RD_RET uart_state = RD_UART_Checker(rs485_obj->uart_obj);
	if (uart_state != RET_OK) return uart_state;

	if (rs485_obj->tx_mode == 1 && HAL_GetTick() - rs485_obj->last_tx_tick > TX_TIMEOUT) {
		HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
		rs485_obj->tx_mode = 0;
		rs485_obj->error_cnt++;
	}

	if (rs485_obj->error_cnt > ERROR_RS485_MAX) {
		rs485_obj->uart_obj->is_fatal = 1;  // 이미 RD_RS485_INIT 내부에서 RD_UART_INIT 복구 시도
		if (RD_RS485_INIT(rs485_obj) != RET_OK) return RET_NOK;
		return RET_WAIT;
	}

	return RET_OK;
}
#endif

