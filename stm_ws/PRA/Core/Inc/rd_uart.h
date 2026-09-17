/*
 * RD_UART.h
 *
 *  Created on: Aug 12, 2025
 *      Author: abc01
 */

#ifndef INC_RD_UART_H_
#define INC_RD_UART_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"

// RS485 -> GPIO_t
#if __has_include("rd_peripheral.h")
	#include "rd_peripheral.h"
	#define RS485_AVAILABLE
#endif

// RTOS 전용 TASK
#if __has_include("cmsis_os2.h")
	#include "cmsis_os2.h"
	extern osThreadId_t rs485TaskHandle;
	#define RTOS_IS_AVAILABLE // RS485 Thread 깨우기용 플래그 미사용시 주석처리
#endif
/* Exported macro ------------------------------------------------------------*/
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128

#define TX_TIMEOUT 10 //ms
#define RX_TIMEOUT 10 //ms

#define ERROR_UART_MAX  100
#define ERROR_RS485_MAX 100

/* Exported types ------------------------------------------------------------*/

typedef struct {
	uint8_t rx_buffer[RX_BUFFER_SIZE];  	// DMA 링버퍼
    volatile uint16_t head;          		// DMA가 쓴 위치 (IDLE ISR에서 갱신)
    volatile uint16_t tail;          		// Task가 읽은 위치
    UART_HandleTypeDef *huart;       		// HAL UART 핸들
    volatile uint16_t rx_length;

    uint8_t temp_buffer[RX_BUFFER_SIZE]; 	//후처리 데이터 저장
    volatile uint8_t rx_new;				//신규 데이터 플래그
    volatile uint32_t last_rx_tick;			// 마지막 수신 시각 (timeout 감지용)

    uint8_t tx_buffer[TX_BUFFER_SIZE];		//DMA 송신버퍼
    volatile uint16_t tx_length;			//DMA 송신예정인 length 정보


    volatile uint16_t rx_error_cnt;
    volatile uint16_t tx_error_cnt;

    volatile uint8_t is_running;
    volatile uint8_t is_fatal;
} UART_Ring_t;

#ifdef RS485_AVAILABLE
typedef struct {
	UART_Ring_t *uart_obj;
	GPIO_IO_t DIR;
	volatile uint8_t  tx_mode;				// 수신 모드 송신 모드 디버그용
    volatile uint32_t last_tx_tick;			// 마지막 송신 시각 (timeout 감지용)

    volatile uint16_t error_cnt;
} RS485_t;
#endif


/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_UART_INIT(UART_Ring_t *uart_obj);
RD_RET RD_UART_IDLEHandler(UART_Ring_t *uart_obj);
RD_RET RD_UART_Transmit(UART_Ring_t *uart_obj);
RD_RET RD_UART_Checker(UART_Ring_t *uart_obj);

#ifdef RS485_AVAILABLE
RD_RET RD_RS485_INIT(RS485_t *rs485_obj);
RD_RET RD_RS485_Transmit(RS485_t *rs485_obj);
RD_RET RD_RS485_Checker(RS485_t *rs485_obj);
#endif
/* UART Error Callback Function
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 1. 문제가 발생한 UART 채널인지 확인 (UART2)
    if (huart->Instance == USART2)
    {
    	ECU_uart2->is_fatal = 1;
		// (선택) 하드웨어 플래그만 살짝 지워줌
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
    }
}
 */
#endif /* INC_RD_UART_H_ */
