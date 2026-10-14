/**
 ******************************************************************************
 * @file    rd_uart.c
 * @author  abc01
 * @date    2025-08-12 (PRA 이식 2026-09-17, Step6)
 * @brief   DMA+IDLE 인터럽트 기반 UART 링버퍼 드라이버 구현부 (ECU/DPC_B advanced).
 *
 *  에러 처리 정책
 *    HAL_UART_ErrorCallback 은 isr_err_code 에 raw 에러코드만 캡처.
 *    Checker 가 isr_err_code → HC_* 변환 후 state.bits.health/lifecycle 갱신.
 *    lifecycle == LS_OFFLINE 이면 RET_NOK 반환.
 *    상위 레이어(systemTask)가 RET_NOK 감지 후 RD_*_RECOVERY 직접 호출.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_uart.h"
#include <string.h>

/* CR1 read-modify-write 원자화: systemTask(CHECKER 의 IDLE IT 토글) / rs485Task(TX 전환 시 RE clear) /
 * TC ISR(RX 복귀 시 RE set)이 같은 CR1 을 비원자 RMW 로 만져 lost-update 가능 — 짧은 비트조작만
 * PRIMASK 로 감싼다. ★ RE 비트를 한 번 잃으면 DMA 는 살아 있는데 수신만 영구 정지 = "무응답". */
static inline uint32_t uart_crit_enter(void) { uint32_t pm = __get_PRIMASK(); __disable_irq(); return pm; }
static inline void     uart_crit_exit(uint32_t pm) { __set_PRIMASK(pm); }

/* ============================================================================
 *                               UART
 * ========================================================================== */

RD_RET RD_UART_INIT(UART_Ring_t *uart_obj, UART_HandleTypeDef *huart)
{
    if (uart_obj == NULL || huart == NULL) return RET_NOK;

#ifdef RTOS_IS_AVAILABLE
    /* memset 을 살아남아야 하는 필드 — 아래에서 되돌린다 (RECOVERY 재호출 시 채널 사망 방지) */
    osThreadId_t keep_wake = uart_obj->wake_task;
#endif

    memset(uart_obj, 0, sizeof(UART_Ring_t));

#ifdef RTOS_IS_AVAILABLE
    uart_obj->wake_task = keep_wake;
#endif

    uart_obj->huart = huart;

    if (HAL_UART_Receive_DMA(huart, uart_obj->rx_buffer, RX_BUFFER_SIZE) != HAL_OK)
    	return RET_NOK;

    uart_obj->error.state.bits.lifecycle = LS_READY;
    uart_obj->error.state.bits.health    = HC_OK;

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    return RET_OK;
}

RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK;

    UART_HandleTypeDef *huart = uart_obj->huart;

    uart_obj->error.state.bits.lifecycle = LS_RECOVERING;

    HAL_UART_Abort(huart);
    HAL_UART_DeInit(huart);
    if (HAL_UART_Init(huart) != HAL_OK) {
        uart_obj->error.state.bits.lifecycle = LS_OFFLINE;
        uart_obj->error.state.bits.health    = HC_FATAL;
        return RET_NOK;
    }

    RD_RET init_ret = RD_UART_INIT(uart_obj, huart);
    if (init_ret != RET_OK) {
        uart_obj->error.state.bits.lifecycle = LS_OFFLINE;
        uart_obj->error.state.bits.health    = HC_FATAL;
    }
    return init_ret;
}

RD_RET RD_UART_IDLE_HANDLER(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK;

    uint16_t dma_tail = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart_obj->huart->hdmarx);

    uart_obj->head      = uart_obj->tail;
    uart_obj->tail      = dma_tail;
    uart_obj->rx_length = (uart_obj->tail - uart_obj->head + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;

    if (uart_obj->rx_length == 0) return RET_WAIT;

    uart_obj->last_rx_tick = HAL_GetTick();

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

#ifdef RTOS_IS_AVAILABLE
    if (uart_obj->wake_task != NULL)
        osThreadFlagsSet(uart_obj->wake_task, 0x0001);
#endif
    return RET_OK;
}

RD_RET RD_UART_TRANSMIT(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL || uart_obj->tx_length == 0) return RET_NOK;

    if (uart_obj->huart->gState != HAL_UART_STATE_READY) return RET_WAIT;

    if (HAL_UART_Transmit_DMA(uart_obj->huart, uart_obj->tx_buffer, uart_obj->tx_length) != HAL_OK)
    {
        uart_obj->error.tx_error_cnt++;
        return RET_NOK;
    }

    uart_obj->error.tx_error_cnt = 0;
    return RET_OK;
}

RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k)
{
    if (uart_obj == NULL) return RET_NOK;
    if (uart_obj->huart == NULL) return RET_WAIT;   /* 아직 INIT 전 (부팅 윈도우) */

    uint8_t health    = HC_OK;
    uint8_t lifecycle = uart_obj->error.state.bits.lifecycle;

    if (lifecycle == LS_RECOVERING) return RET_WAIT;
    if (lifecycle == LS_OFFLINE)    return RET_NOK;

    uint32_t hal_err = isr_err_take(&uart_obj->error.isr_err_code);
    if (hal_err != 0) {
    	if      (hal_err & HAL_UART_ERROR_DMA) health = HC_HW_FAULT;
    	else if (hal_err & HAL_UART_ERROR_ORE) health = HC_OVERRUN;
    	else                                   health = HC_FRAMING_ERR;

    	if (health == HC_HW_FAULT) {
    		uart_obj->error.rx_error_cnt = UART_FATAL_CNT_TH + 1;
    	} else {
    		uint32_t pm = uart_crit_enter();
    		__HAL_UART_DISABLE_IT(uart_obj->huart, UART_IT_IDLE);
    		uart_crit_exit(pm);
    		HAL_UART_AbortReceive(uart_obj->huart);

    		uart_obj->head      = 0;
    		uart_obj->tail      = 0;
    		uart_obj->rx_length = 0;

    		if (HAL_UART_Receive_DMA(uart_obj->huart, uart_obj->rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
    			health = HC_HW_FAULT;
    			uart_obj->error.rx_error_cnt = UART_FATAL_CNT_TH + 1;
    		} else {
    	        uart_obj->error.rx_error_cnt++;
    		}
    	    uint32_t pm2 = uart_crit_enter();
    	    __HAL_UART_CLEAR_IDLEFLAG(uart_obj->huart);
    	    __HAL_UART_ENABLE_IT(uart_obj->huart, UART_IT_IDLE);
    	    uart_crit_exit(pm2);
    	}
    }

    /* Packet layer 에러 (HAL 에러 없을 때만). rx_error_cnt 는 미변경 — degraded_cnt 로만 집계. */
    if (health == HC_OK && uart_obj->comm_err_flag != 0) {
        if      (uart_obj->comm_err_flag & COMM_ERR_CRC_BIT)     health = HC_CRC_ERR;
        else if (uart_obj->comm_err_flag & COMM_ERR_FRAMING_BIT) health = HC_FRAMING_ERR;
        uart_obj->comm_err_flag = 0;
    }

    /* RX 타임아웃 (RUNNING 진입 이후에만 판정) */
    if (health == HC_OK && lifecycle >= LS_RUNNING) {
        if (HAL_GetTick() - uart_obj->last_rx_tick > UART_RX_TIMEOUT_MS)
            health = HC_TIMEOUT;
        else uart_obj->error.rx_error_cnt = 0;
    }

    /* Degraded 카운터 갱신 */
    if (health != HC_OK) {
        uint32_t next = (uint32_t)uart_obj->error.degraded_cnt + degraded_k;
        uart_obj->error.degraded_cnt = (next > DEGRADED_CNT_MAX) ? DEGRADED_CNT_MAX : (uint16_t)next;
    } else if (uart_obj->error.degraded_cnt > 0) {
        uart_obj->error.degraded_cnt = (uart_obj->error.degraded_cnt > DEGRADED_TICK_DECAY)
                                       ? (uart_obj->error.degraded_cnt - DEGRADED_TICK_DECAY) : 0;
    }

    /* Lifecycle 전이 */
    if (lifecycle == LS_READY && (health != HC_OK || uart_obj->last_rx_tick != 0)) {
    	lifecycle = LS_RUNNING;
    }
    if (lifecycle == LS_DEGRADED && uart_obj->error.degraded_cnt == DEGRADED_CNT_MAX)
        lifecycle = LS_OFFLINE;
    if (lifecycle == LS_RUNNING && uart_obj->error.degraded_cnt >= DEGRADED_THRESHOLD_HIGH) {
    	lifecycle = LS_DEGRADED;
    } else if (lifecycle == LS_DEGRADED && uart_obj->error.degraded_cnt <= DEGRADED_THRESHOLD_LOW) {
    	lifecycle = LS_RUNNING;
    }

    /* 즉시 OFFLINE 트리거 */
    if (health >= HC_THRESHOLD_FATAL || uart_obj->error.rx_error_cnt > UART_FATAL_CNT_TH) {
        lifecycle = LS_OFFLINE;
        if (health < HC_FATAL) health = HC_FATAL;
    }

    uart_obj->error.state.bits.health    = health;
    uart_obj->error.state.bits.lifecycle = lifecycle;

    if (lifecycle == LS_OFFLINE)        return RET_NOK;
    if (health   >= HC_THRESHOLD_WARN)  return RET_WAIT;
    return RET_OK;
}

/* ============================================================================
 *                              RS485
 * ========================================================================== */
#ifdef RS485_AVAILABLE

RD_RET RD_RS485_INIT(RS485_t *rs485_obj, UART_HandleTypeDef *huart)
{
    if (rs485_obj == NULL || rs485_obj->uart_obj == NULL) return RET_NOK;

    /* DIR 핀은 드라이버가 하드코딩하지 않는다 — 호출자가 rs485_obj->DIR 을 미리 주입(DIP). */
    rs485_obj->tx_mode      = 0;
    rs485_obj->last_tx_tick = HAL_GetTick();

    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);

    return RD_UART_INIT(rs485_obj->uart_obj, huart);
}

RD_RET RD_RS485_RECOVERY(RS485_t *rs485_obj)
{
    if (rs485_obj == NULL || rs485_obj->uart_obj == NULL) return RET_NOK;

    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
    rs485_obj->tx_mode = 0;

    return RD_UART_RECOVERY(rs485_obj->uart_obj);
}

RD_RET RD_RS485_TRANSMIT(RS485_t *rs485_obj)
{
    if (rs485_obj == NULL) return RET_NOK;

    UART_HandleTypeDef *huart = rs485_obj->uart_obj->huart;
    uint32_t pm = uart_crit_enter();
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_RE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    uart_crit_exit(pm);

    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_SET);
    rs485_obj->tx_mode      = 1;
    rs485_obj->last_tx_tick = HAL_GetTick();

    RD_RET state = RD_UART_TRANSMIT(rs485_obj->uart_obj);

    if (state == RET_OK) return RET_OK;

    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
    rs485_obj->tx_mode = 0;

    return state;
}

RD_RET RD_RS485_IRQ_HANDLER(RS485_t *rs485_obj) {
	if (rs485_obj == NULL) return RET_NOK;
	UART_HandleTypeDef *huart = rs485_obj->uart_obj->huart;
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
	// 버퍼 IC 수신 모드로 복귀 (DIR = Low)
	HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
	rs485_obj->tx_mode = 0;
	// 유도 노이즈 찌꺼기 청소 (DR 더미 read + 에러/IDLE 플래그 클리어)
	volatile uint32_t dummy_dr = huart->Instance->DR;
	(void)dummy_dr;
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE);
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	// 수신 모듈(RE)과 IDLE 인터럽트 재활성화
	SET_BIT(huart->Instance->CR1, USART_CR1_RE);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	// 다음 송신이 1회성으로 멈추지 않도록 TX 잠금 해제 (READY 환원)
	huart->gState = HAL_UART_STATE_READY;
	if (huart->hdmatx != NULL) huart->hdmatx->State = HAL_DMA_STATE_READY;
	return RET_OK;
}

RD_RET RD_RS485_CHECKER(RS485_t *rs485_obj, uint16_t degraded_k)
{
    if (rs485_obj == NULL) return RET_NOK;

    RD_RET uart_state = RD_UART_CHECKER(rs485_obj->uart_obj, degraded_k);
    if (uart_state != RET_OK) return uart_state;

    if (rs485_obj->tx_mode == 1 &&
        HAL_GetTick() - rs485_obj->last_tx_tick > TX_TIMEOUT)
    {
        HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
        rs485_obj->tx_mode = 0;
    }

    return RET_OK;
}

#endif /* RS485_AVAILABLE */
