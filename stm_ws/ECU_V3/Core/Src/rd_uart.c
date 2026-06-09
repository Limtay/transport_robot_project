/**
 ******************************************************************************
 * @file    rd_uart.c
 * @author  abc01
 * @date    2025-08-12
 * @brief   DMA+IDLE 인터럽트 기반 UART 링버퍼 드라이버 구현부.
 *
 *  파일 구성
 *  -----------------------------------------------------------------------
 *    [UART]   RD_UART_RECOVERY / RD_UART_INIT
 *             RD_UART_IDLE_HANDLER / RD_UART_TRANSMIT / RD_UART_CHECKER
 *    [RS485]  RD_RS485_RECOVERY / RD_RS485_INIT
 *             RD_RS485_TRANSMIT / RD_RS485_CHECKER
 *
 *  에러 처리 정책
 *  -----------------------------------------------------------------------
 *    HAL_UART_ErrorCallback 은 isr_err_code 에 raw 에러코드만 캡처.
 *    Checker 가 isr_err_code → HC_* 변환 후 state.bits.health/lifecycle 갱신.
 *    lifecycle == LS_OFFLINE 이면 RET_NOK 반환.
 *    상위 레이어가 RET_NOK 를 감지하고 직접
 *    RD_UART_RECOVERY / RD_RS485_RECOVERY 를 호출해야 한다.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_uart.h"
#include <string.h>

/* ============================================================================
 *                               UART
 * ========================================================================== */

/**
 * @brief  버퍼·DMA·카운터·IDLE 인터럽트를 초기화. 하드웨어는 이미 준비된 상태를 가정.
 *         최초 부팅 시 직접 호출하거나, RD_UART_RECOVERY 에서 내부 호출.
 */




RD_RET RD_UART_INIT(UART_Ring_t *uart_obj, UART_HandleTypeDef *huart)
{
    if (uart_obj == NULL || huart == NULL) return RET_NOK;
    memset(uart_obj, 0, sizeof(UART_Ring_t));
    // 3. 핸들 연결
    uart_obj->huart = huart;

    // 4. 하드웨어 DMA 수신 시작 (필수)
    if (HAL_UART_Receive_DMA(huart, uart_obj->rx_buffer, RX_BUFFER_SIZE) != HAL_OK)
    	return RET_NOK;

    uart_obj->error.state.bits.lifecycle = LS_READY;
    uart_obj->error.state.bits.health    = HC_OK;

    // 5. 하드웨어 IDLE 인터럽트 활성화 (필수)
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    return RET_OK;
}

/**
 * @brief  하드웨어 완전 재초기화 (Abort → DeInit → Init) 후 RD_UART_INIT 호출.
 *         상위 레이어가 Checker 에서 RET_NOK 를 받은 뒤 직접 호출.
 */
RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK;

    /* INIT 시 주입된 huart 재사용. RD_UART_INIT 의 memset 이 huart 필드를 지우므로 로컬 캡처. */
    UART_HandleTypeDef *huart = uart_obj->huart;

    /* 진입 시 lifecycle = LS_RECOVERING 표시 — Checker 는 이 상태를 보호 (덮어쓰지 않음).
     * 성공 시 RD_UART_INIT 가 LS_READY 로 reset, 실패 시 LS_OFFLINE 으로 강제 전이 (무한 RECOVERY 재시도 방지). */
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

/**
 * @brief  IDLE 인터럽트 핸들러에서 호출. DMA 현재 위치로 링버퍼를 처리하여
 *         temp_buffer 에 선형화하고 rx_new = 1 로 설정.
 */
RD_RET RD_UART_IDLE_HANDLER(UART_Ring_t *uart_obj)
{
    if (uart_obj == NULL || uart_obj->huart == NULL) return RET_NOK;

    /* DMA 현재 위치 → tail */
    uint16_t dma_tail = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart_obj->huart->hdmarx);

    uart_obj->head      = uart_obj->tail;
    uart_obj->tail      = dma_tail;
    uart_obj->rx_length = (uart_obj->tail - uart_obj->head + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;

    if (uart_obj->rx_length == 0) return RET_WAIT;

    uart_obj->last_rx_tick = HAL_GetTick();

    /* 링버퍼 선형화 → temp_buffer */
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
    if (uart_obj->huart->Instance == USART2)
        osThreadFlagsSet(rs485TaskHandle, 0x0001);
    else if (uart_obj->huart->Instance == USART1)
        osThreadFlagsSet(rcTaskHandle, 0x0001);
#endif
    return RET_OK;
}

/**
 * @brief  DMA TX 를 시작. gState 가 READY 가 아니면 RET_WAIT 반환.
 */
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

/**
 * @brief  isr_err_code / packet 에러 / 타임아웃을 검사하여 state.bits.health + lifecycle 을 갱신.
 *
 *  처리 순서:
 *    1. ISR 캡처 HAL 에러     → HC_HW_FAULT / HC_OVERRUN / HC_FRAMING_ERR + rx_error_cnt++
 *                               HAL 에러 없는 틱: rx_error_cnt = 0 (연속 카운터 리셋)
 *    2. Packet layer 에러     → HC_CRC_ERR / HC_FRAMING_ERR (HAL 에러 없을 때만)
 *                               rx_error_cnt 는 건드리지 않음 — degraded_cnt 로만 집계
 *    3. RX 타임아웃 (지속)    → HC_TIMEOUT (degraded_cnt 로만 집계)
 *    4. degraded_cnt 갱신     : 에러 있으면 +degraded_k, 없으면 -DEGRADED_TICK_DECAY
 *    5. lifecycle 전이        : INIT/READY → RUNNING 첫 승격,
 *                               RUNNING ↔ DEGRADED 히스테리시스 (THRESHOLD_HIGH/LOW),
 *                               HC_FATAL or rx_error_cnt 임계(연속 HAL 장애) → LS_OFFLINE
 *
 *  lifecycle 보호: LS_RECOVERING / LS_OFFLINE 에서는 Checker 가 일반 전이 시도하지 않음
 *                  (OFFLINE 강제 트리거만 예외 — 이미 OFFLINE 이면 효과 없음).
 *
 *  degraded_k 는 채널 통신 주파수에 맞춰 상위가 결정 (DEGRADED_K_100HZ / _200HZ / _250HZ).
 */
RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k)
{
    if (uart_obj == NULL) return RET_NOK;
    /* huart 미주입 = 아직 task 가 INIT 전 (부팅 윈도우). escalation 유발 금지 → WAIT.
     * (UART INIT 은 의도적으로 각 task 루프 시작 시 수행 — 스케줄러 전 시작 시 딜레이로 DMA 사망 회피) */
    if (uart_obj->huart == NULL) return RET_WAIT;

    uint8_t health    = HC_OK;
    uint8_t lifecycle = uart_obj->error.state.bits.lifecycle;

    /* lifecycle 보호: RECOVERY 진행 중이거나 OFFLINE 인 경우 검사 자체 skip.
     *  - LS_RECOVERING: 상위(RECOVERY)가 state 를 수정 중이므로 race 회피.
     *  - LS_OFFLINE   : 상위가 RD_UART_RECOVERY 호출 전까지 변경할 게 없음.
     *  isr_err_code 는 RD_UART_INIT 에서 자동 클리어 → stale 누적 없음. */
    if (lifecycle == LS_RECOVERING) return RET_WAIT;
    if (lifecycle == LS_OFFLINE)    return RET_NOK;

    uint32_t hal_err = isr_err_take(&uart_obj->error.isr_err_code);
    if (hal_err != 0) {
    	if      (hal_err & HAL_UART_ERROR_DMA) health = HC_HW_FAULT;   /* DMA 컨트롤러 결함 */
    	else if (hal_err & HAL_UART_ERROR_ORE) health = HC_OVERRUN;
    	else                                   health = HC_FRAMING_ERR; /* PE/FE/NE */

    	if (health == HC_HW_FAULT) {
    		/* DMA HW 결함은 가벼운 재무장으로 못 살림 → 상위 RECOVERY 로 escalate */
    		uart_obj->error.rx_error_cnt = UART_FATAL_CNT_TH + 1;
    	} else {
    		/* 노이즈성(ORE/PE/FE/NE) → 가벼운 재무장 시도.
    		 * 재무장 전에 IDLE IT 를 끄고 진행 중인 RX DMA 를 확실히 정지(RxState→READY)한다.
    		 * 그래야 (1) Receive_DMA 재시작이 HAL_BUSY 로 실패하지 않고
    		 *       (2) head/tail/rx_length 리셋과 IDLE/DMA ISR 간 race 가 제거된다. */
    		__HAL_UART_DISABLE_IT(uart_obj->huart, UART_IT_IDLE);
    		HAL_UART_AbortReceive(uart_obj->huart);

    		uart_obj->head      = 0;
    		uart_obj->tail      = 0;
    		uart_obj->rx_length = 0;

    		if (HAL_UART_Receive_DMA(uart_obj->huart, uart_obj->rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
    			health = HC_HW_FAULT;                          /* 재무장 실패 = HW 문제 */
    			uart_obj->error.rx_error_cnt = UART_FATAL_CNT_TH + 1;  /* escalate */
    		} else {
    	        uart_obj->error.rx_error_cnt++;
    		}
    	    __HAL_UART_CLEAR_IDLEFLAG(uart_obj->huart);
    	    __HAL_UART_ENABLE_IT(uart_obj->huart, UART_IT_IDLE);
    	}
    }
    /* ★ clean tick 에서 rx_error_cnt 를 0 으로 리셋하지 않는다.
           에러는 항상 clean tick 으로 분리되므로 여기서 리셋하면 임계치에 영원히 도달 못함.
           빈도는 degraded_cnt 가 담당. rx_error_cnt 는 "성공 수신" 시점에만 리셋. */

    /* 1b. Packet layer 에러 (HAL 에러 없을 때만 — HAL 우선).
     *     rx_error_cnt 는 건드리지 않음 — 노이즈로 인한 소프트 에러는 degraded_cnt 로만 집계. */
    if (health == HC_OK && uart_obj->comm_err_flag != 0) {
        if      (uart_obj->comm_err_flag & COMM_ERR_CRC_BIT)     health = HC_CRC_ERR;
        else if (uart_obj->comm_err_flag & COMM_ERR_FRAMING_BIT) health = HC_FRAMING_ERR;
        uart_obj->comm_err_flag = 0;
    }

    /* 2. RX 타임아웃 (RUNNING 진입 이후에만 판정) — 지속 조건. 매 tick 카운터 증가로 fast saturate. */
    if (health == HC_OK && lifecycle >= LS_RUNNING) {
        if (HAL_GetTick() - uart_obj->last_rx_tick > UART_RX_TIMEOUT_MS)
            health = HC_TIMEOUT;
        else uart_obj->error.rx_error_cnt = 0;
    }

    /* 3. Degraded 카운터 갱신 — 이번 tick 에 에러 있으면 +K (포화), 없으면 -DECAY (0 하한) */
    if (health != HC_OK) {
        uint32_t next = (uint32_t)uart_obj->error.degraded_cnt + degraded_k;
        uart_obj->error.degraded_cnt = (next > DEGRADED_CNT_MAX) ? DEGRADED_CNT_MAX : (uint16_t)next;
    } else if (uart_obj->error.degraded_cnt > 0) {
        uart_obj->error.degraded_cnt = (uart_obj->error.degraded_cnt > DEGRADED_TICK_DECAY)
                                       ? (uart_obj->error.degraded_cnt - DEGRADED_TICK_DECAY) : 0;
    }

    /* 4. Lifecycle 전이 */
    /* 4a. READY → RUNNING 첫 승격: health OK + 실제 수신 이력(last_rx_tick != 0) 필수.
     *     last_rx_tick == 0 이면 아직 IDLE_HANDLER 가 한 번도 불리지 않은 것 → LS_READY 유지. */
    if (health == HC_OK && lifecycle == LS_READY && uart_obj->last_rx_tick != 0) {
    	lifecycle = LS_RUNNING;
    }
    /* 4b. RUNNING ↔ DEGRADED 히스테리시스 (counter 기반, 4× 갭으로 flapping 방지) */
    if (lifecycle == LS_DEGRADED && uart_obj->error.degraded_cnt == DEGRADED_CNT_MAX)
        lifecycle = LS_OFFLINE;
    if (lifecycle == LS_RUNNING && uart_obj->error.degraded_cnt >= DEGRADED_THRESHOLD_HIGH) {
    	lifecycle = LS_DEGRADED;
    } else if (lifecycle == LS_DEGRADED && uart_obj->error.degraded_cnt <= DEGRADED_THRESHOLD_LOW) {
    	lifecycle = LS_RUNNING;
    }

    /* 5. 즉시 OFFLINE 트리거 — 치명 단발 OR HAL 에러 누적 임계 초과 */
    if (health >= HC_THRESHOLD_FATAL || uart_obj->error.rx_error_cnt > UART_FATAL_CNT_TH) {
        lifecycle = LS_OFFLINE;
        if (health < HC_FATAL) health = HC_FATAL;   /* OFFLINE 사유 표시 */
    }

    /* 6. state 업데이트 */
    uart_obj->error.state.bits.health    = health;
    uart_obj->error.state.bits.lifecycle = lifecycle;

    if (lifecycle == LS_OFFLINE)        return RET_NOK;   /* 복구 필요 */
    if (health   >= HC_THRESHOLD_WARN)  return RET_WAIT;  /* 경고/에러 — 상위 레이어가 모니터링 */
    return RET_OK;
}

/* ============================================================================
 *                              RS485
 * ========================================================================== */
#ifdef RS485_AVAILABLE

/**
 * @brief  RS485 핸들 초기화. DIR 핀/필드 설정 후 RD_UART_INIT 호출.
 *         최초 부팅 시 직접 호출하거나, RD_RS485_RECOVERY 에서 내부 호출.
 */
RD_RET RD_RS485_INIT(RS485_t *rs485_obj, UART_HandleTypeDef *huart)
{
    if (rs485_obj == NULL || rs485_obj->uart_obj == NULL) return RET_NOK;

    rs485_obj->DIR.per_GPIO_Pin = RS485_DIR_Pin;
    rs485_obj->DIR.per_GPIOx    = RS485_DIR_GPIO_Port;
    rs485_obj->tx_mode      = 0;
    rs485_obj->last_tx_tick = HAL_GetTick();

    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);

    return RD_UART_INIT(rs485_obj->uart_obj, huart);
}

/**
 * @brief  DIR 핀 RX 복귀 + error_cnt 초기화 후 RD_UART_RECOVERY 호출.
 *         상위 레이어가 Checker 에서 RET_NOK 를 받은 뒤 직접 호출.
 */
RD_RET RD_RS485_RECOVERY(RS485_t *rs485_obj)
{
    if (rs485_obj == NULL || rs485_obj->uart_obj == NULL) return RET_NOK;

    /* RD_UART_RECOVERY 가 내부에서 lifecycle = LS_RECOVERING 표시. 여기서는 DIR/tx_mode 만 리셋. */
    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
    rs485_obj->tx_mode = 0;

    return RD_UART_RECOVERY(rs485_obj->uart_obj);
}

/**
 * @brief  RS485 송신 시작. DIR 핀을 TX 모드(SET)로 전환 후 DMA TX 시작.
 *         송신 실패 시 DIR 핀을 즉시 RX 모드로 복귀.
 */
RD_RET RD_RS485_TRANSMIT(RS485_t *rs485_obj)
{
    if (rs485_obj == NULL) return RET_NOK;

    // RX block
    UART_HandleTypeDef *huart = rs485_obj->uart_obj->huart;
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_RE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);

    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_SET);
    rs485_obj->tx_mode      = 1;
    rs485_obj->last_tx_tick = HAL_GetTick();

    RD_RET state = RD_UART_TRANSMIT(rs485_obj->uart_obj);

    if (state == RET_OK) return RET_OK;

    /* 송신 실패 시 즉시 RX 모드 복귀 */
    HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
    rs485_obj->tx_mode = 0;

    return state;
}

RD_RET RD_RS485_IRQ_HANDLER(RS485_t *rs485_obj) {
	if (rs485_obj == NULL) return RET_NOK;
	UART_HandleTypeDef *huart = rs485_obj->uart_obj->huart;
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
	// 버퍼 IC 수신 모드로 복귀 (GPIO = Low)
	HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
	rs485_obj->tx_mode = 0;
	// [오타 수정] huartx -> huart1 로 변경 및 0.2ms 유도 노이즈 찌꺼기 원천 청소
	volatile uint32_t dummy_dr = huart->Instance->DR;
	(void)dummy_dr;
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE);
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//수신 모듈(RE)과 IDLE 인터럽트 재활성화
	SET_BIT(huart->Instance->CR1, USART_CR1_RE);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	// [주석 해제] 다음 송신이 1회성으로 멈추지 않도록 TX 잠금 해제 (READY 환원)
	huart->gState = HAL_UART_STATE_READY;
	if (huart->hdmatx != NULL) huart->hdmatx->State = HAL_DMA_STATE_READY;
	return RET_OK;
}
/**
 * @brief  UART state 확인 + TX 타임아웃 시 DIR 핀 강제 복귀.
 *         lifecycle == LS_OFFLINE 이면 RET_NOK 반환 (상위 레이어가 RECOVERY 호출).
 *         degraded_k 는 RD_UART_CHECKER 에 그대로 전달 — RS485 채널 주파수에 맞춰 상위가 결정.
 */
RD_RET RD_RS485_CHECKER(RS485_t *rs485_obj, uint16_t degraded_k)
{
    if (rs485_obj == NULL) return RET_NOK;

    /* UART 레벨 state 확인 */
    RD_RET uart_state = RD_UART_CHECKER(rs485_obj->uart_obj, degraded_k);
    if (uart_state != RET_OK) return uart_state;

    /* TX 타임아웃 — 강제 수신 모드 복귀 */
    if (rs485_obj->tx_mode == 1 &&
        HAL_GetTick() - rs485_obj->last_tx_tick > TX_TIMEOUT)
    {
        HAL_GPIO_WritePin(rs485_obj->DIR.per_GPIOx, rs485_obj->DIR.per_GPIO_Pin, GPIO_PIN_RESET);
        rs485_obj->tx_mode = 0;
    }

    return RET_OK;
}

#endif /* RS485_AVAILABLE */
