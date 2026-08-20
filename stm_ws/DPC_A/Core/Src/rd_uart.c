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

/* Exported includes
 * ----------------------------------------------------------*/

/* Exported typedef
 * -----------------------------------------------------------*/

/* Exported define
 * ------------------------------------------------------------*/

/* Exported variables
 * ---------------------------------------------------------*/

/* Exported function prototypes
 * -----------------------------------------------*/

RD_RET RD_UART_INIT(UART_Ring_t *uart_obj);
RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj);
RD_RET RD_UART_IDLEHandler(UART_Ring_t *uart_obj);
RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k);
RD_RET RD_UART_Transmit(UART_Ring_t *uart_obj);

/* Private user code ---------------------------------------------------------*/

RD_RET RD_UART_INIT(UART_Ring_t *uart_obj) {
  // 수신초기화
  memset(uart_obj->rx_buffer, 0, RX_BUFFER_SIZE);
  uart_obj->head = 0;
  uart_obj->tail = 0;
  uart_obj->last_rx_tick = 0;
  uart_obj->rx_new = 0; // 신규데이터 여부 0

  // 송신초기화
  memset(uart_obj->tx_buffer, 0, TX_BUFFER_SIZE);
  uart_obj->tx_length = 0;

  // 에러초기화
  memset((void *)&uart_obj->error, 0, sizeof(ERROR_STATUS_t));
  uart_obj->error.state.bits.lifecycle = LS_READY;
  uart_obj->comm_err_flag = 0;

  // 시작세팅
  if (HAL_UART_Receive_DMA(uart_obj->huart, uart_obj->rx_buffer,
                           RX_BUFFER_SIZE) != HAL_OK)
    return RET_NOK;
  __HAL_UART_ENABLE_IT(uart_obj->huart, UART_IT_IDLE);

  return RET_OK;
}

RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj) {
  if (uart_obj == NULL || uart_obj->huart == NULL)
    return RET_NOK;

  /* INIT 시 주입된 huart 재사용. RD_UART_INIT 의 memset 이 huart 필드를
   * 지우므로 로컬 캡처. */
  UART_HandleTypeDef *huart = uart_obj->huart;

  /* 진입 시 lifecycle = LS_RECOVERING 표시 — Checker 는 이 상태를 보호
   * (덮어쓰지 않음). 성공 시 RD_UART_INIT 가 LS_READY 로 reset, 실패 시
   * LS_OFFLINE 으로 강제 전이 (무한 RECOVERY 재시도 방지). */
  uart_obj->error.state.bits.lifecycle = LS_RECOVERING;

  HAL_UART_Abort(huart);
  HAL_UART_DeInit(huart);
  if (HAL_UART_Init(huart) != HAL_OK) {
    uart_obj->error.state.bits.lifecycle = LS_OFFLINE;
    uart_obj->error.state.bits.health = HC_FATAL;
    return RET_NOK;
  }

  RD_RET init_ret = RD_UART_INIT(uart_obj);
  if (init_ret != RET_OK) {
    uart_obj->error.state.bits.lifecycle = LS_OFFLINE;
    uart_obj->error.state.bits.health = HC_FATAL;
  }
  return init_ret;
}

RD_RET RD_UART_IDLEHandler(UART_Ring_t *uart_obj) {
  if (uart_obj == NULL || uart_obj->huart == NULL)
    return RET_NOK; // 널 에러
  // DMA현재값 추출
  uint16_t dma_tail =
      RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart_obj->huart->hdmarx);

  // head tail 갱신
  uart_obj->head = uart_obj->tail; // 현재 head 갱신
  uart_obj->tail = dma_tail;       // 현재 tail 갱신
  uart_obj->last_rx_tick = HAL_GetTick();
  uart_obj->rx_length = (uart_obj->tail - uart_obj->head + RX_BUFFER_SIZE) %
                        RX_BUFFER_SIZE; // 길이정보 갱신

  // 버퍼 리인덱싱 후 입력
  if (uart_obj->tail > uart_obj->head) {
    memcpy(uart_obj->temp_buffer,
           (const uint8_t *)&uart_obj->rx_buffer[uart_obj->head],
           uart_obj->rx_length);
  } else {
    uint16_t first_len = RX_BUFFER_SIZE - uart_obj->head;
    memcpy(uart_obj->temp_buffer,
           (const uint8_t *)&uart_obj->rx_buffer[uart_obj->head], first_len);
    memcpy(uart_obj->temp_buffer + first_len,
           (const uint8_t *)uart_obj->rx_buffer, uart_obj->tail);
  }
  uart_obj->rx_new = 1;

  return RET_OK;
}

RD_RET RD_UART_Transmit(UART_Ring_t *uart_obj) {
  if (uart_obj == NULL || uart_obj->huart == NULL)
    return RET_NOK; // 널 에러

  if (uart_obj->huart->gState != HAL_UART_STATE_READY)
    return RET_WAIT;

  if (HAL_UART_Transmit_DMA(uart_obj->huart, uart_obj->tx_buffer,
                            uart_obj->tx_length) != HAL_OK) {
    return RET_NOK;
  }
  return RET_OK;
}

RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k) {
  if (uart_obj == NULL)
    return RET_NOK;
  /* huart 미주입 = 아직 task 가 INIT 전 (부팅 윈도우). escalation 유발 금지 →
   * WAIT. (UART INIT 은 의도적으로 각 task 루프 시작 시 수행 — 스케줄러 전 시작
   * 시 딜레이로 DMA 사망 회피) */
  if (uart_obj->huart == NULL)
    return RET_WAIT;

  uint8_t health = HC_OK;
  uint8_t lifecycle = uart_obj->error.state.bits.lifecycle;

  /* lifecycle 보호: RECOVERY 진행 중이거나 OFFLINE 인 경우 검사 자체 skip.
   *  - LS_RECOVERING: 상위(RECOVERY)가 state 를 수정 중이므로 race 회피.
   *  - LS_OFFLINE   : 상위가 RD_UART_RECOVERY 호출 전까지 변경할 게 없음.
   *  isr_err_code 는 RD_UART_INIT 에서 자동 클리어 → stale 누적 없음. */
  if (lifecycle == LS_RECOVERING)
    return RET_WAIT;
  if (lifecycle == LS_OFFLINE)
    return RET_NOK;

  uint32_t hal_err = isr_err_take(&uart_obj->error.isr_err_code);
  if (hal_err != 0) {
    if (hal_err & HAL_UART_ERROR_DMA)
      health = HC_HW_FAULT; /* DMA 컨트롤러 결함 */
    else if (hal_err & HAL_UART_ERROR_ORE)
      health = HC_OVERRUN;
    else
      health = HC_FRAMING_ERR; /* PE/FE/NE */

    if (health == HC_HW_FAULT) {
      /* DMA HW 결함은 가벼운 재무장으로 못 살림 → 상위 RECOVERY 로 escalate */
      uart_obj->error.rx_error_cnt = UART_FATAL_CNT_TH + 1;
    } else {
      /* 노이즈성(ORE/PE/FE/NE) → 가벼운 재무장 시도.
       * 재무장 전에 IDLE IT 를 끄고 진행 중인 RX DMA 를 확실히
       * 정지(RxState→READY)한다. 그래야 (1) Receive_DMA 재시작이 HAL_BUSY 로
       * 실패하지 않고 (2) head/tail/rx_length 리셋과 IDLE/DMA ISR 간 race 가
       * 제거된다. */
      __HAL_UART_DISABLE_IT(uart_obj->huart, UART_IT_IDLE);
      HAL_UART_AbortReceive(uart_obj->huart);

      uart_obj->head = 0;
      uart_obj->tail = 0;
      uart_obj->rx_length = 0;

      if (HAL_UART_Receive_DMA(uart_obj->huart, uart_obj->rx_buffer,
                               RX_BUFFER_SIZE) != HAL_OK) {
        health = HC_HW_FAULT; /* 재무장 실패 = HW 문제 */
        uart_obj->error.rx_error_cnt = UART_FATAL_CNT_TH + 1; /* escalate */
      } else {
        uart_obj->error.rx_error_cnt++;
      }
      __HAL_UART_CLEAR_IDLEFLAG(uart_obj->huart);
      __HAL_UART_ENABLE_IT(uart_obj->huart, UART_IT_IDLE);
    }
  }
  /* ★ clean tick 에서 rx_error_cnt 를 0 으로 리셋하지 않는다.
         에러는 항상 clean tick 으로 분리되므로 여기서 리셋하면 임계치에 영원히
     도달 못함. 빈도는 degraded_cnt 가 담당. rx_error_cnt 는 "성공 수신"
     시점에만 리셋. */

  /* 1b. Packet layer 에러 (HAL 에러 없을 때만 — HAL 우선).
   *     rx_error_cnt 는 건드리지 않음 — 노이즈로 인한 소프트 에러는
   * degraded_cnt 로만 집계. */
  if (health == HC_OK && uart_obj->comm_err_flag != 0) {
    if (uart_obj->comm_err_flag & COMM_ERR_CRC_BIT)
      health = HC_CRC_ERR;
    else if (uart_obj->comm_err_flag & COMM_ERR_FRAMING_BIT)
      health = HC_FRAMING_ERR;
    uart_obj->comm_err_flag = 0;
  }

  /* 2. RX 타임아웃 (RUNNING 진입 이후에만 판정) — 지속 조건. 매 tick 카운터
   * 증가로 fast saturate. */
  if (health == HC_OK && lifecycle >= LS_RUNNING) {
    if (HAL_GetTick() - uart_obj->last_rx_tick > UART_RX_TIMEOUT_MS)
      health = HC_TIMEOUT;
    else
      uart_obj->error.rx_error_cnt = 0;
  }

  /* 3. Degraded 카운터 갱신 — 이번 tick 에 에러 있으면 +K (포화), 없으면 -DECAY
   * (0 하한) */
  if (health != HC_OK) {
    uint32_t next = (uint32_t)uart_obj->error.degraded_cnt + degraded_k;
    uart_obj->error.degraded_cnt =
        (next > DEGRADED_CNT_MAX) ? DEGRADED_CNT_MAX : (uint16_t)next;
  } else if (uart_obj->error.degraded_cnt > 0) {
    uart_obj->error.degraded_cnt =
        (uart_obj->error.degraded_cnt > DEGRADED_TICK_DECAY)
            ? (uart_obj->error.degraded_cnt - DEGRADED_TICK_DECAY)
            : 0;
  }

  /* 4. Lifecycle 전이 */
  /* 4a. READY → RUNNING 승격:
   *     - health OK : 실제 수신 이력(last_rx_tick != 0) 필수.
   *       last_rx_tick == 0 이면 아직 IDLE_HANDLER 가 한 번도 불리지 않은 것 →
   * LS_READY 유지.
   *     - health != OK : 에러가 났다는 건 채널이 동작은 했다는 뜻이므로 RUNNING
   * 으로 승격해 RUNNING→DEGRADED→OFFLINE→RECOVERY escalation 경로를 타게 한다.
   *       (READY 에 머문 채 health 만 에러로 굳어 복구가 영영 안 걸리는 freeze
   * 방지) */
  if (lifecycle == LS_READY &&
      (health != HC_OK || uart_obj->last_rx_tick != 0)) {
    lifecycle = LS_RUNNING;
  }
  /* 4b. RUNNING ↔ DEGRADED 히스테리시스 (counter 기반, 4× 갭으로 flapping 방지)
   */
  if (lifecycle == LS_DEGRADED &&
      uart_obj->error.degraded_cnt == DEGRADED_CNT_MAX)
    lifecycle = LS_OFFLINE;
  if (lifecycle == LS_RUNNING &&
      uart_obj->error.degraded_cnt >= DEGRADED_THRESHOLD_HIGH) {
    lifecycle = LS_DEGRADED;
  } else if (lifecycle == LS_DEGRADED &&
             uart_obj->error.degraded_cnt <= DEGRADED_THRESHOLD_LOW) {
    lifecycle = LS_RUNNING;
  }

  /* 5. 즉시 OFFLINE 트리거 — 치명 단발 OR HAL 에러 누적 임계 초과 */
  if (health >= HC_THRESHOLD_FATAL ||
      uart_obj->error.rx_error_cnt > UART_FATAL_CNT_TH) {
    lifecycle = LS_OFFLINE;
    if (health < HC_FATAL)
      health = HC_FATAL; /* OFFLINE 사유 표시 */
  }

  /* 6. state 업데이트 */
  uart_obj->error.state.bits.health = health;
  uart_obj->error.state.bits.lifecycle = lifecycle;

  if (lifecycle == LS_OFFLINE)
    return RET_NOK; /* 복구 필요 */
  if (health >= HC_THRESHOLD_WARN)
    return RET_WAIT; /* 경고/에러 — 상위 레이어가 모니터링 */
  return RET_OK;
}
