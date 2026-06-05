/**
 ******************************************************************************
 * @file    rd_can_motor.c
 * @author  Kyeongtae
 * @date    2026-05-28
 * @brief   CAN AK 모터 드라이버 래퍼 — DATA_MOTOR_t 직접 갱신 (memcpy-only marshal).
 ******************************************************************************
 */

#include "rd_can_motor.h"
#include "cmsis_os.h"
#include <string.h>

/* ── 전역 — ECU_AK 단독 정의 ────────────────────────────────────────────── */
CAN_Ak_Handle_t ECU_AK[NUM_AK_MOTORS];

/* ── 공개 함수 ─────────────────────────────────────────────────────────── */
/**
 * @brief  모터 핸들 + CAN 페리페럴 + 상태머신 초기화.
 */
RD_RET RD_CAN_MOTOR_INIT(CAN_HandleTypeDef *hcan, volatile PERIPHERAL_ERROR_t *err)
{
    if (hcan == NULL || err == NULL) return RET_NOK;

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        CAN_AK_INIT(&ECU_AK[i], hcan, (uint8_t)(i + 1));   /* CAN ID 1~4 */
    }

    if (CAN_Init(hcan) != HAL_OK) {
        err->can.state.bits.lifecycle = LS_OFFLINE;
        err->can.state.bits.health    = HC_FATAL;
        return RET_NOK;
    }

    /* 상태머신 초기화 */
    err->can.rx_error_cnt        = 0;
    err->can.degraded_cnt        = 0;
    err->can.isr_err_code        = 0;
    err->can.state.bits.lifecycle = LS_READY;
    err->can.state.bits.health    = HC_OK;
    return RET_OK;
}


RD_RET RD_CAN_MOTOR_RECOVERY(PERIPHERAL_t *peripheral, volatile PERIPHERAL_ERROR_t *err)
{
    if (peripheral == NULL || err == NULL) return RET_NOK;
	err->can.state.bits.lifecycle = LS_RECOVERING;

#ifdef USE_RTOS_CAN_QUEUE
	osMessageQueueReset(canTxQueueHandle);
#endif
	HAL_CAN_AbortTxRequest(peripheral->hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        CAN_AK_INIT(&ECU_AK[i], peripheral->hcan, (uint8_t)(i + 1));   /* CAN ID 1~4 */
    }

    if (CAN_RECOVERY(peripheral->hcan) != HAL_OK) {
        err->can.state.bits.lifecycle = LS_OFFLINE;   /* 무한 재시도 방지 */
        err->can.state.bits.health    = HC_FATAL;
        return RET_NOK;
    }

    /* 성공 → 상태머신 완전 리셋 (이게 빠지면 lifecycle 이 OFFLINE 에 박혀 무한 RECOVERY) */
    err->can.rx_error_cnt        = 0;
    err->can.degraded_cnt        = 0;
    err->can.isr_err_code        = 0;
    err->can.state.bits.lifecycle = LS_READY;
    err->can.state.bits.health    = HC_OK;
    return RET_OK;
}

RD_RET RD_CAN_MOTOR_UPDATE(volatile DATA_MOTOR_t *data)
{
    if (data == NULL) return RET_NOK;
    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        taskENTER_CRITICAL();
        AK_State_t s = ECU_AK[i].state;     /* 일관된 스냅샷 */
        taskEXIT_CRITICAL();

        data->position[i] = s.position;
        data->velocity[i] = s.velocity;
        data->current[i]  = s.current;
        data->temp[i]     = s.temp_motor;
    }
    return RET_OK;
}

RD_RET RD_CAN_MOTOR_TRANSMIT(const volatile CMD_MOTOR_t *cmd)
{
    if (cmd == NULL) return RET_NOK;

    CMD_MOTOR_t snap;
    taskENTER_CRITICAL();
    memcpy(&snap, (const void *)cmd, sizeof(CMD_MOTOR_t));
    taskEXIT_CRITICAL();

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        ECU_AK[i].cmd.mode    = (AK_Control_Mode_t)snap.ctr_mode[i];
        ECU_AK[i].cmd.rpm     = snap.cmd_velocity[i];
        ECU_AK[i].cmd.current = snap.cmd_current[i];
        ECU_AK[i].cmd.pos     = snap.cmd_position[i];
        CAN_AK_WRITE(&ECU_AK[i]);   /* MODE_ESTOP 인 모터는 내부에서 skip */
    }
    return RET_OK;
}

/* ── 내부 helper ──────────────────────────────────────────────────────── */

static uint16_t pack4_err(const uint8_t v[NUM_AK_MOTORS])
{
    return ((uint16_t)(v[0] & 0x0F))       |
           ((uint16_t)(v[1] & 0x0F) <<  4) |
           ((uint16_t)(v[2] & 0x0F) <<  8) |
           ((uint16_t)(v[3] & 0x0F) << 12);
}

static uint8_t pack4_comm(const uint8_t v[NUM_AK_MOTORS])
{
    return (uint8_t)(((v[0] & 0x03)     ) |
                     ((v[1] & 0x03) << 2) |
                     ((v[2] & 0x03) << 4) |
                     ((v[3] & 0x03) << 6));
}

/* ── CHECKER ──────────────────────────────────────────────────────────── */

RD_RET RD_CAN_MOTOR_CHECKER(volatile DATA_MOTOR_t *data, volatile PERIPHERAL_ERROR_t *err)
{
    if (data == NULL || err == NULL) return RET_NOK;

    uint8_t health    = HC_OK;
    uint8_t lifecycle = err->can.state.bits.lifecycle;

    /* RECOVERING / OFFLINE 일 때는 상위가 RECOVERY 호출 전까지 검사 skip */
    if (lifecycle == LS_RECOVERING) return RET_WAIT;
    if (lifecycle == LS_OFFLINE)    return RET_NOK;

    /* ★ 에러 IT 재무장 — ErrorCallback 이 폭주 차단을 위해 끈 에러 notification 을 매 tick 복구.
     *    (ActivateNotification 은 IER 비트 set 만 하므로 idempotent. 에러가 지속되면
     *     다음 에러프레임에서 콜백이 한 번 더 캡처 후 다시 끄므로 IRQ 는 tick 당 ≤1 로 제한된다.) */
    HAL_CAN_ActivateNotification(ECU_AK[0].hcan,
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE   |
        CAN_IT_BUSOFF        | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);

    /* 1. ISR 캡처 HAL 에러 — atomic read-clear.
     *    [#1][#2] HAL_CAN_ERROR_* 는 비트 OR 마스크. switch 가 아니라 AND + 심각도
     *    우선순위 if-chain 으로 분류한다. (EWG|EPV|BOF 동시 set 시 BOF 우선) */
    uint32_t hal_err = isr_err_take(&err->can.isr_err_code);
    if (hal_err != 0) {
        if      (hal_err & HAL_CAN_ERROR_BOF)   health = HC_BUS_OFF;       /* 최우선: 버스오프 */
        else if (hal_err & HAL_CAN_ERROR_EPV)   health = HC_BUS_PASSIVE;
        else if (hal_err & HAL_CAN_ERROR_EWG)   health = HC_BUS_WARNING;
        else if (hal_err & HAL_CAN_ERROR_PARAM) health = HC_PARAM_ERR;
        else if (hal_err & HAL_CAN_ERROR_ACK)   health = HC_ACK_FAIL;
        else if (hal_err & (HAL_CAN_ERROR_STF | HAL_CAN_ERROR_FOR))
                                                health = HC_FRAMING_ERR;
        else if (hal_err & HAL_CAN_ERROR_CRC)   health = HC_CRC_ERR;
        else                                    health = HC_PROTOCOL_ERR;
        err->can.rx_error_cnt++;
    } else {
        /* CAN 은 EWG/EPV 등으로 페리페럴이 abort 되지 않으므로 연속 카운트가
         * 유효하다. 래칭되는 BOF 는 위에서 health=fatal 로 즉시 잡힘 → clean tick 리셋 OK. */
        err->can.rx_error_cnt = 0;
    }

    /* 2. per-motor 채널 상태 수집 */
    uint32_t tick = HAL_GetTick();
    uint8_t  hw_err_raw[NUM_AK_MOTORS];
    uint8_t  comm_per[NUM_AK_MOTORS];
    uint8_t  any_running  = 0;
    uint8_t  any_comm_err = 0;   /* rx 타임아웃 또는 tx 실패 (구 any_timeout) */
    uint8_t  any_hw_err   = 0;

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        taskENTER_CRITICAL();
        CAN_AK_CHECKER(&ECU_AK[i], tick);
        hw_err_raw[i]       = (uint8_t)ECU_AK[i].state.error_code;
        AK_Error_t err_temp = ECU_AK[i].error;
        taskEXIT_CRITICAL();

        comm_per[i] = 0;
        if (err_temp.rx_err_cnt > 0) comm_per[i] |= AK_COMM_RX_BIT;
        if (err_temp.tx_err_cnt > 0) comm_per[i] |= AK_COMM_TX_BIT;

        err->can_rx_cnt[i] = err_temp.rx_err_cnt;
        err->can_tx_cnt[i] = err_temp.tx_err_cnt;

        if (err_temp.last_rx_tick != 0) any_running  = 1;
        if (comm_per[i] != 0)           any_comm_err = 1;
        if (hw_err_raw[i] != 0)         any_hw_err   = 1;
    }
    data->error_code = pack4_err(hw_err_raw);
    data->comm_err   = pack4_comm(comm_per);

    /* 3. health 가중치 — HAL 에러 > 모터 hw fault > 통신 에러 */
    if (health == HC_OK && any_hw_err)                               health = HC_HW_FAULT;
    if (health == HC_OK && any_comm_err && lifecycle >= LS_RUNNING)  health = HC_TIMEOUT;

    /* 4. degraded counter — 200Hz CAN polling 기준 K */
    if (health != HC_OK) {
        uint32_t next = (uint32_t)err->can.degraded_cnt + DEGRADED_K_100HZ;
        err->can.degraded_cnt = (next > DEGRADED_CNT_MAX) ? DEGRADED_CNT_MAX : (uint16_t)next;
    } else if (err->can.degraded_cnt > 0) {
        err->can.degraded_cnt = (err->can.degraded_cnt > DEGRADED_TICK_DECAY)
                              ? (err->can.degraded_cnt - DEGRADED_TICK_DECAY) : 0;
    }

    /* 5. lifecycle 전이 */
    if (health == HC_OK && lifecycle == LS_READY && any_running) lifecycle = LS_RUNNING;

    if (lifecycle == LS_DEGRADED && err->can.degraded_cnt == DEGRADED_CNT_MAX)
        lifecycle = LS_OFFLINE;
    else if (lifecycle == LS_RUNNING && err->can.degraded_cnt >= DEGRADED_THRESHOLD_HIGH)
        lifecycle = LS_DEGRADED;
    else if (lifecycle == LS_DEGRADED && err->can.degraded_cnt <= DEGRADED_THRESHOLD_LOW)
        lifecycle = LS_RUNNING;

    /* 6. 즉시 OFFLINE 트리거 — 치명 단발(버스오프 등) OR HAL 에러 누적 임계 초과 */
    if (health >= HC_THRESHOLD_FATAL || err->can.rx_error_cnt > HAL_FATAL_CNT_TH) {
        lifecycle = LS_OFFLINE;
        if (health < HC_FATAL) health = HC_FATAL;   /* OFFLINE 사유 표시 */
    }

    /* 7. state 갱신 — ERROR_STATUS_t.can.state + data->state mirror */
    err->can.state.bits.health    = health;
    err->can.state.bits.lifecycle = lifecycle;
    data->state                   = err->can.state;

    if (lifecycle == LS_OFFLINE)        return RET_NOK;   /* 복구 필요 */
    if (health   >= HC_THRESHOLD_WARN)  return RET_WAIT;  /* 경고/에러 — 상위 모니터링 */
    return RET_OK;
}
