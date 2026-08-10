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

RD_RET RD_CAN_MOTOR_TRANSMIT(const CMD_MOTOR_t *cmd, uint8_t motor_mask)
{
    if (cmd == NULL) return RET_NOK;

    CMD_MOTOR_t snap;
    taskENTER_CRITICAL();
    memcpy(&snap, cmd, sizeof(CMD_MOTOR_t));
    taskEXIT_CRITICAL();

    /* 실효 마스크 = 설정 마스크 ∩ 응답 중인 모터 (A, 2026-08-03).
     * 부재 모터로 프레임이 나가면 버스에 ACK 를 줄 노드가 없어 HAL_CAN_ERROR_ACK 이 폭주하고,
     * rx_error_cnt>HAL_FATAL_CNT_TH → LS_OFFLINE → fatal escalation → SYS_STATE_FAULT 로 이어졌다.
     * (구 코드: ESTOP 경로의 CAN_AK_ESTOP 이 존재 게이트 없이 motor_on=1 을 세워 이 경로가 열렸다.)
     * 여기서 걸러 두면 어떤 상위 경로도 부재 모터에 TX 할 수 없다.
     * per-motor 로 거르는 이유: 4개 중 1개만 끊겨도 전체 TX 를 막으면 살아있는 모터의 제동까지
     * 못 걸게 되어 오히려 위험 — 살아있는 모터에는 정상적으로 명령/제동이 나가야 한다. */
    motor_mask = RD_CAN_MOTOR_READY_MASK(motor_mask);

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        if (!(motor_mask & (1u << i))) continue;   /* 마스크 제외/부재 모터 TX skip */
        ECU_AK[i].cmd.mode    = (AK_Control_Mode_t)snap.ctr_mode[i];
        ECU_AK[i].cmd.rpm     = snap.cmd_velocity[i];
        ECU_AK[i].cmd.current = snap.cmd_current[i];
        ECU_AK[i].cmd.pos     = snap.cmd_position[i];
        CAN_AK_WRITE(&ECU_AK[i]);   /* MODE_ESTOP 인 모터는 내부에서 skip */
    }
    return RET_OK;
}

/* 존재/신선도 게이트 (H1 개정, failsafe_analysis_260717.md §8-P1):
 * mask 된 모든 모터의 상시 피드백(AK 설정: 명령 무관 100Hz 송신)이
 * MOTOR_COMM_FAULT_MS 이내로 신선한가.
 *
 * 세 용도:
 *   1) motor_on 전제조건 (ALL_READY, rd_system.c) — 모터 전원이 아직 없으면 TX 자체를
 *      시작하지 않아 빈 버스 ACK 에러 폭주→FAULT (전원 인가 순서 문제) 가 원천 차단.
 *      늦게 켜진 모터는 피드백이 보이는 즉시 자동 합류.
 *   2) 구동 중(motor_on==1) !ALL_READY = 주행 중 통신 상실 → ESTOP_SW (자동복귀형).
 *   3) TX 실효 마스크 (READY_MASK, TRANSMIT 내부) — 1)은 상위 정책이라 ESTOP 경로가
 *      우회할 수 있었다 (CAN_AK_ESTOP 의 무조건 motor_on=1). 드라이버에서 한 번 더
 *      per-motor 로 걸러 어떤 경로도 부재 모터에 프레임을 내지 못하게 한다 (A).
 *
 * tick==0 (INIT/RECOVERY 후 미접촉) 은 별도 분기 불필요 — now-0 이 항상 임계 초과라
 * 자연히 "not ready". 기동 유예도 불필요 — 미접촉 모터는 게이트가 motor_on 을 막아
 * 주행이 시작되지 않으므로 오탐 자체가 성립하지 않는다. */
uint8_t RD_CAN_MOTOR_READY_MASK(uint8_t motor_mask)
{
    uint32_t now   = HAL_GetTick();
    uint8_t  ready = 0;
    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        if (!(motor_mask & (1u << i))) continue;
        if (now - ECU_AK[i].error.last_rx_tick <= MOTOR_COMM_FAULT_MS) ready |= (uint8_t)(1u << i);
    }
    return ready;
}

uint8_t RD_CAN_MOTOR_ALL_READY(uint8_t motor_mask)
{
    motor_mask &= (uint8_t)((1u << NUM_AK_MOTORS) - 1u);   /* 상위 미사용 비트 무시 */
    if (motor_mask == 0) return 0;                          /* 활성 모터 0 = 구동 불가 */
    return (RD_CAN_MOTOR_READY_MASK(motor_mask) == motor_mask) ? 1 : 0;
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

RD_RET RD_CAN_MOTOR_CHECKER(volatile DATA_MOTOR_t *data, volatile PERIPHERAL_ERROR_t *err, uint8_t motor_mask)
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
    uint8_t  any_hw_err   = 0;

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        /* 마스크 제외 모터 (H1): TX 도 없고 응답 기대도 없음 — 타임아웃/에러/worst 집계에서
         * 제외하고 발행 필드도 0 으로. (제외 모터의 delta_tick 은 자연히 0xFF stale) */
        if (!(motor_mask & (1u << i))) {
            hw_err_raw[i] = 0;
            comm_per[i]   = 0;
            err->can_rx_cnt[i] = 0;
            err->can_tx_cnt[i] = 0;
            continue;
        }

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

        /* ever_seen 래치 (C): 첫 RX 를 본 순간 set, 이후 유지 (RECOVERY 로도 지워지지 않음).
         * any_running 은 RECOVERY 마다 리셋되는 "이번 사이클 수신 이력", motor_seen 은
         * "전원 인가 이후 한 번이라도 존재를 확인했는가" — escalation 허용 여부의 근거. */
        if (err_temp.last_rx_tick != 0) {
            any_running      = 1;
            err->motor_seen |= (uint8_t)(1u << i);
        }
        if (hw_err_raw[i] != 0)         any_hw_err   = 1;
    }
    /* mask 된 모터 중 한 번이라도 확인된 적이 있는가 = 이 채널이 실제로 증명된 적 있는가 */
    uint8_t seen = (uint8_t)(err->motor_seen & motor_mask);
    data->error_code = pack4_err(hw_err_raw);
    data->comm_err   = pack4_comm(comm_per);   /* per-motor 발행은 유지 (Orin 진단용) */

    /* 3. health 가중치 — HAL 에러 > 모터 hw fault.
     * per-motor RX 타임아웃(comm_err 발행값)은 채널 health/degraded 에서 분리 (H1):
     * 모터 무응답으로 채널 전체가 DEGRADED→OFFLINE→RECOVERY 순환(0.5s 주기 큐리셋)하던
     * 원인 제거 — 채널 escalation 은 HAL/버스 에러 전용, 모터 무응답 정지는
     * RD_CAN_MOTOR_ALL_READY (게이트 + ESTOP_SW, systemTask) 가 담당. */
    if (health == HC_OK && any_hw_err) health = HC_HW_FAULT;

    /* 4. degraded counter — 200Hz CAN polling 기준 K */
    if (health != HC_OK) {
        uint32_t next = (uint32_t)err->can.degraded_cnt + DEGRADED_K_100HZ;
        err->can.degraded_cnt = (next > DEGRADED_CNT_MAX) ? DEGRADED_CNT_MAX : (uint16_t)next;
    } else if (err->can.degraded_cnt > 0) {
        err->can.degraded_cnt = (err->can.degraded_cnt > DEGRADED_TICK_DECAY)
                              ? (err->can.degraded_cnt - DEGRADED_TICK_DECAY) : 0;
    }

    /* 5. lifecycle 전이 */
    /* READY → RUNNING 승격:
     *   - health OK + 수신 이력(any_running) : 정상 승격
     *   - health != OK : 에러 = 버스가 동작은 했다는 뜻 → RUNNING 으로 승격해
     *     RUNNING→DEGRADED→OFFLINE→RECOVERY escalation 경로를 타게 한다.
     *     (READY 에 머문 채 health 만 에러로 굳어 복구가 영영 안 걸리는 freeze 방지)
     *     단 seen(C) 조건 추가 — 한 번도 모터를 본 적 없는 채널은 에러만으로 승격시키지 않는다.
     *     아직 안 켜진 모터를 향한 에러로 escalation 사다리를 오르면 부팅 순서가 FAULT 가 된다.
     *     freeze 우려는 없음: 첫 RX 가 들어오면 any_running 으로 정상 승격하고,
     *     그 전까지는 TX 자체가 없어(A) 복구할 대상도 없다. */
    if (lifecycle == LS_READY && (any_running || (health != HC_OK && seen))) lifecycle = LS_RUNNING;

    if (lifecycle == LS_DEGRADED && err->can.degraded_cnt == DEGRADED_CNT_MAX)
        lifecycle = LS_OFFLINE;
    else if (lifecycle == LS_RUNNING && err->can.degraded_cnt >= DEGRADED_THRESHOLD_HIGH)
        lifecycle = LS_DEGRADED;
    else if (lifecycle == LS_DEGRADED && err->can.degraded_cnt <= DEGRADED_THRESHOLD_LOW)
        lifecycle = LS_RUNNING;

    /* 6. 즉시 OFFLINE 트리거 — 치명 단발(버스오프 등) OR HAL 에러 누적 임계 초과.
     *    seen(C) 게이트: 한 번도 확인된 적 없는 채널은 OFFLINE 으로 떨어뜨리지 않는다
     *    (RET_NOK → 상위 fatal_can1_cnt → SYS_STATE_FAULT 사다리의 입구).
     *    미확인 상태의 이상은 health/hw.error.bit.can 으로 Orin 에 보고만 하고,
     *    주행은 ALL_READY 게이트가 이미 막고 있으므로 안전하다. */
    if (seen && (health >= HC_THRESHOLD_FATAL || err->can.rx_error_cnt > HAL_FATAL_CNT_TH)) {
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
