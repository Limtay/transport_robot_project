/*
 * rd_can_bms.c
 *
 *  PRA BMS CAN(CAN1) 드라이버 — Step 8-1 (HW 브링업 + 수신 확인).
 *  설계/Step: pra_config_wiki/pra_bms_can.md
 *
 *  🔒 격리: 본 파일의 어떤 경로도 PRA_CONTROL.STATE·전원 출력·NVIC_SystemReset 에
 *     쓰지 않는다. 통신 단절 시 CAN 서브시스템만 자체 복구.
 *
 *  인터럽트 소유(2026-10-10 변경): CAN1 RX0/SCE 인터럽트를 **.ioc(CubeMX)에서 활성** →
 *     IRQ 핸들러(CAN1_RX0/SCE_IRQHandler → HAL_CAN_IRQHandler)는 `stm32f4xx_it.c`,
 *     NVIC enable 은 `HAL_CAN_MspInit`(stm32f4xx_hal_msp.c)이 소유한다.
 *     본 파일은 **HAL weak 콜백(RxFifo0MsgPending/Error)만** 정의한다 — 핸들러/NVIC 를
 *     여기서 중복 정의하지 말 것(링크 에러).
 */

#include "rd_can_bms.h"
#include "rd_map_pra.h"   /* reg (REGISTER_t) — SENSOR/BMS 발행 대상 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"     /* taskENTER_CRITICAL — reg 쓰기 보호(프로젝트 규약) */
#include "task.h"

/* main.c (CubeMX) 전역 CAN 핸들 — rd_system.h 가 huart6 를 extern 하는 패턴과 동일 */
extern CAN_HandleTypeDef hcan1;

/* 폴링 주기 — 저율 로깅(결정 §4-2). 3요청 동시 적재 후 이 간격 대기 */
#define BMS_POLL_PERIOD_MS   1000u

/* 응답 타임아웃 — now - last_rx_tick 이 이 값 초과 시 CAN 자기복구.
 * 유저 사양 "1주기(1000ms) 무응답" + 틱 지터 마진(응답하는 BMS 오탐 방지). */
#define BMS_RX_TIMEOUT_MS    1500u

/* 단일 BMS 핸들 (본 파일 소유) */
BMS_Handle_t BMS = {0};

/* ── RX 필터 (확장 ID, 29-bit mask 모드) ──────────────────────────────────────
 * 응답 ID 패턴 0x18[데이터ID]4001 만 통과(FIFO0). 데이터ID 바이트[23:16]=don't-care.
 *   ID   (29b) = 0x18004001  : 상위 0x18 · 호스트 0x40 · BMS 0x01 고정, 데이터ID 0
 *   MASK (29b) = 0x1F00FFFF  : [28:24]·[15:0] 매치, [23:16](데이터ID) don't-care
 *     ※ 29-bit 공간 마스크(0x1F..) — 바이트 관점 0xFF00FFFF 와 유효 ID 에선 동일
 *       (상위 비트[31:29]는 29-bit ID 에서 항상 0). <<3 오버플로 회피 위해 0x1F 사용.
 * bxCAN 필터 레지스터는 CAN_RIxR 배치(= id<<3 | IDE<<2 | RTR<<1)를 따른다. */
#define BMS_FILTER_ID_29B    0x18004001u
#define BMS_FILTER_MASK_29B  0x1F00FFFFu

/* ── 디버그/관찰 전역 ─────────────────────────────────────────────────────── */
volatile uint32_t      BMS_rx_count     = 0;
volatile uint32_t      BMS_err_count    = 0;
volatile uint32_t      BMS_isr_err_code = 0;
volatile BMS_RxDebug_t BMS_last_rx      = {0};

/* ── 시계 포트 ────────────────────────────────────────────────────────────── */
uint32_t HW_NowTick(void)
{
    return HAL_GetTick();   /* TIM5 부재 — ms 단위. register ×0.1ms 스케일 불일치는 기존 미결 계승 */
}

/* ── INIT : 필터 + 핸들 clear + Start + Notification ──────────────────────────
 * NVIC enable·IRQ 핸들러는 CubeMX(MspInit/it.c) 소유 — 여기선 건드리지 않는다. */
RD_RET RD_CAN_BMS_INIT(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return RET_NOK;

    /* 1) RX 필터 (bank0, 32-bit mask, FIFO0) */
    uint32_t id_reg   = (BMS_FILTER_ID_29B   << 3) | 0x4u;   /* IDE=1, RTR=0 */
    uint32_t mask_reg = (BMS_FILTER_MASK_29B << 3) | 0x4u;   /* IDE 매치 요구 → 확장 프레임만 */

    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (uint16_t)(id_reg   >> 16);
    f.FilterIdLow          = (uint16_t)(id_reg   & 0xFFFFu);
    f.FilterMaskIdHigh     = (uint16_t)(mask_reg >> 16);
    f.FilterMaskIdLow      = (uint16_t)(mask_reg & 0xFFFFu);
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;   /* F446 dual-CAN: CAN1 bank 0~13 */

    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return RET_NOK;

    /* 2) BMS 핸들 clear + Start */
    for (int i = 0; i < 8; i++) {
        BMS.msg_vic.data[i]   = 0;
        BMS.msg_temp.data[i]  = 0;
        BMS.msg_fault.data[i] = 0;
    }
    BMS.msg_vic.valid = BMS.msg_temp.valid = BMS.msg_fault.valid = 0;
    BMS.last_rx_tick  = HW_NowTick();   /* init/recovery 시점부터 타임아웃 측정(즉시 오탐 방지) */

    if (HAL_CAN_Start(hcan) != HAL_OK) return RET_NOK;

    /* 3) Notification — RX FIFO0 pending + 버스헬스 에러만.
     *    CAN_IT_LAST_ERROR_CODE(LEC)는 구독 제외: form/stuff/CRC 등 단발 프레임 에러까지
     *    콜백을 띄워 오탐이 됨(수신 자체는 정상). 복구는 타임아웃(8-3) + EWG/EPV/BOF 로 판정. */
    if (HAL_CAN_ActivateNotification(hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK) {
        return RET_NOK;
    }
    return RET_OK;
}

/* ── RECOVERY : CAN 서브시스템만 재초기화 (자기복구) ──────────────────────────
 * Stop → DeInit(MspDeInit: NVIC/GPIO 해제) → Init(MspInit: 재설정) → INIT(필터/Start/
 * Notification 재적용 + 에러 notification 재무장). 🔒 시스템/전원에 전파하지 않는다.
 * ECU_V3 CAN_RECOVERY 와 동일 개념 — 통신 단절(타임아웃/HAL 에러) 시 RD_TASK_CAN 이 호출. */
RD_RET RD_CAN_BMS_RECOVERY(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return RET_NOK;

    HAL_CAN_Stop(hcan);                                  /* best-effort (에러 상태여도 진행) */
    if (HAL_CAN_DeInit(hcan) != HAL_OK) return RET_NOK;
    if (HAL_CAN_Init(hcan)   != HAL_OK) return RET_NOK;  /* hcan->Init(MX 설정)은 유지됨 */

    BMS_isr_err_code = 0;                                /* 복구 사이클 — 에러 캡처 리셋 */
    return RD_CAN_BMS_INIT(hcan);                        /* 필터/clear/Start/Notification 재무장 */
}

/* IRQ 핸들러(CAN1_RX0/SCE)는 stm32f4xx_it.c(CubeMX 생성)가 소유 — 여기서 정의하지 않는다.
 * 그 핸들러의 HAL_CAN_IRQHandler(&hcan1) → 아래 HAL weak 콜백으로 디스패치된다. */

/* ── 요청 송신 (메일박스 직접) ────────────────────────────────────────────── */
/* 3 요청(0x90/0x92/0x98)을 TX 메일박스 3개에 직접 적재. 큐 없음(결정 §4-1).
 * AutoRetransmission DISABLE — 무ACK(BMS 부재) 시 1회 시도 후 메일박스 자동 해제. */
RD_RET CAN_BMS_REQUEST_ALL(CAN_HandleTypeDef *hcan)
{
    static const uint8_t ids[3] = { BMS_ID_VIC, BMS_ID_TEMP, BMS_ID_FAULT };
    uint8_t zero[8] = {0};
    uint8_t ok = 0;

    for (int i = 0; i < 3; i++) {
        CAN_TxHeaderTypeDef tx = {0};
        tx.StdId = 0x00;
        tx.IDE   = CAN_ID_EXT;
        tx.ExtId = 0x18000140u | ((uint32_t)ids[i] << 16);   /* 0x18[id]0140 */
        tx.RTR   = CAN_RTR_DATA;
        tx.DLC   = 8;
        tx.TransmitGlobalTime = DISABLE;

        uint32_t mailbox;
        if (HAL_CAN_AddTxMessage(hcan, &tx, zero, &mailbox) == HAL_OK) ok++;
    }
    return (ok == 3) ? RET_OK : (ok > 0 ? RET_WAIT : RET_NOK);
}

/* ── RX pop / apply (ISR 컨텍스트) ────────────────────────────────────────── */
uint8_t CAN_BMS_RX_POP(CAN_HandleTypeDef *hcan, BMS_RxFrame_t *frame)
{
    if (frame == NULL) return 0;

    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) return 0;

    frame->ext_id  = (hdr.IDE == CAN_ID_EXT) ? hdr.ExtId : hdr.StdId;
    frame->data_id = (uint8_t)((frame->ext_id >> 16) & 0xFFu);
    frame->dlc     = (uint8_t)hdr.DLC;
    for (int i = 0; i < 8; i++) frame->data[i] = data[i];
    return 1;
}

uint8_t CAN_BMS_RX_APPLY(BMS_Handle_t *h, const BMS_RxFrame_t *frame)
{
    if (h == NULL || frame == NULL) return 0;

    BMS_Msg_t *m;
    switch (frame->data_id) {
        case BMS_ID_VIC:   m = &h->msg_vic;   break;
        case BMS_ID_TEMP:  m = &h->msg_temp;  break;
        case BMS_ID_FAULT: m = &h->msg_fault; break;
        default: return 0;   /* 대상 3종 외(필터는 통과하나 미요청) 무시 */
    }

    for (int i = 0; i < 8; i++) m->data[i] = frame->data[i];
    m->valid        = 1;
    h->last_rx_tick = HW_NowTick();   /* 신선도/타임아웃 기준 — 임의 3종 수신 시 갱신 */
    return 1;
}

/* ── 디코드 → 레지스터 발행 (SENSOR/BMS, addr 45~59) ──────────────────────────
 * 핸들 raw(ISR 적재) → big-endian 조립 + 오프셋/스케일 → reg.sensor_bms.
 * 변환 규칙: BMS_protocol_can.md §7.
 *   voltage  : BE(B0,B1)             uint16 ×0.1V (오프셋 없음)
 *   current  : (int16)(BE(B4,B5)-30000)  — CAN uint16(30000 offset) → 레지스터 int16(오프셋 제거, 0=무부하)
 *   soc      : BE(B6,B7)             uint16 ×0.1%
 *   max_temp : 0x92 B0 그대로        uint8 (40 offset **보존** — 리더가 -40. uint8 라 음수 못 담아 offset 유지, current 와 비대칭)
 *   err_lsb  : 0x98 B0~3, err_hsb : B4~7  (바이트별 비트플래그, LE 패킹으로 addr52~59 에 B0~7 순서 매핑)
 * reg 쓰기는 프로젝트 규약(taskENTER_CRITICAL). 미수신(valid=0) 필드는 0 발행. */
static inline uint16_t be16(uint8_t hi, uint8_t lo) { return (uint16_t)(((uint16_t)hi << 8) | lo); }

void RD_CAN_BMS_UPDATE(void)
{
    uint8_t vic[8], fault[8], temp0;
    uint8_t vic_ok, temp_ok, fault_ok;

    /* 1) ISR 가 쓰는 raw 캐시 일관 스냅샷 (CAN IRQ prio5 ≤ syscall 임계 → critical 로 마스킹) */
    taskENTER_CRITICAL();
    for (int i = 0; i < 8; i++) { vic[i] = BMS.msg_vic.data[i]; fault[i] = BMS.msg_fault.data[i]; }
    temp0    = BMS.msg_temp.data[0];
    vic_ok   = BMS.msg_vic.valid;
    temp_ok  = BMS.msg_temp.valid;
    fault_ok = BMS.msg_fault.valid;
    taskEXIT_CRITICAL();

    /* 2) 디코드 */
    uint16_t voltage = 0, soc = 0;
    int16_t  current = 0;
    uint8_t  max_temp = 0;
    uint32_t err_lsb = 0, err_hsb = 0;

    if (vic_ok) {
        voltage = be16(vic[0], vic[1]);
        current = (int16_t)((int32_t)be16(vic[4], vic[5]) - 30000);   /* 오프셋 제거 → 0=무부하 */
        soc     = be16(vic[6], vic[7]);
    }
    if (temp_ok) {
        max_temp = temp0;   /* 40 offset 보존 (리더가 -40) */
    }
    if (fault_ok) {
        err_lsb = (uint32_t)fault[0] | ((uint32_t)fault[1] << 8) |
                  ((uint32_t)fault[2] << 16) | ((uint32_t)fault[3] << 24);
        err_hsb = (uint32_t)fault[4] | ((uint32_t)fault[5] << 8) |
                  ((uint32_t)fault[6] << 16) | ((uint32_t)fault[7] << 24);
    }

    /* 3) reg SENSOR/BMS 발행 */
    taskENTER_CRITICAL();
    reg.sensor_bms.voltage       = voltage;
    reg.sensor_bms.current       = current;
    reg.sensor_bms.soc           = soc;
    reg.sensor_bms.max_cell_temp = max_temp;
    reg.sensor_bms.error_lsb     = err_lsb;
    reg.sensor_bms.error_hsb     = err_hsb;
    /* delta_tick(60)/state(61) 는 Step 8-5/8-6 */
    taskEXIT_CRITICAL();
}

/* ── Checker : 채널 상태(STATE_t) + 신선도 발행 (로깅 전용, 격리) ──────────────
 * 🔒 addr61 state / addr60 delta_tick 발행만 — 어떤 시스템 카운터/FSM 에도 가산 금지.
 * (ECU 의 OFFLINE→fatal→SYS_STATE_FAULT escalation·degraded decay 모델은 미이식 —
 *  1Hz 로깅 채널엔 과함. 복구는 RD_TASK_CAN 자기완결(8-3)이 단독 담당.)
 *   health : HAL 에러(BOF/EPV/EWG) > 응답 타임아웃(HC_TIMEOUT) > HC_OK
 *   lifecycle : 첫 응답 전=LS_READY / 신선=LS_RUNNING / 끊김=LS_DEGRADED */
volatile STATE_t BMS_ch_state = {0};   /* 디버그 관찰용 CAN1 채널 상태 */

void RD_CAN_BMS_CHECKER(void)
{
    uint32_t now  = HW_NowTick();
    uint32_t dt   = now - BMS.last_rx_tick;
    uint8_t fresh = (dt <= BMS_RX_TIMEOUT_MS);
    uint8_t seen  = (BMS.msg_vic.valid | BMS.msg_temp.valid | BMS.msg_fault.valid);
    uint32_t e    = BMS_isr_err_code;   /* recovery 가 clear (읽기만) */

    uint8_t health;
    if      (e & HAL_CAN_ERROR_BOF) health = HC_BUS_OFF;
    else if (e & HAL_CAN_ERROR_EPV) health = HC_BUS_PASSIVE;
    else if (e & HAL_CAN_ERROR_EWG) health = HC_BUS_WARNING;
    else if (seen && !fresh)        health = HC_TIMEOUT;
    else                            health = HC_OK;

    uint8_t lc;
    if      (!seen) lc = LS_READY;      /* 첫 응답 전(BMS 미연결/부팅) */
    else if (fresh) lc = LS_RUNNING;    /* 정상 수신 */
    else            lc = LS_DEGRADED;   /* 응답 끊김(복구 루프 재시도 중) */

    STATE_t st;
    st.raw = CH_PACK(lc, health);
    BMS_ch_state = st;

    /* delta_tick: **BMS 예외 스케일 ×0.1s** (register CSV 변경 반영 — 타 센서 ×0.1ms 와 다름).
     * dt[ms]/100 = 0.1s 단위. 0xFF 포화(=25.5s↑ 지연=오염). 1Hz 폴링서 정상치 ~10(1.0s). */
    uint32_t d01s  = dt / 100u;
    uint8_t  delta = (d01s > 0xFFu) ? 0xFFu : (uint8_t)d01s;

    taskENTER_CRITICAL();
    reg.sensor_bms.state      = st;      /* addr61 — CAN1 채널 종합 상태(로깅) */
    reg.sensor_bms.delta_tick = delta;   /* addr60 — 신선도(ms 포화) */
    taskEXIT_CRITICAL();
}

/* ── HAL 콜백 (weak override) ─────────────────────────────────────────────── */
/* RX FIFO0 수신 — FIFO 를 비우며 pop → apply(핸들 적재). 디버그 캡처(BMS_last_rx) 유지. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    BMS_RxFrame_t f;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (!CAN_BMS_RX_POP(hcan, &f)) break;

        BMS_rx_count++;

        /* 디버그 스냅샷 (최근 임의 프레임, volatile 단일 store) */
        BMS_last_rx.ext_id  = f.ext_id;
        BMS_last_rx.data_id = f.data_id;
        BMS_last_rx.dlc     = f.dlc;
        for (int i = 0; i < 8; i++) BMS_last_rx.data[i] = f.data[i];

        CAN_BMS_RX_APPLY(&BMS, &f);   /* 3종 매칭 시 핸들 적재 + last_rx_tick latch */
    }
}

/* CAN 에러 — Step 8-1: 코드 캡처 + 카운터. IRQ 폭주 차단 위해 에러 notification 을
 * 끈다(ECU 패턴). Step 8-5 Checker 가 매 tick 재무장(ActivateNotification)할 예정 —
 * 현 단계엔 재무장 없음(브링업 1회 캡처로 충분, RX notification 은 유지됨). */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    BMS_isr_err_code |= hcan->ErrorCode;   /* HAL_CAN_ERROR_* 비트 OR */
    BMS_err_count++;

    HAL_CAN_DeactivateNotification(hcan,
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF | CAN_IT_ERROR);
}

/* ── 태스크 진입점 ────────────────────────────────────────────────────────── */
/* main `Start_can1` 에 유저가 `RD_TASK_CAN();` 수동 추가.
 * INIT 성공까지 CAN 만 재시도(상위/전원 무전파) → ~1s 주기로 3요청 동시 적재 + 자기복구 판정.
 * 수신은 ISR 이 BMS 핸들/BMS_last_rx 에 비동기 적재.
 * 🔒 격리: 복구는 CAN 서브시스템만 — PRA 동작·전원·리붓과 무관. */
void RD_TASK_CAN(void)
{
    while (RD_CAN_BMS_INIT(&hcan1) != RET_OK) {
        osDelay(100);   /* 격리: 실패해도 CAN 재시도만, 시스템 리셋/전파 없음 */
    }

    for (;;) {
        CAN_BMS_REQUEST_ALL(&hcan1);   /* 0x90/0x92/0x98 요청 → 메일박스 3개 */
        osDelay(BMS_POLL_PERIOD_MS);   /* 이 동안 ISR 이 응답 적재 */

        RD_CAN_BMS_UPDATE();           /* 핸들 raw → reg.sensor_bms 발행(디코드) */
        RD_CAN_BMS_CHECKER();          /* 채널 state(addr61) + delta_tick(addr60) 발행(로깅) */

        /* 자기완결 복구 판정 (시스템 무전파):
         *   ① 응답 타임아웃(1 폴링 주기+마진 동안 무응답) 또는
         *   ② HAL 버스헬스 에러(EWG/EPV/BOF — ErrorCallback 캡처) */
        uint8_t rx_timeout = ((HW_NowTick() - BMS.last_rx_tick) > BMS_RX_TIMEOUT_MS);
        uint8_t hal_error  = (BMS_isr_err_code != 0);

        if (rx_timeout || hal_error) {
            RD_CAN_BMS_RECOVERY(&hcan1);   /* CAN 만 재초기화 → last_rx_tick/에러 리셋됨 */
        }
    }
}
