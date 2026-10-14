/*
 * rd_register_pra.h
 *
 *  PRA(=Orin PCU, PacketID 0xA1) 256-byte 레지스터 맵 (register_map_pra.CSV 기준).
 *  - Little-endian, 1 addr = 1 byte, 실제값 = raw × Scale.
 *  - 외부 RS485 마스터(Orin) ↔ 내부 PERIPHERAL 의 단일 source-of-truth.
 *  - 명칭 규약: STM 개발문서=PRA / Orin·외부 관점=PCU (동일 노드).
 *
 *  영역 레이아웃 (총 256 byte) — #define 기준:
 *      [   0 :  15] DEFINE           ( 16 byte)  시스템 설정 파라미터   R/W (잠금키 필요)
 *      [  16 :  28] SYSTEM           ( 13 byte)  시스템 상태            R/O
 *      [  29 :  37] RSVD0            (  9 byte)  미래 확장용            R/O
 *      [  38 :  47] SENSOR/ADC       ( 10 byte)  자체 ADC (NTC/전류/전압) R/O
 *      [  48 :  64] SENSOR/BMS       ( 17 byte)  BMS(CAN1) 피드백       R/O
 *      [  65 :  67] CMD/POWER        (  3 byte)  전원 스위치 제어 명령  R/W
 *      [  68 :  79] RSVD1            ( 12 byte)  미래 확장용            R/O
 *      [  80 :  99] DIAG             ( 20 byte)  진단 카운터 + 예약     R/O
 *      [ 100 : 255] RSVD2            (156 byte)  미래 확장용            R/O
 *
 *  ⚠ 의존성: `state` 필드는 rd_common.h 의 STATE_t 를 사용한다. 현 PRA rd_common.h
 *     는 최소본(RD_RET 만)이라, 통신/Checker 이식(Step 4/6) 시 DPC_B/ECU 의
 *     rd_common.h(STATE_t / LS_* / HC_*) 를 함께 이식해야 본 헤더가 컴파일된다.
 *
 *  Created on: 2026-09-17
 *      Author: swarm
 */

#ifndef INC_RD_REGISTER_PRA_H_
#define INC_RD_REGISTER_PRA_H_

#include <stdint.h>
#include <assert.h>
#include "rd_common.h"   /* STATE_t (lifecycle+health), LS_* / HC_* 정의 (이식 예정) */

/*
 * ===== "state" 필드 공통 의미 =====
 *  본 레지스터 맵의 모든 `state` 필드는 rd_common.h 의 `STATE_t` (uint8_t union).
 *      bit[3:0] = lifecycle  (LS_INIT=0 ... LS_RUNNING=2 ... LS_OFFLINE=15)
 *      bit[7:4] = health     (HC_OK=0 ... HC_FATAL=15)
 *  외부 마스터는 1 byte 를 읽어 분해: lifecycle = state & 0x0F / health = (state>>4)&0x0F
 */

/* ===== Region offset / size (bytes) — dispatch/marshal LUT 용 ===== */
#define REG_TOTAL_SIZE              256

#define REG_DEFINE_OFFSET             0
#define REG_DEFINE_SIZE              16

#define REG_SYS_OFFSET               16
#define REG_SYS_SIZE                 13

#define REG_RSVD0_OFFSET             29
#define REG_RSVD0_SIZE                9

#define REG_SENSOR_ADC_OFFSET        38
#define REG_SENSOR_ADC_SIZE          10   /* 2026-10-13: 7→10 (current/vin/vout int8→int16, ×0.1 단위) */

#define REG_SENSOR_BMS_OFFSET        48   /* 2026-10-13: 45→48 (SENSOR/ADC +3 밀림) */
#define REG_SENSOR_BMS_SIZE          17

#define REG_CMD_POWER_OFFSET         65   /* 2026-10-13: 62→65 */
#define REG_CMD_POWER_SIZE            3

#define REG_RSVD1_OFFSET             68   /* 2026-10-13: 65→68 */
#define REG_RSVD1_SIZE               12   /* 2026-10-13: 15→12 (SENSOR/ADC +3 흡수, DIAG 80 고정, 총 256 유지) */

#define REG_DIAG_OFFSET              80
#define REG_DIAG_SIZE                20

#define REG_RSVD2_OFFSET            100
#define REG_RSVD2_SIZE              156

/* CMD/POWER 쓰기 감시 범위 (cmd_write_tick 갱신 판정용) */
#define REG_CMD_POWER_S_OFFSET       65   /* ctr_mode[0] 시작 (2026-10-13: 62→65) */
#define REG_CMD_POWER_E_OFFSET       67   /* ctr_mode[2] 마지막 바이트 (2026-10-13: 64→67) */

/* ===== sys_write_mode 값 ===== */
#define SYS_WRITE_LOCK              0    /* DEFINE 영역 쓰기 잠금 (기본) */
#define SYS_WRITE_UNLOCK           1    /* DEFINE 영역 쓰기 허용 */

/* ===== HW bitfield 비트 정의 (hw_reset / hw_fatal / hw_error 공통) =====
 *  비트 순서 = SYSTEM.degraded_cnt 인덱스 순서와 통일: CAN1(bit0) | uart2(bit1) | I2C1(bit2) | RSVD×5.
 *  rd_system.h 의 HARDWARE_STATUS_t 비트 순서와 일치해야 한다. bit[3..7] reserved.
 *  (2026-09-17 Step3: degraded_cnt 순서에 맞춰 CAN1→uart2 로 정렬) */
#define HW_BIT_CAN1                 (1u << 0)   /* BMS CAN (CAN1)  — degraded_cnt[0] */
#define HW_BIT_UART2                (1u << 1)   /* Orin RS485 (USART6, 레거시 명칭 uart2) — degraded_cnt[1] */
#define HW_BIT_I2C1                 (1u << 2)   /* MCP23017 (I2C1) */

/* ===== Default 값 (RD_MAP_INIT 에서 세팅) ===== */
#define DEF_ERR_TIMEOUT            15   /* [ms]     RX timeout error 임계 (10~500) */
#define DEF_FATAL_TIMEOUT          10   /* [100ms]  RX timeout fatal 임계 (0=감지 X) */
#define DEF_ERR_CNT                 5   /* [count]  Error 임계 카운트 (0~10)  */
#define DEF_FATAL_CNT              10   /* [count]  Fatal 임계 카운트 (0=감지 X) */

/* ===== CMD/POWER.ctr_mode 값 (addr 61~63) ===== */
#define CTR_MODE_OFF               0
#define CTR_MODE_ON                1

/* ===== SENSOR/BMS.bms_state 값 (addr 53) — BMS 가 CAN 으로 보고한 상태 ===== */
#define BMS_STATE_STOP           0
#define BMS_STATE_CHARGE         1
#define BMS_STATE_DISCHARGE      2

/* ===== sys_state(addr 23, R/O) 값 =====
 *  값 정의용. 레지스터 필드는 uint8_t 유지(packing 보존).
 *  실제 enum(SYSTEM_STATE_e)은 rd_system.h 소유. **PRA_CONTROL.STATE 미러**(2026-10-04).
 *      0=INIT / 1=RUN_OFF / 2=RUN_TRN / 3=RUN_ON / 255=FAULT
 *  (통신/채널 health 는 hw_error/hw_fatal/degraded_cnt 로 별도 노출) */


/* ===== [DEFINE] addr   0~15 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ uint8_t sys_write_mode;  /* R/W: 0=LOCK / 1=UNLOCK (default 0) — DEFINE 영역 쓰기 허용 키 */
    /* addr   1 */ uint8_t err_timeout;     /* R/W: RX timeout error 임계 [ms]     (default 15, range 10~500) */
    /* addr   2 */ uint8_t fatal_timeout;   /* R/W: RX timeout fatal 임계 [100ms]  (default 10, 0=감지 X) */
    /* addr   3 */ uint8_t err_cnt;         /* R/W: Error 임계 카운트 [count]      (default 5,  range 0~10) */
    /* addr   4 */ uint8_t fatal_cnt;       /* R/W: Fatal 임계 카운트 [count]      (default 10, 0=감지 X) */
    /* addr   5 */ uint8_t hw_reset;        /* R/W: Orin 이 PRA 특정 채널 수동 리셋 — bitfield HW_BIT_UART2/CAN1/I2C1 */
    /* addr   6 */ uint8_t reserved[10];    /* 미래 확장용 */
} DEFINE_t;

/* ===== [SYSTEM] addr  16~28 (13 bytes) — R/O ===== */
typedef struct __attribute__((packed)) {
    /* addr  16 */ uint8_t  degraded_cnt[4];  /* R/O 통신 오염도 [%]: idx0=CAN1 / idx1=uart2(Orin) / idx2~3=RSVD
                                               *      값 = (uint8_t)((degraded_cnt_raw * 26) >> 8) — 0~1000 raw → 0~100% */
    /* addr  20 */ uint8_t  hw_error;         /* R/O bitfield: HW_BIT_* — 현재 활성 에러 (HC_WARN 이상) */
    /* addr  21 */ uint8_t  hw_fatal;         /* R/O bitfield: 재초기화 필요 수준의 치명 에러 */
    /* addr  22 */ uint8_t  hw_reset;         /* R/O bitfield: 현재 활성 에러 상태 미러 (Hardware_Error_FLAG_t 매핑) */
    /* addr  23 */ uint8_t  sys_state;        /* R/O 시스템 상태 SYSTEM_STATE_e (0=INIT/1=MANUAL/2=AUTO/3=ESTOP_SW/4=ESTOP_HW/5=FAULT) */
    /* addr  24 */ uint8_t  rs485_proc_delta; /* R/O ×0.1 [ms] Orin request→response 소요시간 */
    /* addr  25 */ uint32_t realtime_tick;    /* R/O ×0.1 [ms] TIM5 10kHz 카운터 (tim_cnt) — PRA alive time */
} DATA_SYSTEM_t;

/* ===== [RSVD0] addr  29~37 (9 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 (LUT 에서 R/O 로 차단) */

/* ===== [SENSOR/ADC] addr  38~47 (10 bytes) — 자체 ADC1 (CH2~CH7) 측정 =====
 *  2026-10-13: current/voltage 를 int8(×1)→int16(×0.1) 로 변경(정밀도). float→int16 변환은
 *              MARSHAL_PUBLISH 에서 ×10. NTC 는 int8(×1) 유지(Orin 은 최대값만 텔레메트리). */
typedef struct __attribute__((packed)) {
    /* addr  38 */ int8_t  ntc[3];         /* R/O ×1 [°C] 온도 — ntc[0]=CH2 / [1]=CH3 / [2]=CH4 (인덱스 그대로, 순서 무관) */
    /* addr  41 */ int16_t current_input;  /* R/O ×0.1 [A] 전류 — CH5(PA5, IOUT) */
    /* addr  43 */ int16_t voltage_input;  /* R/O ×0.1 [V] 입력측 전압 — 의미 기준 매핑(SENSOR_Voltage_IN) */
    /* addr  45 */ int16_t voltage_output; /* R/O ×0.1 [V] 출력측 전압 — 의미 기준 매핑(SENSOR_Voltage_OUT), 프리차징 완료 판정 */
    /* addr  47 */ STATE_t state;          /* R/O ADC 센서 블록 채널 종합 상태 */
} DATA_SENSOR_ADC_t;

/* ===== [SENSOR/BMS] addr  48~64 (17 bytes) — BMS(CAN1) 피드백 (2026-10-13: 45→48 밀림) ===== */

typedef struct __attribute__((packed)) {
    /* addr  48 */ uint16_t voltage;        /* R/O ×0.1 [V] 팩 총전압 */
    /* addr  50 */ int16_t current;        /* R/O ×0.1 [A], signed */
    /* addr  52 */ uint16_t soc;            /* R/O ×0.1 [%] State of Charge */
    /* addr  54 */ uint8_t  max_cell_temp;  /* R/O ×1 [°C] 최고 셀 온도 (40 offset) */
    /* addr  55 */ uint32_t error_lsb;      /* R/O error code [0:3] */
    /* addr  59 */ uint32_t error_hsb;      /* R/O error code [4:7] */
    /* addr  63 */ uint8_t  delta_tick;     /* R/O ×0.1 [s] (**BMS 예외 스케일** — 1Hz 로깅용, 타 센서 ×0.1ms 와 다름) realtime_tick - time_stamp (0xFF: 25.5s↑ 지연=오염) */
    /* addr  64 */ STATE_t  state;          /* R/O CAN1 포트 채널 종합 상태 (STATE_t) */
} DATA_SENSOR_BMS_t;

/* ===== [CMD/POWER] addr  65~67 (3 bytes) — 전원 스위치 제어 명령 (2026-10-13: 62→65 밀림) ===== */
typedef struct __attribute__((packed)) {
    /* addr  65 */ uint8_t ctr_mode[3];     /* R/W: 전원 on/off (CTR_MODE_OFF/ON)
                                             *      ctr_mode[0]=메인파워 / [1]=AUX1(MCU) / [2]=AUX2(ORIN) */
} CMD_POWER_t;

/* ===== [RSVD1] addr  68~79 (12 bytes) — 2026-10-13: 65~79(15)→68~79(12), SENSOR/ADC +3 흡수 ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 */

/* ===== [DIAG] addr  80~99 (20 bytes) — R/O =====
 *   진단 카운터(4 byte) + DIAG_RSVD(16 byte) 구조 */
typedef struct __attribute__((packed)) {
    /* addr  80 */ uint32_t cmd_write_tick;  /* 진단 카운터: 상위(Orin)가 CMD/POWER 영역에 마지막으로 쓴 시각 */
    /* addr  84 */ uint8_t  diag_rsvd[16];   /* DIAG_RSVD — 향후 센서 진단 확장용 */
} DIAG_t;

/* ===== [RSVD2] addr 100~255 (156 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 */


/* ===== 전체 레지스터 맵 — Total 256 bytes (멤버 순서 = #define offset 순) ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ DEFINE_t           reg_df;         /*  16 bytes */
    /* addr  16 */ DATA_SYSTEM_t      sys;            /*  13 bytes */
    /* addr  29 */ uint8_t            reserved0[9];   /*   9 bytes */
    /* addr  38 */ DATA_SENSOR_ADC_t  sensor_adc;     /*  10 bytes */
    /* addr  48 */ DATA_SENSOR_BMS_t  sensor_bms;     /*  17 bytes */
    /* addr  65 */ CMD_POWER_t        cmd_power;      /*   3 bytes */
    /* addr  68 */ uint8_t            reserved1[12];  /*  12 bytes */
    /* addr  80 */ DIAG_t             diag;           /*  20 bytes */
    /* addr 100 */ uint8_t            reserved2[156]; /* 156 bytes */
} REGISTER_t;

/* ===== Compile-time size 검증 (총 256 byte) — 2026-10-10 활성화 (C11 static_assert, <assert.h>) ===== */
static_assert(sizeof(DEFINE_t)           == REG_DEFINE_SIZE,      "DEFINE_t size mismatch");
static_assert(sizeof(DATA_SYSTEM_t)      == REG_SYS_SIZE,         "DATA_SYSTEM_t size mismatch");
static_assert(sizeof(DATA_SENSOR_ADC_t)  == REG_SENSOR_ADC_SIZE,  "DATA_SENSOR_ADC_t size mismatch");
static_assert(sizeof(DATA_SENSOR_BMS_t)  == REG_SENSOR_BMS_SIZE,  "DATA_SENSOR_BMS_t size mismatch");
static_assert(sizeof(CMD_POWER_t)        == REG_CMD_POWER_SIZE,   "CMD_POWER_t size mismatch");
static_assert(sizeof(DIAG_t)             == REG_DIAG_SIZE,        "DIAG_t size mismatch");
static_assert(sizeof(REGISTER_t)         == REG_TOTAL_SIZE,       "REGISTER_t total size mismatch");

#endif /* INC_RD_REGISTER_PRA_H_ */
