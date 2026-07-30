/*
 * rd_register_dpcb.h
 *
 *  DPC_B 256-byte 레지스터 맵 (Modbus_MAP_DPCB_v2.csv 기준).
 *  - Little-endian, 1 addr = 1 byte, 실제값 = raw × Scale.
 *  - 외부 RS485 마스터(Orin) ↔ 내부 PERIPHERAL 의 단일 source-of-truth.
 *
 *  영역 레이아웃 (총 256 byte) — #define 기준:
 *      [   0 :  15] DEFINE           ( 16 byte)  시스템 설정 파라미터   R/W (잠금키 필요)
 *      [  16 :  45] RSVD0            ( 30 byte)  미래 확장용            R/O
 *      [  46 :  61] SYSTEM           ( 16 byte)  시스템 상태            R/O
 *      [  62 :  64] SENSOR/DPCA      (  3 byte)  DPC-A 센서 피드백      R/O
 *      [  65 :  65] UART2/data       (  1 byte)  Orin RS485 채널 상태   R/O
 *      [  66 :  73] GPIO/data        (  8 byte)  DPC-B 패널/GPIO 상태   R/O
 *      [  74 : 110] MOTOR/data       ( 37 byte)  Dynamixel 피드백       R/O
 *      [ 111 : 119] RSVD1            (  9 byte)  미래 확장용            R/O
 *      [ 120 : 121] CMD/DPCA         (  2 byte)  DPC-A 제어 명령        R/W
 *      [ 122 : 127] CMD/DPCB         (  6 byte)  DPC-B GPIO/모드 명령   R/W
 *      [ 128 : 142] CMD/MOT          ( 15 byte)  Dynamixel 제어 명령    R/W
 *      [ 143 : 174] RSVD2            ( 32 byte)  미래 확장용            R/O
 *      [ 175 : 178] DIAG             (  4 byte)  진단 카운터            R/O
 *      [ 179 : 206] DIAG_RSVD        ( 28 byte)  진단 확장 예약         R/O
 *      [ 207 : 255] RSVD3            ( 49 byte)  미래 확장용            R/O
 *
 *  Created on: 2026-06-25
 *      Author: swarm
 */

#ifndef INC_RD_REGISTER_DPCB_H_
#define INC_RD_REGISTER_DPCB_H_

#include <stdint.h>
#include <assert.h>
#include "rd_common.h"   /* STATE_t (lifecycle+health), LS_* / HC_* 정의 */

/*
 * ===== "state" 필드 공통 의미 =====
 *  본 레지스터 맵에 등장하는 모든 `state` 필드는 rd_common.h 의 `STATE_t`
 *  (uint8_t union) 를 의미한다.
 *
 *      bit[3:0] = lifecycle  (LS_INIT=0 ... LS_RUNNING=2 ... LS_OFFLINE=15)
 *      bit[7:4] = health     (HC_OK=0 ... HC_FATAL=15)
 *
 *  외부 마스터는 1 byte 를 읽어 다음 식으로 분해:
 *      lifecycle = state & 0x0F
 *      health    = (state >> 4) & 0x0F
 */

/* ===== Region offset / size (bytes) — dispatch/marshal LUT 용 ===== */
#define REG_TOTAL_SIZE              256

#define REG_DEFINE_OFFSET             0
#define REG_DEFINE_SIZE              16

#define REG_RSVD0_OFFSET             16
#define REG_RSVD0_SIZE               30

#define REG_SYS_OFFSET               46
#define REG_SYS_SIZE                 16

#define REG_SENSOR_DPCA_OFFSET       62
#define REG_SENSOR_DPCA_SIZE          3

#define REG_UART2_OFFSET             65
#define REG_UART2_SIZE                1

#define REG_SENSOR_DPCB_OFFSET       66
#define REG_SENSOR_DPCB_SIZE          8

#define REG_MOTOR_DATA_OFFSET        74
#define REG_MOTOR_DATA_SIZE          37

#define REG_RSVD1_OFFSET            111
#define REG_RSVD1_SIZE                9

#define REG_CMD_DPCA_OFFSET         120
#define REG_CMD_DPCA_SIZE             2

#define REG_CMD_DPCB_OFFSET         122
#define REG_CMD_DPCB_SIZE             6

#define REG_CMD_MOT_OFFSET          128
#define REG_CMD_MOT_SIZE             15

#define REG_RSVD2_OFFSET            143
#define REG_RSVD2_SIZE               32

#define REG_DIAG_OFFSET             175
#define REG_DIAG_SIZE                32

#define REG_RSVD3_OFFSET            207
#define REG_RSVD3_SIZE               49

/* CMD/MOT 쓰기 감시 범위 (cmd_write_tick 갱신 판정용) */
#define REG_CMD_MOT_S_OFFSET        128   /* torque_en[0] 시작 */
#define REG_CMD_MOT_E_OFFSET        142   /* goal_current[2] 마지막 바이트 */

/* ===== sys_write_mode 값 ===== */
#define SYS_WRITE_LOCK              0    /* DEFINE 영역 쓰기 잠금 (기본) */
#define SYS_WRITE_UNLOCK            1    /* DEFINE 영역 쓰기 허용 */

/* ===== HW bitfield 비트 정의 (hw_reset / hw_fatal / hw_error 공통) =====
 *  rd_system.h 의 HARDWARE_STATUS_t 비트 순서와 일치해야 한다.
 *  bit[4..7] reserved */
#define HW_BIT_UART2                (1u << 0)   /* Orin RS485 (USART2)  */
#define HW_BIT_UART4                (1u << 1)   /* DPC-A UART  (UART4)  */
#define HW_BIT_UART6                (1u << 2)   /* Dynamixel   (USART6) */
#define HW_BIT_I2C1                 (1u << 3)   /* MCP23017    (I2C1)   */

/* ===== Default 값 (RD_MAP_INIT 에서 세팅) ===== */
#define DEF_ERR_TIMEOUT             15   /* [ms]     RX timeout error 임계 (10~500) */
#define DEF_FATAL_TIMEOUT           10   /* [100ms]  RX timeout fatal 임계 (0=감지 X) */
#define DEF_ERR_CNT                  5   /* [count]  Error 임계 카운트 (0~10)  */
#define DEF_FATAL_CNT               10   /* [count]  Fatal 임계 카운트 (0=감지 X) */
#define DEF_GOAL_CURRENT           750   /* [raw]    ×2.69 mA ≈ 2.0 A (기본 전류 제한) */

/* ===== CMD/DPCB.servo_cmd 값 ===== */
#define SERVO_CMD_IDLE               0
#define SERVO_CMD_LOCK               1
#define SERVO_CMD_UNLOCK             2

/* ===== CMD/DPCB.mode 값 (addr 126) ===== */
typedef enum {
    DPCB_MODE_MANUAL = 0,   /* 패널 기반 직접 제어 (default) */
    DPCB_MODE_AUTO   = 1,   /* Orin RS485 관제 — 세부 동작은 sys_state 가 결정 */
} DPCB_MODE_e;

/* ===== sys_state(addr 57, R/O 실제) / sys_state_target(addr 127, R/W 목표) 값 =====
 *  값 정의용 enum. 레지스터 필드는 uint8_t 유지(packing 보존).
 *  Orin 이 sys_state_target 에 입력 가능한 값(mask): CTRL(0)/HOLD(1)/INIT(2)/ASCEND_1(6). */
typedef enum {
    DPCB_STATE_CTRL      = 0,   /* Orin 직접 관제 */
    DPCB_STATE_HOLD      = 1,   /* 수송 고정 (포지션 고정) */
    DPCB_STATE_INIT      = 2,   /* ┐ FSM 자동 전개 (구 DEPLOY_*) */
    DPCB_STATE_DESCEND_1 = 3,   /* │ */
    DPCB_STATE_DESCEND_2 = 4,   /* │ */
    DPCB_STATE_WAIT      = 5,   /* │ Orin 이 target=ASCEND_1(6) 기입해야 전이 */
    DPCB_STATE_ASCEND_1  = 6,   /* │ */
    DPCB_STATE_ASCEND_2  = 7,   /* │ */
    DPCB_STATE_FINISH    = 8,   /* ┘ 완료 후 CTRL(0) 자동 복귀 */
    DPCB_STATE_RSVD      = 9,   /* 예약 — 향후 확장 */
    DPCB_STATE_ERROR     = 10,  /* FSM 실패 또는 통신 FAULT 시 진입 */
} DPCB_STATE_e;


/* ===== [DEFINE] addr   0~15 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ uint8_t sys_write_mode;  /* R/W: 0=LOCK / 1=UNLOCK (default 0) — DEFINE 영역 쓰기 허용 키 */
    /* addr   1 */ uint8_t err_timeout;     /* R/W: RX timeout error 임계 [ms]      (default 15, range 10~500) */
    /* addr   2 */ uint8_t fatal_timeout;   /* R/W: RX timeout fatal 임계 [100ms]   (default 10, 0=감지 X) */
    /* addr   3 */ uint8_t err_cnt;         /* R/W: Error 임계 카운트  [count]      (default 5,  range 0~10) */
    /* addr   4 */ uint8_t fatal_cnt;       /* R/W: Fatal 임계 카운트  [count]      (default 10, 0=감지 X) */
    /* addr   5 */ uint8_t hw_reset;        /* R/W: 소프트 리셋 트리거 bitfield — HW_BIT_UART2/4/6/I2C1 */
    /* addr   6 */ uint8_t reserved[10];    /* 미래 확장용 */
} DEFINE_t;

/* ===== [RSVD0] addr  16~45 (30 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 (LUT 에서 R/O 로 차단) */

/* ===== [SYSTEM] addr  46~61 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr  46 */ uint8_t  degraded_cnt[8]; /* R/O 통신 오염도 [%]: idx0=uart2 / idx1=uart4 / idx2=uart6 / idx3=i2c / idx4~7=RSVD
                                              *      값 = (uint8_t)((degraded_cnt_raw * 26) >> 8)  — 0~1000 raw → 0~100% */
    /* addr  54 */ uint8_t  hw_reset;        /* R/O bitfield: HW_BIT_* — 1 세트 시 소프트 리셋 트리거 */
    /* addr  55 */ uint8_t  hw_fatal;        /* R/O bitfield: 재초기화 필요 수준의 치명 에러 */
    /* addr  56 */ uint8_t  hw_error;        /* R/O bitfield: 현재 활성 에러 (HARDWARE_STATUS_t 매핑) */
    /* addr  57 */ uint8_t  sys_state;       /* R/O 운용 상태 DPCB_STATE_e (0=CTRL/1=HOLD/2~8=FSM/9=RSVD/10=ERROR)
                                              *      ※ 발행 소스 payload_state→DPC_CTL.STATE 교체는 별도 TODO(현재 미반영) */
    /* addr  58 */ uint32_t realtime_tick;   /* R/O [ms] TIM5 카운터 (tim_cnt) — DPC_B alive time */
} DATA_SYSTEM_t;

/* ===== [SENSOR/DPCA] addr  62~64 (3 bytes) — DPC-A 센서 피드백 ===== */
typedef struct __attribute__((packed)) {
    /* addr  62 */ uint8_t prox_contact;    /* R/O 3bit raw: 지면 근접 센서 (DPC-A → DPC-B 전달)
                                             *      bit[0~2] = prox 채널별 접촉 여부 */
    /* addr  63 */ uint8_t lock_contact;    /* R/O 4bit raw: DPC-A 집게 접점 상태
                                             *      bit[0~3] = 접점 채널별 잠금 여부 */
    /* addr  64 */ STATE_t uart4_state;     /* R/O UART4(DPC-A) 채널 종합 상태 */
} DATA_SENSOR_DPCA_t;

/* ===== [UART2/data] addr  65 (1 byte) — Orin RS485 채널 상태 ===== */
typedef struct __attribute__((packed)) {
    /* addr  65 */ STATE_t state;           /* R/O STATE_t — Orin RS485 (USART2) 통신 상태 */
} DATA_UART2_t;

/* ===== [GPIO/data] addr  66~73 (8 bytes) — DPC-B 패널/GPIO 상태 ===== */
typedef struct __attribute__((packed)) {
    /* addr  66 */ uint8_t lock_contact;    /* R/O 4bit raw: DPC-B 측 lock 접점 (집게 고정 확인)
                                             *      bit[0~3] = 접점 채널별 잠금 여부 */
    /* addr  67 */ uint8_t ex_sw[6];        /* R/O MCP23017 패널 스위치 상태 [idx 0~5 = SW1~SW6]
                                             *      ex_sw[0] SW1 (SPST): 0=OFF / 1=ON
                                             *      ex_sw[1] SW2 (SPST): 0=OFF / 1=ON
                                             *      ex_sw[2] SW3 (SPDT): 0=IDLE(mid) / 1=UP / 2=DOWN
                                             *      ex_sw[3] SW4 (SPDT): 0=IDLE(mid) / 1=UP / 2=DOWN
                                             *      ex_sw[4] SW5 (SPDT): 0=IDLE(mid) / 1=UP / 2=DOWN
                                             *      ex_sw[5] SW6 (SPDT): 0=IDLE(mid) / 1=UP / 2=DOWN */
    /* addr  73 */ STATE_t panel_state;     /* R/O I2C1(MCP23017) 채널 종합 상태 */
} DATA_SENSOR_DPCB_t;

/* ===== [MOTOR/data] addr  74~110 (37 bytes) — Dynamixel 피드백 ===== */
typedef struct __attribute__((packed)) {
    /* addr  74 */ int32_t  present_position[3];    /* R/O ×1 [pulse] (멀티턴, CUR_POSITION) — ID=2/3/4 순서 */
    /* addr  86 */ int32_t  present_velocity[3];    /* R/O ×0.229 [rev/min]                  */
    /* addr  98 */ int16_t  present_current[3];     /* R/O ×2.69 [mA]                        */
    /* addr 104 */ uint8_t  present_temperature[3]; /* R/O ×1 [°C]                           */
    /* addr 107 */ uint8_t  hardware_error[3];      /* R/O Dynamixel HW 에러 코드 raw         */
    /* addr 110 */ STATE_t  uart6_state;            /* R/O USART6(Dynamixel RS485) 채널 종합 상태 */
} DATA_MOTOR_DYN_t;

/* ===== [RSVD1] addr 111~119 (9 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 */

/* ===== [CMD/DPCA] addr 120~121 (2 bytes) — DPC-A 제어 명령 ===== */
typedef struct __attribute__((packed)) {
    /* addr 120 */ uint8_t locker_en;   /* R/W: DPC-A 집게 잠금 (0=OFF / 1=ON) — dpcaTask 가 DPCA_PACKET 으로 전달 */
    /* addr 121 */ uint8_t boot_en;     /* R/W: DPC-A 부트 동작 (0=OFF / 1=ON)                                    */
} CMD_DPCA_t;

/* ===== [CMD/DPCB] addr 122~127 (6 bytes) — DPC-B GPIO/모드 명령 ===== */
typedef struct __attribute__((packed)) {
    /* addr 122 */ uint8_t locker_en;   /* R/W: DPC-B 로커 솔레노이드 (0=OFF / 1=ON) */
    /* addr 123 */ uint8_t boot_en;     /* R/W: DPC-B 부트 솔레노이드  (0=OFF / 1=ON) */
    /* addr 124 */ uint8_t light_en;    /* R/W: 조명 LED               (0=OFF / 1=ON) */
    /* addr 125 */ uint8_t servo_cmd;   /* R/W: 서보 명령 — SERVO_CMD_IDLE(0)/LOCK(1)/UNLOCK(2) */
    /* addr 126 */ uint8_t mode;             /* R/W: 운용 모드 — DPCB_MODE_MANUAL(0)/AUTO(1) */
    /* addr 127 */ uint8_t sys_state_target; /* R/W: 운용 목표 상태 — DPCB_STATE_e (구 deploy_fsm)
                                              *      Orin 입력 가능 mask: CTRL(0)/HOLD(1)/INIT(2)/ASCEND_1(6) */
} CMD_DPCB_t;

/* ===== [CMD/MOT] addr 128~142 (15 bytes) — Dynamixel 제어 명령 ===== */
typedef struct __attribute__((packed)) {
    /* addr 128 */ uint8_t torque_en[3];        /* R/W: 토크 ON/OFF per motor (0=OFF / 1=ON) */
    /* addr 131 */ int16_t goal_position[3];    /* R/W: ×1 [pulse] 목표 위치 — ID=2/3/4 순서 */
    /* addr 137 */ int16_t goal_current[3];     /* R/W: ×2.69 [mA] 전류 제한 (default 750 ≈ 2.0 A) */
} CMD_MOT_t;

/* ===== [RSVD2] addr 143~174 (32 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 */

/* ===== [DIAG] addr 175~206 (32 bytes) — R/O =====
 *   진단 카운터(4 byte) + DIAG_RSVD(28 byte) 구조 */
typedef struct __attribute__((packed)) {
    /* addr 175 */ uint32_t cmd_write_tick;  /* 진단 카운터: 상위(Orin)가 CMD/MOT 영역에 마지막으로 쓴 시각 (osKernelGetTickCount) */
    /* addr 179 */ uint8_t  diag_rsvd[28];   /* DIAG_RSVD — 향후 진단 확장용 */
} DIAG_t;

/* ===== [RSVD3] addr 207~255 (49 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 */


/* ===== 전체 레지스터 맵 — Total 256 bytes (멤버 순서 = #define offset 순) ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ DEFINE_t             reg_df;          /*  16 bytes */
    /* addr  16 */ uint8_t              reserved0[30];   /*  30 bytes */
    /* addr  46 */ DATA_SYSTEM_t        sys;             /*  16 bytes */
    /* addr  62 */ DATA_SENSOR_DPCA_t   sensor_dpca;     /*   3 bytes */
    /* addr  65 */ DATA_UART2_t         uart2;           /*   1 byte  */
    /* addr  66 */ DATA_SENSOR_DPCB_t   sensor_dpcb;     /*   8 bytes */
    /* addr  74 */ DATA_MOTOR_DYN_t     motor_data;      /*  37 bytes */
    /* addr 111 */ uint8_t              reserved1[9];    /*   9 bytes */
    /* addr 120 */ CMD_DPCA_t           cmd_dpca;        /*   2 bytes */
    /* addr 122 */ CMD_DPCB_t           cmd_dpcb;        /*   6 bytes */
    /* addr 128 */ CMD_MOT_t            cmd_mot;         /*  15 bytes */
    /* addr 143 */ uint8_t              reserved2[32];   /*  32 bytes */
    /* addr 175 */ DIAG_t               diag;            /*  32 bytes */
    /* addr 207 */ uint8_t              reserved3[49];   /*  49 bytes */
} REGISTER_t;

/* ===== Compile-time size 검증 (총 256 byte) — STM32CubeIDE 인식 불가로 주석 처리 ===== */
//static_assert(sizeof(DEFINE_t)            == REG_DEFINE_SIZE,       "DEFINE_t size mismatch");
//static_assert(sizeof(DATA_SYSTEM_t)       == REG_SYS_SIZE,          "DATA_SYSTEM_t size mismatch");
//static_assert(sizeof(DATA_SENSOR_DPCA_t)  == REG_SENSOR_DPCA_SIZE,  "DATA_SENSOR_DPCA_t size mismatch");
//static_assert(sizeof(DATA_UART2_t)        == REG_UART2_SIZE,        "DATA_UART2_t size mismatch");
//static_assert(sizeof(DATA_SENSOR_DPCB_t)  == REG_SENSOR_DPCB_SIZE,  "DATA_SENSOR_DPCB_t size mismatch");
//static_assert(sizeof(DATA_MOTOR_DYN_t)    == REG_MOTOR_DATA_SIZE,   "DATA_MOTOR_DYN_t size mismatch");
//static_assert(sizeof(CMD_DPCA_t)          == REG_CMD_DPCA_SIZE,     "CMD_DPCA_t size mismatch");
//static_assert(sizeof(CMD_DPCB_t)          == REG_CMD_DPCB_SIZE,     "CMD_DPCB_t size mismatch");
//static_assert(sizeof(CMD_MOT_t)           == REG_CMD_MOT_SIZE,      "CMD_MOT_t size mismatch");
//static_assert(sizeof(DIAG_t)              == REG_DIAG_SIZE,         "DIAG_t size mismatch");
//static_assert(sizeof(REGISTER_t)          == REG_TOTAL_SIZE,        "REGISTER_t total size mismatch");

#endif /* INC_RD_REGISTER_DPCB_H_ */
