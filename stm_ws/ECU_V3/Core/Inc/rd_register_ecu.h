/*
 * rd_register_ecu.h
 *
 *  ECU 256-byte 레지스터 맵 (Google Sheet 기준).
 *  - Little-endian, 1 addr = 1 byte, 실제값 = raw × Scale.
 *  - 외부 RS485 마스터 ↔ 내부 PERIPHERAL 의 단일 source-of-truth.
 *
 *  영역 레이아웃 (총 256 byte) — #define 기준:
 *      [   0 :  15] DEFINE      ( 16 byte)  시스템 설정 파라미터    R/W
 *      [  16 :  45] RSVD0       ( 30 byte)  미래 확장용            R/O
 *      [  46 :  61] SYSTEM      ( 16 byte)  시스템 상태 (+degraded_cnt[8])  R/O
 *      [  62 :  82] IMU         ( 21 byte)  쿼터니언/자이로/가속도      R/O (UART6 EBIMU-9DOFV6)
 *      [  83 :  93] ENCODER     ( 11 byte)  AS5600 5ch + state     R/O
 *      [  94 :  94] UART2       (  1 byte)  RS485 채널 STATE_t      R/O
 *      [  95 :  95] SENSOR/RC   (  1 byte)  RC 수신기 채널 STATE_t  R/O
 *      [  96 : 127] MOTOR/data  ( 32 byte)  모터 피드백              R/O
 *      [ 128 : 179] MOTOR/cmd   ( 52 byte)  모터 제어 명령           R/W
 *      [ 180 : 191] SYSTEM/cmd  ( 12 byte)  시스템 명령 (lin/ang/mode) R/W
 *      [ 192 : 223] RSVD1       ( 32 byte)  미래 확장용              R/O
 *      [ 224 : 255] DIAG        ( 32 byte)  진단/디버그 카운터        R/O
 *
 *  Created on: 2026. 5. 14.
 *  Updated   : 2026. 5. 28. — #define 기준으로 구조체 멤버 재배치 + IMU/CMD 필드 주석 정정
 *      Author: Kyeongtae
 */
#ifndef INC_RD_REGISTER_ECU_H_
#define INC_RD_REGISTER_ECU_H_

#include <stdint.h>
#include <assert.h>
#include "rd_common.h"   /* STATE_t (lifecycle+health), LS_* / HC_* 정의 */

/*
 * ===== "state" 필드 공통 의미 (중요) =====
 *  본 레지스터 맵에 등장하는 모든 `state` 필드 (encoder.state, imu.state,
 *  motor_data.state, uart2.state, rc.state) 는 모두 rd_common.h 의 `STATE_t`
 *  (uint8 union) 를 의미한다.
 *
 *      bit[3:0] = lifecycle  (LS_INIT=0 ... LS_RUNNING=2 ... LS_OFFLINE=15)
 *      bit[7:4] = health     (HC_OK=0 ... HC_BUS_OFF=13 ... HC_FATAL=15)
 *
 *  외부 마스터는 1 byte 를 읽어 다음 식으로 분해:
 *      lifecycle = state & 0x0F
 *      health    = (state >> 4) & 0x0F
 *
 *  주의 — 다음 두 필드는 `state` 가 아니며 의미가 다르다:
 *      sys_state (addr 57)  : SYSTEM_STATE_e (FSM, rd_system.h)
 *      comm_err  (addr 126) : AK_COMM_ERR_t 4 모터 분의 2bit×4 packed (can_ak.h
 *                             통신 에러만, STATE_t.bits.health 와 별개)
 */

/* ===== Region offset / size (bytes) — dispatch/marshal LUT 용 ===== */
#define REG_TOTAL_SIZE          256

#define REG_DEFINE_OFFSET         0
#define REG_DEFINE_SIZE          16

#define REG_RSVD0_OFFSET         16
#define REG_RSVD0_SIZE           30

#define REG_SYS_OFFSET           46
#define REG_SYS_SIZE             16

#define REG_IMU_OFFSET           62
#define REG_IMU_SIZE             21

#define REG_ENCODER_OFFSET       83
#define REG_ENCODER_SIZE         11

#define REG_UART2_OFFSET         94
#define REG_UART2_SIZE            1

#define REG_SENSOR_RC_OFFSET     95
#define REG_SENSOR_RC_SIZE        1

#define REG_MOTOR_DATA_OFFSET    96
#define REG_MOTOR_DATA_SIZE      32

#define REG_CMD_MOTOR_OFFSET    128
#define REG_CMD_MOTOR_SIZE       52

#define REG_CMD_SYSTEM_OFFSET   180
#define REG_CMD_SYSTEM_SIZE      12

#define REG_RSVD1_OFFSET        192
#define REG_RSVD1_SIZE           32

#define REG_DIAG_OFFSET         224
#define REG_DIAG_SIZE            32

#define REG_CMD_VEL_S_OFFSET	132	// Start of 'cmd_position[0]' index
#define REG_CMD_VEL_E_OFFSET	187 // 	 End of 'cmd_ang_vel' index

/* ===== sys_write_mode 값 ===== */
#define SYS_WRITE_LOCK            0
#define SYS_WRITE_UNLOCK          1

/* ===== soft_estop (addr 189) 값 — Orin 용 소프트 ESTOP ===== */
#define SOFT_ESTOP_ACTIVE         0  /* ESTOP 작동: AUTO 모드에서 CAN_AK_ESTOP 소프트 제동 (FSM 전이 없음) */
#define SOFT_ESTOP_RELEASE        1  /* 해제 (default) — 정상 주행                                         */

/* ===== Default 값 (RD_MAP_INIT 에서 세팅 필요) ===== */
#define DEF_ERR_TIMEOUT          15  /* [ms]      RX timeout error 임계 (10~500) */
#define DEF_FATAL_TIMEOUT        10  /* [100ms]   RX timeout fatal 임계 (0=감지 X, 0~100) */
#define DEF_ERR_CNT               5  /* [count]   Error 임계 카운트 (0~10) */
#define DEF_FATAL_CNT            10  /* [count]   Fatal 임계 카운트 (0=감지 X, 0~100) */
#define DEF_CTR_MODE              3  /* AK_Control_Mode_t — 3=VELOCITY (기본 모드) */

/* ===== HW bitfield 비트 정의 (hw_reset / hw_fatal / hw_error 공통) ===== */
//#define HW_BIT_UART1            (1u << 0)
//#define HW_BIT_UART2            (1u << 1)
//#define HW_BIT_UART4            (1u << 2)
//#define HW_BIT_CAN1             (1u << 3)
//#define HW_BIT_I2C1             (1u << 4)
/* bit[5..7] reserved */

/* ===== [DEFINE] addr   0~15 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr   0 */ uint8_t sys_write_mode;  /* R/W: 0=LOCK / 1=UNLOCK (default 0) — DEFINE 영역 외 R/W 쓰기 허용 키 */
	/* addr   1 */ uint8_t err_timeout;     /* R/W: RX timeout error 임계 [ms]      (default 15, range 10~500) */
	/* addr   2 */ uint8_t fatal_timeout;   /* R/W: RX timeout fatal 임계 [100ms]   (default 10, 0=감지 X, range 0~100) */
	/* addr   3 */ uint8_t err_cnt;         /* R/W: Error 임계 카운트  [count]      (default 5,  range 0~10) */
	/* addr   4 */ uint8_t fatal_cnt;       /* R/W: Fatal 임계 카운트  [count]      (default 10, 0=감지 X, range 0~100) */
	/* addr   5 */ uint8_t hw_reset;   		/* R/W: Soft reset trigger */
	/* addr   6 */ uint8_t reserved[10];    /* 미래 확장용 */
} DEFINE_t;

/* ===== [RSVD0] addr  16~45 (30 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 (LUT 에서 R/O 로 차단) */

/* ===== [SYSTEM] addr  46~61 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr  46 */ uint8_t  degraded_cnt[8]; /* R/O 통신 오염 정도 [%]: idx0=uart1 / idx1=uart2 / idx2=uart6(IMU) / idx3=can1 / idx4=i2c1 / idx5~7=RSVD
	                                          *      값 = (uint8_t)((degraded_cnt_raw * 26) >> 8)  — 0~1000 raw → 0~100% */
	/* addr  54 */ uint8_t  hw_reset;        /* R/W bitfield: HW_BIT_* — 1 세트 시 소프트 리셋 트리거 */
	/* addr  55 */ uint8_t  hw_fatal;        /* R/O bitfield: 재초기화 필요 수준의 치명 에러            */
	/* addr  56 */ uint8_t  hw_error;        /* R/O bitfield: 현재 활성 에러 (Hardware_Error_FLAG_t 매핑) */
	/* addr  57 */ uint8_t  sys_state;       /* R/O SYSTEM_STATE_e: 0=INIT/1=MANUAL/2=AUTO/3=ESTOP_SW/4=ESTOP_HW/5=FAULT */
	/* addr  58 */ uint32_t realtime_tick;   /* R/O [ms] TIM5 1kHz 카운터 (tim_cnt) — ECU alive time */
} DATA_SYSTEM_t;

/* ===== [SENSOR/IMU] addr  62~82 (21 bytes) — UART6 EBIMU-9DOFV6 raw (HEX 모드, 250Hz) ===== */
typedef struct __attribute__((packed)) {
	/* addr  62 */ int16_t  quat_z;          /* R/O ×0.0001 [무단위]  quat[0] */
	/* addr  64 */ int16_t  quat_y;          /* R/O ×0.0001 [무단위]  quat[1] */
	/* addr  66 */ int16_t  quat_x;          /* R/O ×0.0001 [무단위]  quat[2] */
	/* addr  68 */ int16_t  quat_w;          /* R/O ×0.0001 [무단위]  quat[3] */
	/* addr  70 */ int16_t  gyro_x;          /* R/O ×0.1    [deg/s]   gyro[0] */
	/* addr  72 */ int16_t  gyro_y;          /* R/O ×0.1    [deg/s]   gyro[1] */
	/* addr  74 */ int16_t  gyro_z;          /* R/O ×0.1    [deg/s]   gyro[2] */
	/* addr  76 */ int16_t  acc_x;           /* R/O ×0.001  [g]       acc[0]  */
	/* addr  78 */ int16_t  acc_y;           /* R/O ×0.001  [g]       acc[1]  */
	/* addr  80 */ int16_t  acc_z;           /* R/O ×0.001  [g]       acc[2]  */
	/* addr  82 */ STATE_t  state;           /* R/O STATE_t — IMU 모듈 종합 상태 (lifecycle[3:0] + health[7:4]) */
} DATA_IMU_t;

/* ===== [SENSOR/ENCODER] addr  83~93 (11 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr  83 */ uint16_t encoder[5];      /* R/O AS5600 ch0~4, 12bit raw [0:11], [12:15]=RSVD (0~4095)
	                                          *      실제값: (raw - init_offset + ENC_RAW_MAX) % ENC_RAW_MAX */
	/* addr  93 */ STATE_t  state;           /* R/O STATE_t — 5 enc + MUX 중 worst (lifecycle[3:0] + health[7:4]) */
} DATA_ENCODER_t;

/* ===== [UART2] addr  94 (1 byte) — RS485 채널 종합 상태 ===== */
typedef struct __attribute__((packed)) {
	/* addr  94 */ STATE_t  state;           /* R/O STATE_t — RS485 (UART2) 통신 상태 (HAL + packet layer 종합) */
} DATA_UART2_t;

/* ===== [SENSOR/RC] addr  95 (1 byte) — RC 수신기 채널 종합 상태 ===== */
typedef struct __attribute__((packed)) {
	/* addr  95 */ STATE_t  state;           /* R/O STATE_t — UART1 (RC 수신기) 통신 상태 */
} DATA_RC_t;

/* ===== [MOTOR/data] addr  96~127 (32 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr  96 */ int16_t  position[4];     /* R/O ×0.1 [deg]   — AK motor ID=1~4 (raw=AK_State_t.position) */
	/* addr 104 */ int16_t  velocity[4];     /* R/O ×10.0 [RPM]                                              */
	/* addr 112 */ int16_t  current[4];      /* R/O ×0.01 [A]                                                */
	/* addr 120 */ int8_t   temp[4];         /* R/O ×1 [°C]    range -20~127                                 */
	/* addr 124 */ uint16_t error_code;      /* R/O 4bit×4 packed (LSB=M1): AK 에러코드 원시값 (AK_State_t.error_code) */
	/* addr 126 */ uint8_t  comm_err;        /* R/O 2bit×4 packed (LSB=M1): per-motor AK_COMM_ERR_t — bit[0]=RX err, bit[1]=TX err (can_ak.h).
	                                          *      00=OK / 01=RX / 10=TX / 11=BOTH. 모터 자체 fault 는 error_code(addr 124) 별도.
	                                          *      ※ STATE_t.bits.health 와 의미 다름 (헷갈림 회피 위해 health → comm_err 으로 명명). */
	/* addr 127 */ STATE_t  state;           /* R/O STATE_t — 4 모터 중 worst 요약 (lifecycle[3:0] + health[7:4]) */
} DATA_MOTOR_t;

/* ===== [MOTOR/cmd] addr 128~179 (52 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr 128 */ uint8_t  ctr_mode[4];     /* R/W AK_Control_Mode_t (default 3=VELOCITY, range 0~7):
	                                          *      0=ESTOP / 1=CURRENT / 2=CURRENT_BRAKE / 3=VELOCITY
	                                          *      4=POSITION / 5=SET_ORIGIN / 6=POS_VEL_LOOP / 7=MIT */
	/* addr 132 */ float    cmd_position[4]; /* R/W 위치 목표값 [deg] — AK motor ID=1~4   */
	/* addr 148 */ float    cmd_velocity[4]; /* R/W 속도 목표값 [RPM]                     */
	/* addr 164 */ float    cmd_current[4];  /* R/W 전류 목표값 [A]                       */
} CMD_MOTOR_t;

/* ===== [SYSTEM/cmd] addr 180~191 (12 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr 180 */ float    cmd_lin_vel;     /* R/W AUTO 모드 선속도 [m/s]   (cmd_vel[0]) */
	/* addr 184 */ float    cmd_ang_vel;     /* R/W AUTO 모드 각속도 [rad/s] (cmd_vel[1]) */
	/* addr 188 */ uint8_t  weight;          /* R/W throttle scale (0~3): 0=정지 / 1=×0.15 / 2=×0.50 / 3=×1.00     */
	/* addr 189 */ uint8_t  soft_estop;      /* R/W Orin 용 soft ESTOP: 0=작동(AUTO 에서 CAN_AK_ESTOP 제동) / 1=해제(default)
	                                          *      jeongae 전개 시퀀스 등에서 Orin 이 WRITE 로 제어 (구 ctr_flag 재정의) */
	/* addr 190 */ uint8_t  mode;            /* R/W 0=MANUAL / 1=AUTO — GPIO MODE 핀 연동 (PERIPHERAL_DATA_t.MODE) */
	/* addr 191 */ uint8_t  reserved;        /* RSVD                                                                */
} CMD_SYSTEM_t;

/* ===== [RSVD1] addr 192~223 (32 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 (LUT 에서 R/O 로 차단) */

/* ===== [DIAG] addr 224~255 (32 bytes) — R/O ===== */
typedef struct __attribute__((packed)) {
	/* addr 224 */ uint32_t cmd_write_tick;         /* 향후 진단 확장용 (UART err / I2C MUX err 카운터 등) */
	/* addr 228 */ uint8_t  reserved[28];         /* 향후 진단 확장용 (UART err / I2C MUX err 카운터 등) */
} DIAG_t;

/* ===== 전체 레지스터 맵 — Total 256 bytes (멤버 순서 = #define offset 순) ===== */
typedef struct __attribute__((packed)) {
	/* addr   0 */ DEFINE_t       reg_df;        /* 16 bytes */
	/* addr  16 */ uint8_t        reserved0[30]; /* 30 bytes */
	/* addr  46 */ DATA_SYSTEM_t  sys;           /* 16 bytes */
	/* addr  62 */ DATA_IMU_t     imu;           /* 21 bytes */
	/* addr  83 */ DATA_ENCODER_t encoder;       /* 11 bytes */
	/* addr  94 */ DATA_UART2_t   uart2;         /*  1 byte  */
	/* addr  95 */ DATA_RC_t      rc;            /*  1 byte  */
	/* addr  96 */ DATA_MOTOR_t   motor_data;    /* 32 bytes */
	/* addr 128 */ CMD_MOTOR_t    cmd_motor;     /* 52 bytes */
	/* addr 180 */ CMD_SYSTEM_t   cmd_system;    /* 12 bytes */
	/* addr 192 */ uint8_t        reserved1[32]; /* 32 bytes */
	/* addr 224 */ DIAG_t         diag;          /* 32 bytes */
} REGISTER_t;

/* ===== Compile-time size 검증 (총 256 byte) — STM32CubeIDE 인식 불가로 주석 처리 ===== */
//static_assert(sizeof(DEFINE_t)       == REG_DEFINE_SIZE,      "DEFINE_t size mismatch");
//static_assert(sizeof(DATA_SYSTEM_t)  == REG_SYS_SIZE,         "DATA_SYSTEM_t size mismatch");
//static_assert(sizeof(DATA_IMU_t)     == REG_IMU_SIZE,         "DATA_IMU_t size mismatch");
//static_assert(sizeof(DATA_ENCODER_t) == REG_ENCODER_SIZE,     "DATA_ENCODER_t size mismatch");
//static_assert(sizeof(DATA_UART2_t)   == REG_UART2_SIZE,       "DATA_UART2_t size mismatch");
//static_assert(sizeof(DATA_RC_t)      == REG_SENSOR_RC_SIZE,   "DATA_RC_t size mismatch");
//static_assert(sizeof(DATA_MOTOR_t)   == REG_MOTOR_DATA_SIZE,  "DATA_MOTOR_t size mismatch");
//static_assert(sizeof(CMD_MOTOR_t)    == REG_CMD_MOTOR_SIZE,   "CMD_MOTOR_t size mismatch");
//static_assert(sizeof(CMD_SYSTEM_t)   == REG_CMD_SYSTEM_SIZE,  "CMD_SYSTEM_t size mismatch");
//static_assert(sizeof(DIAG_t)         == REG_DIAG_SIZE,        "DIAG_t size mismatch");
//static_assert(sizeof(REGISTER_t)     == REG_TOTAL_SIZE,       "REGISTER_t total size mismatch");

#endif /* INC_RD_REGISTER_ECU_H_ */
