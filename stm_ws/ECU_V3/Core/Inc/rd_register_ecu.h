/*
 * rd_register_ecu.h
 *
 *  ECU 256-byte 레지스터 맵 (Google Sheet 기준).
 *  - Little-endian, 1 addr = 1 byte, 실제값 = raw × Scale.
 *  - 외부 RS485 마스터 ↔ 내부 PERIPHERAL 의 단일 source-of-truth.
 *
 *  영역 레이아웃 (총 256 byte) — #define 기준 (2026-07-16 delta_tick 재배치):
 *      [   0 :  15] DEFINE      ( 16 byte)  시스템 설정 파라미터        R/W
 *      [  16 :  32] SYSTEM      ( 17 byte)  시스템 상태 + realtime_tick + proc_delta R/O
 *      [  33 :  41] RSVD0       (  9 byte)  미래 확장용                R/O
 *      [  42 :  47] LOADCELL    (  6 byte)  로드셀 avg×2 + delta + state  R/O
 *      [  48 :  69] IMU         ( 22 byte)  쿼터니언/자이로/가속도 + delta + state  R/O
 *      [  70 :  85] ENCODER     ( 16 byte)  AS5600 5ch + delta×5 + state  R/O
 *      [  86 :  86] UART2       (  1 byte)  RS485 채널 STATE_t         R/O
 *      [  87 :  87] SENSOR/RC   (  1 byte)  RC 수신기 채널 STATE_t      R/O
 *      [  88 : 127] MOTOR/data  ( 40 byte)  모터 피드백 + delta×4 + cmd_delta×4  R/O
 *      [ 128 : 179] MOTOR/cmd   ( 52 byte)  모터 제어 명령  (불변)      R/W
 *      [ 180 : 192] SYSTEM/cmd  ( 13 byte)  선속도/각속도/모드/mask     R/W
 *      [ 193 : 223] RSVD1       ( 31 byte)  미래 확장용                R/O
 *      [ 224 : 255] DIAG        ( 32 byte)  진단/디버그 카운터 (불변)    R/O
 *
 *  ===== Timestamp / delta_tick 체계 (memo_260716.md) =====
 *      시간축: TIM5 free-run 32bit @10kHz — realtime_tick = 카운터값 × 0.1ms.
 *      delta_tick(uint8) = realtime_tick(발행시점) − 취득시점 tick [×0.1ms].
 *      0xFF = stale (≥25.5ms 또는 미갱신). Orin 은
 *      취득시각 = realtime_tick − delta_tick 으로 복원 (같은 패킷 내 정합 보장).
 *
 *  Created on: 2026. 5. 14.
 *  Updated   : 2026. 7. 16. — delta_tick 필드 삽입 + 전 영역 재배치 (시트 확정본 기준)
 *      Author: Kyeongtae
 */
#ifndef INC_RD_REGISTER_ECU_H_
#define INC_RD_REGISTER_ECU_H_

#include <stdint.h>
#include <assert.h>
#include "rd_common.h"   /* STATE_t (lifecycle+health), LS_* / HC_* 정의 */

/*
 * ===== "state" 필드 공통 의미 (중요) =====
 *  본 레지스터 맵에 등장하는 모든 `state` 필드는 rd_common.h 의 `STATE_t` (uint8 union):
 *      bit[3:0] = lifecycle  (LS_INIT=0 ... LS_RUNNING=2 ... LS_OFFLINE=15)
 *      bit[7:4] = health     (HC_OK=0 ... HC_FATAL=15)
 *
 *  주의 — 다음 두 필드는 `state` 가 아니며 의미가 다르다:
 *      sys_state (addr 27)  : SYSTEM_STATE_e (FSM, rd_system.h)
 *      comm_err  (addr 126) : AK_COMM_ERR_t 4 모터 분의 2bit×4 packed
 */

/* ===== Region offset / size (bytes) — dispatch/marshal LUT 용 ===== */
#define REG_TOTAL_SIZE          256

#define REG_DEFINE_OFFSET         0
#define REG_DEFINE_SIZE          16

#define REG_SYS_OFFSET           16
#define REG_SYS_SIZE             17   /* 2026-07-27 rs485_proc_delta(addr 32) 흡수 — RSVD0 에서 1B 인출 */

#define REG_RSVD0_OFFSET         33
#define REG_RSVD0_SIZE            9

#define REG_LOADCELL_OFFSET      42
#define REG_LOADCELL_SIZE         6

#define REG_IMU_OFFSET           48
#define REG_IMU_SIZE             22

#define REG_ENCODER_OFFSET       70
#define REG_ENCODER_SIZE         16

#define REG_UART2_OFFSET         86
#define REG_UART2_SIZE            1

#define REG_SENSOR_RC_OFFSET     87
#define REG_SENSOR_RC_SIZE        1

#define REG_MOTOR_DATA_OFFSET    88
#define REG_MOTOR_DATA_SIZE      40

#define REG_CMD_MOTOR_OFFSET    128
#define REG_CMD_MOTOR_SIZE       52

#define REG_CMD_SYSTEM_OFFSET   180
#define REG_CMD_SYSTEM_SIZE      13   /* 2026-07-17 motor_mask(addr 192) 추가 — RSVD1 에서 1B 인출 */

#define REG_RSVD1_OFFSET        193
#define REG_RSVD1_SIZE           31

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

/* ===== [SYSTEM] addr  16~32 (17 bytes) — 2026-07-27 재배치 (redesign/01 §9.1) =====
 *  변경 2가지:
 *   ① hw_error / hw_fatal / hw_reset 을 **심각도 순**으로 재배치 (구: reset,fatal,error)
 *      → hw_reset 이 26 으로 내려오면서 Control task 읽기 세그 {26,7} 이
 *        hw_reset + sys_state + realtime_tick(4) + rs485_proc_delta = 정확히 7B 로 맞는다.
 *   ② rs485_proc_delta 를 DIAG(구 addr 228) 에서 addr 32 로 이동 → 시간 동기 필드가
 *      realtime_tick 바로 뒤에 연속 배치된다. LOADCELL(42) 이하 주소는 불변.
 *
 *  ⚠ 순서를 바꿨으므로 rd_map_ecu.c 의 발행부가 memcpy 가 아닌 **필드별 대입**이어야 한다.
 *     HW_ERROR_FLAG_t 는 {reset,fatal,error} 순이라 memcpy 로는 reset↔error 가 뒤바뀐다.
 */
typedef struct __attribute__((packed)) {
	/* addr  16 */ uint8_t  degraded_cnt[8]; /* R/O 통신 오염 정도 [%]: idx0=uart1 / idx1=uart2 / idx2=uart6(IMU) / idx3=can1 / idx4=i2c1 / idx5~7=RSVD
	                                          *      값 = (uint8_t)((degraded_cnt_raw * 26) >> 8)  — 0~1000 raw → 0~100% */
	/* addr  24 */ uint8_t  hw_error;        /* R/O bitfield: 현재 활성 에러 (HW_ERROR_FLAG_t.error) — 구 addr 26 */
	/* addr  25 */ uint8_t  hw_fatal;        /* R/O bitfield: 재초기화 필요 수준의 치명 에러            */
	/* addr  26 */ uint8_t  hw_reset;        /* R/O bitfield: HW_BIT_* — 1 세트 시 소프트 리셋 트리거 — 구 addr 24 */
	/* addr  27 */ uint8_t  sys_state;       /* R/O SYSTEM_STATE_e: 0=INIT/1=MANUAL/2=AUTO/3=ESTOP_SW/4=ESTOP_HW/5=FAULT */
	/* addr  28 */ uint32_t realtime_tick;   /* R/O ×0.1[ms] TIM5 10kHz free-run 카운터 — 발행 시점 tick (delta 기준값) */
	/* addr  32 */ uint8_t  rs485_proc_delta;/* R/O ×0.1[ms] 직전 트랜잭션의 MARSHAL_PUBLISH(realtime_tick latch)
	                                          * → 응답 TX 시작 처리시간. 스냅샷이 처리 전에 찍히므로 이번 응답이
	                                          * 아닌 '다음' 응답에 실린다 (Orin 시계 동기: testbed_spec.md §2.5).
	                                          * 구 addr 228(DIAG) 에서 이동 */
} DATA_SYSTEM_t;

/* ===== [RSVD0] addr  32~41 (10 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 (LUT 에서 R/O 로 차단) */

/* ===== [LOADCELL] addr  42~47 (6 bytes) — 로드셀 ADC (ADC1+DMA), R/O ===== */
typedef struct __attribute__((packed)) {
	/* addr  42 */ uint16_t avg[2];          /* R/O 로드셀 raw 트림평균 ch0~1 (0~4095). 물리값 변환은 상위 */
	/* addr  46 */ uint8_t  delta_tick;      /* R/O ×0.1[ms] 취득 지연 (2ch DMA 동시취득 → 대표 1개). 0xFF=stale */
	/* addr  47 */ STATE_t  state;           /* R/O STATE_t — 로드셀 종합 상태 */
} DATA_LOADCELL_t;

/* ===== [SENSOR/IMU] addr  48~69 (22 bytes) — UART6 EBIMU-9DOFV6 raw (HEX, 250Hz) ===== */
typedef struct __attribute__((packed)) {
	/* addr  48 */ int16_t  quat_z;          /* R/O ×0.0001 [무단위]  quat[0] */
	/* addr  50 */ int16_t  quat_y;          /* R/O ×0.0001 [무단위]  quat[1] */
	/* addr  52 */ int16_t  quat_x;          /* R/O ×0.0001 [무단위]  quat[2] */
	/* addr  54 */ int16_t  quat_w;          /* R/O ×0.0001 [무단위]  quat[3] */
	/* addr  56 */ int16_t  gyro_x;          /* R/O ×0.1    [deg/s]   gyro[0] */
	/* addr  58 */ int16_t  gyro_y;          /* R/O ×0.1    [deg/s]   gyro[1] */
	/* addr  60 */ int16_t  gyro_z;          /* R/O ×0.1    [deg/s]   gyro[2] */
	/* addr  62 */ int16_t  acc_x;           /* R/O ×0.001  [g]       acc[0]  */
	/* addr  64 */ int16_t  acc_y;           /* R/O ×0.001  [g]       acc[1]  */
	/* addr  66 */ int16_t  acc_z;           /* R/O ×0.001  [g]       acc[2]  */
	/* addr  68 */ uint8_t  delta_tick;      /* R/O ×0.1[ms] 취득 지연 (프레임 단위 → 대표 1개). 0xFF=stale */
	/* addr  69 */ STATE_t  state;           /* R/O STATE_t — IMU 모듈 종합 상태 */
} DATA_IMU_t;

/* ===== [SENSOR/ENCODER] addr  70~85 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr  70 */ uint16_t encoder[5];      /* R/O AS5600 ch0~4, 12bit raw (0~4095) */
	/* addr  80 */ uint8_t  delta_tick[5];   /* R/O ×0.1[ms] 채널별 취득 지연 (I2C MUX 순차 read). 0xFF=stale */
	/* addr  85 */ STATE_t  state;           /* R/O STATE_t — 5 enc + MUX 중 worst */
} DATA_ENCODER_t;

/* ===== [UART2] addr  86 (1 byte) — RS485 채널 종합 상태 ===== */
typedef struct __attribute__((packed)) {
	/* addr  86 */ STATE_t  state;           /* R/O STATE_t — RS485 (UART2) 통신 상태 */
} DATA_UART2_t;

/* ===== [SENSOR/RC] addr  87 (1 byte) — RC 수신기 채널 종합 상태 ===== */
typedef struct __attribute__((packed)) {
	/* addr  87 */ STATE_t  state;           /* R/O STATE_t — UART1 (RC 수신기) 통신 상태 */
} DATA_RC_t;

/* ===== [MOTOR/data] addr  88~127 (40 bytes) ===== */
typedef struct __attribute__((packed)) {
	/* addr  88 */ int16_t  position[4];       /* R/O ×0.1 [deg]   — AK motor ID=1~4 */
	/* addr  96 */ int16_t  velocity[4];       /* R/O ×10.0 [RPM]                    */
	/* addr 104 */ int16_t  current[4];        /* R/O ×0.01 [A]                      */
	/* addr 112 */ int8_t   temp[4];           /* R/O ×1 [°C]    range -20~127       */
	/* addr 116 */ uint8_t  delta_tick[4];     /* R/O ×0.1[ms] 모터별 CAN 피드백 수신 지연. 0xFF=stale */
	/* addr 120 */ uint8_t  cmd_delta_tick[4]; /* R/O ×0.1[ms] 모터별 CAN 명령 송출 지연 (TX mailbox 진입 시각). 0xFF=stale */
	/* addr 124 */ uint16_t error_code;        /* R/O 4bit×4 packed (LSB=M1): AK 에러코드 원시값 */
	/* addr 126 */ uint8_t  comm_err;          /* R/O 2bit×4 packed (LSB=M1): per-motor AK_COMM_ERR_t */
	/* addr 127 */ STATE_t  state;             /* R/O STATE_t — 4 모터 중 worst 요약 */
} DATA_MOTOR_t;

/* ===== [MOTOR/cmd] addr 128~179 (52 bytes) — 불변 ===== */
typedef struct __attribute__((packed)) {
	/* addr 128 */ uint8_t  ctr_mode[4];     /* R/W AK_Control_Mode_t (default 3=VELOCITY, range 0~7):
	                                          *      0=ESTOP / 1=CURRENT / 2=CURRENT_BRAKE / 3=VELOCITY
	                                          *      4=POSITION / 5=SET_ORIGIN / 6=POS_VEL_LOOP / 7=MIT */
	/* addr 132 */ float    cmd_position[4]; /* R/W 위치 목표값 [deg] — AK motor ID=1~4   */
	/* addr 148 */ float    cmd_velocity[4]; /* R/W 속도 목표값 [RPM]                     */
	/* addr 164 */ float    cmd_current[4];  /* R/W 전류 목표값 [A]                       */
} CMD_MOTOR_t;

/* ===== 활성 모터 마스크 (addr 192) 기본값 ===== */
#define MOTOR_MASK_ALL         0x0F  /* bit0~3 = M1~4 전체 활성 (default) */

/* ===== [SYSTEM/cmd] addr 180~192 (13 bytes) — 2026-07-17 motor_mask 확장 ===== */
typedef struct __attribute__((packed)) {
	/* addr 180 */ float    cmd_lin_vel;     /* R/W AUTO 모드 선속도 [m/s]   (cmd_vel[0]) */
	/* addr 184 */ float    cmd_ang_vel;     /* R/W AUTO 모드 각속도 [rad/s] (cmd_vel[1]) */
	/* addr 188 */ uint8_t  auto_mode;       /* R/W AUTO 경로 선택 (default 0=KINEMATIC, range 0~3):
	                                          *      0=KINEMATIC (lin/ang → 속도 제어)
	                                          *      1=CURRENT   (cmd_current 직접 입력)
	                                          *      2=DIRECT    (reg 값 무가공 통과 — 주의요망)
	                                          *      3=CONTROL   (TODO 미래 확장 — 현재 motor_on=0) */
	/* addr 189 */ uint8_t  soft_estop;      /* R/W Orin 용 soft ESTOP: 0=작동(AUTO 에서 CAN_AK_ESTOP 제동) / 1=해제(default) */
	/* addr 190 */ uint8_t  mode;            /* R/W 0=MANUAL / 1=AUTO — GPIO MODE 핀 연동 (PERIPHERAL_DATA_t.MODE) */
	/* addr 191 */ uint8_t  use_lpf;         /* R/W cmd_velocity LPF 게이트 (default 1=use).
	                                          *     MANUAL 은 ECU 소유: 속도모드=1 / 전류모드=0 (RC_TO_REGISTER).
	                                          *     AUTO 는 Orin 소유. 0→1 상승엣지에 LPF 리셋 (RD_TASK_CONTROL) */
	/* addr 192 */ uint8_t  motor_mask;      /* R/W 활성 모터 비트필드 (bit0~3 = M1~4, default 0x0F).
	                                          *     제외 모터: TX skip + comm/worst 집계 제외 (단일 트랙 테스트용).
	                                          *     활성 모터 지속 무응답(500ms, 구동 중)은 ESTOP_SW
	                                          *     (failsafe_analysis_260717.md §8-P1) */
} CMD_SYSTEM_t;

/* ===== [RSVD1] addr 193~223 (31 bytes) ===== */
/* 읽으면 0x00 반환 / 쓰기 무시 (LUT 에서 R/O 로 차단) */

/* ===== [DIAG] addr 224~255 (32 bytes) — R/O, 불변 ===== */
typedef struct __attribute__((packed)) {
	/* addr 224 */ uint32_t cmd_write_tick;       /* cmd 영역(132~187) WRITE 시각 [osKernel ms] — AUTO 워치독 기준 */
	/* addr 228 */ uint8_t  reserved[28];         /* 향후 진단 확장용.
	                                               * 구 addr 228 = rs485_proc_delta 였으나 2026-07-27 에
	                                               * SYS(addr 32) 로 이동 → 그 자리는 reserved 로 흡수 */
} DIAG_t;

/* ===== 전체 레지스터 맵 — Total 256 bytes (멤버 순서 = #define offset 순) ===== */
typedef struct __attribute__((packed)) {
	/* addr   0 */ DEFINE_t        reg_df;        /* 16 bytes */
	/* addr  16 */ DATA_SYSTEM_t   sys;           /* 17 bytes (addr 16~32) */
	/* addr  33 */ uint8_t         reserved0[9];  /*  9 bytes (addr 33~41) */
	/* addr  42 */ DATA_LOADCELL_t loadcell;      /*  6 bytes */
	/* addr  48 */ DATA_IMU_t      imu;           /* 22 bytes */
	/* addr  70 */ DATA_ENCODER_t  encoder;       /* 16 bytes */
	/* addr  86 */ DATA_UART2_t    uart2;         /*  1 byte  */
	/* addr  87 */ DATA_RC_t       rc;            /*  1 byte  */
	/* addr  88 */ DATA_MOTOR_t    motor_data;    /* 40 bytes */
	/* addr 128 */ CMD_MOTOR_t     cmd_motor;     /* 52 bytes */
	/* addr 180 */ CMD_SYSTEM_t    cmd_system;    /* 13 bytes */
	/* addr 193 */ uint8_t         reserved1[31]; /* 31 bytes */
	/* addr 224 */ DIAG_t          diag;          /* 32 bytes */
} REGISTER_t;

/* ===== Compile-time size 검증 (총 256 byte) — STM32CubeIDE 인식 불가로 주석 처리 ===== */
//static_assert(sizeof(DEFINE_t)       == REG_DEFINE_SIZE,      "DEFINE_t size mismatch");
//static_assert(sizeof(DATA_SYSTEM_t)  == REG_SYS_SIZE,         "DATA_SYSTEM_t size mismatch");
//static_assert(sizeof(DATA_LOADCELL_t)== REG_LOADCELL_SIZE,    "DATA_LOADCELL_t size mismatch");
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
