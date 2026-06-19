/*
 * DYN_xm430_w350.h
 *  XM430-W350 Control Table 정의
 *  - 레지스터 주소 : DYN_ADDR_e
 *  - 레지스터 크기 : DYN_SIZE_<NAME> (bytes)
 *  - 모드/보드레이트 : DYN_OPMODE_e, DYN_BAUDRATE_e
 *  - 단위 변환 상수 : DYN_UNIT_*
 *  - 위치/라디안 한계 : DYN_POS_*, DYN_RAD_*
 *
 *  Created on: Mar 11, 2026
 *      Author: swarm
 */

#ifndef INC_DYN_XM430_W350_H_
#define INC_DYN_XM430_W350_H_

#include <stdint.h>

/* ── 모델 식별 ───────────────────────────────────────────────────────────────*/
#define DYN_MODEL_NUMBER     1020U   /* 0x03FC */
/* ══════════════════════════════════════════════════════════════════════════════
 * Control Table – 레지스터 주소 (EEPROM : 0~63, RAM : 64~)
 * ══════════════════════════════════════════════════════════════════════════════*/
typedef enum {
    /* ── EEPROM 영역 (전원 OFF 후에도 유지) ─────────────────────────────── */
    DYN_ADDR_MODEL_NUMBER         =   0, /* 2 bytes, RO */
    DYN_ADDR_MODEL_INFORMATION    =   2, /* 4 bytes, RO */
    DYN_ADDR_FIRMWARE_VERSION     =   6, /* 1 byte,  RO */
    DYN_ADDR_ID                   =   7, /* 1 byte,  RW */
    DYN_ADDR_BAUD_RATE            =   8, /* 1 byte,  RW */
    DYN_ADDR_RETURN_DELAY_TIME    =   9, /* 1 byte,  RW */
    DYN_ADDR_DRIVE_MODE           =  10, /* 1 byte,  RW */
    DYN_ADDR_OPERATING_MODE       =  11, /* 1 byte,  RW */
    DYN_ADDR_SECONDARY_ID         =  12, /* 1 byte,  RW */
    DYN_ADDR_PROTOCOL_TYPE        =  13, /* 1 byte,  RW */
    DYN_ADDR_HOMING_OFFSET        =  20, /* 4 bytes, RW */
    DYN_ADDR_MOVING_THRESHOLD     =  24, /* 4 bytes, RW */
    DYN_ADDR_TEMPERATURE_LIMIT    =  31, /* 1 byte,  RW */
    DYN_ADDR_MAX_VOLTAGE_LIMIT    =  32, /* 2 bytes, RW */
    DYN_ADDR_MIN_VOLTAGE_LIMIT    =  34, /* 2 bytes, RW */
    DYN_ADDR_PWM_LIMIT            =  36, /* 2 bytes, RW */
    DYN_ADDR_CURRENT_LIMIT        =  38, /* 2 bytes, RW */
    DYN_ADDR_VELOCITY_LIMIT       =  44, /* 4 bytes, RW */
    DYN_ADDR_MAX_POSITION_LIMIT   =  48, /* 4 bytes, RW */
    DYN_ADDR_MIN_POSITION_LIMIT   =  52, /* 4 bytes, RW */
    DYN_ADDR_SHUTDOWN             =  63, /* 1 byte,  RW */

    /* ── RAM 영역 (전원 ON 시 초기화) ───────────────────────────────────── */
    DYN_ADDR_TORQUE_ENABLE        =  64, /* 1 byte,  RW */
    DYN_ADDR_LED                  =  65, /* 1 byte,  RW */
    DYN_ADDR_STATUS_RETURN_LEVEL  =  68, /* 1 byte,  RW */
    DYN_ADDR_REGISTERED_INST      =  69, /* 1 byte,  RO */
    DYN_ADDR_HW_ERROR_STATUS      =  70, /* 1 byte,  RO */
    DYN_ADDR_VELOCITY_I_GAIN      =  76, /* 2 bytes, RW */
    DYN_ADDR_VELOCITY_P_GAIN      =  78, /* 2 bytes, RW */
    DYN_ADDR_POSITION_D_GAIN      =  80, /* 2 bytes, RW */
    DYN_ADDR_POSITION_I_GAIN      =  82, /* 2 bytes, RW */
    DYN_ADDR_POSITION_P_GAIN      =  84, /* 2 bytes, RW */
    DYN_ADDR_FEEDFORWARD_2ND_GAIN =  88, /* 2 bytes, RW */
    DYN_ADDR_FEEDFORWARD_1ST_GAIN =  90, /* 2 bytes, RW */
    DYN_ADDR_BUS_WATCHDOG         =  98, /* 1 byte,  RW */
    DYN_ADDR_GOAL_PWM             = 100, /* 2 bytes, RW */
    DYN_ADDR_GOAL_CURRENT         = 102, /* 2 bytes, RW */
    DYN_ADDR_GOAL_VELOCITY        = 104, /* 4 bytes, RW */
    DYN_ADDR_PROFILE_ACCELERATION = 108, /* 4 bytes, RW */
    DYN_ADDR_PROFILE_VELOCITY     = 112, /* 4 bytes, RW */
    DYN_ADDR_GOAL_POSITION        = 116, /* 4 bytes, RW */
    DYN_ADDR_REALTIME_TICK        = 120, /* 2 bytes, RO */
    DYN_ADDR_MOVING               = 122, /* 1 byte,  RO */
    DYN_ADDR_MOVING_STATUS        = 123, /* 1 byte,  RO */
    DYN_ADDR_PRESENT_PWM          = 124, /* 2 bytes, RO */
    DYN_ADDR_PRESENT_CURRENT      = 126, /* 2 bytes, RO */
    DYN_ADDR_PRESENT_VELOCITY     = 128, /* 4 bytes, RO */
    DYN_ADDR_PRESENT_POSITION     = 132, /* 4 bytes, RO */
    DYN_ADDR_VELOCITY_TRAJECTORY  = 136, /* 4 bytes, RO */
    DYN_ADDR_POSITION_TRAJECTORY  = 140, /* 4 bytes, RO */
    DYN_ADDR_PRESENT_INPUT_VOLTAGE= 144, /* 2 bytes, RO */
    DYN_ADDR_PRESENT_TEMPERATURE  = 146, /* 1 byte,  RO */
} DYN_ADDR_e;

/* ── 레지스터 크기 (bytes) ───────────────────────────────────────────────────*/
#define DYN_SIZE_MODEL_NUMBER         2U
#define DYN_SIZE_MODEL_INFORMATION    4U
#define DYN_SIZE_FIRMWARE_VERSION     1U
#define DYN_SIZE_ID                   1U
#define DYN_SIZE_BAUD_RATE            1U

#define DYN_SIZE_OPERATING_MODE       1U
#define DYN_SIZE_TORQUE_ENABLE        1U

#define DYN_SIZE_HW_ERROR_STATUS      1U

#define DYN_SIZE_GOAL_CURRENT         2U
#define DYN_SIZE_GOAL_VELOCITY        4U
#define DYN_SIZE_PROFILE_ACCELERATION 4U
#define DYN_SIZE_PROFILE_VELOCITY     4U
#define DYN_SIZE_GOAL_POSITION        4U

#define DYN_SIZE_PRESENT_CURRENT      2U
#define DYN_SIZE_PRESENT_VELOCITY     4U
#define DYN_SIZE_PRESENT_POSITION     4U
#define DYN_SIZE_PRESENT_TEMPERATURE  1U

#define DYN_SIZE_POSITION_I_GAIN	  2U

/* ══════════════════════════════════════════════════════════════════════════════
 * RAM 영역 Bulk-Read 전용 Packed Struct  (addr 64 ~ 146, 총 83 bytes)
 *
 *  사용법:
 *    DYN_RAM_t ram;
 *    dxl_read(id, DYN_ADDR_TORQUE_ENABLE, (uint8_t*)&ram, DYN_RAM_SIZE);
 *
 *  주소 갭은 _pad_XX 필드로 채워 레지스터 오프셋을 유지합니다.
 * ══════════════════════════════════════════════════════════════════════════════*/
#define DYN_RAM_START_ADDR     64U   /* TORQUE_ENABLE 주소          */
#define DYN_RAM_SIZE           83U   /* addr 64 ~ 146 inclusive     */

#define DYN_CMD_START_ADDR    100U   /* GOAL_PWM ~ GOAL_POSITION    (addr 100~119, 20 bytes) */
#define DYN_CMD_SIZE           20U   /* 20 bytes                                             */

#define DYN_STATE_START_ADDR  120U   /* REALTIME_TICK ~ PRESENT_POSITION (addr 120~135, 16 bytes) */
#define DYN_STATE_SIZE         16U   /* 16 bytes                                                   */

typedef struct __attribute__((packed)) {
    /* addr 100 */ int16_t   goal_pwm;               /* RW, ±885 */
    /* addr 102 */ int16_t   goal_current;           /* RW, ±1193 */
    /* addr 104 */ int32_t   goal_velocity;          /* RW, signed LSB */
    /* addr 108 */ uint32_t  profile_acceleration;   /* RW */
    /* addr 112 */ uint32_t  profile_velocity;       /* RW */
    /* addr 116 */ int32_t   goal_position;          /* RW, 0~4095 */
} DYN_CMD_t;

typedef struct __attribute__((packed)) {
    /* addr 120 */ uint16_t  realtime_tick;          /* RO, ms */
    /* addr 122 */ uint8_t   moving;                 /* RO, 0/1 */
    /* addr 123 */ uint8_t   moving_status;          /* RO */
    /* addr 124 */ int16_t   present_pwm;            /* RO, signed */
    /* addr 126 */ int16_t   present_current;        /* RO, signed */
    /* addr 128 */ int32_t   present_velocity;       /* RO, signed LSB */
    /* addr 132 */ int32_t   present_position;       /* RO, signed pulse */
} DYN_STATE_t;

typedef struct __attribute__((packed)) {
    /* addr 64 */  uint8_t   torque_enable;          /* RW */
    /* addr 65 */  uint8_t   led;                    /* RW */
    /* addr 66 */  uint8_t   _pad_66[2];             /* reserved */
    /* addr 68 */  uint8_t   status_return_level;    /* RW */
    /* addr 69 */  uint8_t   registered_inst;        /* RO */
    /* addr 70 */  uint8_t   hw_error_status;        /* RO */
    /* addr 71 */  uint8_t   _pad_71[5];             /* reserved */
    /* addr 76 */  uint16_t  velocity_i_gain;        /* RW */
    /* addr 78 */  uint16_t  velocity_p_gain;        /* RW */
    /* addr 80 */  uint16_t  position_d_gain;        /* RW */
    /* addr 82 */  uint16_t  position_i_gain;        /* RW */
    /* addr 84 */  uint16_t  position_p_gain;        /* RW */
    /* addr 86 */  uint8_t   _pad_86[2];             /* reserved */
    /* addr 88 */  uint16_t  feedforward_2nd_gain;   /* RW */
    /* addr 90 */  uint16_t  feedforward_1st_gain;   /* RW */
    /* addr 92 */  uint8_t   _pad_92[6];             /* reserved */
    /* addr 98 */  int8_t    bus_watchdog;            /* RW, -1=disable */
    /* addr 99 */  uint8_t   _pad_99;                /* reserved */

    /* addr 100 */ DYN_CMD_t   cmd;      // RW
    /* addr 120 */ DYN_STATE_t state;    // RO

    /* addr 136 */ int32_t   velocity_trajectory;    /* RO, signed LSB */
    /* addr 140 */ uint32_t  position_trajectory;    /* RO, 0~4095 */
    /* addr 144 */ uint16_t  present_input_voltage;  /* RO, 0.1V/LSB */
    /* addr 146 */ uint8_t   present_temperature;    /* RO, 1°C/LSB */
} DYN_RAM_t;

/* ══════════════════════════════════════════════════════════════════════════════
 * Baud Rate (addr 8) – 레지스터 값 → 실제 bps – 1 byte
 * ══════════════════════════════════════════════════════════════════════════════*/
 typedef enum {
    DYN_BAUD_9600     = 0,
    DYN_BAUD_57600    = 1, /* 공장 초기값 */
    DYN_BAUD_115200   = 2,
    DYN_BAUD_1M       = 3,
    DYN_BAUD_2M       = 4,
    DYN_BAUD_3M       = 5,
    DYN_BAUD_4M       = 6,
} DYN_BAUDRATE_e;

/**
 * @brief Dynamixel Protocol 2.0 Status Error 상세 정의
 * Status Packet의 Error 필드(1바이트) 값을 해석하기 위한 열거형입니다.
 */
typedef enum {
   ERR_NONE         = 0, // 에러 없음: 정상적으로 명령이 처리됨
   ERR_RESULT_FAIL  = 1, // 결과 실패: 전송된 Instruction Packet을 처리하는 데 실패함
   ERR_INSTRUCTION  = 2, // 명령 에러: 정의되지 않은 Instruction 사용 또는 Reg Write 없이 Action 사용
   ERR_CRC          = 3, // CRC 에러: 수신된 패킷의 CRC 값이 계산 결과와 일치하지 않음
   ERR_DATA_RANGE   = 4, // 데이터 범위 에러: 쓰려는 데이터가 컨트롤 테이블에 정의된 범위를 벗어남
   ERR_DATA_LENGTH  = 5, // 데이터 길이 에러: Instruction Packet에 명시된 데이터 길이가 실제와 맞지 않음
   ERR_DATA_LIMIT   = 6, // 데이터 한계 에러: 쓰려는 데이터가 해당 주소의 최대/최소 한계치를 초과함
   ERR_ACCESS       = 7, // 액세스 에러: Read-only 주소에 쓰기 시도, 정의되지 않은 주소 접근 등
} DYN_STATUS_ERROR_e;

/* ══════════════════════════════════════════════════════════════════════════════
 * Operationg mode (addr 11) – 1 byte
 * ══════════════════════════════════════════════════════════════════════════════*/

typedef enum {
    DYN_MODE_CURRENT      = 0,  /* 전류 제어 모드 */
    DYN_MODE_VELOCITY     = 1,  /* 속도 제어 모드 */
    DYN_MODE_POSITION     = 3,  /* 위치 제어 모드 (0~360°) */
    DYN_MODE_EXT_POSITION = 4,  /* 확장 위치 모드 (멀티턴) */
    DYN_MODE_CUR_POSITION = 5,  /* 전류 기반 위치 모드 */
    DYN_MODE_PWM          = 16  /* PWM 모드 */
} DYN_MODE_e;

/* ══════════════════════════════════════════════════════════════════════════════
 * 위치 / 라디안 변환
 * ══════════════════════════════════════════════════════════════════════════════*/
 #define DYN_POS_ZERO          2048    /* 0 rad에 해당하는 pulse 값 */
 #define DYN_POS_MAX           4095    /* +π rad에 해당하는 pulse 값 */
 #define DYN_POS_MIN              0    /* -π rad에 해당하는 pulse 값 */
 
 #define DYN_RAD_MAX           3.14159265f  /*  π [rad] */
 #define DYN_RAD_MIN          -3.14159265f  /* -π [rad] */
 
 /* pulse → rad : rad = (pulse - 2048) * (π / 2048) */
 #define DYN_PULSE_TO_RAD(pulse)   (((int32_t)(pulse) - DYN_POS_ZERO) * (DYN_RAD_MAX / 2048.0f))
 /* rad → pulse : pulse = (int32_t)(rad * 2048 / π) + 2048 */
 #define DYN_RAD_TO_PULSE(rad)     ((int32_t)((rad) * (2048.0f / DYN_RAD_MAX)) + DYN_POS_ZERO)
 /* ══════════════════════════════════════════════════════════════════════════════
  * 단위 변환 상수
  * ══════════════════════════════════════════════════════════════════════════════*/
 /* Velocity : 1 LSB = 0.0239691227 rad/s (양방향 부호) */
 #define DYN_UNIT_VELOCITY     0.0239691227f  /* [rad/s / LSB] */
 /* Current : 1 LSB = 1 raw (모델별 단위 다름, XM430은 약 2.69 mA/LSB) */
 #define DYN_UNIT_CURRENT_MA   2.69f          /* [mA / LSB] */
 
 /* raw → rad/s */
 #define DYN_VEL_TO_RADS(raw)  ((float)(raw) * DYN_UNIT_VELOCITY)
 /* raw → mA */
 #define DYN_CUR_TO_MA(raw)    ((float)(raw) * DYN_UNIT_CURRENT_MA)

#endif /* INC_DYN_DYN_W350_H_ */
