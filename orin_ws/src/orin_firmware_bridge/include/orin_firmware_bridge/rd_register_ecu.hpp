#ifndef ORIN_FIRMWARE_BRIDGE__RD_REGISTER_ECU_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_REGISTER_ECU_HPP_

// stm_ws/ECU_V3/Core/Inc/rd_register_ecu.h 의 C++ 래핑
// 구조체 레이아웃·오프셋·크기는 STM32 측과 100% 동일해야 함 (__attribute__((packed)) 필수)

#include "orin_firmware_bridge/rd_common.hpp"

namespace orin_bridge::ecu {

/* ===== Region offset / size (bytes) ===== */
constexpr uint16_t REG_TOTAL_SIZE         = 256;

constexpr uint16_t REG_DEFINE_OFFSET      =   0;
constexpr uint16_t REG_DEFINE_SIZE        =  16;

constexpr uint16_t REG_RSVD0_OFFSET       =  16;
constexpr uint16_t REG_RSVD0_SIZE         =  30;

constexpr uint16_t REG_SYS_OFFSET         =  46;
constexpr uint16_t REG_SYS_SIZE           =  16;

constexpr uint16_t REG_IMU_OFFSET         =  62;
constexpr uint16_t REG_IMU_SIZE           =  21;

constexpr uint16_t REG_ENCODER_OFFSET     =  83;
constexpr uint16_t REG_ENCODER_SIZE       =  11;

constexpr uint16_t REG_UART2_OFFSET       =  94;
constexpr uint16_t REG_UART2_SIZE         =   1;

constexpr uint16_t REG_SENSOR_RC_OFFSET   =  95;
constexpr uint16_t REG_SENSOR_RC_SIZE     =   1;

constexpr uint16_t REG_MOTOR_DATA_OFFSET  =  96;
constexpr uint16_t REG_MOTOR_DATA_SIZE    =  32;

constexpr uint16_t REG_CMD_MOTOR_OFFSET   = 128;
constexpr uint16_t REG_CMD_MOTOR_SIZE     =  52;

constexpr uint16_t REG_CMD_SYSTEM_OFFSET  = 180;
constexpr uint16_t REG_CMD_SYSTEM_SIZE    =  12;

constexpr uint16_t REG_RSVD1_OFFSET       = 192;
constexpr uint16_t REG_RSVD1_SIZE         =  32;

constexpr uint16_t REG_DIAG_OFFSET        = 224;
constexpr uint16_t REG_DIAG_SIZE          =  32;

/* ===== sys_write_mode 값 ===== */
constexpr uint8_t SYS_WRITE_LOCK   = 0;
constexpr uint8_t SYS_WRITE_UNLOCK = 1;

/* ===== soft_estop (addr 189) — Orin 용 소프트 ESTOP ===== */
constexpr uint16_t REG_SOFT_ESTOP_OFFSET = 189;
constexpr uint8_t  SOFT_ESTOP_ACTIVE     = 0;  // ESTOP 작동: AUTO 모드에서 CAN_AK_ESTOP 소프트 제동
constexpr uint8_t  SOFT_ESTOP_RELEASE    = 1;  // 해제 (default)

/* ===== [DEFINE] addr 0~15 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr  0 */ uint8_t sys_write_mode;
    /* addr  1 */ uint8_t err_timeout;
    /* addr  2 */ uint8_t fatal_timeout;
    /* addr  3 */ uint8_t err_cnt;
    /* addr  4 */ uint8_t fatal_cnt;
    /* addr  5 */ uint8_t hw_reset;
    /* addr  6 */ uint8_t reserved[10];
} DEFINE_t;

/* ===== [SYSTEM] addr 46~61 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 46 */ uint8_t  degraded_cnt[8]; // 통신 오염도 [%]: idx0=uart1/1=uart2/2=uart4/3=can1/4=i2c1/5~7=RSVD
    /* addr 54 */ uint8_t  hw_reset;        // bitfield: HW_BIT_* 세트 시 소프트 리셋
    /* addr 55 */ uint8_t  hw_fatal;        // bitfield: 재초기화 필요 치명 에러
    /* addr 56 */ uint8_t  hw_error;        // bitfield: 현재 활성 에러
    /* addr 57 */ uint8_t  sys_state;       // SYSTEM_STATE_e: 0=INIT/1=MANUAL/2=AUTO/3=ESTOP_SW/4=ESTOP_HW/5=FAULT
    /* addr 58 */ uint32_t realtime_tick;   // [ms] TIM5 1kHz 카운터 — ECU alive time
} DATA_SYSTEM_t;

/* ===== [SENSOR/IMU] addr 62~82 (21 bytes) — TODO: 미구현 ===== */
typedef struct __attribute__((packed)) {
    /* addr 62 */ int16_t quat_z;   // ×0.0001
    /* addr 64 */ int16_t quat_y;
    /* addr 66 */ int16_t quat_x;
    /* addr 68 */ int16_t quat_w;
    /* addr 70 */ int16_t gyro_x;   // ×0.1 [deg/s]
    /* addr 72 */ int16_t gyro_y;
    /* addr 74 */ int16_t gyro_z;
    /* addr 76 */ int16_t acc_x;    // ×0.001 [g]
    /* addr 78 */ int16_t acc_y;
    /* addr 80 */ int16_t acc_z;
    /* addr 82 */ orin_bridge::STATE_t state;
} DATA_IMU_t;

/* ===== [SENSOR/ENCODER] addr 83~93 (11 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 83 */ uint16_t encoder[5]; // AS5600 ch0~4, 12bit raw [0:11] (0~4095)
    /* addr 93 */ orin_bridge::STATE_t  state;      // 5 enc + MUX 중 worst
} DATA_ENCODER_t;

/* ===== [UART2] addr 94 (1 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 94 */ orin_bridge::STATE_t state; // RS485 (UART2) 통신 상태
} DATA_UART2_t;

/* ===== [SENSOR/RC] addr 95 (1 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 95 */ orin_bridge::STATE_t state; // UART1 (RC 수신기) 통신 상태
} DATA_RC_t;

/* ===== [MOTOR/data] addr 96~127 (32 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr  96 */ int16_t  position[4];  // ×0.1 [deg]
    /* addr 104 */ int16_t  velocity[4];  // ×10.0 [RPM]
    /* addr 112 */ int16_t  current[4];   // ×0.01 [A]
    /* addr 120 */ int8_t   temp[4];      // ×1 [°C]
    /* addr 124 */ uint16_t error_code;   // 4bit×4 packed (LSB=M1): AK 에러코드
    /* addr 126 */ uint8_t  comm_err;     // 2bit×4 packed (LSB=M1): bit[0]=RX err, bit[1]=TX err
    /* addr 127 */ orin_bridge::STATE_t  state;        // 4모터 중 worst 요약
} DATA_MOTOR_t;

/* ===== [MOTOR/cmd] addr 128~179 (52 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 128 */ uint8_t ctr_mode[4];      // AK_Control_Mode_t (default 3=VELOCITY)
    /* addr 132 */ float   cmd_position[4];  // [deg]
    /* addr 148 */ float   cmd_velocity[4];  // [RPM]
    /* addr 164 */ float   cmd_current[4];   // [A]
} CMD_MOTOR_t;

/* ===== [SYSTEM/cmd] addr 180~191 (12 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 180 */ float   cmd_lin_vel;  // AUTO 모드 선속도 [m/s]
    /* addr 184 */ float   cmd_ang_vel;  // AUTO 모드 각속도 [rad/s]
    /* addr 188 */ uint8_t weight;       // throttle scale (0~3)
    /* addr 189 */ uint8_t soft_estop;   // Orin soft ESTOP: 0=작동(AUTO 에서 CAN_AK_ESTOP 제동) / 1=해제(default)
    /* addr 190 */ uint8_t mode;         // 0=MANUAL / 1=AUTO
    /* addr 191 */ uint8_t reserved;
} CMD_SYSTEM_t;

/* ===== [DIAG] addr 224~255 (32 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 224 */ uint32_t cmd_write_tick;
    /* addr 228 */ uint8_t  reserved[28];
} DIAG_t;

/* ===== 전체 레지스터 맵 — Total 256 bytes ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ DEFINE_t       reg_df;
    /* addr  16 */ uint8_t        reserved0[30];
    /* addr  46 */ DATA_SYSTEM_t  sys;
    /* addr  62 */ DATA_IMU_t     imu;
    /* addr  83 */ DATA_ENCODER_t encoder;
    /* addr  94 */ DATA_UART2_t   uart2;
    /* addr  95 */ DATA_RC_t      rc;
    /* addr  96 */ DATA_MOTOR_t   motor_data;
    /* addr 128 */ CMD_MOTOR_t    cmd_motor;
    /* addr 180 */ CMD_SYSTEM_t   cmd_system;
    /* addr 192 */ uint8_t        reserved1[32];
    /* addr 224 */ DIAG_t         diag;
} REGISTER_t;

static_assert(sizeof(DEFINE_t)       == REG_DEFINE_SIZE,     "DEFINE_t size mismatch");
static_assert(sizeof(DATA_SYSTEM_t)  == REG_SYS_SIZE,        "DATA_SYSTEM_t size mismatch");
static_assert(sizeof(DATA_IMU_t)     == REG_IMU_SIZE,        "DATA_IMU_t size mismatch");
static_assert(sizeof(DATA_ENCODER_t) == REG_ENCODER_SIZE,    "DATA_ENCODER_t size mismatch");
static_assert(sizeof(DATA_UART2_t)   == REG_UART2_SIZE,      "DATA_UART2_t size mismatch");
static_assert(sizeof(DATA_RC_t)      == REG_SENSOR_RC_SIZE,  "DATA_RC_t size mismatch");
static_assert(sizeof(DATA_MOTOR_t)   == REG_MOTOR_DATA_SIZE, "DATA_MOTOR_t size mismatch");
static_assert(sizeof(CMD_MOTOR_t)    == REG_CMD_MOTOR_SIZE,  "CMD_MOTOR_t size mismatch");
static_assert(sizeof(CMD_SYSTEM_t)   == REG_CMD_SYSTEM_SIZE, "CMD_SYSTEM_t size mismatch");
static_assert(sizeof(DIAG_t)         == REG_DIAG_SIZE,       "DIAG_t size mismatch");
static_assert(sizeof(REGISTER_t)     == REG_TOTAL_SIZE,      "REGISTER_t total size mismatch");

} // namespace orin_bridge::ecu

#endif
