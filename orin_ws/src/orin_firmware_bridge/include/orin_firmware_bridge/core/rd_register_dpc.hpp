#ifndef ORIN_FIRMWARE_BRIDGE__RD_REGISTER_DPC_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_REGISTER_DPC_HPP_

// DPC-B 레지스터 맵 — 256B. **진실 원천은 펌웨어 헤더 `rd_register_dpcb.h`** 이고
// 이 파일은 그 미러다 (2026-07-30 반영, Modbus_MAP_DPCB_v2.csv 기준).
// Little-Endian / 1 addr = 1 byte / 실제값 = raw × Scale.
//
// ⚠ ECU 미러(`rd_register_ecu.hpp`)와 같은 관계다: 두 헤더는 **손으로 동기화**되고,
//   어긋나도 빌드·테스트가 통과하면서 브리지가 엉뚱한 주소를 읽는다 (01 §9.1).
//   그래서 크기뿐 아니라 **`offsetof` 로 절대주소까지** 못박는다.
//
// ## 이 맵은 DPC-B 것이고 DPC-A 를 **중계**한다
//
// DPC-A 는 DPC-B 에 UART4 로 물려 있다. Orin 은 DPC-A 를 직접 부르지 않고,
// 이 맵의 `[SENSOR/DPCA]`(62~64) 로 상태를 읽고 `[CMD/DPCA]`(120~121) 로 명령한다.
//
// ## ⚠ ECU 와 레이아웃이 다르다 — 04 §6.2(C5) 의 "SYS 통일" 은 채택되지 않았다
//
// | | ECU | DPC-B |
// |---|---|---|
// | SYS 위치·크기 | `16 : 17` | **`46 : 16`** |
// | hw_* 순서 | error(24) fatal(25) reset(26) | **reset(54) fatal(55) error(56)** ← 역순 |
// | `rs485_proc_delta` | 있음(28) | **없음** |
// | `realtime_tick` | ×0.1ms (TIM5 10kHz) | **µs (TIM5 1MHz)** ← 단위가 다르다 |
// | `sys_state` | SYSTEM_STATE_e (0~5) | **DPCB_STATE_e (0~10)** — 아래 참조 |
// | degraded_cnt 채널 | 0=uart1 1=uart2 2=uart6 3=can1 4=i2c1 | **0=uart2 1=uart4 2=uart6 3=i2c1** |
//
// 그래서 04 §6.2 가 예고한 대가를 그대로 치른다: `NodeStatus` 매핑을 **보드별로** 해야 하고,
// 시간 동기(B1) 확장은 `proc_delta` 부재 + 단위 차이를 먼저 풀어야 한다.

#include <cstddef>

#include "orin_firmware_bridge/core/rd_common.hpp"

namespace orin_bridge::dpc {

/* ===== Region offset / size (bytes) — 펌웨어 헤더의 #define 과 1:1 ===== */
constexpr uint16_t REG_TOTAL_SIZE        = 256;

constexpr uint16_t REG_DEFINE_OFFSET     =   0;
constexpr uint16_t REG_DEFINE_SIZE       =  16;

constexpr uint16_t REG_RSVD0_OFFSET      =  16;
constexpr uint16_t REG_RSVD0_SIZE        =  30;

constexpr uint16_t REG_SYS_OFFSET        =  46;
constexpr uint16_t REG_SYS_SIZE          =  16;

constexpr uint16_t REG_SENSOR_DPCA_OFFSET =  62;
constexpr uint16_t REG_SENSOR_DPCA_SIZE   =   3;

constexpr uint16_t REG_UART2_OFFSET      =  65;
constexpr uint16_t REG_UART2_SIZE        =   1;

constexpr uint16_t REG_SENSOR_DPCB_OFFSET =  66;
constexpr uint16_t REG_SENSOR_DPCB_SIZE   =   8;

constexpr uint16_t REG_MOTOR_DATA_OFFSET =  74;
constexpr uint16_t REG_MOTOR_DATA_SIZE   =  37;

constexpr uint16_t REG_RSVD1_OFFSET      = 111;
constexpr uint16_t REG_RSVD1_SIZE        =   9;

constexpr uint16_t REG_CMD_DPCA_OFFSET   = 120;
constexpr uint16_t REG_CMD_DPCA_SIZE     =   2;

constexpr uint16_t REG_CMD_DPCB_OFFSET   = 122;
constexpr uint16_t REG_CMD_DPCB_SIZE     =   6;

constexpr uint16_t REG_CMD_MOT_OFFSET    = 128;
constexpr uint16_t REG_CMD_MOT_SIZE      =  15;

constexpr uint16_t REG_RSVD2_OFFSET      = 143;
constexpr uint16_t REG_RSVD2_SIZE        =  32;

// ⚠ DIAG 는 **32B 다** (진단 카운터 4 + 확장 예약 28). 시트만 보고 4 로 적었다가
//   펌웨어 헤더로 정정했다 — 시트의 마지막 RESERVED "48" 도 오타였고 49 가 맞다.
constexpr uint16_t REG_DIAG_OFFSET       = 175;
constexpr uint16_t REG_DIAG_SIZE         =  32;

constexpr uint16_t REG_RSVD3_OFFSET      = 207;
constexpr uint16_t REG_RSVD3_SIZE        =  49;

/* CMD/MOT 쓰기 감시 범위 (DPC 의 cmd_write_tick 갱신 판정) */
constexpr uint16_t REG_CMD_MOT_S_OFFSET  = 128;   // torque_en[0]
constexpr uint16_t REG_CMD_MOT_E_OFFSET  = 142;   // goal_current[2] 마지막 바이트

/* ===== 주요 필드 절대주소 — 시퀀스·커맨드가 직접 쓴다 ===== */
constexpr uint16_t REG_SYS_STATE_OFFSET        =  57;   // R/O  현재 상태
constexpr uint16_t REG_DPCA_LOCKER_EN_OFFSET   = 120;
constexpr uint16_t REG_DPCA_BOOT_EN_OFFSET     = 121;
constexpr uint16_t REG_DPCB_LOCKER_EN_OFFSET   = 122;
constexpr uint16_t REG_DPCB_BOOT_EN_OFFSET     = 123;
constexpr uint16_t REG_DPCB_LIGHT_EN_OFFSET    = 124;
constexpr uint16_t REG_DPCB_SERVO_CMD_OFFSET   = 125;
constexpr uint16_t REG_MODE_OFFSET             = 126;
constexpr uint16_t REG_SYS_STATE_TARGET_OFFSET = 127;   // R/W  목표 상태

/* ===== sys_write_mode ===== */
constexpr uint8_t SYS_WRITE_LOCK   = 0;
constexpr uint8_t SYS_WRITE_UNLOCK = 1;

/* ===== HW bitfield (hw_reset / hw_fatal / hw_error 공통) =====
 * ⚠ 이름을 `HW_BIT_*` 로 두면 안 된다: ECU 쪽 같은 이름이 `rd_common.hpp` 의
 *   **전역 매크로**라 네임스페이스와 무관하게 치환된다 (실제로 컴파일이 깨졌다). */
constexpr uint8_t kHwBitUart2 = (1u << 0);   // Orin RS485 (USART2)
constexpr uint8_t kHwBitUart4 = (1u << 1);   // DPC-A UART (UART4)
constexpr uint8_t kHwBitUart6 = (1u << 2);   // Dynamixel  (USART6)
constexpr uint8_t kHwBitI2c1  = (1u << 3);   // MCP23017   (I2C1)

/* ===== 기본값 (펌웨어 RD_MAP_INIT) ===== */
constexpr uint8_t  DEF_ERR_TIMEOUT   =  15;   // [ms]
constexpr uint8_t  DEF_FATAL_TIMEOUT =  10;   // [×100ms], 0=감지 안 함
constexpr uint8_t  DEF_ERR_CNT       =   5;
constexpr uint8_t  DEF_FATAL_CNT     =  10;
constexpr int16_t  DEF_GOAL_CURRENT  = 750;   // ×2.69mA ≈ 2.0A

/* ===== CMD/DPCB.servo_cmd ===== */
constexpr uint8_t SERVO_CMD_IDLE   = 0;
constexpr uint8_t SERVO_CMD_LOCK   = 1;
constexpr uint8_t SERVO_CMD_UNLOCK = 2;

/* ===== CMD/DPCB.mode (addr 126) ===== */
constexpr uint8_t MODE_MANUAL = 0;   // 패널 기반 직접 제어
constexpr uint8_t MODE_AUTO   = 1;   // Orin RS485 관제 — 세부 동작은 sys_state 가 정한다

/* ===== DPCB_STATE_e — sys_state(57, R/O) / sys_state_target(127, R/W) =====
 *
 *   CTRL(0) ──target=INIT(2)──▶ INIT(2) ─▶ DESCEND_1(3) ─▶ DESCEND_2(4) ─▶ WAIT(5)
 *                                                                            │
 *                                        Orin 이 target=ASCEND_1(6) 을 써야 전이
 *                                                                            ▼
 *   CTRL(0) ◀──자동 복귀── FINISH(8) ◀─ ASCEND_2(7) ◀─ ASCEND_1(6) ◀──────────┘
 *
 * ⚠ **`WAIT(5)` 가 카메라 위치다.** FSM 이 거기서 멈춰 Orin 을 기다린다 —
 *   전개 시퀀스의 CAMERA_ACTION 이 들어갈 자리가 바로 이 지점이다.
 * ⚠ **FINISH(8) 이후 CTRL(0) 로 자동 복귀한다.** 5Hz 폴링으로는 FINISH 를 놓칠 수
 *   있으므로 회수 완료 판정은 둘 다 받는다 (rd_sequence.cpp `IsRetractDone`). */
enum : uint8_t {
    STATE_CTRL      =  0,   // Orin 직접 관제 (기본)
    STATE_HOLD      =  1,   // 수송 고정 (포지션 고정)
    STATE_INIT      =  2,   // ┐ FSM 자동 전개
    STATE_DESCEND_1 =  3,   // │
    STATE_DESCEND_2 =  4,   // │
    STATE_WAIT      =  5,   // │ ★ 카메라 위치 — Orin 이 target=ASCEND_1 을 써야 진행
    STATE_ASCEND_1  =  6,   // │
    STATE_ASCEND_2  =  7,   // │
    STATE_FINISH    =  8,   // ┘ 완료 → CTRL 자동 복귀
    STATE_RSVD      =  9,   // 예약
    STATE_ERROR     = 10,   // FSM 실패 또는 통신 FAULT
};

/* FSM 자동 전개 구간(2~8) 안인가 — 여기 있으면 이전 시퀀스가 진행 중이다. */
inline bool IsDeployState(uint8_t s) { return s >= STATE_INIT && s <= STATE_FINISH; }

/* Orin 이 `sys_state_target` 에 **쓸 수 있는 값**은 넷뿐이다 (펌웨어 헤더 mask).
 * 나머지는 FSM 이 스스로 밟는 중간 상태라, 밖에서 쓰면 단계를 건너뛰게 된다. */
inline bool IsWritableTarget(uint8_t s) {
    return s == STATE_CTRL || s == STATE_HOLD || s == STATE_INIT || s == STATE_ASCEND_1;
}

inline const char* SysStateName(uint8_t s) {
    switch (s) {
        case STATE_CTRL:      return "CTRL";
        case STATE_HOLD:      return "HOLD";
        case STATE_INIT:      return "INIT";
        case STATE_DESCEND_1: return "DESCEND_1";
        case STATE_DESCEND_2: return "DESCEND_2";
        case STATE_WAIT:      return "WAIT";
        case STATE_ASCEND_1:  return "ASCEND_1";
        case STATE_ASCEND_2:  return "ASCEND_2";
        case STATE_FINISH:    return "FINISH";
        case STATE_RSVD:      return "RSVD";
        case STATE_ERROR:     return "ERROR";
        default:              return "?";
    }
}

/* ===== [DEFINE] addr 0~15 (16 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr  0 */ uint8_t sys_write_mode;   // 0=LOCK / 1=UNLOCK — DEFINE 영역 쓰기 허용 키
    /* addr  1 */ uint8_t err_timeout;      // [ms]     기본 15  (10~500)
    /* addr  2 */ uint8_t fatal_timeout;    // [×100ms] 기본 10  (0=감지 안 함)
    /* addr  3 */ uint8_t err_cnt;          // 기본 5  (0~10)
    /* addr  4 */ uint8_t fatal_cnt;        // 기본 10 (0=감지 안 함)
    /* addr  5 */ uint8_t hw_reset;         // bitfield: kHwBit*
    /* addr  6 */ uint8_t reserved[10];
} DEFINE_t;

/* ===== [SYSTEM] addr 46~61 (16 byte) =====
 * ⚠ ECU 의 DATA_SYSTEM_t 와 **필드 순서가 다르다** (hw_reset/fatal/error 역순, proc_delta 없음).
 *   공용 매핑 함수를 쓰지 말 것. */
typedef struct __attribute__((packed)) {
    /* addr 46 */ uint8_t  degraded_cnt[8];  // [%] 0=uart2 1=uart4 2=uart6 3=i2c1 / 4~7=RSVD
    /* addr 54 */ uint8_t  hw_reset;
    /* addr 55 */ uint8_t  hw_fatal;
    /* addr 56 */ uint8_t  hw_error;
    /* addr 57 */ uint8_t  sys_state;        // DPCB_STATE_e
    // ⚠ [µs] 다 — DPC_B 펌웨어 `rd_register_dpcb.h:169` 주석은 `[ms]` 라고 적혀 있지만
    //   TIM5 는 APB1 타이머클럭 80MHz / Prescaler 80 = 1MHz 로 돌아간다 (2026-08-13 확인).
    //   uint32 라 **약 71.6분에 한 바퀴 돈다** — alive_time 은 그 주기로 0 으로 되감긴다.
    /* addr 58 */ uint32_t realtime_tick;    // [µs] TIM5 1MHz — DPC-B alive time
} DATA_SYSTEM_t;

/* ===== [SENSOR/DPCA] addr 62~64 (3 byte) — DPC-A 중계 피드백 ===== */
typedef struct __attribute__((packed)) {
    /* addr 62 */ uint8_t prox_contact;              // 3bit — 지면 근접 센서 (bit0~2 채널별)
    /* addr 63 */ uint8_t lock_contact;              // 4bit — DPC-A 집게 접점 (bit0~3)
    /* addr 64 */ orin_bridge::STATE_t uart4_state;  // UART4(DPC-A) 채널 상태
} DATA_SENSOR_DPCA_t;

/* ===== [UART2] addr 65 (1 byte) — Orin RS485 채널 상태 ===== */
typedef struct __attribute__((packed)) {
    /* addr 65 */ orin_bridge::STATE_t state;
} DATA_UART2_t;

/* ===== [GPIO] addr 66~73 (8 byte) — DPC-B 패널/GPIO ===== */
typedef struct __attribute__((packed)) {
    /* addr 66 */ uint8_t lock_contact;              // 4bit — DPC-B lock 접점 (bit0~3)
    /* addr 67 */ uint8_t ex_sw[6];                  // MCP23017 패널 SW1~SW6
                                                     //   [0][1] SPST: 0=OFF 1=ON
                                                     //   [2]~[5] SPDT: 0=IDLE(mid) 1=UP 2=DOWN
    /* addr 73 */ orin_bridge::STATE_t panel_state;  // I2C1(MCP23017) 채널 상태
} DATA_SENSOR_DPCB_t;

/* ===== [MOTOR/data] addr 74~110 (37 byte) — Dynamixel ID=2/3/4 =====
 * ⚠ ECU 의 AK 모터와 **스케일이 전부 다르다.** 공용 변환을 쓰지 말 것. */
typedef struct __attribute__((packed)) {
    /* addr  74 */ int32_t present_position[3];     // ×1 [pulse] 멀티턴
    /* addr  86 */ int32_t present_velocity[3];     // ×0.229 [rev/min]
    /* addr  98 */ int16_t present_current[3];      // ×2.69 [mA]
    /* addr 104 */ uint8_t present_temperature[3];  // ×1 [°C]
    /* addr 107 */ uint8_t hardware_error[3];       // Dynamixel HW 에러 raw
    /* addr 110 */ orin_bridge::STATE_t uart6_state;
} DATA_MOTOR_DYN_t;

constexpr float VEL_TO_RPM    = 0.229f;
constexpr float CURRENT_TO_MA = 2.69f;

/* ===== [CMD/DPCA] addr 120~121 (2 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 120 */ uint8_t locker_en;   // DPC-A 집게 잠금 0=OFF 1=ON
    /* addr 121 */ uint8_t boot_en;     // DPC-A 부트 동작 0=OFF 1=ON
} CMD_DPCA_t;

/* ===== [CMD/DPCB] addr 122~127 (6 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 122 */ uint8_t locker_en;          // 로커 솔레노이드 0=OFF 1=ON
    /* addr 123 */ uint8_t boot_en;            // 부트 솔레노이드 0=OFF 1=ON
    /* addr 124 */ uint8_t light_en;           // 조명 LED       0=OFF 1=ON
    /* addr 125 */ uint8_t servo_cmd;          // SERVO_CMD_*
    /* addr 126 */ uint8_t mode;               // MODE_MANUAL / MODE_AUTO
    /* addr 127 */ uint8_t sys_state_target;   // DPCB_STATE_e — IsWritableTarget() 만 허용
} CMD_DPCB_t;

/* ===== [CMD/MOT] addr 128~142 (15 byte) — Dynamixel ID=2/3/4 ===== */
typedef struct __attribute__((packed)) {
    /* addr 128 */ uint8_t torque_en[3];       // 0=OFF 1=ON
    /* addr 131 */ int16_t goal_position[3];   // ×1 [pulse]
    /* addr 137 */ int16_t goal_current[3];    // ×2.69 [mA], 기본 750
} CMD_MOT_t;

/* ===== [DIAG] addr 175~206 (32 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 175 */ uint32_t cmd_write_tick;    // 상위가 CMD/MOT 에 마지막으로 쓴 시각
    /* addr 179 */ uint8_t  diag_rsvd[28];
} DIAG_t;

/* ===== 전체 256B (멤버 순서 = offset 순) ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ DEFINE_t           reg_df;
    /* addr  16 */ uint8_t            reserved0[REG_RSVD0_SIZE];
    /* addr  46 */ DATA_SYSTEM_t      sys;
    /* addr  62 */ DATA_SENSOR_DPCA_t sensor_dpca;
    /* addr  65 */ DATA_UART2_t       uart2;
    /* addr  66 */ DATA_SENSOR_DPCB_t sensor_dpcb;
    /* addr  74 */ DATA_MOTOR_DYN_t   motor_data;
    /* addr 111 */ uint8_t            reserved1[REG_RSVD1_SIZE];
    /* addr 120 */ CMD_DPCA_t         cmd_dpca;
    /* addr 122 */ CMD_DPCB_t         cmd_dpcb;
    /* addr 128 */ CMD_MOT_t          cmd_mot;
    /* addr 143 */ uint8_t            reserved2[REG_RSVD2_SIZE];
    /* addr 175 */ DIAG_t             diag;
    /* addr 207 */ uint8_t            reserved3[REG_RSVD3_SIZE];
} REGISTER_t;

/* 크기 검증 — 펌웨어 헤더 쪽은 CubeIDE 제약으로 주석 처리돼 있다. **이쪽이 그 몫을 한다.** */
static_assert(sizeof(DEFINE_t)           == REG_DEFINE_SIZE,      "DEFINE_t size mismatch");
static_assert(sizeof(DATA_SYSTEM_t)      == REG_SYS_SIZE,         "DATA_SYSTEM_t size mismatch");
static_assert(sizeof(DATA_SENSOR_DPCA_t) == REG_SENSOR_DPCA_SIZE, "DATA_SENSOR_DPCA_t size mismatch");
static_assert(sizeof(DATA_UART2_t)       == REG_UART2_SIZE,       "DATA_UART2_t size mismatch");
static_assert(sizeof(DATA_SENSOR_DPCB_t) == REG_SENSOR_DPCB_SIZE, "DATA_SENSOR_DPCB_t size mismatch");
static_assert(sizeof(DATA_MOTOR_DYN_t)   == REG_MOTOR_DATA_SIZE,  "DATA_MOTOR_DYN_t size mismatch");
static_assert(sizeof(CMD_DPCA_t)         == REG_CMD_DPCA_SIZE,    "CMD_DPCA_t size mismatch");
static_assert(sizeof(CMD_DPCB_t)         == REG_CMD_DPCB_SIZE,    "CMD_DPCB_t size mismatch");
static_assert(sizeof(CMD_MOT_t)          == REG_CMD_MOT_SIZE,     "CMD_MOT_t size mismatch");
static_assert(sizeof(DIAG_t)             == REG_DIAG_SIZE,        "DIAG_t size mismatch");
static_assert(sizeof(REGISTER_t)         == REG_TOTAL_SIZE,       "REGISTER_t total size mismatch");

/* 절대주소 검증 — 크기만 보면 **필드 순서 변경을 못 잡는다** (01 §9.1 의 교훈). */
static_assert(offsetof(REGISTER_t, sys)         == REG_SYS_OFFSET,         "sys 위치 어긋남");
static_assert(offsetof(REGISTER_t, sensor_dpca) == REG_SENSOR_DPCA_OFFSET, "sensor_dpca 위치 어긋남");
static_assert(offsetof(REGISTER_t, uart2)       == REG_UART2_OFFSET,       "uart2 위치 어긋남");
static_assert(offsetof(REGISTER_t, sensor_dpcb) == REG_SENSOR_DPCB_OFFSET, "sensor_dpcb 위치 어긋남");
static_assert(offsetof(REGISTER_t, motor_data)  == REG_MOTOR_DATA_OFFSET,  "motor_data 위치 어긋남");
static_assert(offsetof(REGISTER_t, cmd_dpca)    == REG_CMD_DPCA_OFFSET,    "cmd_dpca 위치 어긋남");
static_assert(offsetof(REGISTER_t, cmd_dpcb)    == REG_CMD_DPCB_OFFSET,    "cmd_dpcb 위치 어긋남");
static_assert(offsetof(REGISTER_t, cmd_mot)     == REG_CMD_MOT_OFFSET,     "cmd_mot 위치 어긋남");
static_assert(offsetof(REGISTER_t, diag)        == REG_DIAG_OFFSET,        "diag 위치 어긋남");
/* 시퀀스가 쓰는 두 주소는 특히 못박는다 — 밀리면 전개 명령이 엉뚱한 곳에 간다. */
static_assert(offsetof(REGISTER_t, sys) + offsetof(DATA_SYSTEM_t, sys_state)
                  == REG_SYS_STATE_OFFSET, "sys_state 절대주소 어긋남");
static_assert(offsetof(REGISTER_t, cmd_dpcb) + offsetof(CMD_DPCB_t, sys_state_target)
                  == REG_SYS_STATE_TARGET_OFFSET, "sys_state_target 절대주소 어긋남");
static_assert(offsetof(REGISTER_t, cmd_mot) == REG_CMD_MOT_S_OFFSET, "CMD_MOT 시작 어긋남");
static_assert(offsetof(REGISTER_t, cmd_mot) + REG_CMD_MOT_SIZE - 1 == REG_CMD_MOT_E_OFFSET,
              "CMD_MOT 끝 어긋남");

}  // namespace orin_bridge::dpc

#endif  // ORIN_FIRMWARE_BRIDGE__RD_REGISTER_DPC_HPP_
