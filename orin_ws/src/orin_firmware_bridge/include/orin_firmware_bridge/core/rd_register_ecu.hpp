#ifndef ORIN_FIRMWARE_BRIDGE__RD_REGISTER_ECU_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_REGISTER_ECU_HPP_

// stm_ws/ECU_V3/Core/Inc/rd_register_ecu.h 의 C++ 래핑
// 구조체 레이아웃·오프셋·크기는 STM32 측과 100% 동일해야 함 (__attribute__((packed)) 필수)
//
// ===== Timestamp / delta_tick 체계 (memo_260716.md, 2026-07-16 재배치) =====
//   시간축: STM TIM5 free-run 32bit @10kHz — realtime_tick = 카운터 × 0.1ms.
//   delta_tick(uint8) = realtime_tick(발행시점) − 취득시점 tick [×0.1ms], 0xFF = stale.
//   취득시각 복원: acquire_tick = realtime_tick − delta_tick (같은 패킷 내 정합 보장).

#include <cstring>

#include "orin_firmware_bridge/core/rd_common.hpp"

namespace orin_bridge::ecu {

/* ===== Region offset / size (bytes) ===== */
constexpr uint16_t REG_TOTAL_SIZE         = 346;  // 2026-11-05 POS_FB 추가, 2026-11-08 theta3 u32 로 256~345 — 0~255 주소 불변

constexpr uint16_t REG_DEFINE_OFFSET      =   0;
constexpr uint16_t REG_DEFINE_SIZE        =  16;

constexpr uint16_t REG_SYS_OFFSET         =  16;
constexpr uint16_t REG_SYS_SIZE           =  17;  // 2026-07-27 rs485_proc_delta(32) 흡수

constexpr uint16_t REG_RSVD0_OFFSET       =  33;
constexpr uint16_t REG_RSVD0_SIZE         =   9;

constexpr uint16_t REG_LOADCELL_OFFSET    =  42;
constexpr uint16_t REG_LOADCELL_SIZE      =   6;

constexpr uint16_t REG_IMU_OFFSET         =  48;
constexpr uint16_t REG_IMU_SIZE           =  22;

constexpr uint16_t REG_ENCODER_OFFSET     =  70;
constexpr uint16_t REG_ENCODER_SIZE       =  16;

constexpr uint16_t REG_UART2_OFFSET       =  86;
constexpr uint16_t REG_UART2_SIZE         =   1;

constexpr uint16_t REG_SENSOR_RC_OFFSET   =  87;
constexpr uint16_t REG_SENSOR_RC_SIZE     =   1;

constexpr uint16_t REG_MOTOR_DATA_OFFSET  =  88;
constexpr uint16_t REG_MOTOR_DATA_SIZE    =  40;

constexpr uint16_t REG_CMD_MOTOR_OFFSET   = 128;
constexpr uint16_t REG_CMD_MOTOR_SIZE     =  52;

constexpr uint16_t REG_CMD_SYSTEM_OFFSET  = 180;
constexpr uint16_t REG_CMD_SYSTEM_SIZE    =  14;  // motor_mask(192), lin_zero_req(193, 2026-11-05)

constexpr uint16_t REG_RSVD1_OFFSET       = 194;
constexpr uint16_t REG_RSVD1_SIZE         =  30;

constexpr uint16_t REG_DIAG_OFFSET        = 224;
constexpr uint16_t REG_DIAG_SIZE          =  32;

constexpr uint16_t REG_POS_FB_OFFSET      = 256;  // INST_POS 고정 응답 (redesign/11 §2)
constexpr uint16_t REG_POS_FB_SIZE        =  90;

/* ===== sys_write_mode 값 ===== */
constexpr uint8_t SYS_WRITE_LOCK   = 0;
constexpr uint8_t SYS_WRITE_UNLOCK = 1;

/* ===== ctr_mode (CMD_MOTOR_t, addr 128~131) — AK_Control_Mode_t ===== */
constexpr uint8_t CTR_MODE_ESTOP    = 0;
constexpr uint8_t CTR_MODE_CURRENT  = 1;   // AUTO 에서 이 값이면 STM 이 kinematics 대신 직접 전류 소비 (P2)
constexpr uint8_t CTR_MODE_VELOCITY = 3;   // default
constexpr uint8_t CTR_MODE_POSITION = 4;
constexpr uint8_t CTR_MODE_SET_ORIGIN = 5;  // 엣지 명령 — 펄스로만 보낸다 (01 §6.3 C-1)

/* ===== soft_estop (addr 189) — Orin 용 소프트 ESTOP ===== */
constexpr uint16_t REG_SOFT_ESTOP_OFFSET = 189;
constexpr uint8_t  SOFT_ESTOP_ACTIVE     = 0;  // ESTOP 작동: AUTO 모드에서 CAN_AK_ESTOP 소프트 제동
constexpr uint8_t  SOFT_ESTOP_RELEASE    = 1;  // 해제 (default)

/* ===== auto_mode (addr 188) — AUTO 경로 선택. write 범위가 여기서 파생된다 (spec §2.6) =====
 * 0=KINEMATIC: ECU 가 100Hz 로 ctr_mode 를 VELOCITY 로 덮어씀 → bridge write 와 경쟁. 제어 금지.
 * 1=CURRENT  : ECU 가 ctr_mode 를 CURRENT 로 강제(자가치유) → bridge 는 cmd_current(164:16)만 쓰면 된다.
 * 2=DIRECT   : ECU 미개입(무가공 통과) → bridge 가 ctr_mode 까지 소유(128:52).
 * 3=CONTROL  : ECU 미구현(motor off). 금지. */
constexpr uint16_t REG_AUTO_MODE_OFFSET = 188;
constexpr uint8_t  AUTO_MODE_KINEMATIC  = 0;
constexpr uint8_t  AUTO_MODE_CURRENT    = 1;
constexpr uint8_t  AUTO_MODE_DIRECT     = 2;
constexpr uint8_t  AUTO_MODE_CONTROL    = 3;
/* 2026-07-27 신규 (redesign/01 §3.2) — CURRENT 와 대칭. ECU 가 ctr_mode 를 강제 자가치유한다.
 * 2026-07-28 실기 확인: auto_mode=4 -> ctr_mode 3(VELOCITY), =5 -> 4(POSITION). */
constexpr uint8_t  AUTO_MODE_VELOCITY   = 4;
constexpr uint8_t  AUTO_MODE_POSITION   = 5;

/* ===== auto_mode 파생 write 범위 (redesign/01 §3, §6.1.2 표) =====
 * CURRENT 164:16 / VELOCITY 148:16 / POSITION 132:16 / DIRECT 128:52 (ctr_mode 포함 전체) */
constexpr uint16_t REG_CMD_POSITION_OFFSET = 132;
constexpr uint16_t REG_CMD_VELOCITY_OFFSET = 148;
constexpr uint16_t REG_CMD_CURRENT_OFFSET  = 164;
constexpr uint16_t REG_CMD_VALUE_SIZE      = 16;   // float x 4

/* auto_mode 의 사람이 읽는 이름과 write 범위 문자열.
 *
 * **왜 함수로 두는가**: VELOCITY(4)/POSITION(5) 가 2026-07-28 에 추가됐을 때 실제 셀렉터
 * (rd_schedule.cpp 의 4-way switch)는 갱신됐지만, 여섯 군데에 흩어져 있던 표시용
 * 삼항식 `am == DIRECT ? "DIRECT" : "CURRENT"` 는 그대로 남았다. 그래서 실기에서
 * `config auto_mode 5` 가 성공해 놓고 **"auto_mode=1(CURRENT), write 164:16"** 이라고
 * 답했다 — 동작은 옳고 보고만 틀린, 조작자가 알아챌 방법이 없는 상태다.
 * 이름을 한 곳에서만 만들면 다음에 모드가 늘어도 같은 방식으로 갈라지지 않는다. */
inline const char* AutoModeName(uint8_t am) {
    switch (am) {
        case AUTO_MODE_KINEMATIC: return "KINEMATIC";
        case AUTO_MODE_CURRENT:   return "CURRENT";
        case AUTO_MODE_DIRECT:    return "DIRECT";
        case AUTO_MODE_CONTROL:   return "CONTROL";
        case AUTO_MODE_VELOCITY:  return "VELOCITY";
        case AUTO_MODE_POSITION:  return "POSITION";
        // 0xFF = 브리지 쪽 표식 `AUTO_MODE_NONE` (rd_config.hpp — 이 헤더보다 뒤에 정의된다).
        // ECU 레지스터 값이 아니라 "아무것도 쓰지 않는다" 는 구성이다 (01 §4.2).
        case 0xFF:                return "NONE";
        default:                  return "?";
    }
}

/* write 범위 "offset:size". 금지 모드(KINEMATIC/CONTROL)는 bridge 가 cmd 를 쓰지 않으므로
 * 범위가 없다 — 0:0 같은 그럴듯한 숫자 대신 "-" 로 둔다 (미판독 표현 규약과 같은 이유:
 * "없음" 과 "0" 을 섞지 않는다). */
inline const char* AutoModeWriteSpan(uint8_t am) {
    switch (am) {
        case AUTO_MODE_CURRENT:   return "164:16";
        case AUTO_MODE_DIRECT:    return "128:52";
        case AUTO_MODE_VELOCITY:  return "148:16";
        case AUTO_MODE_POSITION:  return "132:16";
        default:                  return "-";
    }
}

/* 비-DIRECT auto_mode 에서 **ECU 가 강제하는** ctr_mode (2026-07-28 실기 확인, 06 §4.7).
 *
 * 이 모드들에서 ctr_mode(128~131) 는 bridge 의 write 범위 밖이라 섀도 값이 ECU 로 나가지
 * 않는다. 그래서 섀도를 CURRENT 로 고정해 두면 `status` 가 ECU 실제 상태와 다른 값을
 * 보고한다 — auto_mode=5 인데 ctr_mode=[1,1,1,1](CURRENT) 로 보이는 식이다.
 * 게이팅(AcceptsAutoMode)은 DIRECT 에서만 ctr_mode 를 보므로 판정에는 영향이 없지만,
 * **실험 중 조작자가 보는 값이 틀리는 것** 자체가 문제다.
 * DIRECT 는 bridge 가 ctr_mode 를 소유하므로 강제값이 없다 — 호출자가 섀도를 유지한다. */
inline uint8_t AutoModeForcedCtrMode(uint8_t am) {
    switch (am) {
        case AUTO_MODE_VELOCITY:  return CTR_MODE_VELOCITY;   // 4 -> 3
        case AUTO_MODE_POSITION:  return CTR_MODE_POSITION;   // 5 -> 4
        default:                  return CTR_MODE_CURRENT;    // 1(CURRENT) 및 그 외
    }
}

/* ===== mode (addr 190) — 모드의 단일 진실원천 (GPIO 스위치·Orin write 양쪽이 갱신) ===== */
constexpr uint16_t REG_MODE_OFFSET   = 190;
constexpr uint8_t  MODE_MANUAL       = 0;
constexpr uint8_t  MODE_AUTO         = 1;   // RC 없이 레지스터 write 로 진입 가능 (testbed_spec §3.1)

/* ===== use_lpf (addr 191) — cmd_velocity LPF. CMD_SYSTEM_t 의 필드에 대응 ===== */
constexpr uint16_t REG_USE_LPF_OFFSET = 191;

/* ===== motor_mask (addr 192) — 활성 모터 비트필드 (bit0~3 = M1~M4) ===== */
constexpr uint16_t REG_MOTOR_MASK_OFFSET = 192;

/* ===== sys_state (SYSTEM_STATE_e, STM rd_system.h) — RUNNING 중 ECU 이탈 판정용 ===== */
constexpr uint8_t SYS_STATE_AUTO = 2;

/* ===== lin_zero_req (addr 193) — linear encoder 0 초기화 요청 (redesign/11 §2.2) ===== */
constexpr uint16_t REG_LIN_ZERO_REQ_OFFSET = 193;

/* ===== RLS raw → deg (redesign/11 §2). 3채널 모두 AksIM 18bit (2026-11-08 ch2 Orbis → AksIM) ===== */
constexpr float kRlsAksimToDeg = 360.0f / 262144.0f;
/* rls_ew: 채널당 2bit, b1 = Error. 값은 Error=0 일 때만 유효하다. */
constexpr bool RlsChError(uint8_t rls_ew, int ch) { return ((rls_ew >> (ch * 2 + 1)) & 0x01) != 0; }

/* ===== delta_tick stale sentinel ===== */
constexpr uint8_t DELTA_STALE = 0xFF;   // ≥25.5ms 또는 미갱신

/* ===== 시간 동기·지연 계측 (testbed_spec.md §2.5) ===== */
constexpr uint16_t REG_PROC_DELTA_OFFSET =  32;  // sys.rs485_proc_delta — 2026-07-27 DIAG(228) 에서 이동

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

/* ===== [SYSTEM] addr 16~32 (17 bytes) — 2026-07-27 재배치 (redesign/01 §9.1) =====
 *  ① hw_error/hw_fatal/hw_reset 을 심각도 순으로 재배치 (구: reset,fatal,error)
 *     → hw_reset 이 26 으로 내려와 control 읽기 세그 {26,7} 이 정확히 7B 로 맞는다
 *  ② rs485_proc_delta 를 DIAG(구 228) 에서 addr 32 로 이동 → realtime_tick 뒤에 연속
 *  LOADCELL(42) 이하 주소는 불변이므로 기존 bag·분석의 오프셋 가정이 깨지지 않는다.
 *  ⚠ STM `Core/Inc/rd_register_ecu.h` 와 **바이트 단위로 동일**해야 한다. */
typedef struct __attribute__((packed)) {
    /* addr 16 */ uint8_t  degraded_cnt[8]; // 통신 오염도 [%]: idx0=uart1/1=uart2/2=uart6/3=can1/4=i2c1/5=로드셀 예약/6=uart4(RLS)/7=RSVD
    /* addr 24 */ uint8_t  hw_error;        // bitfield: 현재 활성 에러          — 구 addr 26
    /* addr 25 */ uint8_t  hw_fatal;        // bitfield: 재초기화 필요 치명 에러
    /* addr 26 */ uint8_t  hw_reset;        // bitfield: HW_BIT_* 세트 시 소프트 리셋 — 구 addr 24
    /* addr 27 */ uint8_t  sys_state;       // SYSTEM_STATE_e: 0=INIT/1=MANUAL/2=AUTO/3=ESTOP_SW/4=ESTOP_HW/5=FAULT
    /* addr 28 */ uint32_t realtime_tick;   // ×0.1[ms] TIM5 10kHz free-run — 발행 시점 tick (delta 기준값)
    /* addr 32 */ uint8_t  rs485_proc_delta;// ×0.1[ms] 직전 트랜잭션 publish→응답TX 처리시간 (spec §2.5, 0xFF=stale)
} DATA_SYSTEM_t;

/* ===== [LOADCELL] addr 42~47 (6 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 42 */ uint16_t avg[2];              // 로드셀 raw 트림평균 ch0~1 (0~4095)
    /* addr 46 */ uint8_t  delta_tick;          // ×0.1[ms] 취득 지연 (2ch 동시취득 → 대표 1개). 0xFF=stale
    /* addr 47 */ orin_bridge::STATE_t state;   // 로드셀 종합 상태
} DATA_LOADCELL_t;

/* ===== [SENSOR/IMU] addr 48~69 (22 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 48 */ int16_t quat_z;   // ×0.0001
    /* addr 50 */ int16_t quat_y;
    /* addr 52 */ int16_t quat_x;
    /* addr 54 */ int16_t quat_w;
    /* addr 56 */ int16_t gyro_x;   // ×0.1 [deg/s]
    /* addr 58 */ int16_t gyro_y;
    /* addr 60 */ int16_t gyro_z;
    /* addr 62 */ int16_t acc_x;    // ×0.001 [g]
    /* addr 64 */ int16_t acc_y;
    /* addr 66 */ int16_t acc_z;
    /* addr 68 */ uint8_t delta_tick;           // ×0.1[ms] 취득 지연 (프레임 대표 1개). 0xFF=stale
    /* addr 69 */ orin_bridge::STATE_t state;
} DATA_IMU_t;

/* ===== [SENSOR/ENCODER] addr 70~85 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 70 */ uint16_t encoder[5];          // AS5600 ch0~4, 12bit raw (0~4095)
    /* addr 80 */ uint8_t  delta_tick[5];       // ×0.1[ms] 채널별 취득 지연 (I2C MUX 순차). 0xFF=stale
    /* addr 85 */ orin_bridge::STATE_t  state;  // 5 enc + MUX 중 worst
} DATA_ENCODER_t;

/* ===== [UART2] addr 86 (1 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 86 */ orin_bridge::STATE_t state; // RS485 (UART2) 통신 상태
} DATA_UART2_t;

/* ===== [SENSOR/RC] addr 87 (1 byte) ===== */
typedef struct __attribute__((packed)) {
    /* addr 87 */ orin_bridge::STATE_t state; // UART1 (RC 수신기) 통신 상태
} DATA_RC_t;

/* ===== [MOTOR/data] addr 88~127 (40 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr  88 */ int16_t  position[4];       // ×0.1 [deg]
    /* addr  96 */ int16_t  velocity[4];       // ×10.0 [RPM]
    /* addr 104 */ int16_t  current[4];        // ×0.01 [A]
    /* addr 112 */ int8_t   temp[4];           // ×1 [°C]
    /* addr 116 */ uint8_t  delta_tick[4];     // ×0.1[ms] 모터별 CAN 피드백 수신 지연. 0xFF=stale
    /* addr 120 */ uint8_t  cmd_delta_tick[4]; // ×0.1[ms] 모터별 CAN 명령 송출 지연. 0xFF=stale
    /* addr 124 */ uint16_t error_code;        // 4bit×4 packed (LSB=M1): AK 에러코드
    /* addr 126 */ uint8_t  comm_err;          // 2bit×4 packed (LSB=M1): bit[0]=RX err, bit[1]=TX err
    /* addr 127 */ orin_bridge::STATE_t  state;// 4모터 중 worst 요약
} DATA_MOTOR_t;

/* ===== [MOTOR/cmd] addr 128~179 (52 bytes) — 불변 ===== */
typedef struct __attribute__((packed)) {
    /* addr 128 */ uint8_t ctr_mode[4];      // AK_Control_Mode_t (default 3=VELOCITY)
    /* addr 132 */ float   cmd_position[4];  // [deg]
    /* addr 148 */ float   cmd_velocity[4];  // [RPM]
    /* addr 164 */ float   cmd_current[4];   // [A]
} CMD_MOTOR_t;

/* ===== [SYSTEM/cmd] addr 180~193 (14 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 180 */ float   cmd_lin_vel;  // AUTO 모드 선속도 [m/s]
    /* addr 184 */ float   cmd_ang_vel;  // AUTO 모드 각속도 [rad/s]
    /* addr 188 */ uint8_t auto_mode;    // AUTO 경로 (default 0): 0=KINEMATIC / 1=CURRENT / 2=DIRECT / 3=CONTROL(TODO)
    /* addr 189 */ uint8_t soft_estop;   // Orin soft ESTOP: 0=작동 / 1=해제(default)
    /* addr 190 */ uint8_t mode;         // 0=MANUAL / 1=AUTO
    /* addr 191 */ uint8_t use_lpf;      // cmd_velocity LPF (default 1). MANUAL 은 ECU 소유 / AUTO 는 Orin 소유
    /* addr 192 */ uint8_t motor_mask;   // 활성 모터 비트필드 (bit0~3=M1~4, default 0x0F) — 단일 트랙 테스트용
    /* addr 193 */ uint8_t lin_zero_req; // 1 = linear encoder 0 초기화 1회 요청. ECU 가 읽고 곧바로 지운다 (read-back 검증 불가)
} CMD_SYSTEM_t;

/* ===== [DIAG] addr 224~255 (32 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 224 */ uint32_t cmd_write_tick;
    /* addr 228 */ uint32_t origin_jump_rej;   // INST_POS 점프 가드가 cmd 를 거부한 누적 횟수 (구 228 = proc_delta → sys(32) 로 이동)
    /* addr 232 */ uint32_t overspeed_cnt;     // POSITION 과속 제동 래치 누적 횟수
    /* addr 236 */ uint8_t  reserved[20];
} DIAG_t;

/* ===== [POS_FB] addr 256~345 (90 bytes) — INST_POS 고정 응답 (redesign/11 §2) =====
 *  ECU 가 한 CRIT 안에서 채우므로 전 필드가 같은 시점의 스냅샷이다. */
typedef struct __attribute__((packed)) {
    /* addr 256 */ uint32_t realtime_tick;      // ×0.1[ms]
    /* addr 260 */ uint8_t  rs485_proc_delta;   // ×0.1[ms] 직전 트랜잭션 처리시간
    /* addr 261 */ uint8_t  sys_state;          // SYSTEM_STATE_e
    /* addr 262 */ uint8_t  hw_reset;           // bitfield: HW_BIT_*
    /* addr 263 */ uint8_t  origin_flag;        // bit0~3 = ORIGIN_PEND(M1~4) / bit4~7 = FRAME_LOST(M1~4)
    /* addr 264 */ uint8_t  degraded_cnt[8];    // sys.degraded_cnt 와 같은 값
    /* addr 272 */ int16_t  mtr_position[4];    // ×0.1 [deg] 모터 현재 좌표 (원점 이후)
    /* addr 280 */ int16_t  mtr_velocity[4];    // motor_data.velocity 와 같은 AK raw
    /* addr 288 */ int16_t  mtr_current[4];     // ×0.01 [A]
    /* addr 296 */ int16_t  pre_origin_pose[4]; // ×0.1 [deg] 모터별 마지막 확정 원점 이동량 S
    /* addr 304 */ uint8_t  origin_seq[4];      // 모터별 확정 원점 이벤트 카운터 (부팅 0, mod 256)
    /* addr 308 */ uint16_t loadcell_avg[2];    // raw 0~4095. ch0=지면 총 접선력 / ch1=윗벽 y힘
    /* addr 312 */ orin_bridge::STATE_t loadcell_state;
    /* addr 313 */ orin_bridge::STATE_t motor_state;  // 4모터 worst
    /* addr 314 */ orin_bridge::STATE_t rls_state;    // UART4 채널
    /* addr 315 */ uint8_t  rls_ew;             // 2bit×4 (b1=Error, b0=Warning), LSB=ch0
    /* addr 316 */ uint32_t rls_psi1;           // AksIM raw 18bit
    /* addr 320 */ uint32_t rls_psi2;           // AksIM raw 18bit
    /* addr 324 */ uint32_t rls_theta3;         // AksIM raw 18bit (2026-11-08 Orbis u16 → AksIM u32)
    /* addr 328 */ uint8_t  lin_zero_result;    // 0=없음/1=성공/2=보드 실패/3=무응답/4=형식 불일치/5=잘못된 요청
    /* addr 329 */ uint8_t  lin_zero_seq;       // 'i' 완료 카운터 (부팅 0, mod 256)
    /* addr 330 */ int32_t  lin_x;              // [count] 10 um. rls_ew ch3 Error=0 일 때만 유효
    /* addr 334 */ int32_t  lin_y;              // [count] 20 um. 〃
    /* addr 338 */ uint8_t  odiag_motor;        // 마지막 원점 이벤트 모터 idx (0xFF=없음)
    /* addr 339 */ uint8_t  odiag_dt;           // ×0.1[ms] 원점 TX − fb_old 취득
    /* addr 340 */ int16_t  odiag_e_track;      // ×0.1 [deg] 원점 tick 의 cmd − fb_old
    /* addr 342 */ int16_t  odiag_residual;     // ×0.1 [deg] 사후 검증 오차
    /* addr 344 */ int16_t  odiag_vel;          // AK raw 원점 시점 속도
} PAGE_POS_FB_t;

/* ===== 전체 레지스터 맵 — Total 346 bytes ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ DEFINE_t        reg_df;
    /* addr  16 */ DATA_SYSTEM_t   sys;           // 17 bytes (16~32)
    /* addr  33 */ uint8_t         reserved0[9];  //  9 bytes (33~41)
    /* addr  42 */ DATA_LOADCELL_t loadcell;
    /* addr  48 */ DATA_IMU_t      imu;
    /* addr  70 */ DATA_ENCODER_t  encoder;
    /* addr  86 */ DATA_UART2_t    uart2;
    /* addr  87 */ DATA_RC_t       rc;
    /* addr  88 */ DATA_MOTOR_t    motor_data;
    /* addr 128 */ CMD_MOTOR_t     cmd_motor;
    /* addr 180 */ CMD_SYSTEM_t    cmd_system;
    /* addr 194 */ uint8_t         reserved1[30];
    /* addr 224 */ DIAG_t          diag;
    /* addr 256 */ PAGE_POS_FB_t   pos_fb;
} REGISTER_t;

static_assert(sizeof(DEFINE_t)       == REG_DEFINE_SIZE,     "DEFINE_t size mismatch");
static_assert(sizeof(DATA_SYSTEM_t)  == REG_SYS_SIZE,        "DATA_SYSTEM_t size mismatch");
static_assert(sizeof(DATA_LOADCELL_t)== REG_LOADCELL_SIZE,   "DATA_LOADCELL_t size mismatch");
static_assert(sizeof(DATA_IMU_t)     == REG_IMU_SIZE,        "DATA_IMU_t size mismatch");
static_assert(sizeof(DATA_ENCODER_t) == REG_ENCODER_SIZE,    "DATA_ENCODER_t size mismatch");
static_assert(sizeof(DATA_UART2_t)   == REG_UART2_SIZE,      "DATA_UART2_t size mismatch");
static_assert(sizeof(DATA_RC_t)      == REG_SENSOR_RC_SIZE,  "DATA_RC_t size mismatch");
static_assert(sizeof(DATA_MOTOR_t)   == REG_MOTOR_DATA_SIZE, "DATA_MOTOR_t size mismatch");
static_assert(sizeof(CMD_MOTOR_t)    == REG_CMD_MOTOR_SIZE,  "CMD_MOTOR_t size mismatch");
static_assert(sizeof(CMD_SYSTEM_t)   == REG_CMD_SYSTEM_SIZE, "CMD_SYSTEM_t size mismatch");
static_assert(sizeof(DIAG_t)         == REG_DIAG_SIZE,       "DIAG_t size mismatch");
static_assert(sizeof(PAGE_POS_FB_t)  == REG_POS_FB_SIZE,     "PAGE_POS_FB_t size mismatch");
static_assert(sizeof(REGISTER_t)     == REG_TOTAL_SIZE,      "REGISTER_t total size mismatch");

/* POS_FB → 기존 섀도 자리로 복사 (redesign/12 §2).
 *
 * **왜 복사하는가**: SafeStop·피드백·시계 동기·IDLE 재시드는 전부 기존 자리를 읽는다.
 * 소비자마다 분기를 두면 한 곳만 빠져도 낡은 값을 쓰므로, 입구 한 곳에서 맞춘다.
 *
 * POS_FB 에 없는 값은 건드리지 않는다 → 미판독 유지: hw_error·hw_fatal, 모터
 * temp·error_code·delta_tick, 로드셀 delta_tick, IMU, AS5600. */
inline void MirrorPosFb(REGISTER_t* r) {
    const PAGE_POS_FB_t f = r->pos_fb;
    r->sys.realtime_tick    = f.realtime_tick;
    r->sys.rs485_proc_delta = f.rs485_proc_delta;
    r->sys.sys_state        = f.sys_state;
    r->sys.hw_reset         = f.hw_reset;
    std::memcpy(r->sys.degraded_cnt, f.degraded_cnt, sizeof(f.degraded_cnt));
    for (int i = 0; i < 4; i++) {
        r->motor_data.position[i] = f.mtr_position[i];
        r->motor_data.velocity[i] = f.mtr_velocity[i];
        r->motor_data.current[i]  = f.mtr_current[i];
    }
    r->motor_data.state = f.motor_state;
    r->loadcell.avg[0]  = f.loadcell_avg[0];
    r->loadcell.avg[1]  = f.loadcell_avg[1];
    r->loadcell.state   = f.loadcell_state;
}

/* MirrorPosFb 가 덮는 주소 구간 — read_age 도장용. 위 함수와 어긋나면 test_golden_wire 가 잡는다. */
struct PosFbMirrorSpan {
    uint16_t addr;
    uint16_t len;
};
constexpr PosFbMirrorSpan kPosFbMirrorSpans[] = {
    {16, 8},    // degraded_cnt
    {26, 7},    // hw_reset, sys_state, realtime_tick, rs485_proc_delta
    {42, 4},    // loadcell.avg
    {47, 1},    // loadcell.state
    {88, 24},   // motor_data.position, velocity, current
    {127, 1},   // motor_data.state
};

} // namespace orin_bridge::ecu

#endif
