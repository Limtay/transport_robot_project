#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMMAND_CATALOG_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMMAND_CATALOG_HPP_

// 커맨드 카탈로그 — B6 "커맨드 슬롯은 **의미 단위 명령**" (redesign/00 B6, 03 §7, 07 §2)
//
// ## 왜 이것이 필요했나
//
// 종전 `CommandSet.srv` 는 `start_addr`/`data_len`/`data[]` 를 그대로 받았다. 03 §7.2 는
// **(c) 혼합** — 의미 단위가 기본이고 raw 는 `manual` 전용 — 을 채택했는데, 코드에는 그
// 절반만 들어갔다: 구 raw srv 위에 *"raw 는 manual 전용"* 게이트만 얹혀 있었다.
//
// 그 결과 **control 에서는 레지스터를 읽을 방법이 아예 없었다.** 스케줄 층은 07 §3.2 의
// 양보 tick 으로 뚫렸는데 정책 층이 전부 막고 있었던 것이다. 읽기는 아무것도 바꾸지
// 않으므로 막을 이유가 없었고, 막힌 것은 "raw 주소" 라는 표현 방식 때문이었다.
//
// → **주소를 아는 주체를 브리지 하나로 좁힌다.** 호출자는 "무엇을" 만 말하고
//    (`CMD_READ_MOTOR`), 그것이 어느 구간인지는 이 표가 안다.
//
// ## 03 §7.3 과 다른 점 — `CMD_SET_MOTOR_MASK` 등을 두지 않았다
//
// 03 §7.3 의 목록에는 `CMD_SET_MOTOR_MASK` / `CMD_SET_MODE` / `CMD_SET_AUTO_MODE` /
// `CMD_SET_ORIGIN` 도 있다. 그런데 그 문서가 쓰인 뒤 **`ControlConfig` 가 같은 네 가지를
// 갖게 됐다** (`OP_SET_ACTIVE_MOTORS`·`OP_SET_MODE`·`OP_SET_AUTO_MODE`·`OP_SET_ORIGIN`),
// 그쪽에는 in-span/out-of-span 처리와 IDLE 게이트, write 범위 전환 시 shadow 소독까지
// 붙어 있다.
//
// 여기에 같은 이름을 또 만들면 **auto_mode 를 바꾸는 길이 둘이 되고, 게이트가 다르다.**
// 슬롯 경로는 그 소독을 하지 않으므로 조용히 더 약한 길이 된다. 그래서 넣지 않고
// `ControlConfig` 로 안내한다 (`rd_carrier_api.cpp` 의 거부 메시지). 표에 없는 것이
// 곧 "여기 없다" 는 뜻이 되게 두는 편이, 있는데 거부되는 것보다 정직하다.
//
// `CMD_SET_SOFT_ESTOP`/`CMD_SET_USE_LPF` 는 `ControlConfig` 에 대응이 없어 여기 남는다.

#include <cstdint>

#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_read_preset.hpp"

namespace orin_bridge {
namespace cmdcat {

// ── 읽기 구간 — 의미 단위 이름이 곧 구간이다 ────────────────────────────────
namespace spans {

// SYS 전체 {16,17}. `hw_error`·`degraded_cnt`·`lc`/`hs` 가 여기 있다.
constexpr ReadPreset kSys = {"sys", {{ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE}}, 1};

// 모터 데이터 {88,40}.
constexpr ReadPreset kMotor = {"motor", {{ecu::REG_MOTOR_DATA_OFFSET,
                                          ecu::REG_MOTOR_DATA_SIZE}}, 1};

// 센서 {42,44} — 로드셀(42:6) + IMU(48:22) + 엔코더(70:16) 가 **연속**이라 한 구간이다.
constexpr ReadPreset kSensor = {
    "sensor",
    {{ecu::REG_LOADCELL_OFFSET,
      static_cast<uint16_t>(ecu::REG_LOADCELL_SIZE + ecu::REG_IMU_SIZE + ecu::REG_ENCODER_SIZE)}},
    1};
static_assert(kSensor.spans[0].len == 44, "센서 구간은 42:44 (로드셀+IMU+엔코더)");

// 진단 = DEFINE+SYS(0:33) + DIAG(224:32).
constexpr ReadPreset kDiag = {
    "diag",
    {{ecu::REG_DEFINE_OFFSET,
      static_cast<uint16_t>(ecu::REG_DEFINE_SIZE + ecu::REG_SYS_SIZE)},
     {ecu::REG_DIAG_OFFSET, ecu::REG_DIAG_SIZE}},
    2};

// 전 범위 — **예약 구간은 뺀다.** 256B 를 통째로 읽으면 응답이 err(1)+256 = 257B 로
// `MAX_DATA_LEN`(248)을 넘어 한 트랜잭션에 안 들어간다. 그런데 RSVD0(33:9)·RSVD1(193:31)
// 은 어차피 표시할 것이 없으므로, 빼면 **216B 로 한 번에 들어간다.**
// 03 §7.3 의 `{0,256}` "버퍼 확장 후 1 트랜잭션" 은 이렇게 성립한다.
constexpr ReadPreset kAll = {
    "all",
    {{ecu::REG_DEFINE_OFFSET,
      static_cast<uint16_t>(ecu::REG_DEFINE_SIZE + ecu::REG_SYS_SIZE)},          // 0:33
     {ecu::REG_LOADCELL_OFFSET,
      static_cast<uint16_t>(ecu::REG_RSVD1_OFFSET - ecu::REG_LOADCELL_OFFSET)},  // 42:151
     {ecu::REG_DIAG_OFFSET, ecu::REG_DIAG_SIZE}},                                // 224:32
    3};
static_assert(kAll.RespPayload() == 216, "all = 33 + 151 + 32");
static_assert(kAll.RespPayload() <= kMaxRespPayload,
              "all 이 한 트랜잭션에 안 들어간다 — 예약 구간을 뺀 이유가 이것이다");
// 예약 구간이 정말 빠졌는가. 여기가 어긋나면 응답 길이가 조용히 늘어난다.
static_assert(!kAll.Covers(ecu::REG_RSVD0_OFFSET, 1) && !kAll.Covers(ecu::REG_RSVD1_OFFSET, 1),
              "all 에 예약 구간이 들어왔다");

}  // namespace spans

// ── 명령 종류 ───────────────────────────────────────────────────────────────
enum class CmdKind : uint8_t {
    READ,        // 의미 단위 READ — 아무것도 바꾸지 않는다
    WRITE1,      // 1바이트 의미 단위 WRITE
    REBOOT,
    RAW_READ,
    RAW_WRITE,
};

struct CmdDef {
    uint8_t           cmd;
    const char*       name;
    CmdKind           kind;
    const ReadPreset* read;          // READ 계열
    uint16_t          waddr;         // WRITE1 계열
    bool              manual_only;   // raw 만 true (B6)
    bool              needs_safe_stop;
};

// ⚠ 값은 **03 §7.3 의 번호를 그대로 쓴다.** 다시 매기면 문서와 코드가 갈라진다.
constexpr uint8_t CMD_READ_SYS       = 0;
constexpr uint8_t CMD_READ_MOTOR     = 1;
constexpr uint8_t CMD_READ_SENSOR    = 2;
constexpr uint8_t CMD_READ_DIAG      = 3;
constexpr uint8_t CMD_READ_ALL       = 4;
constexpr uint8_t CMD_SET_SOFT_ESTOP = 13;
constexpr uint8_t CMD_SET_USE_LPF    = 14;
constexpr uint8_t CMD_REBOOT         = 20;
constexpr uint8_t CMD_RAW_READ       = 30;
constexpr uint8_t CMD_RAW_WRITE      = 31;

constexpr CmdDef kCatalog[] = {
    // READ 계열 — **safe_stop 불필요, 모든 모드 허용.** 읽기는 아무것도 바꾸지 않는다.
    {CMD_READ_SYS,    "read_sys",    CmdKind::READ, &spans::kSys,    0, false, false},
    {CMD_READ_MOTOR,  "read_motor",  CmdKind::READ, &spans::kMotor,  0, false, false},
    {CMD_READ_SENSOR, "read_sensor", CmdKind::READ, &spans::kSensor, 0, false, false},
    {CMD_READ_DIAG,   "read_diag",   CmdKind::READ, &spans::kDiag,   0, false, false},
    {CMD_READ_ALL,    "read_all",    CmdKind::READ, &spans::kAll,    0, false, false},

    // WRITE 계열 — safe_stop 필요 (01 §6.2).
    // ⚠ soft_estop 은 예외다: 값 0(=작동)은 **감속 방향**이라 무조건 허용해야 한다.
    //    그 판정은 값에 달렸으므로 표가 아니라 호출부가 한다 (03 §7.3 주석과 같은 취지).
    {CMD_SET_SOFT_ESTOP, "set_soft_estop", CmdKind::WRITE1, nullptr,
     ecu::REG_SOFT_ESTOP_OFFSET, false, true},
    {CMD_SET_USE_LPF,    "set_use_lpf",    CmdKind::WRITE1, nullptr,
     ecu::REG_USE_LPF_OFFSET,    false, true},

    {CMD_REBOOT, "reboot", CmdKind::REBOOT, nullptr, 0, false, true},

    // raw — **manual 전용** (B6). 주소를 아는 주체를 늘리는 유일한 통로다.
    {CMD_RAW_READ,  "raw_read",  CmdKind::RAW_READ,  nullptr, 0, true, false},
    {CMD_RAW_WRITE, "raw_write", CmdKind::RAW_WRITE, nullptr, 0, true, true},
};
constexpr uint8_t kCatalogCount = sizeof(kCatalog) / sizeof(kCatalog[0]);

inline const CmdDef* Find(uint8_t cmd) {
    for (uint8_t i = 0; i < kCatalogCount; i++)
        if (kCatalog[i].cmd == cmd) return &kCatalog[i];
    return nullptr;
}

inline const char* CmdName(uint8_t cmd) {
    const CmdDef* d = Find(cmd);
    return d ? d->name : "?";
}

// ── 게이트 — "지금 이 명령을 받아도 되는가" ────────────────────────────────
//
// **순수 함수로 둔다.** rclcpp 도 FSM 도 모르므로 조합을 전부 유닛 테스트로 고정할 수 있다.
// 그리고 이것이 **서비스 쪽 게이트**다 — 클라이언트에만 있는 가드는 가드가 아니다.
struct GateResult {
    bool        ok;
    const char* why;   // ok 면 nullptr
};

inline GateResult Gate(uint8_t cmd, bool is_manual, bool safe_stop, uint8_t arg0) {
    const CmdDef* d = Find(cmd);
    if (!d) return {false, "알 수 없는 cmd — CommandSet.srv 의 CMD_* 를 쓸 것"};

    if (d->manual_only && !is_manual) {
        return {false, "raw 주소 명령은 manual 전용이다 (B6). "
                       "의미 단위 명령(CMD_READ_* 등)을 쓰거나 `-p bridge_mode:=manual` 로 재기동할 것"};
    }
    if (d->needs_safe_stop && !safe_stop) {
        // ⚠ soft ESTOP **작동**(0)은 감속 방향이라 무조건 허용한다. 멈추라는 명령을
        //    "안 멈춰 있어서" 거부하면 게이트의 목적과 정반대가 된다 (01 §6.2, arm off 와 같은 취지).
        const bool decelerating =
            (cmd == CMD_SET_SOFT_ESTOP && arg0 == ecu::SOFT_ESTOP_ACTIVE);
        if (!decelerating) return {false, "정지 상태가 아니다 (safe_stop 필요 — 01 §6.2)"};
    }
    return {true, nullptr};
}

// 카탈로그의 모든 READ 가 예산 안에 드는가 — 구간을 넓힐 때 컴파일이 먼저 막는다.
static_assert(spans::kSys.RespPayload()    <= kMaxRespPayload, "read_sys 예산 초과");
static_assert(spans::kMotor.RespPayload()  <= kMaxRespPayload, "read_motor 예산 초과");
static_assert(spans::kSensor.RespPayload() <= kMaxRespPayload, "read_sensor 예산 초과");
static_assert(spans::kDiag.RespPayload()   <= kMaxRespPayload, "read_diag 예산 초과");

}  // namespace cmdcat
}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_COMMAND_CATALOG_HPP_
