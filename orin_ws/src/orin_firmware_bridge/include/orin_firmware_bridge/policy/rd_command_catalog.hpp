#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMMAND_CATALOG_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMMAND_CATALOG_HPP_

// 커맨드 카탈로그 — B6 "커맨드 슬롯은 **의미 단위 명령**" (redesign/00 B6, 03 §7, 07 §2)
//
// ## 왜 이것이 필요했나
//
// 종전 `CommandSet.srv` 는 `start_addr`/`data_len`/`data[]` 를 그대로 받았다. 03 §7.2 는
// **(c) 혼합** — 의미 단위가 기본이고 raw 는 `manual` 전용 — 을 채택했는데, 코드에는 그
// 절반만 들어갔다: 구 raw srv 위에 *"raw 는 manual 전용"* 게이트만 얹혀 있었다.
// (그 게이트는 09 §5.4 ② 에서 **해제됐다** — 아래 raw 항목 참조. 의미 단위 명령이라는
//  B6 의 본체는 그대로 살아 있고, 없어진 것은 "모드로 잠그는" 부분뿐이다.)
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
#include "orin_firmware_bridge/core/rd_register_dpc.hpp"

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

// ── DPC 구간 (09 §6) ──────────────────────────────────────────────────────
//
// 전 범위에서 **예약 구간을 뺀다** — ECU 의 `kAll` 과 같은 이유다. 예약은 표시할 것이
// 없고, 빼면 한 트랜잭션에 들어간다.
//   RSVD0 16:30 · RSVD1 111:9 · RSVD2 143:32 · RSVD3 207:49 제외
constexpr ReadPreset kDpcAll = {
    "dpc_all",
    {{dpc::REG_DEFINE_OFFSET, dpc::REG_DEFINE_SIZE},                            // 0:16
     {dpc::REG_SYS_OFFSET,                                                      // 46:65
      static_cast<uint16_t>(dpc::REG_MOTOR_DATA_OFFSET + dpc::REG_MOTOR_DATA_SIZE
                            - dpc::REG_SYS_OFFSET)},
     {dpc::REG_CMD_DPCA_OFFSET,                                                 // 120:23
      static_cast<uint16_t>(dpc::REG_CMD_MOT_OFFSET + dpc::REG_CMD_MOT_SIZE
                            - dpc::REG_CMD_DPCA_OFFSET)},
     {dpc::REG_DIAG_OFFSET, dpc::REG_DIAG_SIZE}},                               // 175:32
    4
};
static_assert(kDpcAll.RespPayload() == 136, "dpc_all = 16 + 65 + 23 + 32");
static_assert(kDpcAll.RespPayload() <= kMaxRespPayload, "dpc_all 이 한 트랜잭션에 안 들어간다");
static_assert(!kDpcAll.Covers(dpc::REG_RSVD0_OFFSET, 1) &&
              !kDpcAll.Covers(dpc::REG_RSVD1_OFFSET, 1) &&
              !kDpcAll.Covers(dpc::REG_RSVD2_OFFSET, 1) &&
              !kDpcAll.Covers(dpc::REG_RSVD3_OFFSET, 1),
              "dpc_all 에 예약 구간이 들어왔다 — 응답이 조용히 커진다");
// TAB3 이 초기 버튼 세팅에 쓰는 값들이 다 들어 있는가 (09 §5.3 ③).
static_assert(kDpcAll.Covers(dpc::REG_SYS_STATE_OFFSET, 1), "dpc_all 이 sys_state 를 놓쳤다");
static_assert(kDpcAll.Covers(dpc::REG_CMD_DPCB_OFFSET, dpc::REG_CMD_DPCB_SIZE),
              "dpc_all 이 CMD/DPCB(122~127) 를 놓쳤다 — TAB3 버튼 초기값이 없어진다");

}  // namespace spans

// ── 명령 종류 ───────────────────────────────────────────────────────────────
enum class CmdKind : uint8_t {
    READ,        // 의미 단위 READ — 아무것도 바꾸지 않는다
    WRITE1,      // 1바이트 의미 단위 WRITE
    REBOOT,
    RAW_READ,
    RAW_WRITE,
};

// **어느 보드에 보내는 명령인가** (09 §6). 0 = 아무 보드나 (raw 전용).
//
// 종전에는 이 축이 없어서 `rd_command.cpp` 가 *"READ 계열은 ECU 전용"* 을 하드코딩으로
// 거부했다. 구간이 ECU 레지스터 맵 기준이라 맞는 판단이었지만, DPC 구간이 확정된 지금은
// **표가 대상을 들고 있는 편이 정직하다** — 표에 적힌 대상이 곧 계약이 된다.
constexpr uint8_t kTargetAny = 0;

struct CmdDef {
    uint8_t           cmd;
    const char*       name;
    CmdKind           kind;
    uint8_t           target;        // TARGET::ECU / DPC / PCU / kTargetAny
    const ReadPreset* read;          // READ 계열
    uint16_t          waddr;         // WRITE1 계열
    // ⚠ **지금은 아무 명령도 true 가 아니다** (09 §5.4 ②, 2026-08-06). 필드는 남긴다 —
    //    "모드로 막는다" 는 축 자체가 없어진 게 아니라 raw 에 안 쓰기로 한 것이다.
    bool              manual_only;
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
// ── DPC 의미 명령 (09 §6) — 03 §7.3 의 미사용 대역에서 잇는다 ──
constexpr uint8_t CMD_DPC_SET_BOOT   = 40;   // 123 microbot booting
constexpr uint8_t CMD_DPC_SET_LIGHT  = 41;   // 124 camera LED
constexpr uint8_t CMD_DPC_SET_SERVO  = 42;   // 125 TOP deploy mount
constexpr uint8_t CMD_DPC_SET_MODE   = 43;   // 126 MANUAL/AUTO
constexpr uint8_t CMD_DPC_SET_SEQ    = 44;   // 127 sys_state_target
constexpr uint8_t CMD_DPC_READ_ALL   = 45;   // 예약 제외 전 구간

constexpr CmdDef kCatalog[] = {
    // READ 계열 — **safe_stop 불필요, 모든 모드 허용.** 읽기는 아무것도 바꾸지 않는다.
    {CMD_READ_SYS,    "read_sys",    CmdKind::READ, TARGET::ECU, &spans::kSys,    0, false, false},
    {CMD_READ_MOTOR,  "read_motor",  CmdKind::READ, TARGET::ECU, &spans::kMotor,  0, false, false},
    {CMD_READ_SENSOR, "read_sensor", CmdKind::READ, TARGET::ECU, &spans::kSensor, 0, false, false},
    {CMD_READ_DIAG,   "read_diag",   CmdKind::READ, TARGET::ECU, &spans::kDiag,   0, false, false},
    {CMD_READ_ALL,    "read_all",    CmdKind::READ, TARGET::ECU, &spans::kAll,    0, false, false},

    // WRITE 계열 — safe_stop 필요 (01 §6.2).
    // ⚠ soft_estop 은 예외다: 값 0(=작동)은 **감속 방향**이라 무조건 허용해야 한다.
    //    그 판정은 값에 달렸으므로 표가 아니라 호출부가 한다 (03 §7.3 주석과 같은 취지).
    {CMD_SET_SOFT_ESTOP, "set_soft_estop", CmdKind::WRITE1, TARGET::ECU, nullptr,
     ecu::REG_SOFT_ESTOP_OFFSET, false, true},
    {CMD_SET_USE_LPF,    "set_use_lpf",    CmdKind::WRITE1, TARGET::ECU, nullptr,
     ecu::REG_USE_LPF_OFFSET,    false, true},

    {CMD_REBOOT, "reboot", CmdKind::REBOOT, kTargetAny, nullptr, 0, false, true},

    // ── DPC 의미 명령 (09 §6, memo_260731 TAB3) ──────────────────────────────
    //
    // **safe_stop 을 요구하지 않는다.** ECU 의 WRITE1 과 다른 판단이다: 그쪽은 모터
    // 명령이라 "달리는 중 설정 변경" 이 위험하지만, 여기는 DPC-B 의 솔레노이드·조명·
    // 서보이고 ECU 주행과 독립이다. safe_stop 을 걸면 주행 중 조명을 못 켜게 되는데
    // 그건 게이트의 목적과 무관한 제약이다.
    //
    // ⚠ **`dpc_set_seq`(127)만 예외로 값 검증이 붙는다** — 호출부에서 한다
    //    (`dpc::IsWritableTarget`). FSM 중간 상태를 밖에서 쓰면 단계를 건너뛰는데,
    //    DPC 펌웨어는 값을 **검증 없이 그대로 대입**하므로 (`rd_map_dpcb.c:264`)
    //    막는 주체가 Orin 밖에 없다.
    {CMD_DPC_SET_BOOT,  "dpc_set_boot",  CmdKind::WRITE1, TARGET::DPC, nullptr,
     dpc::REG_DPCB_BOOT_EN_OFFSET,   false, false},
    {CMD_DPC_SET_LIGHT, "dpc_set_light", CmdKind::WRITE1, TARGET::DPC, nullptr,
     dpc::REG_DPCB_LIGHT_EN_OFFSET,  false, false},
    {CMD_DPC_SET_SERVO, "dpc_set_servo", CmdKind::WRITE1, TARGET::DPC, nullptr,
     dpc::REG_DPCB_SERVO_CMD_OFFSET, false, false},
    {CMD_DPC_SET_MODE,  "dpc_set_mode",  CmdKind::WRITE1, TARGET::DPC, nullptr,
     dpc::REG_MODE_OFFSET,           false, false},
    {CMD_DPC_SET_SEQ,   "dpc_set_seq",   CmdKind::WRITE1, TARGET::DPC, nullptr,
     dpc::REG_SYS_STATE_TARGET_OFFSET, false, false},
    {CMD_DPC_READ_ALL,  "dpc_read_all",  CmdKind::READ,   TARGET::DPC, &spans::kDpcAll,
     0, false, false},

    // ── raw — 주소를 아는 주체를 늘리는 유일한 통로. 대상은 호출자가 정한다 ──
    //
    // ★★ **`manual_only` 를 열었다** (09 §5.4 ②, 2026-08-06 사용자 결정).
    //
    // 03 §7.2 (c) 는 *"raw 는 manual 전용"* 으로 정했고 B6 이 그렇게 구현했다. 그 근거는
    // "주행·실험 모드에서 임의 주소에 쓰면 브리지가 자동으로 쓰는 값과 경쟁해 조용히
    // 덮어쓴다" 였다. **그 위험은 그대로다** — 바뀐 것은 용도다: 웹 TAB4 가 조작자가
    // 주소를 직접 보고 만지는 **Advanced 탭**이 되면서, 모드로 잠그면 탭이 성립하지 않는다.
    //
    // 대신 안전망이 둘 남는다:
    //   ① `needs_safe_stop` (raw_write) — **유지한다.** 달리는 중 임의 주소 쓰기는
    //      모드와 무관하게 막아야 한다. `manual_only` 를 여는 것과 이것을 여는 것은
    //      **다른 결정**이며, 이번에 연 것은 앞의 것뿐이다.
    //   ② target 축 (09 §6) — 남의 보드 주소로 새는 것은 표가 막는다.
    // 그리고 스케줄 층의 상한(control 은 프레임당 양보 1칸 = 최대 5Hz)은 그대로다.
    {CMD_RAW_READ,  "raw_read",  CmdKind::RAW_READ,  kTargetAny, nullptr, 0, false, false},
    {CMD_RAW_WRITE, "raw_write", CmdKind::RAW_WRITE, kTargetAny, nullptr, 0, false, true},
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

inline GateResult Gate(uint8_t cmd, bool is_manual, bool safe_stop, uint8_t arg0,
                       uint8_t target_id = kTargetAny) {
    const CmdDef* d = Find(cmd);
    if (!d) return {false, "알 수 없는 cmd — CommandSet.srv 의 CMD_* 를 쓸 것"};

    // **표에 적힌 대상이 곧 계약이다** (09 §6). `kTargetAny`(raw·reboot)는 아무 보드나 된다.
    // 기본 인자를 kTargetAny 로 둔 것은 대상을 안 넘기는 호출자(유닛 테스트)를 위한
    // 편의이고, 실제 서비스 경로는 항상 넘긴다 (rd_command.cpp).
    if (d->target != kTargetAny && target_id != kTargetAny && target_id != d->target) {
        return {false, "이 명령의 대상 보드가 아니다 — 표가 정한 대상으로만 보낼 수 있다"};
    }

    if (d->manual_only && !is_manual) {
        return {false, "이 명령은 manual 전용이다 — `-p bridge_mode:=manual` 로 재기동할 것"};
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
