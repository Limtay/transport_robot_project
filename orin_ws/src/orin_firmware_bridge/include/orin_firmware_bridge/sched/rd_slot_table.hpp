#ifndef ORIN_FIRMWARE_BRIDGE__RD_SLOT_TABLE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_SLOT_TABLE_HPP_

// 슬롯 테이블 — 스케줄을 코드 분기가 아니라 **데이터**로 (redesign/04 §2.2, 00 C1, 07 §3.1)
//
// 스케줄 스레드는 5ms 마다 깨어나 RS485 트랜잭션을 정확히 1건 한다. 그 1회가 **tick** 이다.
// "이번 tick 에 무엇을 할 것인가" 를 매번 `tick%2`, `odd_idx%2` 같은 산술로 되짚는 대신
// **표에서 찾아본다.**
//
//   tick_in_frame = tick_count % frame.ticks
//   slot          = frame.slots[tick_in_frame]
//
// ## 슬롯이 답하는 것은 넷이다
//
//   ① 누구에게 (`SlotId`)   ② 무엇을 (`SlotInst`)
//   ③ 무엇을 읽고 (`ReadSpan`)  ④ 무엇을 쓰는가 (`WriteSpan`)
//
// ①②를 한 enum 에 섞으면(`ECU_RW`, `ECU_READ`, `DPC`…) 조합이 곱셈으로 불어나고 규칙이
// 바뀔 때 enum 자체를 고쳐야 한다. **직교하는 축으로 나눈다** (04 §2.2.1).
//
// ③④가 슬롯 안에 있는 것이 핵심이다. 프레임이 프리셋 하나를 공유하면 **DPC·PCU 슬롯이
// 무엇을 읽는지 표현할 방법이 없다.** 슬롯이 자기 구간을 들고 있으면 표 한 줄에 대상·명령·
// 구간이 다 보이고, 그 구조가 그대로 커맨드/CLI/웹 명령의 구조가 된다.
//
// ## 왜 constexpr 인가
//
//  ① 런타임에 안 바뀌므로 200Hz 스레드와 레이스가 없다.
//  ② **표 자체를 컴파일 타임에 검산할 수 있다** — wire 예산을 사람이 더해 적지 않는다.
//     04 §2.5 가 지적한 사고("43 을 손으로 옮겨 적어 payload 와 wire 를 섞었다")가
//     바로 그 계산을 사람이 했기 때문에 났다.
//
// ## Q3 재편 완료 (2026-07-29)
//
// project 프레임이 40칸(200ms) → **10칸(50ms)** 이 됐다. ECU 센서 READ(100Hz)와
// cmd_vel WRITE(50Hz)와 sys READ(10Hz) 세 갈래를 **ECU RW×5** 한 갈래로 합친 것이 요점이다.
//
// 종전 주석은 "재편은 이 파일의 숫자만 고치는 일" 이라고 적었는데 **그렇지 않았다.**
// 세 가지가 더 필요했다:
//   ① `SlotInst::RW` 를 굽는 경로 (rd_schedule.cpp 의 표 굽기가 READ/WRITE 만 알았다)
//   ② RW 는 읽기·쓰기가 한 패킷이라 **게이트로 write 만 뺄 수 없다** — manual 을 별도
//      프레임(EcuRd)으로 분리해야 했고, cmd_vel 일시정지용 READ 폴백도 따로 구웠다
//   ③ 양보 마스크가 control 전용으로 하드코딩돼 있어 **활성 프레임 기준**으로 일반화
// 표가 데이터인 것과 표를 바꾸는 게 공짜인 것은 다른 이야기였다.

#include <cstdint>

#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_read_preset.hpp"
#include "orin_firmware_bridge/core/rd_register_dpc.hpp"
#include "orin_firmware_bridge/core/rd_register_pcu.hpp"

namespace orin_bridge {

// ── ① 누구에게 — 트랜잭션 상대 ───────────────────────────────────────────────
enum class SlotId : uint8_t {
    EMPTY = 0,   // 아무에게도 — 이 tick 은 비운다 (커맨드에 양보되는 자리)
    ECU,
    DPC,
    PCU,
    COMMAND,     // 늦은 바인딩 — 대상·명령을 커맨드 슬롯이 런타임에 정한다
};

// ── ② 무엇을 — RS485 명령 종류 ──────────────────────────────────────────────
enum class SlotInst : uint8_t {
    NONE = 0,    // EMPTY / COMMAND 전용 (표가 정하지 않는다)
    READ,
    WRITE,
    RW,          // 한 트랜잭션에 write + read 동시 (01 §3)
};

inline const char* SlotIdName(SlotId s) {
    switch (s) {
        case SlotId::EMPTY:   return "empty";
        case SlotId::ECU:     return "ecu";
        case SlotId::DPC:     return "dpc";
        case SlotId::PCU:     return "pcu";
        case SlotId::COMMAND: return "command";
    }
    return "?";
}

inline const char* SlotInstName(SlotInst s) {
    switch (s) {
        case SlotInst::NONE:  return "none";
        case SlotInst::READ:  return "read";
        case SlotInst::WRITE: return "write";
        case SlotInst::RW:    return "rw";
    }
    return "?";
}

// ── ③ 무엇을 읽는가 ─────────────────────────────────────────────────────────
//
// 구간 목록은 이미 `ReadPreset`(rd_read_preset.hpp)이 표현한다 — 여기서 같은 모양의 타입을
// 또 만들지 않는다. 슬롯은 **미리 만들어 둔 구간 묶음을 가리키기만 한다** (04 §2.4.2
// "런타임에 조립하지 않는다").
enum class ReadSrc : uint8_t {
    NONE = 0,
    FIXED,     // 표가 명시한 구간 묶음
    PRESET,    // ECU 전용 — **런타임에 선택된 읽기 프리셋**을 따른다 (OP_SET_READ_PRESET)
};

struct ReadSpan {
    ReadSrc                src   = ReadSrc::NONE;
    const ReadPreset* fixed = nullptr;   // src == FIXED 일 때만 유효

    constexpr bool Has() const { return src != ReadSrc::NONE; }

    // 예산 검증용 **최악값**. PRESET 이면 고를 수 있는 프리셋 중 최댓값이다 —
    // "지금 고른 것" 이 아니라 "고를 수 있는 전부" 로 잡아야 검증이 성립한다.
    constexpr uint16_t MaxRespPayload() const {
        if (src == ReadSrc::FIXED) return fixed ? fixed->RespPayload() : 0;
        if (src == ReadSrc::PRESET) {
            uint16_t m = 0;
            for (uint8_t i = 0; i < ecu::kPresetCount; i++) {
                const uint16_t v = ecu::kPresets[i]->RespPayload();
                if (v > m) m = v;
            }
            return m;
        }
        return 0;
    }
    constexpr uint8_t MaxSegCount() const {
        if (src == ReadSrc::FIXED) return fixed ? fixed->count : 0;
        if (src == ReadSrc::PRESET) {
            uint8_t m = 0;
            for (uint8_t i = 0; i < ecu::kPresetCount; i++)
                if (ecu::kPresets[i]->count > m) m = ecu::kPresets[i]->count;
            return m;
        }
        return 0;
    }
};

// ── ④ 무엇을 쓰는가 ─────────────────────────────────────────────────────────
enum class WriteSrc : uint8_t {
    NONE = 0,
    FIXED,      // 표가 명시한 고정 구간
    AUTO_MODE,  // ECU 전용 — auto_mode 가 6종 중 런타임에 고른다 (01 §3.3)
};

// auto_mode 파생 write 중 최대 = DIRECT(128:52). 예산 검증의 최악값이다.
constexpr uint16_t kAutoModeMaxWrite = ecu::REG_CMD_MOTOR_SIZE;

struct WriteSpan {
    WriteSrc src  = WriteSrc::NONE;
    uint16_t addr = 0;
    uint16_t len  = 0;      // src == FIXED 일 때만 유효

    constexpr bool Has() const { return src != WriteSrc::NONE; }
    constexpr uint16_t MaxLen() const {
        return src == WriteSrc::AUTO_MODE ? kAutoModeMaxWrite
             : src == WriteSrc::FIXED     ? len
             : 0;
    }
};

struct SlotDef {
    SlotId    id        = SlotId::EMPTY;
    SlotInst  inst      = SlotInst::NONE;
    ReadSpan  read      = {};
    WriteSpan write     = {};
    uint8_t   cmd_index = 0;   // id == COMMAND 일 때 커맨드 슬롯 번호 (0~3)
};

// 쓰기가 막혔을 때 그 tick 을 어떻게 할 것인가 (Q3). **순수 함수로 빼 둔 이유**는
// 실기에서 구분이 안 되기 때문이다: 폴백이 걸려 READ 가 나가든, 게이트가 안 먹어 RW 가
// 나가든 트랜잭션 수는 똑같이 1이라 heartbeat 로는 판별할 수 없다.
enum class SlotAction : uint8_t {
    NORMAL,         // 표대로
    READ_FALLBACK,  // RW 에서 쓰기만 뺀 READ — 읽기까지 잃지 않기 위해
    SKIP,           // 아무것도 안 함
};

constexpr SlotAction ActionFor(const SlotDef& s, bool block_write) {
    if (!block_write || !s.write.Has()) return SlotAction::NORMAL;
    // RW 를 통째로 버리면 **센서 시계열에도 구멍이 난다** — 읽기와 쓰기가 한 패킷이라
    // 쓰기를 막으려고 tick 을 버리는 것은 대가가 너무 크다.
    if (s.inst == SlotInst::RW) return SlotAction::READ_FALLBACK;
    return SlotAction::SKIP;    // 쓰기 전용 슬롯은 읽을 것이 없다
}

// 조합 유효성 — **구조만** 본다 (04 §2.2.4). "ECU 에 WRITE 를 해도 되는가" 같은 정책은
// 여기서 판단하지 않는다. 대상별 거부 규칙을 표에 넣으면 ID×INST 로 나눈 이유가 사라진다.
constexpr bool SlotValid(const SlotDef& s) {
    const bool has_r = s.read.Has();
    const bool has_w = s.write.Has();
    if (s.id == SlotId::EMPTY || s.id == SlotId::COMMAND)
        return s.inst == SlotInst::NONE && !has_r && !has_w;
    switch (s.inst) {
        case SlotInst::READ:  return  has_r && !has_w;
        case SlotInst::WRITE: return !has_r &&  has_w;
        case SlotInst::RW:    return  has_r &&  has_w;
        case SlotInst::NONE:  return false;   // 실보드 슬롯은 명령이 있어야 한다
    }
    return false;
}

// ── 프레임 ──────────────────────────────────────────────────────────────────
// 프레임 = 슬롯을 N개 늘어놓은 **반복 단위**.
constexpr uint8_t kMaxFrameTicks = 40;

struct FrameDef {
    const char* name;
    uint8_t     ticks;                    // 반복 주기 [tick]. 5ms/tick.
    SlotDef     slots[kMaxFrameTicks];

    // **커맨드에 양보 가능한 tick** (04 §2.2, memo_260727 :82). bit t == 1 이면 tick t 를
    // 커맨드 슬롯이 훔쳐 갈 수 있다. 0 이면 그 칸은 절대 대체되지 않는다.
    //
    // 이것이 control 모드에서 사용자 명령(레지스터 READ/WRITE·REBOOT)을 낼 **유일한 경로**다.
    // control 프레임은 전 칸이 ECU RW 라 비울 칸이 없고, 커맨드 전용 슬롯도 0개다.
    // "비울 수 없으면 한 칸을 훔친다" 를 표에 적는다 — 코드 분기가 아니라.
    //
    // ⚠ **언제 훔쳐도 되는가는 여기서 정하지 않는다.** 표는 "이번에 무엇을 할 차례인가" 만
    //    답한다. RUNNING/STREAM 중 금지 같은 판단은 정책층 게이트가 한다 (07 §3.2).
    uint64_t    user_slot_mask = 0;

    constexpr const SlotDef& At(uint64_t tick_count) const {
        return slots[tick_count % ticks];
    }
    constexpr bool UserSlotAt(uint64_t tick_count) const {
        return (user_slot_mask >> (tick_count % ticks)) & 1ULL;
    }
    // 이 프레임에서 해당 슬롯이 프레임당 몇 번 도는가 — 주기 계산·검산용.
    constexpr uint8_t CountOf(SlotId want) const {
        uint8_t n = 0;
        for (uint8_t i = 0; i < ticks; i++) if (slots[i].id == want) n++;
        return n;
    }
    constexpr uint8_t CountOf(SlotId want, SlotInst inst) const {
        uint8_t n = 0;
        for (uint8_t i = 0; i < ticks; i++)
            if (slots[i].id == want && slots[i].inst == inst) n++;
        return n;
    }
    constexpr uint8_t UserSlotCount() const {
        uint8_t n = 0;
        for (uint8_t i = 0; i < ticks; i++) if ((user_slot_mask >> i) & 1ULL) n++;
        return n;
    }
    constexpr bool AllValid() const {
        for (uint8_t i = 0; i < ticks; i++) if (!SlotValid(slots[i])) return false;
        return true;
    }
    // 프레임 안에서 가장 큰 응답 payload — 200Hz 예산 검증의 최악값.
    constexpr uint16_t MaxRespPayload() const {
        uint16_t m = 0;
        for (uint8_t i = 0; i < ticks; i++) {
            const uint16_t v = slots[i].read.MaxRespPayload();
            if (v > m) m = v;
        }
        return m;
    }
};

// ── 표가 가리키는 읽기 구간 (프리셋과 같은 표현, 사용자 선택 대상은 아니다) ──
namespace spans {

// 100Hz 센서 일괄 READ — IMU(48:22) + 엔코더(70:16) + uart2(86:1) + RC(87:1) + 모터(88:40)
// = 48:80. 종전 `task_100hz_` 가 손으로 더해 만들던 값이며, 이제 표가 소유한다.
constexpr ReadPreset kEcuSensor100Hz = {
    "ecu_sensor_100hz",
    {{ecu::REG_IMU_OFFSET,
      static_cast<uint16_t>(ecu::REG_IMU_SIZE + ecu::REG_ENCODER_SIZE + ecu::REG_UART2_SIZE +
                            ecu::REG_SENSOR_RC_SIZE + ecu::REG_MOTOR_DATA_SIZE)}},
    1
};
static_assert(kEcuSensor100Hz.spans[0].len == 80, "100Hz 센서 구간은 80B (48~127)");

// 10Hz 시스템 상태 READ (16:17 — 2026-07-27 proc_delta 흡수)
constexpr ReadPreset kEcuSys10Hz = {
    "ecu_sys_10hz", {{ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE}}, 1
};

// TODO(04 §6): PCU·DPC 레지스터 미확정 — 임시 구간이며 enable 플래그로 차단돼 있다.
constexpr ReadPreset kPcuPower10Hz = {
    "pcu_power_10hz", {{pcu::REG_DATA_POWER_OFFSET, pcu::REG_DATA_POWER_SIZE}}, 1
};
constexpr ReadPreset kDpcSys10Hz = {
    "dpc_sys_10hz", {{dpc::REG_SYS_OFFSET, dpc::REG_SYS_SIZE}}, 1
};

}  // namespace spans

// ── 슬롯 생성 도우미 — 표를 읽을 수 있게 유지한다 ───────────────────────────
constexpr SlotDef EcuRd(const ReadPreset* p) {
    return {SlotId::ECU, SlotInst::READ, {ReadSrc::FIXED, p}, {}, 0};
}
constexpr SlotDef EcuWr(uint16_t addr, uint16_t len) {
    return {SlotId::ECU, SlotInst::WRITE, {}, {WriteSrc::FIXED, addr, len}, 0};
}
// project 100Hz RW (Q3) — 고정 구간을 읽으면서 cmd_vel 을 쓴다. control 의 RW 와 다른 점은
// 읽기가 **런타임 프리셋이 아니라 표가 소유한 고정 구간**이라는 것이다: project 의 읽기
// 구간은 조작자가 바꾸는 대상이 아니다 (프리셋 축은 control 실험용이다 — 04 §2.4.2).
constexpr SlotDef EcuRw(const ReadPreset* p, uint16_t waddr, uint16_t wlen) {
    return {SlotId::ECU, SlotInst::RW, {ReadSrc::FIXED, p}, {WriteSrc::FIXED, waddr, wlen}, 0};
}
// control 200Hz RW — 읽기는 런타임 프리셋, 쓰기는 auto_mode 파생.
constexpr SlotDef EcuRwPreset() {
    return {SlotId::ECU, SlotInst::RW, {ReadSrc::PRESET, nullptr}, {WriteSrc::AUTO_MODE, 0, 0}, 0};
}
// auto_mode: none — 쓰지 않고 같은 구간을 읽기만 한다.
constexpr SlotDef EcuRdPreset() {
    return {SlotId::ECU, SlotInst::READ, {ReadSrc::PRESET, nullptr}, {}, 0};
}
constexpr SlotDef PcuRd(const ReadPreset* p) {
    return {SlotId::PCU, SlotInst::READ, {ReadSrc::FIXED, p}, {}, 0};
}
constexpr SlotDef DpcRd(const ReadPreset* p) {
    return {SlotId::DPC, SlotInst::READ, {ReadSrc::FIXED, p}, {}, 0};
}
constexpr SlotDef Cmd(uint8_t i) {
    return {SlotId::COMMAND, SlotInst::NONE, {}, {}, i};
}

namespace frames {

// ── project (기본) — 10 tick = 50ms 주기 (Q3, 04 §2.4) ──────────────────────
//
// ECU RW×5 (100Hz) + DPC×1 + PCU×1 (각 20Hz) + Command×3 (각 20Hz).
//
// **종전 40칸(200ms)에서 무엇이 달라졌나**
//
// | | 구 (40칸) | 신 (10칸) | 왜 |
// |---|---|---|---|
// | ECU 센서 READ | 100Hz (20칸) | RW 에 흡수 (5칸) | 읽기와 쓰기가 같은 트랜잭션이면 왕복이 반이다 |
// | cmd_vel WRITE | 50Hz (10칸)  | RW 에 흡수 → **100Hz** | 제어 주기가 센서 주기와 어긋나지 않는다 |
// | ECU sys READ | 10Hz (2칸)   | **슬롯 소멸** | 모든 프리셋이 `{16,17}` 로 시작한다 (04 §2.3) — RW 에 딸려 온다 |
// | DPC / PCU    | 각 10Hz (2칸)| 각 **20Hz** (1칸) | 프레임이 짧아진 만큼 그냥 올라간다 |
// | 커맨드 슬롯  | 4개 × 5Hz    | **3개 × 20Hz**    | 개수보다 응답성이다 — 4번째 명령은 다음 프레임(50ms)을 기다린다 |
//
// ECU 트랜잭션 수는 200ms 당 32건(20 READ + 10 WRITE + 2 sys) → 20건(RW)으로 **줄었다.**
// 건당 페이로드는 커졌지만(요청 8B 쓰기 + 응답 81B) 왕복 횟수가 지배적이라 총 점유는 내려간다.
//
// user_slot_mask = tick 4·6·8 — `slots[]` 의 Cmd 자리와 일치시킨다 (04 §2.4.3:
// 용량은 slots[] 가 아니라 마스크가 정한다). project 는 "전용 3칸" 이므로 둘이 같다.
constexpr uint64_t kProjectUserMask = (1ULL << 4) | (1ULL << 6) | (1ULL << 8);

constexpr FrameDef kProject = {
    "project", 10, {
        /* 0 */ DpcRd(&spans::kDpcSys10Hz),
        /* 1 */ EcuRw(&ecu::kPresetProject, ecu::REG_CMD_SYSTEM_OFFSET, 8),
        /* 2 */ PcuRd(&spans::kPcuPower10Hz),
        /* 3 */ EcuRw(&ecu::kPresetProject, ecu::REG_CMD_SYSTEM_OFFSET, 8),
        /* 4 */ Cmd(0),
        /* 5 */ EcuRw(&ecu::kPresetProject, ecu::REG_CMD_SYSTEM_OFFSET, 8),
        /* 6 */ Cmd(1),
        /* 7 */ EcuRw(&ecu::kPresetProject, ecu::REG_CMD_SYSTEM_OFFSET, 8),
        /* 8 */ Cmd(2),
        /* 9 */ EcuRw(&ecu::kPresetProject, ecu::REG_CMD_SYSTEM_OFFSET, 8),
    },
    kProjectUserMask
};

// ── manual — 배치는 project 와 같지만 **ECU 는 READ** 이고 10칸 전부 양보 가능 ──────
//
// 종전에는 `kManual = kProject` 로 두고 "브리지가 자동으로 쓰지 않는다" 를 게이트
// (`cfg_.AutoConfiguresEcu()`)로만 표현했다. 그러면 표를 봐도 manual 이 무엇인지 알 수 없고,
// RW 로 바뀐 지금은 **게이트로 write 만 뺄 수가 없다** — RW 는 읽기와 쓰기가 한 패킷이라
// write 를 막으면 읽기까지 사라진다. 그래서 04 §2.4 대로 별도 프레임으로 분리한다.
//
// 마스크가 10칸 전부인 것은 01 §5.1 이다: manual 은 **ECU 정기 READ 슬롯도 조작자가 비울 수
// 있다.** slots[] 는 "아무것도 안 넣었을 때의 기본 동작" 일 뿐이다 (04 §2.4.3).
constexpr FrameDef kManual = {
    "manual", 10, {
        /* 0 */ DpcRd(&spans::kDpcSys10Hz),
        /* 1 */ EcuRd(&ecu::kPresetProject),
        /* 2 */ PcuRd(&spans::kPcuPower10Hz),
        /* 3 */ EcuRd(&ecu::kPresetProject),
        /* 4 */ Cmd(0),
        /* 5 */ EcuRd(&ecu::kPresetProject),
        /* 6 */ Cmd(1),
        /* 7 */ EcuRd(&ecu::kPresetProject),
        /* 8 */ Cmd(2),
        /* 9 */ EcuRd(&ecu::kPresetProject),
    },
    /*user_slot_mask*/ 0x3FF   // 10칸 전부 (01 §5.1)
};

// ── control — 매 tick 이 ECU RW 다 (200Hz) ──────────────────────────────────
//
// 프레임을 40칸으로 편 이유는 **양보 가능 tick 을 표로 표현하기 위해서다.** 1칸 프레임에
// 마스크를 세우면 "매 tick 양보 가능" 이 되어 `forever` 커맨드 하나가 제어를 통째로 굶긴다.
// 40칸 중 **tick 39 한 칸**만 양보하면 상한이 표에서 바로 읽힌다:
//
//   최대 5Hz(200ms 주기) · 제어 tick 손실 2.5% · 5ms 결손 (AK CAN timeout 200ms 대비 40배 여유)
//
// 양보가 실제로 일어나는 것은 커맨드가 대기 중이고 정책 게이트가 허용할 때뿐이다.
// 커맨드가 없으면 이 칸도 평소대로 ECU RW 다 — 즉 **현행 동작과 바이트 단위로 같다.**
constexpr FrameDef MakeControlFrame(const char* name, SlotDef s, uint64_t mask) {
    FrameDef f{name, kMaxFrameTicks, {}, mask};
    for (uint8_t i = 0; i < kMaxFrameTicks; i++) f.slots[i] = s;
    return f;
}
constexpr uint64_t kControlUserMask = 1ULL << 39;

constexpr FrameDef kControl     = MakeControlFrame("control",      EcuRwPreset(), kControlUserMask);
// control + auto_mode:none — 쓰지 않고 읽기만 한다. 프레임 모양은 control 과 같다.
constexpr FrameDef kControlRead = MakeControlFrame("control_read", EcuRdPreset(), kControlUserMask);

}  // namespace frames

// ── 컴파일 타임 검산 (04 §2.5) — 사람이 더해 적지 않는다 ─────────────────────
static_assert(frames::kProject.ticks <= kMaxFrameTicks, "project 프레임이 배열보다 길다");
static_assert(frames::kProject.AllValid(), "project 프레임에 구조적으로 불가능한 슬롯이 있다");
// Q3 (04 §2.4) — 이 다섯 줄이 "10칸 = ECU RW×5 + DPC×1 + PCU×1 + Command×3" 이다.
// 주기는 프레임 길이에서 파생된다: 10 tick = 50ms → RW 5칸 = 100Hz, 1칸짜리 = 20Hz.
static_assert(frames::kProject.ticks == 10, "project 프레임은 10 tick = 50ms (Q3)");
static_assert(frames::kProject.CountOf(SlotId::ECU, SlotInst::RW) == 5, "ECU RW 는 100Hz");
static_assert(frames::kProject.CountOf(SlotId::PCU)     ==  1, "PCU READ 는 20Hz");
static_assert(frames::kProject.CountOf(SlotId::DPC)     ==  1, "DPC READ 는 20Hz");
static_assert(frames::kProject.CountOf(SlotId::COMMAND) ==  3, "커맨드 슬롯 3개, 각 20Hz");
// 읽기 전용/쓰기 전용 ECU 슬롯은 남지 않아야 한다 — 남아 있으면 RW 흡수가 덜 된 것이다.
static_assert(frames::kProject.CountOf(SlotId::ECU, SlotInst::READ)  == 0,
              "project 의 ECU 읽기는 전부 RW 에 흡수됐다 (sys 전용 슬롯도 없어졌다)");
static_assert(frames::kProject.CountOf(SlotId::ECU, SlotInst::WRITE) == 0,
              "project 의 cmd_vel WRITE 도 RW 에 흡수됐다");
// 프레임이 빈칸 없이 꽉 찼는가 — EMPTY 가 남아 있으면 tick 하나가 놀고 있다는 뜻이다.
static_assert(frames::kProject.CountOf(SlotId::EMPTY) == 0, "project 프레임에 빈 tick 이 있다");
// 04 §2.4.3 — 용량은 slots[] 가 아니라 **마스크**가 정한다. project 는 전용 3칸이므로
// 둘이 일치해야 한다. 어긋나면 "표에는 Cmd 자리인데 못 쓰는 칸" 이 생긴다.
static_assert(frames::kProject.UserSlotCount() == 3,
              "project 의 커맨드 용량은 3 (04 §2.4.3)");
static_assert(frames::kProject.UserSlotAt(4) && frames::kProject.UserSlotAt(6) &&
              frames::kProject.UserSlotAt(8),
              "project 의 양보 칸은 Cmd 자리(4·6·8)와 같아야 한다");
static_assert(!frames::kProject.UserSlotAt(1) && !frames::kProject.UserSlotAt(0),
              "project 의 ECU RW·DPC 칸은 양보 대상이 아니다");

// manual — 배치는 같고 ECU 가 READ 이며, **10칸 전부** 양보 가능하다 (01 §5.1).
static_assert(frames::kManual.ticks == frames::kProject.ticks,
              "manual 은 project 와 같은 배치다 — ECU 명령만 다르다");
static_assert(frames::kManual.AllValid(), "manual 프레임에 구조적으로 불가능한 슬롯이 있다");
static_assert(frames::kManual.CountOf(SlotId::ECU, SlotInst::READ) == 5,
              "manual 의 ECU 는 READ 다 — 브리지가 자동으로 쓰지 않는다 (01 §4.1)");
static_assert(frames::kManual.CountOf(SlotId::ECU, SlotInst::RW) == 0 &&
              frames::kManual.CountOf(SlotId::ECU, SlotInst::WRITE) == 0,
              "manual 에 쓰기가 남아 있으면 그 모드의 정의가 깨진다");
static_assert(frames::kManual.UserSlotCount() == 10,
              "manual 은 10칸 전부 조작자가 덮어쓸 수 있다 (01 §5.1)");

static_assert(frames::kControl.AllValid() && frames::kControlRead.AllValid(),
              "control 프레임에 구조적으로 불가능한 슬롯이 있다");
static_assert(frames::kControl.CountOf(SlotId::ECU, SlotInst::RW) == kMaxFrameTicks,
              "control 은 매 tick ECU RW (200Hz)");
static_assert(frames::kControlRead.CountOf(SlotId::ECU, SlotInst::READ) == kMaxFrameTicks,
              "auto_mode:none 은 매 tick ECU READ (200Hz)");
static_assert(frames::kControl.CountOf(SlotId::COMMAND) == 0,
              "control 은 커맨드 전용 슬롯이 0개다 (04 §2.4.1) — 양보 tick 이 유일한 경로");
// 양보 상한을 표가 보증한다. 이 수가 늘면 제어 주기 손실이 그만큼 늘어난다.
static_assert(frames::kControl.UserSlotCount() == 1,
              "control 의 양보 tick 은 프레임당 1칸 (최대 5Hz, 제어 손실 2.5%)");
static_assert(frames::kControl.UserSlotAt(39) && !frames::kControl.UserSlotAt(0),
              "control 의 양보 칸은 tick 39 다");

// wire 예산 — 응답이 STM 버퍼를 넘으면 애초에 빌드가 안 된다 (04 §2.4.2).
static_assert(frames::kProject.MaxRespPayload() <= kMaxRespPayload,
              "project 프레임 응답 예산 초과");
static_assert(frames::kControl.MaxRespPayload() <= kMaxRespPayload,
              "control 프레임 응답 예산 초과");
static_assert(frames::kManual.MaxRespPayload() <= kMaxRespPayload,
              "manual 프레임 응답 예산 초과");

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_SLOT_TABLE_HPP_
