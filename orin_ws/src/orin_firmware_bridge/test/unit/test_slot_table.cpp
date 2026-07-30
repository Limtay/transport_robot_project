// 슬롯 테이블 = 스케줄 골든 (redesign/04 §2.2, 00 C1, 07 §3.1)
//
// **이 테스트가 리팩터링의 안전장치다.**
// 골든 바이트 테스트는 각 태스크의 *바이트* 를 고정하지만, **어느 tick 에 어느 태스크가
// 나가는지**(스케줄)는 보지 않는다. 100Hz READ 와 50Hz WRITE 의 순서가 뒤바뀌어도
// 다른 테스트가 전부 통과하고, 실기에서 타이밍으로만 드러난다.
//
// 그래서 표를 **기대 배치와 한 칸씩 대조**한다.
//
// ⚠ 2026-07-29 Q3 재편으로 project 프레임이 40칸(200ms) → 10칸(50ms)이 됐다. 종전 이 파일은
//   구 rd_schedule.cpp 의 분기식(`tick%2`, `odd_idx%2`)을 옮겨 적고 대조했는데, 그 기준은
//   **이제 존재하지 않는 스케줄**이다. 기대값을 Q3 배치로 갈아 끼우되, 무엇이 어떻게
//   달라졌는지를 `ProjectQ3AbsorbedTheOldThreeStreams` 가 명시적으로 남긴다 —
//   기대값만 조용히 바꾸면 "테스트를 통과시키려고 고친 것" 과 구분되지 않는다.

#include <gtest/gtest.h>

#include "orin_firmware_bridge/sched/rd_slot_table.hpp"

namespace {

using orin_bridge::ReadSrc;
using orin_bridge::SlotDef;
using orin_bridge::SlotId;
using orin_bridge::SlotIdName;
using orin_bridge::SlotInst;
using orin_bridge::SlotInstName;
using orin_bridge::SlotValid;
using orin_bridge::ReadPreset;
using orin_bridge::WriteSrc;
namespace frames = orin_bridge::frames;
namespace spans  = orin_bridge::spans;
namespace ecu    = orin_bridge::ecu;

struct Legacy { SlotId id; SlotInst inst; uint8_t cmd_index; };

// 04 §2.4 가 적어 놓은 Q3 배치를 **여기 하드코딩**한다. 표에서 읽어와 비교하면 표가 바뀔 때
// 기대값도 같이 바뀌어 아무것도 증명하지 못한다 (06 §2.1 의 함정).
//
//   tick 0 DPC / 1 ECU RW / 2 PCU / 3 ECU RW / 4 Cmd0 / 5 ECU RW / 6 Cmd1 / 7 ECU RW /
//   8 Cmd2 / 9 ECU RW
Legacy Q3Dispatch(uint64_t tick) {
    switch (tick % 10) {
        case 0: return {SlotId::DPC, SlotInst::READ, 0};
        case 2: return {SlotId::PCU, SlotInst::READ, 0};
        case 4: return {SlotId::COMMAND, SlotInst::NONE, 0};
        case 6: return {SlotId::COMMAND, SlotInst::NONE, 1};
        case 8: return {SlotId::COMMAND, SlotInst::NONE, 2};
        default: return {SlotId::ECU, SlotInst::RW, 0};   // 1·3·5·7·9
    }
}

// ★ 이것이 핵심 — 표가 04 §2.4 의 Q3 배치와 **한 칸도 다르지 않다.**
TEST(SlotTable, ProjectFrameMatchesQ3LayoutTickByTick) {
    for (uint64_t t = 0; t < 40; t++) {
        const Legacy want = Q3Dispatch(t);
        const SlotDef& got = frames::kProject.At(t);
        EXPECT_EQ(got.id, want.id)
            << "tick " << t << ": 표=" << SlotIdName(got.id) << " 기대=" << SlotIdName(want.id);
        EXPECT_EQ(got.inst, want.inst)
            << "tick " << t << ": 표=" << SlotInstName(got.inst)
            << " 기대=" << SlotInstName(want.inst);
        if (want.id == SlotId::COMMAND) {
            EXPECT_EQ(got.cmd_index, want.cmd_index) << "tick " << t << " 의 커맨드 슬롯 번호";
        }
    }
}

// ID·INST 대조만으로는 "무엇을 읽고 무엇을 쓰는가" 가 안 잡힌다. RW 5칸이 전부 같은
// 구간을 보고 같은 곳에 쓰는지 확인한다 — 하나만 달라도 그 tick 의 데이터가 다른 뜻이 된다.
TEST(SlotTable, ProjectRwSlotsAllReadProjectPresetAndWriteCmdVel) {
    int n_rw = 0;
    for (uint64_t t = 0; t < 40; t++) {
        const SlotDef& s = frames::kProject.At(t);
        if (s.id != SlotId::ECU) continue;
        ASSERT_EQ(s.inst, SlotInst::RW) << "tick " << t << " — project 의 ECU 는 전부 RW 다";
        n_rw++;
        // 읽기는 **표가 소유한 고정 구간**이다 (control 과 달리 런타임 프리셋이 아니다).
        ASSERT_EQ(s.read.src, ReadSrc::FIXED) << "tick " << t;
        EXPECT_EQ(s.read.fixed, &ecu::kPresetProject)
            << "tick " << t << " 의 읽기 구간이 " << s.read.fixed->name;
        EXPECT_EQ(s.write.src, WriteSrc::FIXED) << "tick " << t;
        EXPECT_EQ(s.write.addr, ecu::REG_CMD_SYSTEM_OFFSET) << "tick " << t;
        EXPECT_EQ(s.write.len, 8) << "tick " << t << " — cmd_vel 은 180:8";
    }
    EXPECT_EQ(n_rw, 20) << "40 tick = 4 프레임 x 5칸";
}

// ★ Q3 가 **무엇을 흡수했는지**를 못박는다. 기대값을 조용히 갈아 끼운 게 아니라는 근거다.
//
//   구(40칸/200ms): ECU 센서 READ 20 + cmd_vel WRITE 10 + sys READ 2 = ECU 트랜잭션 32건
//   신(10칸/50ms x4): ECU RW 20건
//
// sys 를 별도 슬롯 없이 유지할 수 있는 근거는 **project 프리셋이 {16,17} 로 시작한다**는
// 것 하나다 (04 §2.3). 그게 깨지면 재편 자체가 성립하지 않으므로 여기서 확인한다.
TEST(SlotTable, ProjectQ3AbsorbedTheOldThreeStreams) {
    EXPECT_EQ(frames::kProject.ticks, 10) << "50ms 프레임";

    // ① 구 스케줄의 세 갈래가 전부 RW 로 접혔다.
    EXPECT_EQ(frames::kProject.CountOf(SlotId::ECU, SlotInst::READ), 0)
        << "센서/sys 전용 READ 슬롯이 남아 있다";
    EXPECT_EQ(frames::kProject.CountOf(SlotId::ECU, SlotInst::WRITE), 0)
        << "cmd_vel 전용 WRITE 슬롯이 남아 있다";
    EXPECT_EQ(frames::kProject.CountOf(SlotId::ECU, SlotInst::RW), 5);

    // ② sys 가 매 RW 에 딸려 온다 — 이것이 sys 슬롯을 없앨 수 있었던 유일한 이유다.
    EXPECT_TRUE(ecu::kPresetProject.Covers(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE))
        << "project 프리셋이 SYS 를 안 읽는다 — sys 전용 슬롯을 없애면 안 된다";

    // ③ ECU 왕복 횟수는 **줄었다** (200ms 기준 32 → 20). 늘었다면 재편의 근거가 없다.
    const int old_ecu_per_200ms = 20 + 10 + 2;
    const int new_ecu_per_200ms = frames::kProject.CountOf(SlotId::ECU, SlotInst::RW) * 4;
    EXPECT_LT(new_ecu_per_200ms, old_ecu_per_200ms)
        << "왕복이 늘었다 — RW 로 합친 이득이 없다";
}

// 프레임 경계를 넘어도 같은 패턴이 반복된다 (At 의 나머지 연산 확인).
TEST(SlotTable, FrameRepeatsAcrossBoundaries) {
    for (uint64_t t = 0; t < 200; t++) {
        EXPECT_EQ(frames::kProject.At(t).id, Q3Dispatch(t).id) << "tick " << t;
        EXPECT_EQ(frames::kProject.At(t).id, frames::kProject.At(t + 10).id)
            << "tick " << t << " 와 " << (t + 10) << " 가 다르다";
    }
}

// 주기 검산은 static_assert 가 이미 했다 — 여기서는 **왜 그 수인지**를 남긴다.
TEST(SlotTable, FrequenciesFollowFromCounts) {
    constexpr double kTickHz = 200.0;
    const double frame_hz = kTickHz / frames::kProject.ticks;      // 20Hz (50ms)
    EXPECT_DOUBLE_EQ(frame_hz, 20.0);

    // RW 한 갈래가 센서 읽기와 cmd_vel 쓰기를 **둘 다** 100Hz 로 만든다.
    // 구 스케줄에서는 읽기 100Hz / 쓰기 50Hz 로 어긋나 있었다.
    const double rw_hz = frames::kProject.CountOf(SlotId::ECU, SlotInst::RW) * frame_hz;
    EXPECT_DOUBLE_EQ(rw_hz, 100.0) << "ECU 읽기·쓰기 모두 100Hz";
    EXPECT_DOUBLE_EQ(frames::kProject.CountOf(SlotId::PCU) * frame_hz, 20.0);
    EXPECT_DOUBLE_EQ(frames::kProject.CountOf(SlotId::DPC) * frame_hz, 20.0);
    // 커맨드 슬롯은 3개이고 각각 프레임당 1회 = 20Hz (구 5Hz 에서 4배).
    EXPECT_DOUBLE_EQ(1 * frame_hz, 20.0);
}

// 커맨드 슬롯 0~2 가 **각각 정확히 한 번씩** 나온다 — 하나가 빠지면 그 슬롯은 영영 안 돈다.
// (Q3 로 4개 x 5Hz → 3개 x 20Hz. 개수가 줄고 응답성이 4배가 됐다.)
TEST(SlotTable, EveryCommandSlotAppearsExactlyOnce) {
    int seen[3] = {0, 0, 0};
    for (uint8_t t = 0; t < frames::kProject.ticks; t++) {
        const SlotDef& s = frames::kProject.slots[t];
        if (s.id == SlotId::COMMAND) {
            ASSERT_LT(s.cmd_index, 3u) << "tick " << int(t) << " 의 슬롯 번호가 범위 밖";
            seen[s.cmd_index]++;
        }
    }
    for (int i = 0; i < 3; i++) EXPECT_EQ(seen[i], 1) << "커맨드 슬롯 " << i;
}

// control 은 매 tick 이 ECU RW, auto_mode:none 은 매 tick READ — 서브슬롯이 없다.
TEST(SlotTable, ControlFramesAreAllEcuEveryTick) {
    for (uint64_t t = 0; t < 80; t++) {
        EXPECT_EQ(frames::kControl.At(t).id,   SlotId::ECU)   << "tick " << t;
        EXPECT_EQ(frames::kControl.At(t).inst, SlotInst::RW)  << "tick " << t;
        EXPECT_EQ(frames::kControl.At(t).read.src,  ReadSrc::PRESET);
        EXPECT_EQ(frames::kControl.At(t).write.src, WriteSrc::AUTO_MODE);

        EXPECT_EQ(frames::kControlRead.At(t).id,   SlotId::ECU)    << "tick " << t;
        EXPECT_EQ(frames::kControlRead.At(t).inst, SlotInst::READ) << "tick " << t;
        EXPECT_FALSE(frames::kControlRead.At(t).write.Has())
            << "auto_mode:none 이 쓰기를 갖고 있다";
    }
}

// ── 07 §3.2 양보 tick ──────────────────────────────────────────────────────
// control 에서 사용자 명령이 나갈 수 있는 유일한 경로다. 상한이 표에서 읽혀야 한다.
TEST(SlotTable, ControlYieldsExactlyOneTickPerFrame) {
    EXPECT_EQ(frames::kControl.UserSlotCount(), 1u);
    EXPECT_EQ(frames::kControlRead.UserSlotCount(), 1u);

    int n = 0;
    for (uint64_t t = 0; t < frames::kControl.ticks; t++)
        if (frames::kControl.UserSlotAt(t)) n++;
    EXPECT_EQ(n, 1) << "프레임당 양보 칸은 하나여야 한다";

    // 40 tick(200ms) 마다 한 번 = 최대 5Hz, 제어 tick 손실 2.5%.
    EXPECT_DOUBLE_EQ(200.0 / frames::kControl.ticks, 5.0);
    EXPECT_DOUBLE_EQ(1.0 / frames::kControl.ticks * 100.0, 2.5);
}

// 프레임 경계를 넘어도 양보 칸이 같은 자리에 온다 — tick_count 가 계속 증가하기 때문에
// 나머지 연산이 틀리면 양보가 불규칙해지거나 아예 안 온다.
TEST(SlotTable, YieldTickRepeatsEveryFrame) {
    for (uint64_t t = 0; t < 400; t++) {
        EXPECT_EQ(frames::kControl.UserSlotAt(t), (t % 40) == 39) << "tick " << t;
    }
}

// 04 §2.4.3 — **용량은 slots[] 가 아니라 마스크가 정한다.** project 는 전용 3칸이라 둘이
// 일치해야 하고, manual 은 10칸 전부 열려 있어 일치하지 않는 것이 정상이다.
TEST(SlotTable, UserSlotMaskMatchesCommandCapacity) {
    EXPECT_EQ(frames::kProject.UserSlotCount(), 3u);
    for (uint64_t t = 0; t < 40; t++) {
        const bool is_cmd = frames::kProject.At(t).id == SlotId::COMMAND;
        EXPECT_EQ(frames::kProject.UserSlotAt(t), is_cmd)
            << "tick " << t << " — project 는 Cmd 자리만 양보한다";
    }
    // manual 은 ECU READ 칸까지 조작자가 덮어쓸 수 있다 (01 §5.1) — slots[] 와 일치하지 않는다.
    EXPECT_EQ(frames::kManual.UserSlotCount(), 10u);
    for (uint64_t t = 0; t < 20; t++) EXPECT_TRUE(frames::kManual.UserSlotAt(t)) << "tick " << t;
}

// manual 은 project 와 **같은 배치, 다른 명령**이다 (04 §2.4). 게이트가 아니라 표가
// "브리지가 자동으로 쓰지 않는다" 를 표현한다 — RW 는 게이트로 쓰기만 뺄 수 없기 때문이다.
TEST(SlotTable, ManualMirrorsProjectButNeverWrites) {
    ASSERT_EQ(frames::kManual.ticks, frames::kProject.ticks);
    for (uint8_t t = 0; t < frames::kManual.ticks; t++) {
        const SlotDef& m = frames::kManual.slots[t];
        const SlotDef& p = frames::kProject.slots[t];
        EXPECT_EQ(m.id, p.id) << "tick " << int(t) << " — 배치가 어긋났다";
        if (m.id == SlotId::COMMAND) EXPECT_EQ(m.cmd_index, p.cmd_index) << "tick " << int(t);
        EXPECT_FALSE(m.write.Has()) << "tick " << int(t) << " — manual 에 쓰기가 있다";
        if (m.id == SlotId::ECU) {
            EXPECT_EQ(m.inst, SlotInst::READ) << "tick " << int(t);
            EXPECT_EQ(m.read.fixed, p.read.fixed)
                << "tick " << int(t) << " — 읽는 구간은 project 와 같아야 한다";
        }
    }
}

// ── 04 §2.2.4 구조 검증 ────────────────────────────────────────────────────
TEST(SlotTable, SlotValidityRejectsImpossibleCombinations) {
    using orin_bridge::ReadSpan;
    using orin_bridge::WriteSpan;
    const ReadSpan  r{ReadSrc::FIXED, &spans::kEcuSys10Hz};
    const WriteSpan w{WriteSrc::FIXED, 180, 8};

    EXPECT_TRUE(SlotValid({SlotId::ECU, SlotInst::READ,  r, {}, 0}));
    EXPECT_TRUE(SlotValid({SlotId::ECU, SlotInst::WRITE, {}, w, 0}));
    EXPECT_TRUE(SlotValid({SlotId::ECU, SlotInst::RW,    r,  w, 0}));
    EXPECT_TRUE(SlotValid({SlotId::COMMAND, SlotInst::NONE, {}, {}, 2}));
    EXPECT_TRUE(SlotValid({SlotId::EMPTY, SlotInst::NONE, {}, {}, 0}));

    // READ 인데 쓰기 구간이 붙어 있다 — 그 구간은 조용히 무시된다.
    EXPECT_FALSE(SlotValid({SlotId::ECU, SlotInst::READ, r, w, 0}));
    // RW 인데 읽을 것이 없다 — 응답이 err 1바이트뿐이라 RW 일 이유가 없다.
    EXPECT_FALSE(SlotValid({SlotId::ECU, SlotInst::RW, {}, w, 0}));
    // 실보드인데 명령이 없다.
    EXPECT_FALSE(SlotValid({SlotId::DPC, SlotInst::NONE, {}, {}, 0}));
    // COMMAND 는 표가 구간을 정하지 않는다 (늦은 바인딩).
    EXPECT_FALSE(SlotValid({SlotId::COMMAND, SlotInst::READ, r, {}, 0}));
}

// 예산은 **고를 수 있는 프리셋 전부**의 최악값으로 잡아야 한다 — "지금 고른 것" 으로
// 검증하면 프리셋을 바꾸는 순간 예산을 넘길 수 있다.
TEST(SlotTable, PresetReadSpanUsesWorstCaseBudget) {
    uint16_t worst = 0;
    for (uint8_t i = 0; i < ecu::kPresetCount; i++)
        worst = std::max(worst, ecu::kPresets[i]->RespPayload());
    EXPECT_EQ(frames::kControl.At(0).read.MaxRespPayload(), worst);
    EXPECT_LE(frames::kControl.MaxRespPayload(), orin_bridge::kMaxRespPayload);
}

// ⚠ 종전 `ManualSharesProjectFrame` (manual 과 project 가 같은 프레임 객체) 은 Q3 로
//    성립하지 않는다. project 의 ECU 슬롯이 RW 가 되면서 **게이트로 write 만 뺄 수 없게**
//    됐기 때문이다 — 쓰기를 막으려고 tick 을 버리면 읽기까지 사라진다.
//    "차이는 게이트이지 스케줄이 아니다" 라는 전제 자체가 바뀌었다.
//    두 프레임의 관계는 위 `ManualMirrorsProjectButNeverWrites` 가 대신 지킨다.
TEST(SlotTable, ManualIsItsOwnFrameNotAnAliasOfProject) {
    EXPECT_NE(&frames::kManual, &frames::kProject)
        << "manual 이 project 의 별칭이면 ECU 에 cmd_vel 이 나간다 (01 §4.1 위반)";
    EXPECT_STREQ(frames::kManual.name, "manual");
}

// ── Q3: 쓰기가 막혔을 때 그 tick 을 어떻게 하는가 ───────────────────────────
//
// ★ **실기로는 이걸 확인할 수 없다.** 폴백이 걸려 READ 가 나가든, 게이트가 안 먹어 RW 가
//   나가든 heartbeat 의 Tx 는 똑같이 1 증가한다. 구분이 되는 곳은 여기뿐이다.
TEST(SlotTable, BlockedWriteDegradesRwToReadInsteadOfDroppingTheTick) {
    using orin_bridge::ActionFor;
    using orin_bridge::SlotAction;

    const SlotDef& rw = frames::kProject.At(1);      // ECU RW (읽기 + cmd_vel)
    ASSERT_EQ(rw.inst, SlotInst::RW);

    EXPECT_EQ(ActionFor(rw, /*block_write*/ false), SlotAction::NORMAL);
    // 핵심 — 쓰기를 막아도 **읽기는 살아야 한다.** SKIP 이면 센서 시계열에 구멍이 난다.
    EXPECT_EQ(ActionFor(rw, /*block_write*/ true), SlotAction::READ_FALLBACK)
        << "RW 를 통째로 버렸다 — cmd_vel 을 막으려다 센서까지 잃는다";

    // 쓰기 전용 슬롯은 읽을 것이 없으므로 건너뛰는 게 맞다 (구 40칸 프레임의 동작).
    const SlotDef wr = orin_bridge::EcuWr(ecu::REG_CMD_SYSTEM_OFFSET, 8);
    EXPECT_EQ(ActionFor(wr, true),  SlotAction::SKIP);
    EXPECT_EQ(ActionFor(wr, false), SlotAction::NORMAL);

    // 읽기 전용(manual)은 애초에 쓰기가 없어 게이트와 무관하다.
    const SlotDef& rd = frames::kManual.At(1);
    ASSERT_EQ(rd.inst, SlotInst::READ);
    EXPECT_EQ(ActionFor(rd, true), SlotAction::NORMAL)
        << "manual 은 쓰기가 없으니 막을 것도 없다";
}

}  // namespace
