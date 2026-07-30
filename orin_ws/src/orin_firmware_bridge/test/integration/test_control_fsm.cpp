// C-2 제어 FSM (testbed_spec.md §2)
// 막는 것: write 소스가 상태를 무시하고 새어나가는 것 — IDLE/LOCKED 에서 0A 가 아닌 값이
//          나가거나, LOCKED 가 자동 해제되거나, RUNNING 중 새 goal 이 실험을 덮어쓰는 것.
#include "rd_test_common.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"

using namespace orin_bridge;

namespace {
bool AllZero(const ControlWrite_t& w) {
    for (int i = 0; i < 4; i++) if (w.current[i] != 0.0f) return false;
    return true;
}
}

TEST(ControlFsm, InitIsZeroAndRejectsConfig) {
    RdControl t;
    EXPECT_EQ(t.State(), ControlState::INIT);
    EXPECT_TRUE(AllZero(t.SelectWrite())) << "INIT write 는 0A";
    EXPECT_FALSE(t.AcceptsConfig(false));
    EXPECT_FALSE(t.AcceptsConfig(true)) << "INIT 에서는 REARM 도 불가";
}

TEST(ControlFsm, IdleHoldsZeroEvenIfSampleInjected) {
    RdControl t; t.MarkInitDone();
    EXPECT_EQ(t.State(), ControlState::IDLE);
    EXPECT_TRUE(AllZero(t.SelectWrite())) << "IDLE = 0A 고정 (C-2 완료 기준)";
    EXPECT_TRUE(t.AcceptsConfig(false));
    EXPECT_TRUE(t.AcceptsConfig(true)) << "IDLE 은 config 전부 허용";

    const float c[4] = {5, 5, 5, 5};
    t.SetProfileSample(c, 1.0f, 0);
    EXPECT_TRUE(AllZero(t.SelectWrite())) << "RUNNING 이 아니면 주입 샘플은 무시돼야 한다";

    t.MarkInitDone();
    EXPECT_EQ(t.State(), ControlState::IDLE) << "MarkInitDone 재진입 무해";
}

TEST(ControlFsm, RunningPlaysSampleAndRejectsConcurrentGoal) {
    RdControl t; t.MarkInitDone();
    ASSERT_TRUE(t.BeginProfile(7));
    EXPECT_FALSE(t.BeginProfile(8)) << "동시 goal 불가 (§3.2)";
    EXPECT_FALSE(t.AcceptsConfig(false)) << "RUNNING 중 config 거부";
    EXPECT_FALSE(t.AcceptsConfig(true));

    const float c[4] = {1, 2, 3, 4};
    t.SetProfileSample(c, 2.5f, 3);
    const auto w = t.SelectWrite();
    EXPECT_FLOAT_EQ(w.current[0], 1.0f);
    EXPECT_FLOAT_EQ(w.current[3], 4.0f);
    EXPECT_EQ(w.goal_id, 7u);
    EXPECT_FLOAT_EQ(w.profile_time, 2.5f);
    EXPECT_EQ(w.segment_index, 3);
}

TEST(ControlFsm, EndProfileClearsResidualCommand) {
    RdControl t; t.MarkInitDone(); t.BeginProfile(1);
    const float c[4] = {9, 9, 9, 9};
    t.SetProfileSample(c, 1.0f, 1);
    t.EndProfile();
    EXPECT_EQ(t.State(), ControlState::IDLE);
    const auto w = t.SelectWrite();
    EXPECT_TRUE(AllZero(w)) << "복귀 후 잔류 명령이 남으면 안 된다";
    EXPECT_EQ(w.goal_id, 0u);
}

TEST(ControlFsm, LockedLatchesZeroAndOnlyRearmEscapes) {
    RdControl t; t.MarkInitDone(); t.BeginProfile(1);
    const float c[4] = {9, 9, 9, 9};
    t.SetProfileSample(c, 1.0f, 1);

    t.Lock("test");
    EXPECT_EQ(t.State(), ControlState::LOCKED);
    EXPECT_TRUE(AllZero(t.SelectWrite())) << "LOCKED 는 RUNNING 잔류 명령을 무시하고 0A 래치";
    EXPECT_FALSE(t.AcceptsConfig(false)) << "일반 config 거부";
    EXPECT_TRUE(t.AcceptsConfig(true))   << "REARM 만 허용";
    EXPECT_FALSE(t.BeginProfile(2))      << "LOCKED 에서 goal 거부";

    t.Lock("second");
    EXPECT_EQ(t.LockReason(), "test") << "연쇄 래치 시 최초 사유를 보존해야 원인 추적이 된다";

    t.SetProfileSample(c, 1.0f, 1);
    EXPECT_TRUE(AllZero(t.SelectWrite())) << "LOCKED 중 주입도 무시";

    ASSERT_TRUE(t.Rearm());
    EXPECT_EQ(t.State(), ControlState::IDLE);
    EXPECT_TRUE(t.LockReason().empty());
    EXPECT_FALSE(t.Rearm()) << "IDLE 에서 REARM 은 거부";
    EXPECT_TRUE(t.BeginProfile(3)) << "REARM 후 재실행 가능";
}

TEST(ControlFsm, WriteErrStreakLatchesAndNeverAutoClears) {
    RdControl t; t.MarkInitDone(); t.BeginProfile(1);
    t.NoteWriteErrStreak(RdControl::kWriteErrLockStreak - 1);
    EXPECT_EQ(t.State(), ControlState::RUNNING) << "임계 미만은 유지";

    t.NoteWriteErrStreak(RdControl::kWriteErrLockStreak);
    EXPECT_EQ(t.State(), ControlState::LOCKED);
    EXPECT_NE(t.LockReason().find("write 연속 거부"), std::string::npos);

    t.NoteWriteErrStreak(0);
    EXPECT_EQ(t.State(), ControlState::LOCKED)
        << "스트릭이 회복돼도 자동 해제 금지 — 원인 확인 후 명시 REARM 이 설계 의도";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RegisterRclcppEnv();
    return RUN_ALL_TESTS();
}
