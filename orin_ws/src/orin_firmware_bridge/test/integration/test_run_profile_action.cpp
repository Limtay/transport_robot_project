// C-4b run_profile action (testbed_spec.md §3.2)
// 막는 것: 검증 안 된 프로파일이 RUNNING 에 진입하는 것, 취소가 모터를 0A 로 되돌리지 못하는 것,
//          동시 goal 이 진행 중 실험을 덮어쓰는 것, result 집계가 틀리는 것.
#include "rd_test_fixture.hpp"

using namespace orin_bridge;

namespace {
struct Outcome {
    bool accepted = false, got_result = false, success = false;
    std::string message;
    uint32_t goal_id = 0, ticks = 0, clamp = 0;
    int feedback_cnt = 0;
};
}

class RunProfileTest : public BridgeFixture {
protected:
    Outcome Run(const std::string& yaml, bool cancel_midway = false) {
        Outcome out;
        RunProfileAct::Goal g; g.profile_yaml = yaml;
        rclcpp_action::Client<RunProfileAct>::SendGoalOptions opt;
        opt.feedback_callback = [&out](auto, auto) { out.feedback_cnt++; };
        auto f = act_cli_->async_send_goal(g, opt);
        if (f.wait_for(5s) != std::future_status::ready) return out;
        auto gh = f.get();
        if (!gh) return out;
        out.accepted = true;
        if (cancel_midway) {
            std::this_thread::sleep_for(300ms);
            act_cli_->async_cancel_goal(gh);
        }
        auto rf = act_cli_->async_get_result(gh);
        if (rf.wait_for(30s) != std::future_status::ready) return out;
        auto r = rf.get();
        out.got_result = true;
        out.success = r.result->success;
        out.message = r.result->message;
        out.goal_id = r.result->goal_id;
        out.ticks   = r.result->ticks_executed;
        out.clamp   = r.result->clamp_cnt;
        return out;
    }
};

TEST_F(RunProfileTest, PlaysWholeProfileAndReturnsToIdle) {
    auto o = Run("motors: {m1: [{type: hold, duration: 1.0, value: 0},"
                 "{type: ramp, duration: 1.0, from: 0, to: 5}]}");
    EXPECT_TRUE(o.accepted);
    ASSERT_TRUE(o.got_result);
    EXPECT_TRUE(o.success);
    EXPECT_EQ(o.message, "완료");
    EXPECT_EQ(o.goal_id, 1u) << "세션 첫 goal";
    EXPECT_EQ(o.ticks, 400u) << "2s x 200Hz 전 구간 재생";
    EXPECT_GT(o.feedback_cnt, 0);
    EXPECT_EQ(node_->Control().State(), ControlState::IDLE);
}

TEST_F(RunProfileTest, GoalIdIncrementsPerSession) {
    EXPECT_EQ(Run("motors: {m1: [{type: hold, duration: 0.5, value: 1}]}").goal_id, 1u);
    auto o = Run("motors: {m1: [{type: hold, duration: 0.5, value: 1}]}");
    EXPECT_EQ(o.goal_id, 2u);
    EXPECT_EQ(o.ticks, 100u);
}

TEST_F(RunProfileTest, InvalidProfilesRejectedBeforeRunning) {
    EXPECT_FALSE(Run("motors: {m1: [{type: bogus, duration: 1.0}]}").accepted);
    EXPECT_FALSE(Run("motors: {m1: [{type: ramp, duration: 1.0, from: 0}]}").accepted);
    EXPECT_FALSE(Run("this is not: [valid").accepted);
    EXPECT_EQ(node_->Control().State(), ControlState::IDLE) << "reject 후에도 IDLE 유지";
}

TEST_F(RunProfileTest, ClampIsAcceptedNotRejected) {
    auto o = Run("motors: {m1: [{type: hold, duration: 0.5, value: 100}]}");
    EXPECT_TRUE(o.accepted);
    EXPECT_TRUE(o.success);
    EXPECT_EQ(o.clamp, 100u) << "전 tick 클램프가 result 에 집계돼야 한다";
}

TEST_F(RunProfileTest, CancelStopsAndZeroesCommand) {
    auto o = Run("motors: {m1: [{type: hold, duration: 30.0, value: 3}]}", true);
    ASSERT_TRUE(o.accepted);
    ASSERT_TRUE(o.got_result);
    EXPECT_FALSE(o.success);
    EXPECT_EQ(o.message, "canceled");
    EXPECT_GT(o.ticks, 0u);
    EXPECT_LT(o.ticks, 6000u) << "일부만 재생되고 중단";
    EXPECT_EQ(node_->Control().State(), ControlState::IDLE);
    const auto w = node_->Control().SelectWrite();
    EXPECT_FLOAT_EQ(w.current[0], 0.0f) << "취소 즉시 0A (§3.2)";
    EXPECT_EQ(w.goal_id, 0u);
}

TEST_F(RunProfileTest, ConcurrentGoalRejected) {
    RunProfileAct::Goal g;
    g.profile_yaml = "motors: {m1: [{type: hold, duration: 20.0, value: 2}]}";
    auto f = act_cli_->async_send_goal(g);
    ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
    auto gh = f.get();
    ASSERT_NE(gh, nullptr);
    ASSERT_TRUE(WaitFor([&]{ return node_->Control().State() == ControlState::RUNNING; }));

    EXPECT_FALSE(Run("motors: {m1: [{type: hold, duration: 1.0, value: 1}]}").accepted)
        << "RUNNING 중 새 goal 은 기존 실험을 오염시킨다";

    act_cli_->async_cancel_goal(gh);
    act_cli_->async_get_result(gh).wait_for(10s);
}

TEST_F(RunProfileTest, LockedRejectsGoalUntilRearm) {
    node_->Control().Lock("테스트 강제 래치");
    EXPECT_FALSE(Run("motors: {m1: [{type: hold, duration: 1.0, value: 1}]}").accepted);
    ASSERT_TRUE(node_->Control().Rearm());
    auto o = Run("motors: {m1: [{type: hold, duration: 0.5, value: 1}]}");
    EXPECT_TRUE(o.accepted);
    EXPECT_TRUE(o.success) << "REARM 후 정상 재생";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RegisterRclcppEnv();
    return RUN_ALL_TESTS();
}
