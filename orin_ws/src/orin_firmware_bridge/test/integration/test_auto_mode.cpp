// C-8 auto_mode ↔ write 범위 연동 (testbed_spec.md §2.6)
// 막는 것: KINEMATIC 이 살아 있어 ECU 가 ctr_mode 를 덮어쓰는 경쟁, 범위 전환 시 shadow 잔재가
//          ECU 로 새는 것, 그리고 §2.6 제약 0 위반 — bridge 가 소유하지도 않은 섀도를 덮어써
//          ECU 실값을 지우고 그걸 다시 읽어 자기 자신을 검증하는 것.
#include "rd_test_fixture.hpp"

using namespace orin_bridge;
using Req = Cfg::Request;

class AutoModeTest : public BridgeFixture {
protected:
    // auto_mode=2 WRITE 시점에 shadow 가 이미 소독됐는지 관측 (§2.6 제약 2)
    void OnOosWrite(const OosStep_t& step) override {
        if (step.addr != ecu::REG_AUTO_MODE_OFFSET || step.value != ecu::AUTO_MODE_DIRECT) return;
        std::lock_guard<std::mutex> l(state_.state_mutex);
        const auto& cm = state_.ecu.reg.cmd_motor;
        bool ok = true;
        for (int i = 0; i < 4; i++)
            if (cm.ctr_mode[i] != ecu::CTR_MODE_CURRENT ||
                cm.cmd_position[i] != 0.0f || cm.cmd_velocity[i] != 0.0f) ok = false;
        sanitized_at_write_ = ok;
    }
    std::atomic<bool> sanitized_at_write_{false};
};

TEST_F(AutoModeTest, RegisterContractAndDefaults) {
    EXPECT_EQ(ecu::REG_MOTOR_MASK_OFFSET, 192);
    EXPECT_EQ(ecu::REG_AUTO_MODE_OFFSET, 188);
    EXPECT_EQ(ecu::REG_MODE_OFFSET, 190);
    EXPECT_EQ(ecu::AUTO_MODE_CURRENT, 1);
    EXPECT_EQ(ecu::AUTO_MODE_DIRECT, 2);
    EXPECT_EQ(node_->AutoModeParam(), ecu::AUTO_MODE_CURRENT) << "제어 기본은 CURRENT";
    EXPECT_TRUE(node_->AutoModeValid());
    EXPECT_EQ(node_->AutoMode(), ecu::AUTO_MODE_CURRENT);
}

TEST_F(AutoModeTest, SelectorSwitchesAndReportsWriteRange) {
    auto [ok, msg] = Config(Req::OP_SET_AUTO_MODE, {}, 2);
    ASSERT_TRUE(ok) << msg;
    EXPECT_EQ(node_->AutoMode(), ecu::AUTO_MODE_DIRECT);
    EXPECT_NE(msg.find("128:52"), std::string::npos) << "응답에 전환 후 범위 명시 (§3.3)";
    EXPECT_EQ(ecu_reg_[188], 2);

    auto [ok2, msg2] = Config(Req::OP_SET_AUTO_MODE, {}, 1);
    ASSERT_TRUE(ok2) << msg2;
    EXPECT_EQ(node_->AutoMode(), ecu::AUTO_MODE_CURRENT);
    EXPECT_NE(msg2.find("164:16"), std::string::npos);
}

TEST_F(AutoModeTest, SanitizeHappensBeforeExpandingRange) {
    // 과거 값 잔재를 심어 둔다 — 소독이 없으면 이 값이 그대로 ECU 로 나간다
    {
        std::lock_guard<std::mutex> l(state_.state_mutex);
        auto& cm = state_.ecu.reg.cmd_motor;
        for (int i = 0; i < 4; i++) {
            cm.ctr_mode[i] = 3; cm.cmd_position[i] = 99.0f; cm.cmd_velocity[i] = 55.0f;
        }
    }
    echo_ctr_mode_ = false;
    sanitized_at_write_ = false;
    ClearTrace();

    ASSERT_TRUE(Config(Req::OP_SET_AUTO_MODE, {}, 2).first);
    EXPECT_TRUE(sanitized_at_write_.load())
        << "auto_mode WRITE 시점에 shadow 가 이미 소독돼 있어야 한다 (§2.6 제약 2)";
    const auto tr = Trace();
    ASSERT_GE(tr.size(), 2u);
    EXPECT_EQ(tr[0], "W188=2");
    EXPECT_EQ(tr[1], "R188");
}

// §2.6 제약 0 (하네스가 잡은 shadow 소유권 버그의 회귀 테스트).
// auto_mode=1 에서 bridge 는 cmd_current 만 소유한다. ctr_mode 섀도를 덮어쓰면
// read 세그로 들어온 ECU 실값이 지워지고, 가드가 자기가 쓴 값을 읽어 통과해버린다.
TEST_F(AutoModeTest, BridgeMustNotClobberEcuOwnedShadow) {
    // tick 루프를 멈추고 PrepareControlCommand 를 직접 호출한다 — 루프가 돌면 read 세그 모사가
    // 값을 되돌려놔 덮어쓰기를 가려버린다. 계약을 직접 찔러야 회귀가 확실히 잡힌다.
    stall_ = true;
    std::this_thread::sleep_for(20ms);

    // auto_mode=1(CURRENT): write 범위 164:16 — ctr_mode/pos/vel 은 ECU 소유다.
    {
        std::lock_guard<std::mutex> l(state_.state_mutex);
        auto& cm = state_.ecu.reg.cmd_motor;
        cm.ctr_mode[2] = 3;          // ECU 가 보고한 실값 (VELOCITY)
        cm.cmd_velocity[2] = 42.0f;
    }
    node_->Control().PrepareWrite();
    {
        std::lock_guard<std::mutex> l(state_.state_mutex);
        const auto& cm = state_.ecu.reg.cmd_motor;
        EXPECT_EQ(cm.ctr_mode[2], 3)
            << "CURRENT 모드에서 bridge 가 ctr_mode 섀도를 덮어쓰면 ECU 실값이 지워지고, "
               "§2.6-3 가드가 자기가 쓴 값을 읽어 통과해버린다 (§2.6 제약 0)";
        EXPECT_FLOAT_EQ(cm.cmd_velocity[2], 42.0f) << "cmd_velocity 도 ECU 소유";
    }

    // auto_mode=2(DIRECT): write 범위 128:52 — 이제 bridge 소유이므로 반드시 써야 한다.
    stall_ = false;
    ASSERT_TRUE(Config(Req::OP_SET_AUTO_MODE, {}, 2).first);
    echo_ctr_mode_ = false;
    stall_ = true;
    std::this_thread::sleep_for(20ms);
    {
        std::lock_guard<std::mutex> l(state_.state_mutex);
        state_.ecu.reg.cmd_motor.ctr_mode[2] = 3;
        state_.ecu.reg.cmd_motor.cmd_velocity[2] = 42.0f;
    }
    node_->Control().PrepareWrite();
    {
        std::lock_guard<std::mutex> l(state_.state_mutex);
        const auto& cm = state_.ecu.reg.cmd_motor;
        EXPECT_EQ(cm.ctr_mode[2], ecu::CTR_MODE_CURRENT) << "DIRECT 에서는 bridge 가 ctr_mode 소유";
        EXPECT_FLOAT_EQ(cm.cmd_velocity[2], 0.0f) << "DIRECT 에서는 pos/vel 도 bridge 가 0 으로 소유";
    }
    stall_ = false;
}

// 가드가 ECU 실값을 보는가 — **DIRECT 에서** (2026-07-28 갱신).
//
// 판정 기준이 ctr_mode 에서 auto_mode 로 한 단계 올라갔다 (01 §7.2, 05 §2.3):
//   auto_mode==CURRENT 이면 ECU 가 ctr_mode 를 100Hz 로 자가치유하므로 **보장된다**.
//   ctr_mode 를 따로 볼 이유가 없고, 그 확인은 read-back 세그를 요구해 프리셋을 키운다.
//   DIRECT 에서만 ctr_mode 가 bridge 소유가 되고, 그때 shadow 값이 판정 근거가 된다.
//
// 이 테스트가 원래 막던 것(ECU 실값과 어긋난 낙관적 통과)은 **DIRECT 로 옮겨 그대로 지킨다**.
// testbed_spec §2.6-0 이 발견한 실패의 근본 원인은 "범위 밖 shadow 를 덮어쓴 것"(S2)이었고,
// S2 를 지키면 CURRENT 에서의 read-back 은 중복이다.
TEST_F(AutoModeTest, GuardSeesEcuTruthNotBridgeOptimism) {
    ASSERT_TRUE(Config(Req::OP_SET_AUTO_MODE, {}, ecu::AUTO_MODE_DIRECT).first);
    SetEcuCtrMode(2, 3);       // ECU: m3 = VELOCITY
    echo_ctr_mode_ = true;     // read 세그가 매 tick 그 사실을 실어온다
    // 여러 번 시도해도 한 번도 통과하면 안 된다 (덮어쓰기가 있으면 산발적으로 통과한다)
    for (int i = 0; i < 5; i++) {
        EXPECT_FALSE(SendGoalAccepted("motors: {m1: [{type: hold, duration: 5.0, value: 1}]}"))
            << "시도 " << i << ": ECU 가 VELOCITY 인데 goal 이 통과하면 프로파일이 조용히 무효화된다";
    }
}

// CURRENT 에서는 ctr_mode 를 보지 않는다 — 위 규칙의 뒷면 (05 §2.3).
TEST_F(AutoModeTest, CurrentModeTrustsEcuSelfHealing) {
    SetEcuCtrMode(2, 3);       // ECU 가 아직 VELOCITY 로 보이더라도
    echo_ctr_mode_ = true;
    EXPECT_TRUE(SendGoalAccepted("motors: {m1: [{type: hold, duration: 5.0, value: 1}]}"))
        << "auto_mode=CURRENT 면 ECU 가 100Hz 로 자가치유하므로 통과해야 한다 (01 §7.2)";
}

// 활성 모터만 본다 — DIRECT 에서 (ctr_mode 판정이 사는 곳).
TEST_F(AutoModeTest, ProfileGuardChecksActiveMotorsOnly) {
    ASSERT_TRUE(Config(Req::OP_SET_AUTO_MODE, {}, ecu::AUTO_MODE_DIRECT).first);
    for (int i = 0; i < 4; i++) SetEcuCtrMode(i, ecu::CTR_MODE_CURRENT);
    EXPECT_TRUE(SendGoalAccepted("motors: {m1: [{type: hold, duration: 5.0, value: 1}]}"))
        << "전 모터 CURRENT 면 수락";

    SetEcuCtrMode(2, 3);
    EXPECT_FALSE(SendGoalAccepted("motors: {m1: [{type: hold, duration: 5.0, value: 1}]}"))
        << "활성 모터 m3 가 VELOCITY → reject";

    ASSERT_TRUE(Config(Req::OP_SET_ACTIVE_MOTORS, {1, 2}, 0).first);
    EXPECT_TRUE(SendGoalAccepted("motors: {m1: [{type: hold, duration: 5.0, value: 1}]}"))
        << "비활성 모터의 ctr_mode 는 무관해야 한다";

    SetEcuCtrMode(0, 4);   // POSITION
    EXPECT_FALSE(SendGoalAccepted("motors: {m1: [{type: hold, duration: 5.0, value: 1}]}"));
}

TEST_F(AutoModeTest, ForbiddenAutoModesRejectedWithReason) {
    auto [ok0, m0] = Config(Req::OP_SET_AUTO_MODE, {}, 0);
    EXPECT_FALSE(ok0);
    EXPECT_NE(m0.find("KINEMATIC"), std::string::npos) << "왜 금지인지 사유가 나와야 한다";
    auto [ok3, m3] = Config(Req::OP_SET_AUTO_MODE, {}, 3);
    EXPECT_FALSE(ok3);
    EXPECT_NE(m3.find("미구현"), std::string::npos);
    EXPECT_FALSE(Config(Req::OP_SET_AUTO_MODE, {}, 9).first);
    EXPECT_EQ(node_->AutoMode(), ecu::AUTO_MODE_CURRENT) << "거부 후 셀렉터 불변";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RegisterRclcppEnv();
    return RUN_ALL_TESTS();
}

// ── auto_mode 표시 문자열이 셀렉터와 갈라지지 않게 고정 ─────────────────────────
//
// 무엇을 막는가: VELOCITY(4)/POSITION(5) 가 추가됐을 때 실제 셀렉터(rd_schedule 의
// 4-way switch)만 갱신되고 표시용 삼항식 `am == DIRECT ? "DIRECT" : "CURRENT"` 은
// 여섯 군데에 그대로 남았다. 그래서 2026-07-28 실기에서 `config auto_mode 5` 가
// **성공해 놓고 "auto_mode=1(CURRENT), write 164:16" 이라고 답했다.**
// 동작은 옳고 보고만 틀렸으므로 조작자가 알아챌 방법이 없었다 — 그것이 위험한 지점이다.
TEST(AutoModeLabels, NameAndSpanCoverEveryMode) {
    EXPECT_STREQ(ecu::AutoModeName(ecu::AUTO_MODE_KINEMATIC), "KINEMATIC");
    EXPECT_STREQ(ecu::AutoModeName(ecu::AUTO_MODE_CURRENT),   "CURRENT");
    EXPECT_STREQ(ecu::AutoModeName(ecu::AUTO_MODE_DIRECT),    "DIRECT");
    EXPECT_STREQ(ecu::AutoModeName(ecu::AUTO_MODE_CONTROL),   "CONTROL");
    EXPECT_STREQ(ecu::AutoModeName(ecu::AUTO_MODE_VELOCITY),  "VELOCITY");
    EXPECT_STREQ(ecu::AutoModeName(ecu::AUTO_MODE_POSITION),  "POSITION");

    // 범위는 01 §6.1.2 표 그대로. 이 숫자가 바뀌면 wire 계약이 바뀐 것이다.
    EXPECT_STREQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_CURRENT),  "164:16");
    EXPECT_STREQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_DIRECT),   "128:52");
    EXPECT_STREQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_VELOCITY), "148:16");
    EXPECT_STREQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_POSITION), "132:16");

    // 금지 모드는 bridge 가 cmd 를 쓰지 않는다 — "없음" 을 0:0 으로 위장하지 않는다.
    EXPECT_STREQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_KINEMATIC), "-");
    EXPECT_STREQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_CONTROL),   "-");
}

// 표시 범위가 스케줄러가 실제로 고르는 태스크와 일치하는지 — 문자열과 셀렉터의 교차 검증.
TEST(AutoModeLabels, SpanStringMatchesRegisterOffsets) {
    auto span = [](uint16_t off, uint16_t sz) {
        return std::to_string(off) + ":" + std::to_string(sz);
    };
    EXPECT_EQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_CURRENT),
              span(ecu::REG_CMD_CURRENT_OFFSET,  ecu::REG_CMD_VALUE_SIZE));
    EXPECT_EQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_VELOCITY),
              span(ecu::REG_CMD_VELOCITY_OFFSET, ecu::REG_CMD_VALUE_SIZE));
    EXPECT_EQ(ecu::AutoModeWriteSpan(ecu::AUTO_MODE_POSITION),
              span(ecu::REG_CMD_POSITION_OFFSET, ecu::REG_CMD_VALUE_SIZE));
}

// 비-DIRECT 에서 ECU 가 강제하는 ctr_mode — 2026-07-28 실기값 (06 §4.7).
// 섀도를 전부 CURRENT 로 고정해 두면 status 가 auto_mode=5 인데 ctr_mode=[1,1,1,1] 로
// 보고한다. 게이팅에는 영향이 없지만(AcceptsAutoMode 는 DIRECT 에서만 ctr_mode 를 본다)
// **실험 중 조작자가 보는 값이 틀린다.**
TEST(AutoModeLabels, ForcedCtrModeMatchesHardware) {
    EXPECT_EQ(ecu::AutoModeForcedCtrMode(ecu::AUTO_MODE_CURRENT),  ecu::CTR_MODE_CURRENT);
    EXPECT_EQ(ecu::AutoModeForcedCtrMode(ecu::AUTO_MODE_VELOCITY), ecu::CTR_MODE_VELOCITY);
    EXPECT_EQ(ecu::AutoModeForcedCtrMode(ecu::AUTO_MODE_POSITION), ecu::CTR_MODE_POSITION);
}
