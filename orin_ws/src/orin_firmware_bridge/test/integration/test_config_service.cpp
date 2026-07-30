// C-5 config service (testbed_spec.md §2 단발 2경로, §3.3)
// 막는 것: RUNNING 중 설정 변경이 실험을 오염시키는 것, out-of-span 대체가 tick 을 과소비하거나
//          큐잉돼 뒤늦게 반영되는 것, 검증 실패가 무한 대기/조용한 성공으로 끝나는 것.
#include "rd_test_fixture.hpp"

using namespace orin_bridge;
using Req = Cfg::Request;

class ConfigServiceTest : public BridgeFixture {};

TEST_F(ConfigServiceTest, GetStatusAlwaysAllowed) {
    auto [ok, msg] = Config(Req::OP_GET_STATUS, {}, 0);
    EXPECT_TRUE(ok);
    // 04 §4 (C3) — 응답은 **정형 JSON** 이다. 종전 "state=IDLE motor_mask=0x0F" 문장은
    // 소비자에게 정규식 파싱을 시켰고, 그것이 웹의 기능 한계를 만들었다 (06 §9.1).
    EXPECT_NE(msg.find("\"control_state\":\"IDLE\""), std::string::npos) << msg;
    EXPECT_NE(msg.find("\"motor_mask\":15"), std::string::npos) << msg;
}

TEST_F(ConfigServiceTest, SetActiveMotorsUsesExactlyTwoTicks) {
    ClearTrace();
    auto [ok, msg] = Config(Req::OP_SET_ACTIVE_MOTORS, {2, 3}, 0);
    EXPECT_TRUE(ok) << msg;
    EXPECT_EQ(ecu_reg_[192], 0x06);
    const auto tr = Trace();
    ASSERT_EQ(tr.size(), 2u) << "WRITE tick + READ tick = 최대 2 tick (§6.2-4)";
    EXPECT_EQ(tr[0], "W192=6");
    EXPECT_EQ(tr[1], "R192");
    EXPECT_NE(Config(Req::OP_GET_STATUS, {}, 0).second.find("\"motor_mask\":6"),
              std::string::npos);
}

TEST_F(ConfigServiceTest, SetModeAcceptsZeroOneRejectsOthers) {
    EXPECT_TRUE(Config(Req::OP_SET_MODE, {}, 0).first);
    EXPECT_EQ(ecu_reg_[190], 0);
    EXPECT_TRUE(Config(Req::OP_SET_MODE, {}, 1).first);
    EXPECT_EQ(ecu_reg_[190], 1);
    EXPECT_FALSE(Config(Req::OP_SET_MODE, {}, 7).first);
}

TEST_F(ConfigServiceTest, WriteFailureDoesNotStealReadTick) {
    fail_write_ = true;
    ClearTrace();
    auto [ok, msg] = Config(Req::OP_SET_ACTIVE_MOTORS, {1}, 0);
    EXPECT_FALSE(ok);
    EXPECT_NE(msg.find("WRITE 실패"), std::string::npos);
    EXPECT_EQ(Trace().size(), 1u) << "WRITE 실패 시 재시도·READ 없이 즉시 복귀 (§6.2-3)";
    fail_write_ = false;

    corrupt_ = true;
    auto [ok2, msg2] = Config(Req::OP_SET_ACTIVE_MOTORS, {1}, 0);
    EXPECT_FALSE(ok2);
    EXPECT_NE(msg2.find("불일치"), std::string::npos);
    corrupt_ = false;

    EXPECT_TRUE(Config(Req::OP_SET_ACTIVE_MOTORS, {1,2,3,4}, 0).first)
        << "실패 후 in-flight 가 풀려 다음 요청이 정상 동작해야 한다";
    EXPECT_EQ(ecu_reg_[192], 0x0F);
}

TEST_F(ConfigServiceTest, VerifyTimeoutRespondsAndReleasesSlot) {
    stall_ = true;
    const auto t0 = std::chrono::steady_clock::now();
    auto [ok, msg] = Config(Req::OP_SET_MODE, {}, 1);
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
    EXPECT_FALSE(ok);
    EXPECT_NE(msg.find("시간초과"), std::string::npos);
    EXPECT_GE(dt, 45) << "10 tick(50ms) 은 기다려야 한다";
    EXPECT_LT(dt, 2000) << "무한 대기 금지";
    stall_ = false;
    EXPECT_TRUE(Config(Req::OP_SET_MODE, {}, 1).first) << "타임아웃 후 슬롯 해제";
}

TEST_F(ConfigServiceTest, SetCtrModeRequiresDirectAndValidatesArgs) {
    // §3.3 확정: auto_mode=1 에서는 ECU 가 CURRENT 를 강제하므로 사유 명시 후 즉시 거부
    auto [ok0, m0] = Config(Req::OP_SET_CTR_MODE, {1}, 3);
    EXPECT_FALSE(ok0);
    EXPECT_NE(m0.find("DIRECT"), std::string::npos) << "안내 없는 거부는 원인 불명 실패가 된다";

    ASSERT_TRUE(Config(Req::OP_SET_AUTO_MODE, {}, 2).first);
    echo_ctr_mode_ = false;   // DIRECT 에서는 bridge 가 ctr_mode 를 소유 (ECU 에코 off)

    EXPECT_TRUE(Config(Req::OP_SET_CTR_MODE, {2, 3}, 3).first) << "VELOCITY 설정";
    EXPECT_TRUE(Config(Req::OP_SET_CTR_MODE, {2}, 0).first) << "0=ESTOP 허용 (모터별 임시 정지)";
    EXPECT_FALSE(Config(Req::OP_SET_CTR_MODE, {2}, 5).first) << "5(SET_ORIGIN) 거부";
    EXPECT_FALSE(Config(Req::OP_SET_CTR_MODE, {2}, 9).first);
    EXPECT_FALSE(Config(Req::OP_SET_CTR_MODE, {5}, 1).first) << "모터 번호 범위";

    ASSERT_TRUE(Config(Req::OP_SET_CTR_MODE, {1,2,3,4}, 1).first);
    ASSERT_TRUE(Config(Req::OP_SET_ACTIVE_MOTORS, {2, 3}, 0).first);
    EXPECT_FALSE(Config(Req::OP_SET_CTR_MODE, {1}, 1).first) << "비활성 모터 거부 (§3.3)";
}

TEST_F(ConfigServiceTest, RunningRejectsEverythingButStatus) {
    RunProfileAct::Goal g;
    g.profile_yaml = "motors: {m1: [{type: hold, duration: 20.0, value: 1}]}";
    auto f = act_cli_->async_send_goal(g);
    ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
    auto gh = f.get();
    ASSERT_NE(gh, nullptr);
    ASSERT_TRUE(WaitFor([&]{ return node_->Control().State() == ControlState::RUNNING; }));

    auto [ok1, m1] = Config(Req::OP_SET_ACTIVE_MOTORS, {1}, 0);
    EXPECT_FALSE(ok1);
    EXPECT_NE(m1.find("RUNNING"), std::string::npos);
    EXPECT_FALSE(Config(Req::OP_SET_CTR_MODE, {1}, 1).first);
    EXPECT_FALSE(Config(Req::OP_SET_MODE, {}, 0).first);
    EXPECT_FALSE(Config(Req::OP_SET_AUTO_MODE, {}, 2).first);
    EXPECT_FALSE(Config(Req::OP_REARM, {}, 0).first) << "LOCKED 가 아니면 REARM 도 거부";

    auto [ok5, m5] = Config(Req::OP_GET_STATUS, {}, 0);
    EXPECT_TRUE(ok5) << "상태 조회는 부작용이 없어 허용 (CLI·웹이 이걸로 산다)";
    EXPECT_NE(m5.find("\"control_state\":\"RUNNING\""), std::string::npos) << m5;

    act_cli_->async_cancel_goal(gh);
    act_cli_->async_get_result(gh).wait_for(10s);
}

TEST_F(ConfigServiceTest, LockedOnlyAcceptsRearm) {
    node_->Control().Lock("config 테스트 래치");
    auto [ok1, m1] = Config(Req::OP_SET_MODE, {}, 1);
    EXPECT_FALSE(ok1);
    EXPECT_NE(m1.find("REARM"), std::string::npos);
    EXPECT_NE(Config(Req::OP_GET_STATUS, {}, 0).second.find("lock_reason"), std::string::npos);
    EXPECT_TRUE(Config(Req::OP_REARM, {}, 0).first);
    EXPECT_EQ(node_->Control().State(), ControlState::IDLE);
    EXPECT_TRUE(Config(Req::OP_SET_MODE, {}, 1).first);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RegisterRclcppEnv();
    return RUN_ALL_TESTS();
}

// ── OP_SET_READ_PRESET (04 §2.4.2) ────────────────────────────────────────────
//
// 프리셋 교체는 **슬롯을 쓰지 않는 경로**다. control 은 커맨드 슬롯이 0개라(04 §2.4.1)
// "IDLE 에서 다른 구간을 계속 보고 싶다" 를 푸는 유일한 수단이 이것이다.
//
// 실제 교체 동작은 L1(RdSchedule)이 소유하므로 여기서는 **가짜 setter** 를 꽂아
// 서비스 → 콜백 배선과 게이트만 본다. 프리셋 테이블 자체는 test_read_preset 가 본다.
class ReadPresetService : public BridgeFixture {
protected:
    void SetUp() override {
        BridgeFixture::SetUp();
        node_->AttachReadPresetSetter([this](uint8_t id, std::string* why) {
            last_requested_ = id;
            if (id >= orin_bridge::ecu::kPresetCount) {
                if (why) *why = "프리셋 id " + std::to_string(id) + " 없음";
                return false;
            }
            applied_ = id;
            if (why) *why = std::string("읽기 프리셋 = ") + orin_bridge::ecu::kPresets[id]->name;
            return true;
        });
    }
    int last_requested_ = -1;
    int applied_ = 0;
};

TEST_F(ReadPresetService, SwapsToKnownPreset) {
    auto [ok, msg] = Config(Req::OP_SET_READ_PRESET, {}, 1);
    ASSERT_TRUE(ok) << msg;
    EXPECT_EQ(applied_, 1);
    EXPECT_NE(msg.find("diag"), std::string::npos) << "응답에 적용된 프리셋 이름이 있어야 한다";
}

// ★ 없는 id 를 수락하면 "바꿨다" 고 응답해 놓고 아무 일도 안 일어난다.
TEST_F(ReadPresetService, RejectsUnknownPresetId) {
    auto [ok, msg] = Config(Req::OP_SET_READ_PRESET, {}, 99);
    EXPECT_FALSE(ok);
    EXPECT_EQ(applied_, 0) << "거부됐는데 적용됐다";
    EXPECT_NE(msg.find("99"), std::string::npos) << "사유에 문제의 값이 있어야 한다";
}

TEST_F(ReadPresetService, RejectedWhenNotIdle) {
    // RUNNING/STREAM 중 교체를 막는 이유는 안전이 아니라 **데이터**다 —
    // 발행 메시지의 필드 구성이 중간에 바뀌면 실험 시계열이 갈라진다.
    node_->Control().Lock("테스트");
    auto [ok, msg] = Config(Req::OP_SET_READ_PRESET, {}, 1);
    EXPECT_FALSE(ok);
    EXPECT_EQ(last_requested_, -1) << "게이트를 통과해 setter 까지 갔다";
    EXPECT_NE(msg.find("LOCKED"), std::string::npos) << msg;
}

// setter 가 안 꽂힌 구성(비 control 기동)에서는 **거부**해야 한다 —
// 조용히 성공으로 응답하면 조작자는 바뀐 줄 안다.
TEST_F(ConfigServiceTest, ReadPresetWithoutWiringIsRejected) {
    auto [ok, msg] = Config(Req::OP_SET_READ_PRESET, {}, 1);
    EXPECT_FALSE(ok);
    EXPECT_NE(msg.find("연결되지 않았다"), std::string::npos) << msg;
}
