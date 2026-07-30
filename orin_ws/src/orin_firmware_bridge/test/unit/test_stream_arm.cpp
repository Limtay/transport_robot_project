// STREAM + arm 게이트 (redesign/01 §6.1.3 — 웹 run/stop 버튼)
//
// 막는 사고: STREAM 을 "메시지가 도착하면 RUNNING" 으로 두면, 노드 기동 직후 남아 있던
// 발행자(웹 탭, 죽다 만 MPC)가 **조작자 모르게 모터를 움직인다.** arm 은 그 사이에
// 사람의 명시적 의사표시를 끼워 넣는 장치다.
//
// 고정하는 계약:
//   arm=off  CmdMotor 수신해도 소비하지 않는다 (구독은 하되 버린다)
//   arm=on   신선한 스트림이 오면 STREAM 으로 전이, 그 값이 write 소스가 된다
//   스테일   -> IDLE + **arm 도 off**. LOCKED 가 아니다 (정상 조작이므로 REARM 불필요)
//   arm on   IDLE 에서만. stop(off)은 언제나 허용 — 감속 방향이다 (01 §6.2)
//
// L2 만 쓴다 — 가짜 시계를 넣어 스테일을 결정론적으로 만든다 (A5 의 성과).

#include <gtest/gtest.h>

#include "orin_firmware_bridge/policy/rd_control.hpp"

namespace {

using orin_bridge::IClock;
using orin_bridge::RdControl;
using orin_bridge::RobotState_t;
using orin_bridge::StreamCmd_t;
using orin_bridge::ControlState;
namespace ecu = orin_bridge::ecu;

// 시간을 손으로 돌린다 — sleep 으로 스테일을 재현하면 느리고 흔들린다.
class FakeClock : public IClock {
public:
    std::chrono::steady_clock::time_point NowSteady() const override {
        return std::chrono::steady_clock::time_point{} + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::duration<double>(t_));
    }
    double NowEpoch() const override { return t_; }
    void Advance(double dt) { t_ += dt; }
private:
    double t_ = 1000.0;
};

class StreamArm : public ::testing::Test {
protected:
    void SetUp() override {
        st_ = std::make_unique<RobotState_t>();
        std::memset(&st_->ecu.reg, 0, sizeof(st_->ecu.reg));
        ctrl_.Bind(st_.get(), 30.0f);
        ctrl_.SetClock(&clk_);
        ctrl_.SetStreamTimeout(0.1);
        ctrl_.SetAutoMode(ecu::AUTO_MODE_CURRENT);
        ctrl_.MarkInitDone();
        ASSERT_EQ(ctrl_.State(), ControlState::IDLE);
    }
    StreamCmd_t Cmd(float amps) {
        StreamCmd_t c;
        for (int i = 0; i < 4; i++) c.current[i] = amps;
        c.stamp = clk_.NowEpoch();
        return c;
    }
    std::unique_ptr<RobotState_t> st_;
    FakeClock clk_;
    RdControl ctrl_;
};

// ★ 핵심 — arm 없이는 아무리 명령이 와도 소비하지 않는다.
TEST_F(StreamArm, DisarmedIgnoresStream) {
    ctrl_.PushStreamCommand(Cmd(5.0f));
    const auto w = ctrl_.SelectWrite();
    EXPECT_EQ(ctrl_.State(), ControlState::IDLE) << "arm 없이 STREAM 으로 갔다";
    for (int i = 0; i < 4; i++)
        EXPECT_FLOAT_EQ(w.current[i], 0.0f) << "arm 없이 스트림 값이 write 소스가 됐다";
}

TEST_F(StreamArm, ArmedFreshStreamBecomesWriteSource) {
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why)) << why;
    ctrl_.PushStreamCommand(Cmd(3.0f));

    const auto w = ctrl_.SelectWrite();
    EXPECT_EQ(ctrl_.State(), ControlState::STREAM);
    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(w.current[i], 3.0f);
}

// 스테일 -> IDLE **+ arm off**. 시스템이 "손 뗀 것" 을 기억하면 안 된다.
TEST_F(StreamArm, StaleFallsBackToIdleAndDisarms) {
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why));
    ctrl_.PushStreamCommand(Cmd(3.0f));
    ctrl_.SelectWrite();
    ASSERT_EQ(ctrl_.State(), ControlState::STREAM);

    clk_.Advance(0.15);                       // timeout 0.1s 초과
    const auto w = ctrl_.SelectWrite();
    EXPECT_EQ(ctrl_.State(), ControlState::IDLE) << "스테일인데 STREAM 에 머문다";
    EXPECT_FALSE(ctrl_.StreamArmed())           << "arm 이 켜진 채 남으면 다음 메시지에 갑자기 재개된다";
    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(w.current[i], 0.0f);
}

// LOCKED 가 아니라 IDLE 이다 — 슬라이더에서 손을 떼는 것은 **정상 조작**이라
// REARM 을 요구할 이유가 없다 (01 §6.1.3).
TEST_F(StreamArm, StaleDoesNotLock) {
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why));
    ctrl_.PushStreamCommand(Cmd(3.0f));
    ctrl_.SelectWrite();
    clk_.Advance(1.0);
    ctrl_.SelectWrite();
    EXPECT_NE(ctrl_.State(), ControlState::LOCKED);
}

TEST_F(StreamArm, ArmOnlyFromIdle) {
    ASSERT_TRUE(ctrl_.BeginProfile(1));           // IDLE -> RUNNING
    std::string why;
    EXPECT_FALSE(ctrl_.SetStreamArm(true, &why)) << "RUNNING 중에 arm 이 켜졌다";
    EXPECT_NE(why.find("IDLE"), std::string::npos) << "why=" << why;
}

// stop 은 언제나 허용 — 감속 방향이다 (01 §6.2).
TEST_F(StreamArm, DisarmAlwaysAllowed) {
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why));
    ctrl_.PushStreamCommand(Cmd(3.0f));
    ctrl_.SelectWrite();
    ASSERT_EQ(ctrl_.State(), ControlState::STREAM);

    EXPECT_TRUE(ctrl_.SetStreamArm(false, &why)) << "STREAM 중에 stop 이 거부됐다";
    EXPECT_EQ(ctrl_.State(), ControlState::IDLE);
    EXPECT_FALSE(ctrl_.StreamArmed());
}

// arm 하는 순간 남아 있던 옛 명령이 나가면 안 된다.
TEST_F(StreamArm, ArmDiscardsStaleCommandAtEntry) {
    ctrl_.PushStreamCommand(Cmd(9.0f));           // arm 전에 도착한 값
    clk_.Advance(5.0);                          // 한참 묵었다
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why));

    const auto w = ctrl_.SelectWrite();
    EXPECT_EQ(ctrl_.State(), ControlState::IDLE) << "묵은 명령으로 STREAM 에 진입했다";
    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(w.current[i], 0.0f);
}

// 단위는 auto_mode 가 정한다 (01 §6.1.2) — POSITION 이면 position 필드를 쓴다.
TEST_F(StreamArm, StreamValueFollowsAutoModeUnit) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_POSITION);
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why));

    StreamCmd_t c;
    for (int i = 0; i < 4; i++) { c.current[i] = 7.0f; c.position[i] = 42.0f; }
    c.stamp = clk_.NowEpoch();
    ctrl_.PushStreamCommand(c);

    const auto w = ctrl_.SelectWrite();
    ASSERT_EQ(ctrl_.State(), ControlState::STREAM);
    for (int i = 0; i < 4; i++)
        EXPECT_FLOAT_EQ(w.current[i], 42.0f) << "POSITION 인데 current 필드를 쓰고 있다";
}


// ===== SET_ORIGIN 펄스 + safe_stop (01 §6.3 C-1, §6.2) =====
//
// 펄스가 필요한 이유: SET_ORIGIN 은 **동작은 1회성인데 그것을 담는 그릇(ctr_mode)은
// 매 tick 전송되는 레벨 레지스터**다. "지속" 으로 두면 원점이 매 tick 다시 잡힌다.
// 2026-07-28 실기에서 펄스로 24.2도 -> 0.0도 확인.

class OriginPulse : public StreamArm {
protected:
    void SetUp() override {
        StreamArm::SetUp();
        ctrl_.SetAutoMode(ecu::AUTO_MODE_DIRECT);   // ctr_mode 가 bridge 소유여야 한다
        ctrl_.SetCtrMode(0, ecu::CTR_MODE_CURRENT);
        ctrl_.SetCtrMode(1, ecu::CTR_MODE_CURRENT);
    }
};

// ctr_mode 를 ECU 가 자가치유하는 모드에서는 펄스가 지워진다 — 애초에 거부한다.
TEST_F(OriginPulse, RejectedUnlessDirect) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_CURRENT);
    std::string why;
    EXPECT_FALSE(ctrl_.RequestOriginPulse(&why));
    EXPECT_NE(why.find("DIRECT"), std::string::npos) << "why=" << why;
}

// 딱 1 tick 만 나가고 원복된다.
TEST_F(OriginPulse, SentExactlyOneTickThenRestored) {
    std::string why;
    ASSERT_TRUE(ctrl_.RequestOriginPulse(&why)) << why;
    EXPECT_TRUE(ctrl_.OriginPulsePending());

    ctrl_.PrepareWrite();                                     // tick 1 — 펄스 전송
    EXPECT_EQ(st_->ecu.reg.cmd_motor.ctr_mode[0], ecu::CTR_MODE_SET_ORIGIN);

    ctrl_.PrepareWrite();                                     // tick 2 — 무조건 원복
    EXPECT_EQ(st_->ecu.reg.cmd_motor.ctr_mode[0], ecu::CTR_MODE_CURRENT)
        << "원복되지 않았다 — 원점이 매 tick 다시 잡힌다";
    EXPECT_FALSE(ctrl_.OriginPulsePending());

    ctrl_.PrepareWrite();                                     // tick 3 — 재전송 없음
    EXPECT_EQ(st_->ecu.reg.cmd_motor.ctr_mode[0], ecu::CTR_MODE_CURRENT);
}

TEST_F(OriginPulse, ConcurrentRequestRejected) {
    std::string why;
    ASSERT_TRUE(ctrl_.RequestOriginPulse(&why));
    EXPECT_FALSE(ctrl_.RequestOriginPulse(&why)) << "동시 2건이 수락됐다";
    EXPECT_NE(why.find("진행 중"), std::string::npos) << "why=" << why;
}

// safe_stop — 회전 중에는 설정 변경을 막는다 (01 §6.2).
// **상태만 보고 판정하면 관성으로 도는 중에 원점이 바뀐다.**
TEST_F(OriginPulse, RejectedWhileSpinning) {
    st_->ecu.reg.cmd_system.motor_mask = 0x01;
    st_->ecu.reg.motor_data.velocity[0] = 5;      // x10 = 50 RPM
    std::string why;
    EXPECT_FALSE(ctrl_.RequestOriginPulse(&why));
    EXPECT_NE(why.find("fb_velocity"), std::string::npos)
        << "위반 조건이 응답에 안 실렸다 — 원인 불명 실패를 만들면 안 된다. why=" << why;
}

TEST_F(OriginPulse, RejectedWhileTorqued) {
    st_->ecu.reg.cmd_system.motor_mask = 0x01;
    st_->ecu.reg.motor_data.current[0] = 150;     // x0.01 = 1.5 A
    std::string why;
    EXPECT_FALSE(ctrl_.RequestOriginPulse(&why));
    EXPECT_NE(why.find("fb_current"), std::string::npos) << "why=" << why;
}

// 비활성 모터는 보지 않는다 — 안 쓰는 모터 때문에 설정이 영영 막히면 안 된다.
TEST_F(OriginPulse, IgnoresInactiveMotors) {
    st_->ecu.reg.cmd_system.motor_mask = 0x01;    // M1 만 활성
    st_->ecu.reg.motor_data.velocity[2] = 100;    // M3 이 돌고 있어도
    std::string why;
    EXPECT_TRUE(ctrl_.RequestOriginPulse(&why)) << why;
}

TEST_F(OriginPulse, RejectedInStream) {
    std::string why;
    ASSERT_TRUE(ctrl_.SetStreamArm(true, &why));
    ctrl_.PushStreamCommand(Cmd(1.0f));
    ctrl_.SelectWrite();
    ASSERT_EQ(ctrl_.State(), ControlState::STREAM);
    EXPECT_FALSE(ctrl_.RequestOriginPulse(&why)) << "STREAM 중에 원점이 바뀐다";
}

}  // namespace
