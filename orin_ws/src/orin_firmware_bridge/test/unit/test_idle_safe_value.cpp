// auto_mode 별 IDLE 안전값 (redesign/01 §6.1.2)
//
// 왜 이 테스트가 필요한가 — 2026-07-28 실기에서 실제로 벌어진 일이다 (06 §4.7):
//   auto_mode 를 5(POSITION)로 바꾸는 순간 모터가 **41.6도에서 0도로 슬루했다.**
//   cmd_position 이 0 이었기 때문인데, POSITION 범위에서 0 은 "정지" 가 아니라
//   **"원점으로 가라"** 는 명령이다. 초안이 안전값을 "0A write 유지" 라고 단위째
//   박아 둔 것이 그대로 위험이 됐다.
//
// 고정하는 계약:
//   CURRENT  IDLE -> 0
//   VELOCITY IDLE -> 0
//   POSITION IDLE -> **fb_position 재시드** (매 tick, latch 아님)
//   DIRECT   모터별 ctr_mode 를 따른다
//
// L2 만 쓴다 — rclcpp 도 노드도 필요 없다 (A5 의 성과가 여기서 값을 한다).

#include <gtest/gtest.h>

#include "orin_firmware_bridge/policy/rd_control.hpp"

namespace {

using orin_bridge::RdControl;
using orin_bridge::RobotState_t;
namespace ecu = orin_bridge::ecu;

// fb_position 은 x0.1 [deg] 스케일의 int16 이다.
constexpr int16_t kRaw416 = 416;      // = 41.6 deg — 실기에서 슬루가 일어난 그 값
constexpr float   kDeg416 = 41.6f;

class IdleSafeValue : public ::testing::Test {
protected:
    void SetUp() override {
        st_ = std::make_unique<RobotState_t>();
        std::memset(&st_->ecu.reg, 0, sizeof(st_->ecu.reg));
        for (int i = 0; i < 4; i++) st_->ecu.reg.motor_data.position[i] = kRaw416;

        ctrl_.Bind(st_.get(), 30.0f);
        ctrl_.MarkInitDone();          // INIT -> IDLE
        ASSERT_EQ(ctrl_.State(), orin_bridge::ControlState::IDLE);
    }
    std::unique_ptr<RobotState_t> st_;
    RdControl ctrl_;
};

TEST_F(IdleSafeValue, CurrentIdleIsZero) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_CURRENT);
    ctrl_.PrepareWrite();
    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_current[i], 0.0f);
}

TEST_F(IdleSafeValue, VelocityIdleIsZero) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_VELOCITY);
    ctrl_.PrepareWrite();
    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_velocity[i], 0.0f);
}

// ★ 핵심 — 0 이 아니라 현재 자세여야 한다.
TEST_F(IdleSafeValue, PositionIdleReseedsToFeedback) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_POSITION);
    ctrl_.PrepareWrite();
    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_position[i], kDeg416)
            << "m" << i << " 의 IDLE 명령이 fb_position 이 아니다 — 0 이면 원점으로 슬루한다";
    }
}

// 재시드는 **매 tick** 이다. latch 면 외력으로 밀린 뒤 run 을 누를 때 변위가 생긴다.
TEST_F(IdleSafeValue, PositionReseedTracksMovementEveryTick) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_POSITION);
    ctrl_.PrepareWrite();
    EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_position[0], kDeg416);

    // IDLE 중 외력으로 트랙이 밀렸다고 하자
    for (int i = 0; i < 4; i++) st_->ecu.reg.motor_data.position[i] = 900;   // 90.0 deg
    ctrl_.PrepareWrite();
    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_position[i], 90.0f)
            << "재시드가 latch 라 실측을 안 따라온다 — run 순간 변위가 생긴다";
    }
}

// auto_mode 전환 직후에도 즉시 안전해야 한다 (01 §3.3 shadow 소독을 재시드가 겸한다).
// cmd_position 은 그전까지 어느 프리셋에도 안 읽히던 자리라 낡은 값이 남아 있다.
TEST_F(IdleSafeValue, PositionReseedSanitizesStaleShadow) {
    for (int i = 0; i < 4; i++) st_->ecu.reg.cmd_motor.cmd_position[i] = -12345.0f;  // 낡은 값
    ctrl_.SetAutoMode(ecu::AUTO_MODE_POSITION);
    ctrl_.PrepareWrite();
    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_position[i], kDeg416)
            << "전환 직후 낡은 shadow 가 그대로 나간다";
    }
}

// DIRECT 는 모터별 ctr_mode 를 따른다 (01 §6.1.2 표 마지막 행).
TEST_F(IdleSafeValue, DirectAppliesRulePerMotor) {
    ctrl_.SetAutoMode(ecu::AUTO_MODE_DIRECT);
    ctrl_.SetCtrMode(0, ecu::CTR_MODE_CURRENT);
    ctrl_.SetCtrMode(1, ecu::CTR_MODE_POSITION);
    ctrl_.PrepareWrite();

    const auto& cm = st_->ecu.reg.cmd_motor;
    EXPECT_EQ(cm.ctr_mode[0], ecu::CTR_MODE_CURRENT);
    EXPECT_FLOAT_EQ(cm.cmd_current[0],  0.0f);
    EXPECT_FLOAT_EQ(cm.cmd_position[0], 0.0f) << "CURRENT 모터는 위치 명령이 0 이어야 한다";

    EXPECT_EQ(cm.ctr_mode[1], ecu::CTR_MODE_POSITION);
    EXPECT_FLOAT_EQ(cm.cmd_position[1], kDeg416) << "POSITION 모터만 재시드된다";
    EXPECT_FLOAT_EQ(cm.cmd_current[1],  0.0f);
}

// 05 §2.1 — 클램프는 [A] 단위다. RPM·deg 에 걸면 단위가 다른 값을 잘라낸다.
TEST_F(IdleSafeValue, ClampAppliesToCurrentOnly) {
    ctrl_.Bind(st_.get(), 0.5f);                    // cmd_current_max = 0.5 A
    ctrl_.SetAutoMode(ecu::AUTO_MODE_POSITION);
    ctrl_.PrepareWrite();
    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(st_->ecu.reg.cmd_motor.cmd_position[i], kDeg416)
            << "41.6 deg 가 0.5 로 잘렸다 — 전류 클램프가 각도에 걸리고 있다";
    }
}

}  // namespace
