// DIRECT 에서 **모터별 ctr_mode 가 단위를 정한다** (01 §6.1.2 표 마지막 행)
//
// 무엇을 막는가 — 구 코드는 명령 중(RUNNING/STREAM)이면 ctr_mode 를 CURRENT 로 강제하고
// 값을 단위와 무관하게 cmd_current 에 꽂았다:
//
//     const uint8_t mode = cmd_active ? ecu::CTR_MODE_CURRENT : ctr_mode_[i];
//
// 결과 두 가지가 실제로 도달 가능했다:
//   ① DIRECT + stream position/velocity -> **경고 없이 버려짐.** FSM 은 STREAM 으로 가고
//      모터는 안 움직여, 조작자가 "arm 이 안 먹었다" 와 구분할 수 없었다 (2026-07-28 실측).
//   ② DIRECT + `mode: position` 프로파일 -> **각도가 암페어로 나갔다.** AcceptsAutoMode 는
//      바로 이 조합을 수락한다(rd_profile.cpp: "DIRECT 에서는 모터별 ctr_mode 가 단위를
//      정한다"). 게다가 클램프가 clamp_prof 가 아니라 clamp_a([A]) 라 90도 샘플이
//      클램프 상한(기본 30A)까지 갔다. 9단계가 SetProfileLimits 로 고친 "클램프의 단위
//      의존"(05 §2.1)이 DIRECT 분기에만 남아 있던 것이다.
//
// L2 만 쓴다 — rclcpp 도 노드도 필요 없다.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>

#include "orin_firmware_bridge/rd_clock.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"

namespace {

using orin_bridge::IClock;
using orin_bridge::RdControl;
using orin_bridge::RobotState_t;
using orin_bridge::StreamCmd_t;
namespace ecu = orin_bridge::ecu;

constexpr int16_t kRaw416 = 416;      // fb_position = 41.6 deg (x0.1 스케일)
constexpr float   kDeg416 = 41.6f;

// 스테일 판정을 결정론적으로 — 실시계를 쓰면 스탬프가 즉시 낡아 STREAM 에 못 든다.
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

class DirectUnits : public ::testing::Test {
protected:
    void SetUp() override {
        st_ = std::make_unique<RobotState_t>();
        std::memset(&st_->ecu.reg, 0, sizeof(st_->ecu.reg));
        for (int i = 0; i < 4; i++) st_->ecu.reg.motor_data.position[i] = kRaw416;

        ctrl_.SetClock(&clk_);
        ctrl_.Bind(st_.get(), 30.0f);          // clamp_max_ = 30 [A]
        ctrl_.MarkInitDone();                  // INIT -> IDLE
        ctrl_.SetAutoMode(ecu::AUTO_MODE_DIRECT);
    }

    // STREAM 진입: arm 을 켜고 명령을 하나 밀어 넣는다.
    void EnterStream(const StreamCmd_t& c) {
        std::string why;
        ASSERT_TRUE(ctrl_.SetStreamArm(true, &why)) << why;
        ctrl_.PushStreamCommand(c);
    }

    const auto& Cmd() const { return st_->ecu.reg.cmd_motor; }

    std::unique_ptr<RobotState_t> st_;
    FakeClock clk_;
    RdControl ctrl_;
};

// ── ① 스트림: 모터별 ctr_mode 가 어느 필드를 읽을지 정한다 ──────────────────────
TEST_F(DirectUnits, StreamRoutesEachMotorByItsCtrMode) {
    ctrl_.SetCtrMode(0, ecu::CTR_MODE_CURRENT);
    ctrl_.SetCtrMode(1, ecu::CTR_MODE_VELOCITY);
    ctrl_.SetCtrMode(2, ecu::CTR_MODE_POSITION);
    ctrl_.SetCtrMode(3, ecu::CTR_MODE_ESTOP);

    StreamCmd_t c;
    for (int i = 0; i < 4; i++) {
        c.current[i]  = 7.0f;
        c.velocity[i] = 300.0f;
        c.position[i] = 25.0f;
    }
    c.stamp = clk_.NowEpoch();
    EnterStream(c);
    ctrl_.PrepareWrite();

    // M1 = CURRENT: 전류만 실린다
    EXPECT_FLOAT_EQ(Cmd().cmd_current[0],  7.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_velocity[0], 0.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_position[0], 0.0f);

    // M2 = VELOCITY: 구 코드에서는 이 값이 통째로 버려졌다
    EXPECT_FLOAT_EQ(Cmd().cmd_velocity[1], 300.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_current[1],  0.0f);

    // M3 = POSITION: 여기서도 마찬가지 — 그리고 25 가 cmd_current 로 새면 안 된다
    EXPECT_FLOAT_EQ(Cmd().cmd_position[2], 25.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_current[2],  0.0f);

    // M4 = ESTOP: 전 채널 0
    EXPECT_FLOAT_EQ(Cmd().cmd_current[3],  0.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_velocity[3], 0.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_position[3], 0.0f);

    // ctr_mode 는 조작자가 정한 값 그대로 나간다 (CURRENT 로 덮이지 않는다)
    EXPECT_EQ(Cmd().ctr_mode[1], ecu::CTR_MODE_VELOCITY);
    EXPECT_EQ(Cmd().ctr_mode[2], ecu::CTR_MODE_POSITION);
}

// ── ② 재생: position 샘플이 암페어로 새지 않는다 ────────────────────────────────
TEST_F(DirectUnits, PositionProfileNeverLeaksIntoCurrent) {
    for (int i = 0; i < 4; i++) ctrl_.SetCtrMode(i, ecu::CTR_MODE_POSITION);
    ctrl_.SetProfileLimits(-90.0f, 90.0f);          // position 프로파일의 실효 한계 [deg]
    ASSERT_TRUE(ctrl_.BeginProfile(1));

    const float sample[4] = {90.0f, 90.0f, 90.0f, 90.0f};
    ctrl_.SetProfileSample(sample, 0.0f, 0);
    ctrl_.PrepareWrite();

    for (int i = 0; i < 4; i++) {
        // 구 코드: cmd_current = clamp_a(90) = 30A **(클램프 상한 전부)**
        EXPECT_FLOAT_EQ(Cmd().cmd_current[i], 0.0f)
            << "M" << i + 1 << ": 각도가 암페어로 샜다";
        EXPECT_FLOAT_EQ(Cmd().cmd_position[i], 90.0f);
    }
}

// 프로파일 한계는 **단위를 따라간다** — position 이면 [deg] 로 자른다 (05 §2.4).
TEST_F(DirectUnits, ProfileLimitClampsInProfileUnitNotAmps) {
    for (int i = 0; i < 4; i++) ctrl_.SetCtrMode(i, ecu::CTR_MODE_POSITION);
    ctrl_.SetProfileLimits(-45.0f, 45.0f);
    ASSERT_TRUE(ctrl_.BeginProfile(1));

    const float sample[4] = {120.0f, 120.0f, 120.0f, 120.0f};
    ctrl_.SetProfileSample(sample, 0.0f, 0);
    ctrl_.PrepareWrite();

    // 45 [deg] 로 잘린다. clamp_a 였다면 30 이 나왔을 것이다 — 그 숫자가 나오면 회귀다.
    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(Cmd().cmd_position[i], 45.0f);
}

// 전류 모터는 전역 [A] 클램프를 그대로 받는다 (단위가 맞으므로).
TEST_F(DirectUnits, CurrentMotorStillHitsAmpClamp) {
    for (int i = 0; i < 4; i++) ctrl_.SetCtrMode(i, ecu::CTR_MODE_CURRENT);
    ASSERT_TRUE(ctrl_.BeginProfile(1));

    const float sample[4] = {999.0f, 999.0f, 999.0f, 999.0f};
    ctrl_.SetProfileSample(sample, 0.0f, 0);
    ctrl_.PrepareWrite();

    for (int i = 0; i < 4; i++) EXPECT_FLOAT_EQ(Cmd().cmd_current[i], 30.0f);
}

// ── IDLE 안전값은 바뀌지 않았다 (이번 수정의 비회귀 조건) ───────────────────────
TEST_F(DirectUnits, IdleSafeValuesUnchanged) {
    ctrl_.SetCtrMode(0, ecu::CTR_MODE_CURRENT);
    ctrl_.SetCtrMode(1, ecu::CTR_MODE_VELOCITY);
    ctrl_.SetCtrMode(2, ecu::CTR_MODE_POSITION);
    ctrl_.SetCtrMode(3, ecu::CTR_MODE_ESTOP);
    ctrl_.PrepareWrite();                       // IDLE 상태

    EXPECT_FLOAT_EQ(Cmd().cmd_current[0],  0.0f);
    EXPECT_FLOAT_EQ(Cmd().cmd_velocity[1], 0.0f);
    // POSITION 은 fb_position 재시드 — 0 을 쓰면 "원점으로 가라" 가 된다 (06 §4.7)
    EXPECT_FLOAT_EQ(Cmd().cmd_position[2], kDeg416);
    EXPECT_FLOAT_EQ(Cmd().cmd_current[3],  0.0f);
}

// SET_ORIGIN 펄스 tick 은 전 채널 0 이어야 한다 — 원점 잡는 중에 명령이 실리면 안 된다.
TEST_F(DirectUnits, OriginPulseTickCarriesNoCommand) {
    for (int i = 0; i < 4; i++) ctrl_.SetCtrMode(i, ecu::CTR_MODE_POSITION);
    std::string why;
    ASSERT_TRUE(ctrl_.RequestOriginPulse(&why)) << why;   // IDLE + safe_stop

    ctrl_.PrepareWrite();                                  // ARMED -> SENT
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(Cmd().ctr_mode[i], ecu::CTR_MODE_SET_ORIGIN);
        EXPECT_FLOAT_EQ(Cmd().cmd_current[i],  0.0f);
        EXPECT_FLOAT_EQ(Cmd().cmd_velocity[i], 0.0f);
        EXPECT_FLOAT_EQ(Cmd().cmd_position[i], 0.0f);
    }

    ctrl_.PrepareWrite();                                  // SENT -> 원복
    for (int i = 0; i < 4; i++) EXPECT_EQ(Cmd().ctr_mode[i], ecu::CTR_MODE_POSITION);
}

}  // namespace
