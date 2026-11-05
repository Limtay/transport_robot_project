// INST_POS 좌표 추적 (redesign/12 §4, 11 §9)
//
// 고정하는 계약:
//   ① offset 은 seq 가 바뀐 응답에서 **딱 한 번만** 더한다 (flag 는 여러 번 보일 수 있다 — D5)
//   ② PEND=1 이고 seq 불변이면 abs 를 유지한다 (안 그러면 한 샘플 −3000° 로 튄다)
//   ③ 좌표를 믿을 수 없으면 잠근다: seq 건너뜀 / FRAME_LOST / RUNNING 중 리부트
//   ④ RdControl 이 cmd_rel = start_abs + s − offset 을 쓴다

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>

#include "orin_firmware_bridge/policy/rd_control.hpp"
#include "orin_firmware_bridge/policy/rd_pos_frame.hpp"

namespace {

using orin_bridge::ControlState;
using orin_bridge::RdControl;
using orin_bridge::RdPosFrame;
using orin_bridge::RobotState_t;
namespace ecu = orin_bridge::ecu;
using Verdict = RdPosFrame::Verdict;

constexpr uint8_t kAll = 0x0F;

// 정상 시퀀스의 응답 하나 (11 §5.6). tick 은 호출마다 자동으로 늘린다.
struct FbMaker {
    ecu::PAGE_POS_FB_t f{};
    FbMaker() { std::memset(&f, 0, sizeof(f)); f.realtime_tick = 100; }
    const ecu::PAGE_POS_FB_t& Next() { f.realtime_tick += 50; return f; }
};

TEST(PosFrame, FirstResponseSeedsWithZeroOffset) {
    RdPosFrame fr;
    FbMaker m;
    m.f.mtr_position[0] = 1234;
    m.f.origin_seq[0]   = 7;       // 브리지가 ECU 보다 늦게 떠도 seq 는 0 이 아닐 수 있다

    EXPECT_FALSE(fr.Initialized());
    EXPECT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);
    EXPECT_TRUE(fr.Initialized());
    EXPECT_DOUBLE_EQ(fr.Offset(0), 0.0);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 123.4);
    EXPECT_EQ(fr.OriginCnt(0), 0) << "기동 시점의 seq 는 이벤트가 아니다";
}

// ★ 정상 원점 시퀀스 전체 — abs 가 끊기지 않고 offset 이 한 번만 더해진다.
TEST(PosFrame, OriginEventAppliesOffsetExactlyOnce) {
    RdPosFrame fr;
    FbMaker m;
    m.f.mtr_position[0] = 30000;   // 3000.0°
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 3000.0);

    // k: ECU 가 S 래치, PEND=1, seq 불변. fb 는 이미 새 좌표로 보일 수 있다.
    m.f.origin_flag     = 0x01;
    m.f.mtr_position[0] = 12;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 3000.0) << "PEND 구간의 fb 를 쓰면 abs 가 −3000° 튄다";
    EXPECT_TRUE(fr.AnyHeld());

    // k+m: seq 변화. 여기서 offset 이 더해지고 abs 가 이어진다. PEND 는 아직 1.
    m.f.origin_seq[0]      = 1;
    m.f.pre_origin_pose[0] = 30000;
    m.f.mtr_position[0]    = 15;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Offset(0), 3000.0);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 3001.5);
    EXPECT_EQ(fr.OriginCnt(0), 1);
    EXPECT_FALSE(fr.AnyHeld()) << "seq 가 바뀐 응답부터 fb·offset 이 일치한다";

    // k+m+1: PEND=0. flag 가 또 보여도, 사라져도 다시 더하지 않는다.
    m.f.origin_flag     = 0x00;
    m.f.mtr_position[0] = 20;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Offset(0), 3000.0);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 3002.0);
    EXPECT_EQ(fr.OriginCnt(0), 1);
}

TEST(PosFrame, OffsetAccumulatesAcrossEvents) {
    RdPosFrame fr;
    FbMaker m;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    for (int e = 1; e <= 3; e++) {
        m.f.origin_seq[1]      = static_cast<uint8_t>(e);
        m.f.pre_origin_pose[1] = 30000;
        ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    }
    EXPECT_DOUBLE_EQ(fr.Offset(1), 9000.0) << "16bit fb 범위를 넘는 연속 이동이 이어져야 한다";
    EXPECT_EQ(fr.OriginCnt(1), 3);
    EXPECT_DOUBLE_EQ(fr.Offset(0), 0.0) << "모터별로 따로 센다";
}

// seq 는 mod 256 이다. 255→0 을 리부트로 읽으면 장시간 런이 멀쩡히 잠긴다.
TEST(PosFrame, SeqWrapIsNotAReboot) {
    RdPosFrame fr;
    FbMaker m;
    m.f.origin_seq[0] = 255;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    m.f.origin_seq[0]      = 0;
    m.f.pre_origin_pose[0] = 30000;
    EXPECT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Offset(0), 3000.0);
}

TEST(PosFrame, TickWrapIsNotAReboot) {
    RdPosFrame fr;
    FbMaker m;
    m.f.realtime_tick = 0xFFFFFFF0u;
    ASSERT_EQ(fr.Update(m.f, kAll, true, nullptr), Verdict::OK);
    m.f.realtime_tick = 0x00000010u;
    EXPECT_EQ(fr.Update(m.f, kAll, true, nullptr), Verdict::OK);
}

TEST(PosFrame, SeqSkipLocks) {
    RdPosFrame fr;
    FbMaker m;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    m.f.origin_seq[2] = 2;
    std::string why;
    EXPECT_EQ(fr.Update(m.Next(), kAll, true, &why), Verdict::LOCK);
    EXPECT_NE(why.find("M3"), std::string::npos) << why;
    EXPECT_TRUE(fr.NeedsReset());
    EXPECT_DOUBLE_EQ(fr.Offset(2), 0.0) << "모르는 이동량을 더하면 안 된다";
}

TEST(PosFrame, FrameLostLocksOnlyForActiveMotors) {
    RdPosFrame fr;
    FbMaker m;
    ASSERT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);

    m.f.origin_flag = 0x40;   // M3 FRAME_LOST
    EXPECT_EQ(fr.Update(m.Next(), /*M1,M2*/ 0x03, false, nullptr), Verdict::OK)
        << "비활성 모터의 FRAME_LOST 로 잠그지 않는다";

    std::string why;
    EXPECT_EQ(fr.Update(m.Next(), /*M3*/ 0x04, false, &why), Verdict::LOCK)
        << "FRAME_LOST 는 IDLE 에서도 잠근다 — ECU 가 제동 중이다";
    EXPECT_TRUE(fr.FrameLost());
    EXPECT_TRUE(fr.NeedsReset());
}

TEST(PosFrame, RebootLocksWhileRunningButReinitsInIdle) {
    {
        RdPosFrame fr;
        FbMaker m;
        m.f.realtime_tick = 100000;
        ASSERT_EQ(fr.Update(m.f, kAll, true, nullptr), Verdict::OK);
        m.f.realtime_tick = 10;
        EXPECT_EQ(fr.Update(m.f, kAll, true, nullptr), Verdict::LOCK);
        EXPECT_TRUE(fr.NeedsReset());
    }
    {
        RdPosFrame fr;
        FbMaker m;
        m.f.origin_seq[0]      = 5;
        m.f.pre_origin_pose[0] = 0;
        m.f.realtime_tick      = 100000;
        ASSERT_EQ(fr.Update(m.f, kAll, false, nullptr), Verdict::OK);
        m.f.origin_seq[0]   = 0;      // 리부트로 seq 리셋 (tick 은 아직 앞서 있다고 가정)
        m.f.realtime_tick   = 100050;
        m.f.mtr_position[0] = 50;
        EXPECT_EQ(fr.Update(m.f, kAll, false, nullptr), Verdict::REINIT);
        EXPECT_FALSE(fr.NeedsReset());
        EXPECT_DOUBLE_EQ(fr.Abs(0), 5.0);
    }
}

// 잠긴 뒤에는 낡은 offset 으로 abs 를 계산하지 않고, Reset 으로만 되살아난다.
TEST(PosFrame, StaysFrozenUntilReset) {
    RdPosFrame fr;
    FbMaker m;
    m.f.mtr_position[0] = 100;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    m.f.origin_seq[0] = 9;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::LOCK);

    m.f.mtr_position[0] = 999;
    EXPECT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 10.0);

    fr.Reset();
    EXPECT_FALSE(fr.NeedsReset());
    EXPECT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);
    EXPECT_DOUBLE_EQ(fr.Abs(0), 99.9);
    EXPECT_DOUBLE_EQ(fr.Offset(0), 0.0);
}

TEST(PosFrame, LatchStartRefusedDuringPend) {
    RdPosFrame fr;
    FbMaker m;
    std::string why;
    EXPECT_FALSE(fr.LatchStart(kAll, &why)) << "첫 응답 전에는 시작점을 모른다";

    m.f.mtr_position[1] = 200;
    ASSERT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);
    m.f.origin_flag = 0x02;
    ASSERT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);
    EXPECT_FALSE(fr.LatchStart(0x02, &why));
    EXPECT_TRUE(fr.LatchStart(0x01, &why)) << "비활성 모터의 PEND 는 막지 않는다";

    m.f.origin_flag = 0x00;
    ASSERT_EQ(fr.Update(m.Next(), kAll, false, nullptr), Verdict::OK);
    EXPECT_TRUE(fr.LatchStart(0x02, &why));
    EXPECT_DOUBLE_EQ(fr.StartAbs(1), 20.0);
}

TEST(PosFrame, CheckTrackRecordsMaxAndSkipsHeld) {
    RdPosFrame fr;
    FbMaker m;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    for (int i = 0; i < 4; i++) fr.SetCmdAbs(i, 0.0);

    fr.SetCmdAbs(0, 5.0);
    EXPECT_TRUE(fr.CheckTrack(30.0, kAll, nullptr));
    EXPECT_DOUBLE_EQ(fr.MaxTrackErr(0), 5.0);

    std::string why;
    fr.SetCmdAbs(0, 31.0);
    EXPECT_FALSE(fr.CheckTrack(30.0, kAll, &why));
    EXPECT_NE(why.find("M1"), std::string::npos) << why;
    EXPECT_DOUBLE_EQ(fr.MaxTrackErr(0), 31.0);

    EXPECT_TRUE(fr.CheckTrack(30.0, /*M2 만*/ 0x02, nullptr)) << "비활성 모터는 보지 않는다";

    m.f.origin_flag = 0x01;
    ASSERT_EQ(fr.Update(m.Next(), kAll, true, nullptr), Verdict::OK);
    fr.SetCmdAbs(0, 500.0);
    EXPECT_TRUE(fr.CheckTrack(30.0, kAll, nullptr)) << "PEND 로 유지된 abs 와 비교하면 오탐이다";
}

// ── RdControl 연결 (12 §4) ──────────────────────────────────────────────────
class PosControl : public ::testing::Test {
protected:
    void SetUp() override {
        st_ = std::make_unique<RobotState_t>();
        std::memset(&st_->ecu.reg, 0, sizeof(st_->ecu.reg));
        st_->ecu.reg.cmd_system.motor_mask = kAll;
        ctrl_.SetAutoMode(ecu::AUTO_MODE_POSITION);
        ctrl_.SetPositionInst(true);
        ctrl_.Bind(st_.get(), 30.0f);
        ctrl_.MarkInitDone();
        Fb().sys_state     = ecu::SYS_STATE_AUTO;
        Fb().realtime_tick = 100;
    }
    ecu::PAGE_POS_FB_t& Fb() { return st_->ecu.reg.pos_fb; }
    // 응답 1건 도착 = 스케줄러가 Decode 후 부르는 순서 그대로
    void Respond(int16_t fb0_raw) {
        Fb().realtime_tick += 50;
        Fb().mtr_position[0] = fb0_raw;
        ecu::MirrorPosFb(&st_->ecu.reg);
        ctrl_.OnPosFb();
    }
    float Cmd(int i) { return st_->ecu.reg.cmd_motor.cmd_position[i]; }

    std::unique_ptr<RobotState_t> st_;
    RdControl ctrl_;
};

TEST_F(PosControl, IdleReseedsFromFeedback) {
    Respond(416);
    ctrl_.PrepareWrite();
    EXPECT_FLOAT_EQ(Cmd(0), 41.6f) << "IDLE 재시드는 기존과 같다";
    EXPECT_TRUE(std::isnan(ctrl_.PosFrameSnapshot().CmdAbs(0))) << "IDLE 의 cmd_abs 는 NaN";
}

// ★ 3000° 통과: 전환 전후로 모터에 나가는 값이 절대 위치와 맞아야 한다.
TEST_F(PosControl, CommandIsStartPlusSampleMinusOffset) {
    Respond(100);    // 10.0°
    ctrl_.SetPosTrackLimit(1000.0);
    ASSERT_TRUE(ctrl_.BeginProfile(1));
    const float zero[4] = {0, 0, 0, 0};
    ctrl_.SetProfileSample(zero, 0.0f, 0);
    ctrl_.PrepareWrite();
    EXPECT_FLOAT_EQ(Cmd(0), 10.0f) << "변위 0 = 시작 위치 유지";

    const float s[4] = {3100.0f, 0, 0, 0};
    ctrl_.SetProfileSample(s, 1.0f, 0);
    ctrl_.PrepareWrite();
    EXPECT_FLOAT_EQ(Cmd(0), 3110.0f) << "seq 변화 전에는 옛 좌표 cmd (ECU 가 보정한다)";

    // seq 변화 응답 → 다음 tick 부터 새 좌표
    Fb().origin_seq[0]      = 1;
    Fb().pre_origin_pose[0] = 30000;
    Respond(1100);
    ctrl_.PrepareWrite();
    EXPECT_FLOAT_EQ(Cmd(0), 110.0f) << "cmd_rel = 10 + 3100 − 3000";
    EXPECT_EQ(ctrl_.State(), ControlState::RUNNING);
}

TEST_F(PosControl, TrackErrorLocksWhileRunningOnly) {
    Respond(0);
    ctrl_.SetPosTrackLimit(30.0);
    ASSERT_TRUE(ctrl_.BeginProfile(1));
    const float s[4] = {31.0f, 0, 0, 0};
    ctrl_.SetProfileSample(s, 0.1f, 0);
    ctrl_.PrepareWrite();
    Respond(0);
    EXPECT_EQ(ctrl_.State(), ControlState::LOCKED);
    EXPECT_NE(ctrl_.LockReason().find("추종 오차"), std::string::npos) << ctrl_.LockReason();
}

TEST_F(PosControl, LeavingAutoLocksWhileRunning) {
    Respond(0);
    ctrl_.SetPosTrackLimit(30.0);
    ASSERT_TRUE(ctrl_.BeginProfile(1));
    Fb().sys_state = 3;   // ESTOP_SW
    Respond(0);
    EXPECT_EQ(ctrl_.State(), ControlState::LOCKED)
        << "ECU 가 ESTOP 인데 플랜 시간만 흐르면 안 된다";
}

TEST_F(PosControl, BeginProfileRefusedDuringPend) {
    Respond(0);
    Fb().origin_flag = 0x01;
    Respond(10);
    EXPECT_FALSE(ctrl_.BeginProfile(1));
    EXPECT_EQ(ctrl_.State(), ControlState::IDLE);
    EXPECT_FALSE(ctrl_.BeginReject().empty());
}

TEST_F(PosControl, RearmResetsInvalidFrame) {
    Respond(100);
    Fb().origin_seq[0] = 5;
    Respond(100);
    ASSERT_EQ(ctrl_.State(), ControlState::LOCKED);
    ASSERT_TRUE(ctrl_.Rearm());
    EXPECT_FALSE(ctrl_.PosFrameSnapshot().Initialized());
    Respond(200);
    EXPECT_EQ(ctrl_.State(), ControlState::IDLE);
    EXPECT_DOUBLE_EQ(ctrl_.PosFrameSnapshot().Abs(0), 20.0);
}

// position_inst 가 꺼져 있으면 기존 경로와 완전히 같다.
TEST(PosControlOff, LegacyPositionPathUnchanged) {
    auto st = std::make_unique<RobotState_t>();
    std::memset(&st->ecu.reg, 0, sizeof(st->ecu.reg));
    st->ecu.reg.motor_data.position[0] = 416;
    RdControl c;
    c.SetAutoMode(ecu::AUTO_MODE_POSITION);
    c.Bind(st.get(), 30.0f);
    c.MarkInitDone();
    c.PrepareWrite();
    EXPECT_FLOAT_EQ(st->ecu.reg.cmd_motor.cmd_position[0], 41.6f);

    ASSERT_TRUE(c.BeginProfile(1));
    const float s[4] = {90.0f, 0, 0, 0};
    c.SetProfileSample(s, 0.0f, 0);
    c.PrepareWrite();
    EXPECT_FLOAT_EQ(st->ecu.reg.cmd_motor.cmd_position[0], 90.0f) << "RW 경로의 프로파일은 절대값이다";
}

}  // namespace
