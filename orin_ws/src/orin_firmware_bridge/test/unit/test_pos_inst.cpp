// INST_POS 전체 경로 — 스케줄러 ~ 프레이밍 ~ 가짜 ECU (redesign/12 §9)
//
// ## 왜 PTY 인가
//
// 기존 통합 픽스처는 tick 을 모사해서 `ExecuteTask` 와 프레이밍을 거치지 않는다. INST_POS 의
// 위험은 정확히 그 사이에 있다: 프로브 순서, RW 와 다른 Encode, 응답 scatter→미러→offset 순서.
// 그래서 시리얼 아래만 가상 터미널로 바꾸고 **그 위 코드는 전부 실제 것을 돌린다.**
//
// 가짜 ECU 는 rd_comm_ecu.c(INST_POS) 와 11 §5.3(원점 FSM) 을 옮긴 것이고, 모터의 **물리 위치 p**
// 를 프레임과 별개로 들고 있다. 그래서 이 테스트의 핵심 판정이 가능해진다:
//
//     원점이 옮겨져도 물리 위치는 한 tick 도 튀지 않는가.
//
// ⚠ 가짜 ECU 는 우리가 이해한 ECU 다. 와이어 호환은 실물 플래시 전까지 증명되지 않는다.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_uart.hpp"
#include "orin_firmware_bridge/policy/rd_command.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"
#include "orin_firmware_bridge/policy/rd_oos.hpp"
#include "orin_firmware_bridge/policy/rd_profile_player.hpp"
#include "orin_firmware_bridge/rd_config.hpp"
#include "orin_firmware_bridge/sched/rd_schedule.hpp"

using namespace std::chrono_literals;
using namespace orin_bridge;

namespace {

constexpr uint8_t kInstRead = 0x02, kInstWrite = 0x03, kInstPos = 0x05;
constexpr uint8_t kErrInst = 0x02, kErrDataRange = 0x04, kErrAccess = 0x07;
constexpr uint8_t kMaskM1M4 = 0x09;

// CRC-16/IBM (0x8005, MSB-first 처리). rd_comm.cpp 의 표를 쓰지 않는다 — 양쪽이 같은 표를
// 공유하면 표가 틀려도 통과한다.
uint16_t Crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= static_cast<uint16_t>(d[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x8005)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

class NullSink : public ITelemetrySink {
public:
    void OnTxn(const TxnTiming_t&) override {}
    void OnFeedback() override {}
    void OnRwErr(uint8_t) override {}
    void OnWriteErrTotal(uint64_t) override {}
    void OnIrregularTick() override {}
    void OnLateTick() override {}
    void SetReadPreset(const ReadPreset*) override {}
};

// 모터 1개 + 원점 FSM (11 §5.3 을 옮겼다). 단위는 전부 deg.
struct FakeMotor {
    double   p = 0.0;          // 물리 위치 — 원점과 무관하다
    double   origin = 0.0;     // 모터 프레임의 0 이 물리적으로 어디인가
    bool     armed = false, pending = false, confirming = false;
    int      confirm_left = 0;
    double   last_tx = 0.0, S = 0.0;
    int16_t  S_pub_raw = 0;
    uint8_t  seq = 0;
    double   max_step = 0.0;   // 물리 위치의 tick 당 최대 변화
    double Fb() const { return p - origin; }
};

class FakeEcu {
public:
    explicit FakeEcu(int master) : fd_(master) {
        std::memset(&reg_, 0, sizeof(reg_));
        reg_.cmd_system.motor_mask = 0x0F;
    }
    ~FakeEcu() { Stop(); }

    void Start() { run_ = true; th_ = std::thread([this] { Loop(); }); }
    void Stop() { run_ = false; if (th_.joinable()) th_.join(); }

    // 테스트 조작 — 전부 mtx_ 아래.
    std::mutex mtx_;
    FakeMotor  m_[4];
    bool old_firmware_ = false;   // 256B 맵, 0x05 미지원
    bool stall_[4] = {false, false, false, false};
    int  force_sys_state_ = -1;
    uint8_t frame_lost_ = 0;
    uint32_t jump_rej_ = 0;
    int pos_cnt_ = 0, write_cnt_ = 0, read_193_cnt_ = 0;
    uint8_t LinZeroReq() const { return reg_.cmd_system.lin_zero_req; }

private:
    void Loop() {
        std::vector<uint8_t> in;
        uint8_t buf[512];
        while (run_) {
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0) in.insert(in.end(), buf, buf + n);
            else std::this_thread::sleep_for(100us);
            Parse(&in);
        }
    }

    void Parse(std::vector<uint8_t>* in) {
        for (;;) {
            while (in->size() >= 2 && !((*in)[0] == HEADER1 && (*in)[1] == HEADER2))
                in->erase(in->begin());
            if (in->size() < HEADER_PART_LEN) return;
            const size_t len = (*in)[3] | ((*in)[4] << 8);
            const size_t total = HEADER_PART_LEN + len;
            if (in->size() < total) return;
            const uint16_t rx_crc = (*in)[total - 2] | ((*in)[total - 1] << 8);
            if (rx_crc == Crc16(in->data(), total - 2)) {
                Handle((*in)[5], in->data() + 6, len - 3);
            } else {
                ADD_FAILURE() << "요청 CRC 불일치 — 프레이밍이 어긋났다";
            }
            in->erase(in->begin(), in->begin() + total);
        }
    }

    void Reply(uint8_t inst, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> f = {HEADER1, HEADER2, static_cast<uint8_t>(PacketID::ORIN)};
        const uint16_t len = static_cast<uint16_t>(data.size() + 3);
        f.push_back(len & 0xFF);
        f.push_back(len >> 8);
        f.push_back(inst);
        f.insert(f.end(), data.begin(), data.end());
        const uint16_t crc = Crc16(f.data(), f.size());
        f.push_back(crc & 0xFF);
        f.push_back(crc >> 8);
        (void)!::write(fd_, f.data(), f.size());
    }

    void Handle(uint8_t inst, const uint8_t* d, size_t n) {
        std::lock_guard<std::mutex> l(mtx_);
        auto* raw = reinterpret_cast<uint8_t*>(&reg_);
        const size_t total = old_firmware_ ? 256u : sizeof(reg_);
        if (inst == kInstRead) {
            for (size_t i = 0; i + 4 <= n; i += 4) {
                const size_t a = d[i] | (d[i + 1] << 8), m = d[i + 2] | (d[i + 3] << 8);
                if (a + m > total) { Reply(inst, {kErrDataRange}); return; }
                if (a == ecu::REG_LIN_ZERO_REQ_OFFSET) read_193_cnt_++;
            }
            PublishBase();
            std::vector<uint8_t> out = {0};
            for (size_t i = 0; i + 4 <= n; i += 4) {
                const size_t a = d[i] | (d[i + 1] << 8), m = d[i + 2] | (d[i + 3] << 8);
                out.insert(out.end(), raw + a, raw + a + m);
            }
            Reply(inst, out);
        } else if (inst == kInstWrite) {
            const size_t a = d[0] | (d[1] << 8);
            std::memcpy(raw + a, d + 2, n - 2);
            write_cnt_++;
            Reply(inst, {0});
        } else if (inst == kInstPos && !old_firmware_) {
            HandlePos(d, n);
        } else {
            Reply(inst, {kErrInst});
        }
    }

    // rd_comm_ecu.c PACKET_INST_POS 와 같은 순서.
    void HandlePos(const uint8_t* d, size_t n) {
        pos_cnt_++;
        uint8_t err_wr = 0;
        const size_t waddr = d[0] | (d[1] << 8);
        if (waddr != ecu::REG_CMD_POSITION_OFFSET || n - 2 != ecu::REG_CMD_VALUE_SIZE) {
            err_wr = 0x05;
        } else if (reg_.cmd_system.auto_mode != ecu::AUTO_MODE_POSITION) {
            err_wr = kErrAccess;
        } else {
            std::memcpy(reg_.cmd_motor.cmd_position, d + 2, ecu::REG_CMD_VALUE_SIZE);
            for (int i = 0; i < 4; i++) {
                if ((reg_.cmd_system.motor_mask >> i) & 0x01)
                    StepMotor(i, reg_.cmd_motor.cmd_position[i]);
            }
        }
        tick_ += 50;
        PublishBase();
        auto& f = reg_.pos_fb;
        std::memset(&f, 0, sizeof(f));
        f.realtime_tick = tick_;
        f.sys_state     = reg_.sys.sys_state;
        f.origin_flag   = frame_lost_;
        for (int i = 0; i < 4; i++) {
            f.mtr_position[i]    = static_cast<int16_t>(std::lround(m_[i].Fb() * 10.0));
            f.pre_origin_pose[i] = m_[i].S_pub_raw;
            f.origin_seq[i]      = m_[i].seq;
            if (m_[i].pending) f.origin_flag |= static_cast<uint8_t>(1u << i);
        }
        f.loadcell_state.raw = 0x02;
        f.motor_state.raw    = 0x02;
        f.rls_ew             = 0xAA;   // 전 채널 Error — 실물 RLS 없음
        std::vector<uint8_t> out = {static_cast<uint8_t>(err_wr << 4)};
        const auto* p = reinterpret_cast<const uint8_t*>(&f);
        out.insert(out.end(), p, p + sizeof(f));
        Reply(kInstPos, out);
    }

    // 11 §5.3 [0]~[6]. 판정 기준은 fb 가 아니라 last_tx (D4).
    void StepMotor(int i, double c) {
        FakeMotor& m = m_[i];
        const double fb = m.Fb();
        if (m.confirming && --m.confirm_left <= 0) {   // [0] 확인 — 새 프레임 fb 를 봤다
            m.S_pub_raw  = static_cast<int16_t>(std::lround(m.S * 10.0));
            m.seq++;
            m.confirming = false;
        }
        if (!m.armed) {                                // [3] 시드
            if (std::fabs(c - fb) > 1000.0) { c = fb; jump_rej_++; }
            m.armed = true;
        } else {                                       // [4] 보정·점프 가드
            if (m.pending && std::fabs(c - m.last_tx) > 1000.0) c -= m.S;
            else if (m.pending) m.pending = false;
            if (std::fabs(c - m.last_tx) > 1000.0) { c = m.last_tx; jump_rej_++; }
        }
        if (!m.pending && !m.confirming && std::fabs(fb) >= 3000.0) {   // [5] 트리거
            m.S = fb;
            m.origin += fb;       // SET_ORIGIN: 지금 물리 위치가 새 0 이다
            m.pending = m.confirming = true;
            m.confirm_left = 2;   // fb 100Hz 라 수 tick 뒤에 확인된다
            c -= m.S;
        }
        m.last_tx = c;
        if (!stall_[i]) {
            const double next = m.origin + c;
            m.max_step = std::max(m.max_step, std::fabs(next - m.p));
            m.p = next;
        }
    }

    void PublishBase() {
        reg_.sys.sys_state = force_sys_state_ >= 0 ? static_cast<uint8_t>(force_sys_state_)
                           : reg_.cmd_system.mode == ecu::MODE_AUTO ? ecu::SYS_STATE_AUTO : 1;
        reg_.sys.realtime_tick = tick_;
        for (int i = 0; i < 4; i++)
            reg_.motor_data.position[i] = static_cast<int16_t>(std::lround(m_[i].Fb() * 10.0));
    }

    int fd_;
    std::atomic<bool> run_{false};
    std::thread th_;
    ecu::REGISTER_t reg_;
    uint32_t tick_ = 1000;
};

// 실제 스택 전체. RdNode(ROS) 만 빠지고 RdControlApi 의 goal 수락 순서는 Run() 이 따른다.
class PosInstStack : public ::testing::Test {
protected:
    void SetUp() override {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        ASSERT_GE(master_, 0);
        ASSERT_EQ(::grantpt(master_), 0);
        ASSERT_EQ(::unlockpt(master_), 0);
        const std::string slave = ::ptsname(master_);
        termios t{};
        ASSERT_EQ(::tcgetattr(master_, &t), 0);
        ::cfmakeraw(&t);
        ASSERT_EQ(::tcsetattr(master_, TCSANOW, &t), 0);

        ecu_  = std::make_unique<FakeEcu>(master_);
        uart_ = std::make_unique<RdUart>(slave);
        comm_ = std::make_unique<RdComm>(uart_.get());
        state_ = std::make_unique<RobotState_t>();
        std::memset(&state_->ecu.reg, 0, sizeof(state_->ecu.reg));
        command_ = std::make_unique<RdCommand>(state_.get());

        cfg_.bridge_mode        = BridgeMode::CONTROL;
        cfg_.auto_mode_param    = ecu::AUTO_MODE_POSITION;
        cfg_.position_inst      = true;
        cfg_.active_motor_mask  = kMaskM1M4;
        control_.SetAutoMode(ecu::AUTO_MODE_POSITION);
        control_.SetPositionInst(true);
        control_.Bind(state_.get(), 30.0f);

        sched_ = std::make_unique<RdSchedule>(
            comm_.get(), &map_, state_.get(), &sink_, command_.get(), cfg_, &oos_, &player_,
            &control_, [this](bool) { init_done_ = true; });
    }

    void TearDown() override {
        StopStack();
        // ⚠ 슬레이브 쪽을 먼저 닫는다. 마스터가 먼저 닫히면 SerialPort 소멸자가 I/O 에러로 던진다.
        sched_.reset();
        comm_.reset();
        uart_.reset();
        ecu_.reset();
        if (master_ >= 0) ::close(master_);
    }

    void StartStack() {
        ecu_->Start();
        loop_ = std::thread([this] { exit_code_ = sched_->MainLoopStart(); finished_ = true; });
    }
    void StopStack() {
        if (sched_) sched_->Stop();
        if (loop_.joinable()) loop_.join();
        if (ecu_) ecu_->Stop();
    }
    template <typename F>
    bool WaitFor(F f, std::chrono::milliseconds limit) {
        const auto end = std::chrono::steady_clock::now() + limit;
        while (std::chrono::steady_clock::now() < end) {
            if (f()) return true;
            std::this_thread::sleep_for(5ms);
        }
        return f();
    }
    // 기동 + 첫 INST_POS 응답까지.
    void BringUp() {
        StartStack();
        ASSERT_TRUE(WaitFor([&] { return init_done_.load(); }, 5s)) << "INIT 미완료";
        ASSERT_TRUE(WaitFor([&] { return control_.PosFrameSnapshot().Initialized(); }, 5s))
            << "첫 INST_POS 응답이 없다";
    }
    // RdControlApi::HandleProfileGoal + ExecuteProfile 의 수락 순서.
    bool Play(const std::string& yaml) {
        std::string err;
        EXPECT_TRUE(player_.Load(yaml, kMaskM1M4, &err)) << err;
        const uint8_t ctr[4] = {4, 4, 4, 4};
        EXPECT_TRUE(player_.AcceptsAutoMode(ecu::AUTO_MODE_POSITION, ctr, kMaskM1M4, &err)) << err;
        EXPECT_TRUE(player_.AcceptsPositionInst(true, kMaskM1M4, &err)) << err;
        control_.SetPosTrackLimit(player_.MaxTrackErr());
        float lo = 0, hi = 0;
        player_.EffectiveLimits(&lo, &hi);
        control_.SetProfileLimits(lo, hi);
        const uint32_t gid = player_.Begin();
        if (!control_.BeginProfile(gid)) return false;
        player_.Activate();
        return true;
    }
    bool PlayerDone() {
        size_t tick = 0; bool done = false;
        player_.Progress(&tick, &done);
        return done || tick >= player_.TickCount();
    }

    int master_ = -1;
    std::unique_ptr<FakeEcu> ecu_;
    std::unique_ptr<RdUart>  uart_;
    std::unique_ptr<RdComm>  comm_;
    std::unique_ptr<RobotState_t> state_;
    std::unique_ptr<RdCommand> command_;
    RdMap map_;
    NullSink sink_;
    RdOos oos_;
    RdProfilePlayer player_;
    RdControl control_;
    BridgeConfig cfg_;
    std::unique_ptr<RdSchedule> sched_;
    std::thread loop_;
    std::atomic<bool> init_done_{false}, finished_{false};
    std::atomic<int>  exit_code_{-1};
};

// ★ 핵심 — ±3000° 를 여러 번 지나도 물리 위치가 튀지 않고 절대 좌표가 이어진다.
TEST_F(PosInstStack, CrossesOriginWithoutJumpAndKeepsAbsolute) {
    {
        std::lock_guard<std::mutex> l(ecu_->mtx_);
        ecu_->m_[0].p = 100.0;     // 시작 위치가 0 이 아니어도 변위로 동작해야 한다
        ecu_->m_[3].p = -50.0;
    }
    BringUp();
    ASSERT_TRUE(Play(R"(
mode: position
position_ref: start
limits: {range: [-8000, 8000], max_track_err: 30.0}
motors:
  m1:
    - {type: ramp, duration: 4.0, from: 0, to: 7000}
  m4:
    - {type: ramp, duration: 4.0, from: 0, to: -3500}
)"));
    ASSERT_TRUE(WaitFor([&] { return PlayerDone(); }, 10s));
    std::this_thread::sleep_for(50ms);

    EXPECT_EQ(control_.State(), ControlState::RUNNING) << control_.LockReason();
    const RdPosFrame fr = control_.PosFrameSnapshot();
    EXPECT_EQ(fr.OriginCnt(0), 2) << "M1 은 3000° 를 두 번 지난다";
    EXPECT_EQ(fr.OriginCnt(3), 1) << "M4 는 음의 방향으로 한 번";
    EXPECT_NEAR(fr.Abs(0), 100.0 + 7000.0, 0.5);
    EXPECT_NEAR(fr.Abs(3), -50.0 - 3500.0, 0.5);
    EXPECT_LT(fr.MaxTrackErr(0), 30.0);

    std::lock_guard<std::mutex> l(ecu_->mtx_);
    // 램프 1750 deg/s = tick 당 8.75 deg. 지터로 tick 이 밀려도 몇 배를 넘으면 점프다.
    EXPECT_LT(ecu_->m_[0].max_step, 40.0) << "원점 통과에서 물리 위치가 튀었다";
    EXPECT_LT(ecu_->m_[3].max_step, 40.0);
    EXPECT_EQ(ecu_->jump_rej_, 0u) << "점프 가드가 걸렸다 — Orin 이 어긋난 프레임으로 보냈다";
    EXPECT_NEAR(ecu_->m_[0].p, 7100.0, 0.5);
    EXPECT_DOUBLE_EQ(ecu_->m_[1].p, 0.0) << "비활성 모터는 안 움직인다";
    EXPECT_GT(ecu_->pos_cnt_, 500);
}

// IDLE 재시드: 명령이 실측을 따라가므로 가만히 있어야 한다.
TEST_F(PosInstStack, IdleHoldsPosition) {
    {
        std::lock_guard<std::mutex> l(ecu_->mtx_);
        ecu_->m_[0].p = 123.4;
    }
    BringUp();
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(control_.State(), ControlState::IDLE);
    std::lock_guard<std::mutex> l(ecu_->mtx_);
    EXPECT_NEAR(ecu_->m_[0].p, 123.4, 0.11);
    EXPECT_GT(ecu_->pos_cnt_, 10);
    EXPECT_EQ(ecu_->m_[0].seq, 0);
}

// 구 펌웨어: 0x05 를 한 번도 보내지 않고, 아무것도 쓰지 않고 exit≠0.
TEST_F(PosInstStack, OldFirmwareIsRefusedAtTheProbe) {
    ecu_->old_firmware_ = true;
    StartStack();
    ASSERT_TRUE(WaitFor([&] { return finished_.load(); }, 8s));
    EXPECT_EQ(exit_code_.load(), 1);
    EXPECT_FALSE(init_done_.load());
    std::lock_guard<std::mutex> l(ecu_->mtx_);
    EXPECT_EQ(ecu_->pos_cnt_, 0) << "구 펌웨어에 0x05 를 보내면 에러 바이트가 어긋난다";
    EXPECT_EQ(ecu_->write_cnt_, 0) << "프로브 전에 ECU 설정을 쓰면 안 된다";
}

// 걸림: 무인 실험이 스스로 멈춘다 (12 Q5).
TEST_F(PosInstStack, StallLocksOnTrackError) {
    BringUp();
    {
        std::lock_guard<std::mutex> l(ecu_->mtx_);
        ecu_->stall_[0] = true;
    }
    ASSERT_TRUE(Play(R"(
mode: position
position_ref: start
limits: {range: [-1000, 1000], max_track_err: 30.0}
motors:
  m1:
    - {type: ramp, duration: 1.0, from: 0, to: 500}
)"));
    ASSERT_TRUE(WaitFor([&] { return control_.State() == ControlState::LOCKED; }, 3s));
    EXPECT_NE(control_.LockReason().find("추종 오차"), std::string::npos) << control_.LockReason();
    EXPECT_GT(control_.PosFrameSnapshot().MaxTrackErr(0), 30.0);
}

// ECU 가 ESTOP 으로 빠졌는데 플랜 시간만 흐르면 안 된다.
TEST_F(PosInstStack, EcuLeavingAutoLocks) {
    BringUp();
    ASSERT_TRUE(Play(R"(
mode: position
position_ref: start
limits: {range: [-1000, 1000], max_track_err: 30.0}
motors:
  m1:
    - {type: hold, duration: 5.0, value: 0}
)"));
    {
        std::lock_guard<std::mutex> l(ecu_->mtx_);
        ecu_->force_sys_state_ = 3;   // ESTOP_SW
    }
    ASSERT_TRUE(WaitFor([&] { return control_.State() == ControlState::LOCKED; }, 2s));
    EXPECT_NE(control_.LockReason().find("AUTO"), std::string::npos) << control_.LockReason();
}

// lin_zero 는 ECU 가 곧바로 지우므로 read-back 하면 항상 불일치로 실패한다 (12 §7).
TEST_F(PosInstStack, LinZeroIsASingleWriteWithoutReadBack) {
    BringUp();
    int writes_before = 0;
    {
        std::lock_guard<std::mutex> l(ecu_->mtx_);
        writes_before = ecu_->write_cnt_;
    }
    std::string msg;
    ASSERT_TRUE(oos_.Request(ecu::REG_LIN_ZERO_REQ_OFFSET, 1, &msg, /*verify=*/false)) << msg;
    std::lock_guard<std::mutex> l(ecu_->mtx_);
    EXPECT_EQ(ecu_->LinZeroReq(), 1);
    EXPECT_EQ(ecu_->write_cnt_, writes_before + 1) << "엣지 명령은 한 번만 나가야 한다";
    EXPECT_EQ(ecu_->read_193_cnt_, 0);
}

TEST_F(PosInstStack, FrameLostLocksAndAsksForEcuRearm) {
    BringUp();
    {
        std::lock_guard<std::mutex> l(ecu_->mtx_);
        ecu_->frame_lost_ = 0x10;   // M1 FRAME_LOST
    }
    ASSERT_TRUE(WaitFor([&] { return control_.State() == ControlState::LOCKED; }, 2s));
    EXPECT_NE(control_.LockReason().find("FRAME_LOST"), std::string::npos);
    EXPECT_TRUE(control_.PosFrameLost());
}

}  // namespace
