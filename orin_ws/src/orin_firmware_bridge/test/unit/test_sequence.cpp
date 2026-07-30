// RdSequence — jeongae 전개 FSM (redesign/02 A3 / 04 §6.3)
//
// 분리의 값어치가 여기서 드러난다: **가짜 슬롯 호스트**만 있으면 시퀀스를 통째로 돌릴 수 있다.
// 종전에는 이 로직이 rd_command 안에 있어서, 시퀀스를 검사하려면 슬롯 관리자(우선순위·
// 차용·blackout·만료)를 전부 함께 띄워야 했다.
//
// ## 2026-07-30 — DPC 레지스터 확정으로 계약이 바뀌었다
//
// 종전 이 파일은 *"DPC 단계는 통과(TODO)하는 것이 현재 계약"* 이라고 적고 그것을 고정했다.
// **그 예고대로 확정 시점에 이 테스트가 먼저 깨졌고**, 무엇을 바꿔야 하는지가 드러났다.
// 지금 계약은 "DPC 를 실제로 몰고 상태를 기다린다" 이다.
//
// 2026-07-30 오후 — 펌웨어 헤더 `rd_register_dpcb.h` 로 **DPCB_STATE_e 가 확정**됐다.
// 값을 주입받던 `DeployTargets` 는 없앴다: 값이 정해진 이상 두 곳에 두면 갈라진다.
//
//   CTRL ──target=INIT──▶ INIT ▶ DESCEND_1 ▶ DESCEND_2 ▶ WAIT(★카메라)
//                                                          │ target=ASCEND_1
//   CTRL ◀─자동복귀─ FINISH ◀ ASCEND_2 ◀ ASCEND_1 ◀─────────┘
//
// 브리지가 쓰는 것은 **두 번뿐**이다 (INIT / ASCEND_1). 나머지는 DPC 가 스스로 밟는다.

#include <gtest/gtest.h>

#include <vector>

#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_register_dpc.hpp"
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"
#include "orin_firmware_bridge/policy/rd_sequence.hpp"

namespace {

using orin_bridge::ISlotHost;
using orin_bridge::RdSequence;
using Seq = RdSequence::Seq;
namespace ecu = orin_bridge::ecu;
namespace dpc = orin_bridge::dpc;
namespace TARGET = orin_bridge::TARGET;


// 슬롯을 흉내낸다 — 명령을 기록하고, 결과를 테스트가 직접 정한다.
class FakeSlotHost : public ISlotHost {
public:
    struct Write { uint8_t target; uint16_t addr; uint8_t value; };

    bool PostAutoWriteTo(uint8_t target, uint16_t addr, uint8_t value) override {
        if (!accept_) return false;
        writes.push_back({target, addr, value});
        done_ = false;
        return true;
    }
    bool AutoCommandDone(bool* ok) const override {
        if (!done_) return false;
        if (ok) *ok = ok_;
        return true;
    }
    void SetCmdVelPaused(bool p) override { paused = p; }
    bool IsCmdVelPaused() const override  { return paused; }

    bool DpcSysState(uint8_t* out) const override {
        if (!dpc_readable) return false;      // 아직 한 번도 안 읽혔다
        if (out) *out = dpc_state;
        return true;
    }

    void Complete(bool ok) { done_ = true; ok_ = ok; }

    // Abort 는 **정상적으로** soft ESTOP 해제를 낸다. "전개 명령이 나갔는가" 를 보려면
    // 전체 write 수가 아니라 **DPC 로 간 write** 를 세야 한다.
    size_t DpcWrites() const {
        size_t n = 0;
        for (const auto& w : writes) if (w.target == TARGET::DPC) n++;
        return n;
    }

    std::vector<Write> writes;
    bool    paused       = false;
    bool    accept_      = true;
    bool    dpc_readable = true;
    uint8_t dpc_state    = dpc::STATE_CTRL;

private:
    bool done_ = false;
    bool ok_   = false;
};

class SequenceTest : public ::testing::Test {
protected:
    void SetUp() override { seq_ = std::make_unique<RdSequence>(&host_); }
    void TickN(int n) { for (int i = 0; i < n; i++) seq_->Tick(); }

    // ESTOP 성공까지 — 여러 테스트의 공통 도입부.
    void ArriveAtDpcCheck() {
        seq_->Trigger(); seq_->Tick();
        host_.Complete(true); seq_->Tick();
        ASSERT_EQ(seq_->State(), Seq::DPC_STATE_CHECK);
    }

    FakeSlotHost host_;
    std::unique_ptr<RdSequence> seq_;
};

TEST_F(SequenceTest, IdleUntilTriggered) {
    TickN(5);
    EXPECT_EQ(seq_->State(), Seq::IDLE);
    EXPECT_TRUE(host_.writes.empty());
    EXPECT_FALSE(seq_->Busy());
}

// ★ 전개는 **soft ESTOP 부터** 시작한다 — 몸체가 움직이는 중에 전개판이 나가면 안 된다.
TEST_F(SequenceTest, TriggerStartsWithSoftEstop) {
    seq_->Trigger();
    seq_->Tick();
    ASSERT_EQ(host_.writes.size(), 1u);
    EXPECT_EQ(host_.writes[0].target, TARGET::ECU);
    EXPECT_EQ(host_.writes[0].addr,   ecu::REG_SOFT_ESTOP_OFFSET);
    EXPECT_EQ(host_.writes[0].value,  ecu::SOFT_ESTOP_ACTIVE);
    EXPECT_EQ(seq_->State(), Seq::ESTOP_SET);
    EXPECT_FALSE(host_.paused) << "ESTOP 이 **성공하기 전에** cmd_vel 을 멈추면 안 된다";
}

TEST_F(SequenceTest, PausesCmdVelOnlyAfterEstopConfirmed) {
    seq_->Trigger(); seq_->Tick();
    seq_->Tick();                       // 아직 미완료
    EXPECT_FALSE(host_.paused);
    host_.Complete(true);
    seq_->Tick();
    EXPECT_TRUE(host_.paused);
    EXPECT_EQ(seq_->State(), Seq::DPC_STATE_CHECK);
}

// ★ ESTOP 이 실패하면 전개로 넘어가지 않는다 — 그대로 진행하면 제동 없이 전개된다.
TEST_F(SequenceTest, AbortsWhenEstopFails) {
    seq_->Trigger(); seq_->Tick();
    host_.Complete(false);
    seq_->Tick();
    EXPECT_FALSE(host_.paused) << "실패했는데 cmd_vel 을 멈췄다";
    EXPECT_EQ(seq_->State(), Seq::IDLE) << "ESTOP 미적용 상태였으므로 해제 없이 종료";
    EXPECT_TRUE(seq_->GetLock()) << "실패 후 재트리거를 막아야 한다";
}

// ─────────────────────── DPC 구간 (2026-07-30 신설) ───────────────────────

// ★ **Orin 이 쓸 수 있는 상태는 넷뿐이다** — CTRL(0)/HOLD(1)/INIT(2)/ASCEND_1(6).
//    나머지(DESCEND_*, WAIT, ASCEND_2, FINISH)는 DPC 가 스스로 밟는 중간 상태라,
//    밖에서 쓰면 단계를 건너뛴다. 시퀀스가 내는 write 는 전부 그 mask 안이어야 한다.
TEST_F(SequenceTest, OnlyWritesTargetsTheFirmwareAllows) {
    ArriveAtDpcCheck();
    seq_->Tick();                                   // 전개 요청
    host_.Complete(true); seq_->Tick();
    host_.dpc_state = dpc::STATE_WAIT; seq_->Tick();
    seq_->Tick();                                   // 회수 요청
    ASSERT_GE(host_.DpcWrites(), 2u);
    for (const auto& w : host_.writes) {
        if (w.target != TARGET::DPC) continue;
        EXPECT_EQ(w.addr, dpc::REG_SYS_STATE_TARGET_OFFSET);
        EXPECT_TRUE(dpc::IsWritableTarget(w.value))
            << "펌웨어가 허용하지 않는 상태 " << int(w.value) << "("
            << dpc::SysStateName(w.value) << ") 를 썼다 — FSM 단계를 건너뛴다";
    }
}

// 전개 시작은 **INIT(2)** 이다 — 그 뒤 DESCEND_1/2 는 DPC 가 스스로 밟는다.
TEST_F(SequenceTest, DeployEntersFsmWithInit) {
    ArriveAtDpcCheck();
    seq_->Tick();
    ASSERT_EQ(host_.writes.size(), 2u);
    EXPECT_EQ(host_.writes[1].value, dpc::STATE_INIT);
}

// ★ DPC 를 한 번도 못 읽었으면 **도달했다고 판정하지 않는다.**
//    섀도 초기값 0 은 `CTRL`(정상값)이라 값만으로는 미판독과 구분되지 않는다.
TEST_F(SequenceTest, UnreadDpcIsNotTreatedAsReady) {
    host_.dpc_readable = false;
    ArriveAtDpcCheck();
    TickN(10);
    EXPECT_EQ(host_.DpcWrites(), 0u)
        << "읽은 적도 없는 섀도의 0(CTRL)을 보고 전개를 시작했다";
    EXPECT_EQ(seq_->State(), Seq::DPC_STATE_CHECK) << "아직 기다리는 중이어야 한다";
}

// 그 대기는 **영원하지 않다** — 전개 도중 매달려 있는 것이 가장 나쁘다.
TEST_F(SequenceTest, UnreadDpcEventuallyTimesOut) {
    host_.dpc_readable = false;
    ArriveAtDpcCheck();
    TickN(RdSequence::kWaitTicksMax + 2);
    EXPECT_NE(seq_->State(), Seq::DPC_STATE_CHECK) << "무한 대기";
    EXPECT_TRUE(seq_->GetLock());
}

TEST_F(SequenceTest, AbortsWhenDpcIsInFault) {
    host_.dpc_state = dpc::STATE_ERROR;
    ArriveAtDpcCheck();
    seq_->Tick();
    EXPECT_EQ(host_.DpcWrites(), 0u) << "FAULT 인데 전개를 요청했다";
    EXPECT_TRUE(seq_->GetLock());
}

// ★ 이미 전개 구간이면 겹쳐 몰지 않는다 — 이전 시퀀스가 안 끝난 것이다.
TEST_F(SequenceTest, AbortsWhenDpcIsAlreadyDeploying) {
    host_.dpc_state = dpc::STATE_INIT;
    ArriveAtDpcCheck();
    seq_->Tick();
    EXPECT_EQ(host_.DpcWrites(), 0u) << "이미 전개 중인데 또 몰았다";
    EXPECT_TRUE(seq_->GetLock());
}

// 전개 요청은 **주입된 목표값을 sys_state_target(127) 에** 쓴다.
TEST_F(SequenceTest, DeployWritesInjectedTargetToDpc) {
    ArriveAtDpcCheck();
    seq_->Tick();
    ASSERT_EQ(host_.writes.size(), 2u);
    EXPECT_EQ(host_.writes[1].target, TARGET::DPC);
    EXPECT_EQ(host_.writes[1].addr,   dpc::REG_SYS_STATE_TARGET_OFFSET);
    EXPECT_EQ(host_.writes[1].value,  dpc::STATE_INIT);
    EXPECT_EQ(seq_->State(), Seq::DPC_DEPLOY);
}

// 카메라 위치에 도달하기 전에는 넘어가지 않는다.
TEST_F(SequenceTest, WaitsForCameraReadyState) {
    ArriveAtDpcCheck();
    seq_->Tick();                       // 전개 요청
    host_.Complete(true); seq_->Tick(); // -> DPC_WAIT_CAMERA
    ASSERT_EQ(seq_->State(), Seq::DPC_WAIT_CAMERA);

    host_.dpc_state = dpc::STATE_INIT;          // 아직 이동 중
    TickN(3);
    EXPECT_EQ(seq_->State(), Seq::DPC_WAIT_CAMERA);

    host_.dpc_state = dpc::STATE_WAIT;
    seq_->Tick();
    EXPECT_EQ(seq_->State(), Seq::CAMERA_ACTION);
}

// ★ 전 구간 완주 — 현재 계약. CAMERA_ACTION 만 미구현(카메라 미연결)이라 통과한다.
TEST_F(SequenceTest, FullRunDrivesDpcAndEndsWithEstopReleaseAndLock) {
    ArriveAtDpcCheck();
    seq_->Tick();                            // 전개 요청 (write#1)
    host_.Complete(true); seq_->Tick();      // -> WAIT_CAMERA
    host_.dpc_state = dpc::STATE_WAIT;
    seq_->Tick();                            // -> CAMERA_ACTION
    seq_->Tick();                            // 카메라 통과 + 회수 요청 (write#2)
    ASSERT_EQ(host_.writes.size(), 3u);
    EXPECT_EQ(host_.writes[2].target, TARGET::DPC);
    EXPECT_EQ(host_.writes[2].addr,   dpc::REG_SYS_STATE_TARGET_OFFSET);
    EXPECT_EQ(host_.writes[2].value,  dpc::STATE_ASCEND_1);

    host_.Complete(true); seq_->Tick();      // -> WAIT_RETRACT
    host_.dpc_state = dpc::STATE_FINISH;
    seq_->Tick();                            // 회수 완료 -> ESTOP 해제 (write#3)
    ASSERT_EQ(host_.writes.size(), 4u) << "해제 write 가 안 나갔다";
    EXPECT_EQ(host_.writes[3].target, TARGET::ECU);
    EXPECT_EQ(host_.writes[3].addr,   ecu::REG_SOFT_ESTOP_OFFSET);
    EXPECT_EQ(host_.writes[3].value,  ecu::SOFT_ESTOP_RELEASE);

    host_.Complete(true);
    seq_->Tick();
    EXPECT_FALSE(host_.paused) << "종료했는데 cmd_vel 이 멈춘 채로 남았다";
    EXPECT_EQ(seq_->State(), Seq::IDLE);
    EXPECT_TRUE(seq_->GetLock()) << "종료 시 lock — 재전개는 unlock 필요 (§3.2)";
}

// ★ 해제가 실패해도 cmd_vel 은 재개한다 — 여기서 멈춰 두면 로봇이 영영 못 움직인다.
TEST_F(SequenceTest, ResumesCmdVelEvenIfReleaseFails) {
    ArriveAtDpcCheck();
    seq_->Tick();
    host_.Complete(true); seq_->Tick();
    host_.dpc_state = dpc::STATE_WAIT; seq_->Tick();
    seq_->Tick();
    host_.Complete(true); seq_->Tick();
    host_.dpc_state = dpc::STATE_FINISH; seq_->Tick();
    host_.Complete(false);
    seq_->Tick();
    EXPECT_FALSE(host_.paused);
    EXPECT_EQ(seq_->State(), Seq::IDLE);
}

// ★ 전개 도중 Abort 하면 **soft ESTOP 을 풀고** 나간다 — 걸어 둔 채 끝내면
//    조작자가 이유를 모른 채 로봇이 안 움직이는 상태가 된다.
TEST_F(SequenceTest, AbortMidDeployReleasesTheSoftEstop) {
    ArriveAtDpcCheck();
    ASSERT_TRUE(host_.paused);
    host_.dpc_state = dpc::STATE_ERROR;
    seq_->Tick();                            // FAULT -> Abort
    EXPECT_EQ(seq_->State(), Seq::ESTOP_RELEASE) << "ESTOP 해제 단계로 가야 한다";
    ASSERT_GE(host_.writes.size(), 2u);
    EXPECT_EQ(host_.writes.back().addr,  ecu::REG_SOFT_ESTOP_OFFSET);
    EXPECT_EQ(host_.writes.back().value, ecu::SOFT_ESTOP_RELEASE);
}

// lock 중에는 토픽을 무시한다 (§3.2).
TEST_F(SequenceTest, LockedTriggerIsIgnored) {
    seq_->SetLock(true);
    seq_->Trigger();
    seq_->Tick();
    EXPECT_TRUE(host_.writes.empty());
    EXPECT_EQ(seq_->State(), Seq::IDLE);

    seq_->SetLock(false);
    seq_->Trigger();
    seq_->Tick();
    EXPECT_EQ(host_.writes.size(), 1u) << "unlock 후에는 받아야 한다";
}

// 슬롯이 없어 명령을 못 넣어도 **죽지 않는다** — 상태만 남고 다음 tick 에 다시 본다.
TEST_F(SequenceTest, SurvivesSlotPostFailure) {
    host_.accept_ = false;
    seq_->Trigger();
    seq_->Tick();
    EXPECT_TRUE(host_.writes.empty());
    EXPECT_EQ(seq_->State(), Seq::ESTOP_SET) << "발행 실패로 넘어가도 상태는 진행한다";
}

}  // namespace
