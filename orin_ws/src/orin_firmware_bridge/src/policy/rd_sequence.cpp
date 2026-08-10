#include "orin_firmware_bridge/policy/rd_sequence.hpp"

#include "orin_firmware_bridge/core/rd_map.hpp"            // TARGET::{ECU,DPC}
#include "orin_firmware_bridge/core/rd_register_dpc.hpp"
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

namespace orin_bridge {

bool ISlotHost::PostAutoWrite(uint16_t addr, uint8_t value) {
    return PostAutoWriteTo(TARGET::ECU, addr, value);
}

// DPC 의 `sys_state_target`(127) 에 목표를 쓴다. 슬롯 확보 실패는 즉시 Abort —
// 전개 명령이 나가지 않았는데 다음 단계로 넘어가면 "안 움직이는데 기다린다" 가 된다.
bool RdSequence::PostDpcTargetLocked(uint8_t target_state, const char* what) {
    // ⚠ FSM 중간 상태를 밖에서 쓰면 단계를 건너뛴다. 펌웨어가 정한 mask 밖이면
    //   보내지 않는다 — 실수로 DESCEND_2 같은 값을 쓰는 일을 여기서 막는다.
    //
    //   **이 가드가 유일한 방어선이다** (2026-08-05 펌웨어 확인): `rd_map_dpcb.c:264` 는
    //   `DPC_CTL.STATE = reg.cmd_dpcb.sys_state_target` 을 **검증도 마스크도 없이** 한다.
    //   즉 DPC 는 무엇을 받든 그대로 상태를 바꾼다. 커맨드 경로(`dpc_set_seq`)도 같은
    //   `IsWritableTarget()` 을 쓴다 — 두 경로가 다른 값을 허용하면 한쪽으로만 사고가 난다.
    if (!dpc::IsWritableTarget(target_state)) {
        RD_ERROR(log_, "RdSequence",
            "[Jeongae] %s — sys_state_target=%u(%s) 는 Orin 이 쓸 수 없는 값이다 "
            "(CTRL/HOLD/INIT/ASCEND_1 만 허용)", what, target_state, dpc::SysStateName(target_state));
        AbortLocked("허용되지 않은 sys_state_target");
        return false;
    }
    if (!host_->PostAutoWriteTo(TARGET::DPC, dpc::REG_SYS_STATE_TARGET_OFFSET, target_state)) {
        AbortLocked(what);
        return false;
    }
    wait_ticks_ = 0;
    RD_INFO(log_, "RdSequence", "[Jeongae] %s — DPC sys_state_target=%u(%s) 요청",
            what, target_state, dpc::SysStateName(target_state));
    return true;
}

// `sys_state` 가 want 가 될 때까지 기다린다.
//
// 세 가지를 **같이** 본다. 하나라도 빠지면 매달리거나 거짓으로 통과한다:
//   ① 신선도 — 안 읽힌 섀도의 0은 `CTRL`(정상값)이라 값만으로 구분되지 않는다
//   ② FAULT  — 목표에 영원히 도달하지 않는다. 기다릴 이유가 없다
//   ③ 타임아웃 — 전개 도중 무한 대기가 가장 나쁘다
bool RdSequence::WaitDpcStateLocked(uint8_t want, const char* what, int also_ok) {
    uint8_t cur = 0;
    if (!host_->DpcSysState(&cur)) {
        // 아직 한 번도 안 읽혔다. 타임아웃까지는 기다려 준다 (기동 직후일 수 있다).
        if (++wait_ticks_ > kWaitTicksMax) {
            AbortLocked("DPC state 미판독 — 읽기가 켜져 있는지 확인 (enable_dpc_read)");
        }
        return false;
    }
    if (cur == dpc::STATE_ERROR) {
        AbortLocked("DPC ERROR — FSM 실패 또는 통신 FAULT");
        return false;
    }
    if (cur == want || (also_ok >= 0 && cur == static_cast<uint8_t>(also_ok))) {
        RD_INFO(log_, "RdSequence", "[Jeongae] %s 도달 (sys_state=%u %s)",
                what, cur, dpc::SysStateName(cur));
        wait_ticks_ = 0;
        return true;
    }
    if (++wait_ticks_ > kWaitTicksMax) {
        RD_ERROR(log_, "RdSequence", "[Jeongae] %s 대기 초과 — 현재 sys_state=%u(%s), 기대 %u(%s)",
                 what, cur, dpc::SysStateName(cur), want, dpc::SysStateName(want));
        AbortLocked("DPC 상태 대기 초과");
    }
    return false;
}

const char* RdSequence::SeqName(Seq s) {
    switch (s) {
        case Seq::IDLE:             return "IDLE";
        case Seq::ESTOP_SET:        return "ESTOP_SET";
        case Seq::DPC_STATE_CHECK:  return "DPC_STATE_CHECK";
        case Seq::DPC_SET_AUTO:     return "DPC_SET_AUTO";
        case Seq::DPC_DEPLOY:       return "DPC_DEPLOY";
        case Seq::DPC_WAIT_CAMERA:  return "DPC_WAIT_CAMERA";
        case Seq::CAMERA_ACTION:    return "CAMERA_ACTION";
        case Seq::DPC_RETRACT:      return "DPC_RETRACT";
        case Seq::DPC_WAIT_RETRACT: return "DPC_WAIT_RETRACT";
        case Seq::ESTOP_RELEASE:    return "ESTOP_RELEASE";
    }
    return "?";
}

void RdSequence::Trigger() { trigger_.store(true); }

void RdSequence::SetLock(bool lock) {
    lock_.store(lock);
    RD_INFO(log_, "RdSequence", "[Jeongae] lock %s", lock ? "ON" : "OFF");
}

RdSequence::Seq RdSequence::State() const {
    std::lock_guard<std::mutex> l(mutex_);
    return seq_;
}

bool RdSequence::Busy() const {
    std::lock_guard<std::mutex> l(mutex_);
    return seq_ != Seq::IDLE;
}

// 09 §5.3 ④ (U12) — 단계·대기·lock 을 **한 번에** 뜬다. 따로 물으면 그 사이에 Tick 이
// 끼어들어 "단계는 DPC_DEPLOY 인데 wait_ticks 는 다음 단계 것" 같은 섞인 값이 나가고,
// 화면은 그걸 진행 상황으로 읽는다.
RdSequence::Snapshot_t RdSequence::SnapshotState() const {
    Snapshot_t s;
    {
        std::lock_guard<std::mutex> l(mutex_);
        s.seq        = seq_;
        s.wait_ticks = wait_ticks_;
    }
    s.locked = lock_.load();   // atomic — 위 락 밖에서 읽어도 찢어지지 않는다
    return s;
}

void RdSequence::AbortLocked(const char* reason) {
    RD_ERROR(log_, "RdSequence", "[Jeongae] 시퀀스 실패: %s — 중단", reason);
    // TODO(§3.2): 각 state 실패 시 정책 미확정. 현재는 soft ESTOP 이 걸린 상태면
    //             해제 시도 후 종료, jeongae lock 을 걸어 재트리거를 막는다.
    lock_.store(true);
    if (host_->IsCmdVelPaused()) {
        host_->PostAutoWrite(ecu::REG_SOFT_ESTOP_OFFSET, ecu::SOFT_ESTOP_RELEASE);
        seq_ = Seq::ESTOP_RELEASE;
    } else {
        seq_ = Seq::IDLE;
    }
}

void RdSequence::Tick() {
    std::lock_guard<std::mutex> lock(mutex_);

    bool ok = false;
    switch (seq_) {
        case Seq::IDLE:
            if (trigger_.exchange(false)) {
                // §3.2: Jeongae Unlock check — lock 상태면 토픽 무시
                if (lock_.load()) {
                    RD_WARN(log_, "RdSequence",
                        "[Jeongae] lock 상태 — 토픽 무시 (macro jeongae_lock off 필요)");
                    break;
                }
                RD_INFO(log_, "RdSequence", "[Jeongae] 전개 시퀀스 시작");
                // ECU soft ESTOP 요청: WRITE addr 189 = 0 (§3.2)
                host_->PostAutoWrite(ecu::REG_SOFT_ESTOP_OFFSET, ecu::SOFT_ESTOP_ACTIVE);
                seq_ = Seq::ESTOP_SET;
            }
            break;

        case Seq::ESTOP_SET:
            if (!host_->AutoCommandDone(&ok)) break;
            if (ok) {
                host_->SetCmdVelPaused(true);   // 성공 시 50Hz loop 명령 정지
                RD_INFO(log_, "RdSequence", "[Jeongae] ECU soft ESTOP OK — 50Hz cmd_vel 정지");
                wait_ticks_ = 0;
                seq_ = Seq::DPC_STATE_CHECK;
            } else {
                AbortLocked("ECU soft ESTOP (addr189=0)");
            }
            break;

        // ── DPC 전개 (04 §6.3) ──────────────────────────────────────────────
        // 레지스터 맵은 확정됐다 (2026-07-30). 남은 미확정은 **DEPLOY_FSM 2~8 의
        // 각 값이 무엇인가** 하나뿐이라, 그 값들은 `DeployTargets` 로 주입받는다.

        case Seq::DPC_STATE_CHECK: {
            uint8_t cur = 0;
            if (!host_->DpcSysState(&cur)) {
                if (++wait_ticks_ > kWaitTicksMax)
                    AbortLocked("DPC state 미판독 — 읽기가 켜져 있는지 확인 (enable_dpc_read)");
                break;   // 아직 안 읽혔다 — 다음 tick 에 다시
            }
            if (cur == dpc::STATE_ERROR) { AbortLocked("DPC ERROR — 전개 불가"); break; }
            // 이미 FSM 구간(INIT~FINISH)이면 이전 시퀀스가 안 끝난 것이다. 겹쳐 몰지 않는다.
            if (dpc::IsDeployState(cur)) {
                AbortLocked("DPC 가 이미 전개 FSM 안에 있다 — 회수 완료 여부 확인 필요");
                break;
            }
            // **CTRL(0) / HOLD(1) 둘 다 여기서 통과한다** (09 §3.1, 펌웨어 확인).
            //   `rd_map_dpcb.c:264` 가 `sys_state_target` 을 **검증도 마스크도 없이**
            //   `DPC_CTL.STATE` 에 대입하므로, CTRL 에서 INIT 을 쓰는 것과 HOLD 에서 쓰는
            //   것이 바이트 단위로 같은 일이다. HOLD 를 굳이 경유할 이유가 없다.
            //   (memo_260731 의 "HOLD Check" 는 표기이지 게이트가 아니었다.)
            // 나머지 값(RSVD 9 등)은 우리가 모르는 상태다 — 모르면 몰지 않는다.
            if (cur != dpc::STATE_CTRL && cur != dpc::STATE_HOLD) {
                AbortLocked("DPC 가 CTRL/HOLD 가 아니다 — 전개를 시작할 수 없는 상태");
                break;
            }
            RD_INFO(log_, "RdSequence", "[Jeongae] DPC 확인 OK (sys_state=%u %s)",
                    cur, dpc::SysStateName(cur));
            // ⚠ **mode=AUTO 를 먼저 쓴다** (09 §3.2). 이게 없으면 DPC 의 FSM switch 가
            //   아예 안 돌아 127 에 무엇을 써도 움직이지 않는다 — 그런데 `sys_state` 는
            //   쓴 값을 그대로 되비추므로 **"전개 중" 으로 보이면서 아무 일도 안 일어난다.**
            if (!host_->PostAutoWriteTo(TARGET::DPC, dpc::REG_MODE_OFFSET, dpc::MODE_AUTO)) {
                AbortLocked("DPC mode=AUTO 요청 슬롯 확보 실패");
                break;
            }
            wait_ticks_ = 0;
            RD_INFO(log_, "RdSequence", "[Jeongae] DPC mode=AUTO 요청 (FSM 구동 전제)");
            seq_ = Seq::DPC_SET_AUTO;
            break;
        }

        case Seq::DPC_SET_AUTO:
            // write 완료만 본다 — **읽어서 확인할 수가 없다.** 126 은 소비 후 0xFF 로
            // 되돌리는 트리거이고 `DPC_CTL.MODE` 를 발행하는 R/O 필드도 없다 (09 §3.2).
            if (!host_->AutoCommandDone(&ok)) break;
            if (!ok) { AbortLocked("DPC mode=AUTO write 실패"); break; }
            // 전개 시작 = FSM 진입. 이후 DESCEND_1/2 는 DPC 가 스스로 밟는다.
            if (PostDpcTargetLocked(dpc::STATE_INIT, "전개 요청")) seq_ = Seq::DPC_DEPLOY;
            break;

        case Seq::DPC_DEPLOY:
            // write 가 끝났는지만 본다. 도달 판정은 다음 단계(WAIT_CAMERA).
            if (!host_->AutoCommandDone(&ok)) break;
            if (ok) { wait_ticks_ = 0; seq_ = Seq::DPC_WAIT_CAMERA; }
            else    { AbortLocked("DPC 전개 요청 write 실패"); }
            break;

        case Seq::DPC_WAIT_CAMERA:
            // DPC 가 INIT→DESCEND_1→DESCEND_2 를 스스로 밟고 **WAIT(5) 에서 멈춘다.**
            // 그 자리가 카메라 위치이고, Orin 이 ASCEND_1 을 써야 다시 움직인다.
            if (WaitDpcStateLocked(dpc::STATE_WAIT, "카메라 위치(WAIT)")) seq_ = Seq::CAMERA_ACTION;
            break;

        case Seq::CAMERA_ACTION:
            // ⚠ **여기만 TODO 다 — 카메라가 아직 연결돼 있지 않다** (4과제 정책도 미정).
            //    Action 클라이언트가 붙으면 그 결과를 기다리는 단계가 된다.
            //    지금은 통과시키되, 통과했다는 사실을 로그로 남긴다 — 조용히 건너뛰면
            //    나중에 "카메라가 왜 안 찍혔지" 를 시퀀스 밖에서 찾게 된다.
            //
            //    DPC 는 이 동안 WAIT(5) 에서 **가만히 기다린다** — 서두를 이유가 없다.
            RD_WARN(log_, "RdSequence",
                "[Jeongae] CAMERA_ACTION 통과 — **카메라 미연결로 미구현**. 촬영 없이 회수로 넘어간다");
            if (PostDpcTargetLocked(dpc::STATE_ASCEND_1, "회수 요청")) seq_ = Seq::DPC_RETRACT;
            break;

        case Seq::DPC_RETRACT:
            if (!host_->AutoCommandDone(&ok)) break;
            if (ok) { wait_ticks_ = 0; seq_ = Seq::DPC_WAIT_RETRACT; }
            else    { AbortLocked("DPC 회수 요청 write 실패"); }
            break;

        case Seq::DPC_WAIT_RETRACT:
            // FINISH(8) 에 도달하면 DPC 가 **CTRL(0) 로 자동 복귀한다.** 폴링으로는
            // FINISH 를 스쳐 지나갈 수 있으므로 **둘 다 완료로 받는다** — 하나만 보면
            // 이미 끝난 시퀀스를 타임아웃으로 실패 처리하게 된다.
            if (!WaitDpcStateLocked(dpc::STATE_FINISH, "회수 완료", dpc::STATE_CTRL)) break;
            // ECU soft ESTOP 해제: WRITE addr 189 = 1 (§3.2)
            host_->PostAutoWrite(ecu::REG_SOFT_ESTOP_OFFSET, ecu::SOFT_ESTOP_RELEASE);
            seq_ = Seq::ESTOP_RELEASE;
            break;

        case Seq::ESTOP_RELEASE:
            if (!host_->AutoCommandDone(&ok)) break;
            host_->SetCmdVelPaused(false);      // 성공이든 아니든 50Hz loop 재개
            if (ok) {
                RD_INFO(log_, "RdSequence", "[Jeongae] ECU soft ESTOP 해제 OK — 50Hz cmd_vel 재개");
            } else {
                RD_ERROR(log_, "RdSequence",
                    "[Jeongae] soft ESTOP 해제 실패 — ECU 상태 확인 필요 (cmd_vel 은 재개)");
            }
            // §3.2: 시퀀스 종료 시 Jeongae Locking — 이후 토픽은 unlock 까지 무시
            lock_.store(true);
            RD_INFO(log_, "RdSequence",
                "[Jeongae] 시퀀스 종료 — jeongae lock ON (재전개는 unlock 필요)");
            seq_ = Seq::IDLE;
            break;
    }
}

}  // namespace orin_bridge
