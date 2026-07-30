#include "orin_firmware_bridge/rd_sequence.hpp"

#include "orin_firmware_bridge/rd_register_ecu.hpp"

namespace orin_bridge {

const char* RdSequence::SeqName(Seq s) {
    switch (s) {
        case Seq::IDLE:             return "IDLE";
        case Seq::ESTOP_SET:        return "ESTOP_SET";
        case Seq::DPC_STATE_CHECK:  return "DPC_STATE_CHECK";
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
                RD_WARN(log_, "RdSequence",
                    "[Jeongae] TODO: DPC 레지스터 미확정 — DPC/카메라 단계는 skip 처리");
                seq_ = Seq::DPC_STATE_CHECK;
            } else {
                AbortLocked("ECU soft ESTOP (addr189=0)");
            }
            break;

        // ---- TODO 구간: DPC/PRA 레지스터 및 카메라 Action 미확정 (04 §6.3 C5) ----
        // **분리와 구현은 별개다.** rd_command 에서 옮겨오면서 내용은 그대로 뒀다.
        case Seq::DPC_STATE_CHECK:   /* TODO: DPC state READ — 완전한 상태인지 check   */ seq_ = Seq::DPC_DEPLOY;       break;
        /* DPC IDX(57) == 1 이면 다음 시퀀스 진행*/
        case Seq::DPC_DEPLOY:        /* TODO: DPC 전개 요청 — 공벽1(1)/공벽2(2)/전개판(3) */ seq_ = Seq::DPC_WAIT_CAMERA;  break;
        /* DPC IDX(127) = 2 Write 후 성공하면 다음 시퀀스 진행 */
        case Seq::DPC_WAIT_CAMERA:   /* TODO: DPC state 대기 — 카메라 위치, LED on      */ seq_ = Seq::CAMERA_ACTION;    break;
        /* DPC IDX(127) == 5 이면 다음 시퀀스 진행 */
        case Seq::CAMERA_ACTION:     /* TODO: deploy camera ROS2 Action (4과제 정책 미정) */ seq_ = Seq::DPC_RETRACT;      break;
        /* CAMERA Action 요청 후 완료 대기*/
        case Seq::DPC_RETRACT:       /* TODO: DPC 회수 요청                              */ seq_ = Seq::DPC_WAIT_RETRACT; break;
        /* DPC IDX(127) = 6 요청 후 다음 시퀀스 진행*/
        case Seq::DPC_WAIT_RETRACT:  /* TODO: DPC 회수 완료 state 대기                   */
            // DPC IDX(127) == 8 이면 밑의 시퀀스 진행
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
