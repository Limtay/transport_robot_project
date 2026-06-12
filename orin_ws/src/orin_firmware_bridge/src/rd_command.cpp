#include "orin_firmware_bridge/rd_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <sstream>
#include <iomanip>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace orin_bridge {

static rclcpp::Logger CmdLogger() { return rclcpp::get_logger("RdCommand"); }

RdCommand::RdCommand(RobotState_t* state) : state_(state) {
    auto past = steady_clock::now() - 1h;
    blackout_until_.fill(past);
}

const char* RdCommand::TargetName(uint8_t target_id) {
    switch (target_id) {
        case TARGET::ECU: return "ECU";
        case TARGET::DPC: return "DPC";
        case TARGET::PCU: return "PCU";
        default:          return "UNKNOWN";
    }
}

int RdCommand::TargetIndex(uint8_t target_id) {
    switch (target_id) {
        case TARGET::ECU: return 0;
        case TARGET::DPC: return 1;
        case TARGET::PCU: return 2;
        default:          return -1;
    }
}

// 대상 보드의 reg 섀도 base 포인터 + 전체 크기
static uint8_t* ShadowBase(RobotState_t* st, uint8_t target_id, uint16_t* total_size) {
    switch (target_id) {
        case TARGET::ECU: *total_size = ecu::REG_TOTAL_SIZE; return reinterpret_cast<uint8_t*>(&st->ecu.reg);
        case TARGET::DPC: *total_size = dpc::REG_TOTAL_SIZE; return reinterpret_cast<uint8_t*>(&st->dpc.reg);
        case TARGET::PCU: *total_size = pcu::REG_TOTAL_SIZE; return reinterpret_cast<uint8_t*>(&st->pcu.reg);
        default:          *total_size = 0; return nullptr;
    }
}

// ============================ ROS 스레드 ============================

bool RdCommand::HandleRequest(const CommandRequest_t& req, std::string* out_msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // ---- RESET ----
    if (req.action == 0) {
        if (req.slot >= CMD_NUM_SLOTS) { *out_msg = "RESET: slot 0~3 필요"; return false; }
        if (slots_[req.slot].is_auto)  { *out_msg = "RESET 거부: auto sequence 사용 중인 슬롯"; return false; }
        ClearSlotLocked(req.slot);
        *out_msg = "slot " + std::to_string(req.slot) + " RESET 완료";
        RCLCPP_INFO(CmdLogger(), "[Command] %s", out_msg->c_str());
        return true;
    }
    if (req.action != 1) { *out_msg = "action 은 0(RESET)/1(SET)"; return false; }

    // ---- SET 검증 ----
    if (TargetIndex(req.target_id) < 0) { *out_msg = "target_id 는 ECU(0xE1)/DPC(0xD2)/PCU(0xA1)"; return false; }
    if (req.inst > static_cast<uint8_t>(CmdInst::REBOOT)) { *out_msg = "inst 는 0~3"; return false; }
    if (req.duration > CMD_DURATION_MAX_SEC) { *out_msg = "duration 은 0/1/2~100"; return false; }

    const CmdInst inst = static_cast<CmdInst>(req.inst);
    uint16_t total = 0;
    ShadowBase(state_, req.target_id, &total);
    if (inst == CmdInst::READ || inst == CmdInst::WRITE_REG) {
        if (req.data_len == 0 || req.start_addr + req.data_len > total) {
            *out_msg = "addr/len 범위 오류 (reg " + std::to_string(total) + "B)";
            return false;
        }
    } else if (inst == CmdInst::WRITE_DATA) {
        if (req.data.empty() || req.start_addr + req.data.size() > total) {
            *out_msg = "WRITE_DATA: data 비어있거나 범위 초과";
            return false;
        }
    }

    int idx = req.slot;
    if (req.slot == CMD_SLOT_AUTO) {
        idx = -1;
        for (int i = 0; i < CMD_NUM_SLOTS; i++) {
            if (!slots_[i].active) { idx = i; break; }
        }
        if (idx < 0) { *out_msg = "모든 슬롯 사용 중 — RESET 후 재시도"; return false; }
    } else if (req.slot >= CMD_NUM_SLOTS) {
        *out_msg = "slot 은 0~3 또는 255(auto)";
        return false;
    } else if (slots_[idx].is_auto) {
        *out_msg = "SET 거부: auto sequence 사용 중인 슬롯";
        return false;
    }

    return SetSlotLocked(idx, req, out_msg);
}

bool RdCommand::SetSlotLocked(int idx, const CommandRequest_t& req, std::string* out_msg) {
    Slot_t& s    = slots_[idx];
    s.active     = true;
    s.is_auto    = false;
    s.target_id  = req.target_id;
    s.inst       = static_cast<CmdInst>(req.inst);
    s.start_addr = req.start_addr;
    s.data_len   = (s.inst == CmdInst::WRITE_DATA)
                       ? static_cast<uint16_t>(req.data.size()) : req.data_len;
    s.user_data  = req.data;
    // REBOOT 는 본질적으로 1회성 — once 로 강제
    s.duration   = (s.inst == CmdInst::REBOOT) ? CMD_DURATION_ONCE : req.duration;
    s.start_time = steady_clock::now();
    s.err_streak = 0;

    std::ostringstream oss;
    oss << "slot " << idx << " SET: " << TargetName(s.target_id)
        << " inst=" << static_cast<int>(s.inst)
        << " addr=" << s.start_addr << " len=" << s.data_len
        << " dur=" << s.duration;
    *out_msg = oss.str();
    RCLCPP_INFO(CmdLogger(), "[Command] %s", out_msg->c_str());
    return true;
}

void RdCommand::ClearSlotLocked(int idx) {
    slots_[idx] = Slot_t{};
}

void RdCommand::TriggerJeongae() {
    jeongae_trigger_.store(true);
}

void RdCommand::SetJeongaeLock(bool lock) {
    jeongae_lock_.store(lock);
    RCLCPP_INFO(CmdLogger(), "[Command] jeongae lock = %s", lock ? "ON" : "OFF");
}

// ============================ 스케줄러 스레드 ============================

bool RdCommand::IsTargetBlackedOut(uint8_t target_id) {
    int ti = TargetIndex(target_id);
    if (ti < 0) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return steady_clock::now() < blackout_until_[ti];
}

bool RdCommand::GetSlotTask(int slot, TaskConfig_t* out_task) {
    std::lock_guard<std::mutex> lock(mutex_);
    Slot_t& s = slots_[slot];
    if (!s.active) return false;

    int ti = TargetIndex(s.target_id);
    if (ti >= 0 && steady_clock::now() < blackout_until_[ti]) return false;

    auto elapsed = steady_clock::now() - s.start_time;

    // once: 2초 timeout 후 포기 (§2.2)
    if (s.duration == CMD_DURATION_ONCE && elapsed > 2s) {
        RCLCPP_ERROR(CmdLogger(),
            "[Command] slot %d TIMEOUT (2s): %s inst=%d addr=%u — 포기",
            slot, TargetName(s.target_id), static_cast<int>(s.inst), s.start_addr);
        if (s.is_auto) {
            seq_cmd_done_ = true;
            seq_cmd_ok_   = false;
            ReleaseAutoSlotLocked(slot);
        } else {
            ClearSlotLocked(slot);
        }
        return false;
    }

    // N초 지속: 만료 시 자동 해제
    if (s.duration >= 2 && elapsed > seconds(s.duration)) {
        RCLCPP_INFO(CmdLogger(), "[Command] slot %d 지속시간(%us) 만료 — 해제", slot, s.duration);
        ClearSlotLocked(slot);
        return false;
    }

    switch (s.inst) {
        case CmdInst::WRITE_REG:
            *out_task = {s.target_id, PacketInst::WRITE, s.start_addr, s.data_len};
            return true;
        case CmdInst::WRITE_DATA: {
            // 사용자 데이터를 reg 섀도에 복사 후 일반 WRITE 로 인코딩
            uint16_t total = 0;
            uint8_t* base = ShadowBase(state_, s.target_id, &total);
            if (!base) return false;
            {
                std::lock_guard<std::mutex> st_lock(state_->state_mutex);
                std::memcpy(base + s.start_addr, s.user_data.data(), s.user_data.size());
            }
            *out_task = {s.target_id, PacketInst::WRITE, s.start_addr, s.user_data.size()};
            return true;
        }
        case CmdInst::READ:
            *out_task = {s.target_id, PacketInst::READ, s.start_addr, s.data_len};
            return true;
        case CmdInst::REBOOT:
            *out_task = {s.target_id, PacketInst::REBOOT, 0, 0};
            return true;
    }
    return false;
}

void RdCommand::ReportResult(int slot, RD_RET ret) {
    std::lock_guard<std::mutex> lock(mutex_);
    Slot_t& s = slots_[slot];
    if (!s.active) return;

    // ---- REBOOT 성공: 3초 blackout + 보드 off 표시 (§2.2) ----
    if (s.inst == CmdInst::REBOOT && ret == RD_OK) {
        int ti = TargetIndex(s.target_id);
        if (ti >= 0) blackout_until_[ti] = steady_clock::now() + 3s;
        {
            std::lock_guard<std::mutex> st_lock(state_->state_mutex);
            switch (s.target_id) {
                case TARGET::ECU: state_->ecu.comm.is_connected = false; break;
                case TARGET::DPC: state_->dpc.comm.is_connected = false; break;
                case TARGET::PCU: state_->pcu.comm.is_connected = false; break;
            }
        }
        RCLCPP_INFO(CmdLogger(), "[Command] slot %d REBOOT OK: %s — 3초간 접근 차단",
                    slot, TargetName(s.target_id));
        if (s.is_auto) { seq_cmd_done_ = true; seq_cmd_ok_ = true; ReleaseAutoSlotLocked(slot); }
        else           { ClearSlotLocked(slot); }
        return;
    }

    if (s.duration == CMD_DURATION_ONCE) {
        if (ret == RD_OK) {
            if (s.inst == CmdInst::READ) {
                // reg 갱신 완료 — 해당 값 terminal 출력 (§2.2 Read)
                std::vector<uint8_t> buf(s.data_len);
                uint16_t total = 0;
                uint8_t* base = ShadowBase(state_, s.target_id, &total);
                {
                    std::lock_guard<std::mutex> st_lock(state_->state_mutex);
                    std::memcpy(buf.data(), base + s.start_addr, s.data_len);
                }
                std::ostringstream oss;
                for (size_t i = 0; i < buf.size(); i++) {
                    if (i % 16 == 0) oss << "\n  [" << std::setw(3) << (s.start_addr + i) << "] ";
                    oss << std::hex << std::setw(2) << std::setfill('0')
                        << static_cast<int>(buf[i]) << " " << std::dec << std::setfill(' ');
                }
                RCLCPP_INFO(CmdLogger(), "[Command] slot %d READ OK %s addr=%u len=%u:%s",
                            slot, TargetName(s.target_id), s.start_addr, s.data_len,
                            oss.str().c_str());
            } else {
                RCLCPP_INFO(CmdLogger(), "[Command] slot %d 성공 (RET_OK): %s inst=%d addr=%u",
                            slot, TargetName(s.target_id), static_cast<int>(s.inst), s.start_addr);
            }
            if (s.is_auto) { seq_cmd_done_ = true; seq_cmd_ok_ = true; ReleaseAutoSlotLocked(slot); }
            else           { ClearSlotLocked(slot); }
        }
        // 실패: RET_OK 까지 5Hz 재시도 — 2s timeout 은 GetSlotTask 에서 처리 (§2.3)
        return;
    }

    // forever / N초 지속 커맨드
    if (ret == RD_OK) {
        s.err_streak = 0;
    } else {
        s.err_streak++;
        if (s.err_streak % 25 == 1) {  // 5Hz 기준 약 5초마다 경고
            RCLCPP_WARN(CmdLogger(), "[Command] slot %d 실패 누적 %u회 (%s addr=%u)",
                        slot, s.err_streak, TargetName(s.target_id), s.start_addr);
        }
    }
}

// ===================== 우선순위 규칙 (§2.3 / §3.1) =====================

int RdCommand::AcquireAutoSlotLocked() {
    // 빈 칸 중 가장 상단
    for (int i = 0; i < CMD_NUM_SLOTS; i++) {
        if (!slots_[i].active) return i;
    }
    // 모두 차있으면 최하위(우선순위 최저) 슬롯 일시정지 후 차용
    saved_slot_     = slots_[CMD_NUM_SLOTS - 1];
    saved_slot_idx_ = CMD_NUM_SLOTS - 1;
    ClearSlotLocked(CMD_NUM_SLOTS - 1);
    RCLCPP_WARN(CmdLogger(), "[Command] 모든 슬롯 사용 중 — slot %d 일시정지 후 auto command 차용",
                CMD_NUM_SLOTS - 1);
    return CMD_NUM_SLOTS - 1;
}

void RdCommand::ReleaseAutoSlotLocked(int idx) {
    ClearSlotLocked(idx);
    if (saved_slot_idx_ == idx) {
        slots_[idx] = saved_slot_;
        slots_[idx].start_time = steady_clock::now();  // 재개 시점부터 지속시간 재계산
        slots_[idx].err_streak = 0;
        saved_slot_idx_ = -1;
        RCLCPP_INFO(CmdLogger(), "[Command] slot %d 일시정지 해제 — 이전 명령 복귀", idx);
    }
}

// ===================== jeongae 자동 전개 시퀀스 (§3) =====================

bool RdCommand::StartSeqWriteLocked(uint16_t addr, uint8_t value) {
    int idx = AcquireAutoSlotLocked();
    Slot_t& s    = slots_[idx];
    s.active     = true;
    s.is_auto    = true;
    s.target_id  = TARGET::ECU;
    s.inst       = CmdInst::WRITE_DATA;
    s.start_addr = addr;
    s.user_data  = {value};
    s.data_len   = 1;
    s.duration   = CMD_DURATION_ONCE;
    s.start_time = steady_clock::now();
    seq_slot_     = idx;
    seq_cmd_done_ = false;
    seq_cmd_ok_   = false;
    return true;
}

void RdCommand::AbortSequenceLocked(const char* reason) {
    RCLCPP_ERROR(CmdLogger(), "[Jeongae] 시퀀스 실패: %s — 중단", reason);
    // TODO(§3.2): 각 state 실패 시 정책 미확정. 현재는 soft ESTOP 이 걸린 상태면
    //             해제 시도 후 종료, jeongae lock 을 걸어 재트리거를 막는다.
    jeongae_lock_.store(true);
    if (cmd_vel_paused_.load()) {
        StartSeqWriteLocked(ecu::REG_SOFT_ESTOP_OFFSET, ecu::SOFT_ESTOP_RELEASE);
        seq_ = Seq::ESTOP_RELEASE;
    } else {
        seq_ = Seq::IDLE;
    }
}

void RdCommand::TickAutoSequence() {
    std::lock_guard<std::mutex> lock(mutex_);

    switch (seq_) {
        case Seq::IDLE:
            if (jeongae_trigger_.exchange(false)) {
                // §3.2: Jeongae Unlock check — lock 상태면 토픽 무시
                if (jeongae_lock_.load()) {
                    RCLCPP_WARN(CmdLogger(),
                        "[Jeongae] lock 상태 — 토픽 무시 (macro jeongae_lock off 필요)");
                    break;
                }
                RCLCPP_INFO(CmdLogger(), "[Jeongae] 전개 시퀀스 시작");
                // ECU soft ESTOP 요청: WRITE addr 189 = 0 (§3.2)
                StartSeqWriteLocked(ecu::REG_SOFT_ESTOP_OFFSET, ecu::SOFT_ESTOP_ACTIVE);
                seq_ = Seq::ESTOP_SET;
            }
            break;

        case Seq::ESTOP_SET:
            if (!seq_cmd_done_) break;
            if (seq_cmd_ok_) {
                cmd_vel_paused_.store(true);  // 성공 시 50Hz loop 명령 정지
                RCLCPP_INFO(CmdLogger(), "[Jeongae] ECU soft ESTOP OK — 50Hz cmd_vel 정지");
                RCLCPP_WARN(CmdLogger(),
                    "[Jeongae] TODO: DPC 레지스터 미확정 — DPC/카메라 단계는 skip 처리");
                seq_ = Seq::DPC_STATE_CHECK;
            } else {
                AbortSequenceLocked("ECU soft ESTOP (addr189=0)");
            }
            break;

        // ---- TODO 구간: DPC/PRA 레지스터 및 카메라 Action 미확정 (Code_modify.md §3.2) ----
        case Seq::DPC_STATE_CHECK:   /* TODO: DPC state READ — 완전한 상태인지 check   */ seq_ = Seq::DPC_DEPLOY;       break;
        case Seq::DPC_DEPLOY:        /* TODO: DPC 전개 요청 — 공벽1(1)/공벽2(2)/전개판(3) */ seq_ = Seq::DPC_WAIT_CAMERA;  break;
        case Seq::DPC_WAIT_CAMERA:   /* TODO: DPC state 대기 — 카메라 위치, LED on      */ seq_ = Seq::CAMERA_ACTION;    break;
        case Seq::CAMERA_ACTION:     /* TODO: deploy camera ROS2 Action (4과제 정책 미정) */ seq_ = Seq::DPC_RETRACT;      break;
        case Seq::DPC_RETRACT:       /* TODO: DPC 회수 요청                              */ seq_ = Seq::DPC_WAIT_RETRACT; break;
        case Seq::DPC_WAIT_RETRACT:  /* TODO: DPC 회수 완료 state 대기                   */
            // ECU soft ESTOP 해제: WRITE addr 189 = 1 (§3.2)
            StartSeqWriteLocked(ecu::REG_SOFT_ESTOP_OFFSET, ecu::SOFT_ESTOP_RELEASE);
            seq_ = Seq::ESTOP_RELEASE;
            break;

        case Seq::ESTOP_RELEASE:
            if (!seq_cmd_done_) break;
            cmd_vel_paused_.store(false);  // 성공 시 50Hz loop 재개
            if (seq_cmd_ok_) {
                RCLCPP_INFO(CmdLogger(), "[Jeongae] ECU soft ESTOP 해제 OK — 50Hz cmd_vel 재개");
            } else {
                RCLCPP_ERROR(CmdLogger(),
                    "[Jeongae] soft ESTOP 해제 실패 — ECU 상태 확인 필요 (cmd_vel 은 재개)");
            }
            // §3.2: 시퀀스 종료 시 Jeongae Locking — 이후 토픽은 unlock 까지 무시
            jeongae_lock_.store(true);
            RCLCPP_INFO(CmdLogger(), "[Jeongae] 시퀀스 종료 — jeongae lock ON (재전개는 unlock 필요)");
            seq_ = Seq::IDLE;
            break;
    }
}

} // namespace orin_bridge
