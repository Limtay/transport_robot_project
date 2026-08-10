#include "orin_firmware_bridge/policy/rd_command.hpp"
#include "orin_firmware_bridge/core/rd_register_dpc.hpp"   // IsWritableTarget (dpc_set_seq)
#include <cstring>
#include <sstream>
#include <iomanip>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace orin_bridge {


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
// ============================ ROS 스레드 ============================

bool RdCommand::HandleRequest(const CommandRequest_t& req, std::string* out_msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // ---- RESET ----
    if (req.action == 0) {
        if (req.slot >= CMD_NUM_SLOTS) { *out_msg = "RESET: slot 0~3 필요"; return false; }
        if (slots_[req.slot].is_auto)  { *out_msg = "RESET 거부: auto sequence 사용 중인 슬롯"; return false; }
        ClearSlotLocked(req.slot);
        *out_msg = "slot " + std::to_string(req.slot) + " RESET 완료";
        RD_INFO(log_, "RdCommand", "[Command] %s", out_msg->c_str());
        return true;
    }
    if (req.action != 1) { *out_msg = "action 은 0(RESET)/1(SET)"; return false; }

    // ---- SET 검증 (B6 — 의미 단위 명령) ----
    if (TargetIndex(req.target_id) < 0) { *out_msg = "target_id 는 ECU(0xE1)/DPC-B(0xD1)/PCU(0xA1)"; return false; }
    if (req.duration > CMD_DURATION_MAX_SEC) { *out_msg = "duration 은 0/1/2~100"; return false; }

    const cmdcat::CmdDef* def = cmdcat::Find(req.cmd);
    if (!def) {
        *out_msg = "알 수 없는 cmd " + std::to_string(req.cmd) +
                   " — CommandSet.srv 의 CMD_* 를 쓸 것";
        return false;
    }

    // **표에 적힌 대상이 곧 계약이다** (09 §6). `kTargetAny`(raw·reboot)는 예외.
    // 여기서 막지 않으면 `dpc_set_light` 를 ECU 로 보내 ECU 의 124번을 건드린다.
    if (def->target != cmdcat::kTargetAny && req.target_id != def->target) {
        *out_msg = std::string(def->name) + " 는 " + TargetName(def->target) +
                   " 전용이다 (요청 대상: " + TargetName(req.target_id) + ")";
        return false;
    }

    uint16_t total = 0;
    ShadowBase(state_, req.target_id, &total);

    switch (def->kind) {
        case cmdcat::CmdKind::READ:
            // **표가 대상을 소유한다** (09 §6). 종전에는 여기서 "READ 는 ECU 전용" 을
            // 하드코딩으로 거부했다 — DPC 구간이 미확정이던 시절엔 맞았지만, 지금은
            // `dpc_read_all` 처럼 DPC 구간을 가진 명령이 있다. 표에 없는 조합은 아래
            // 공통 검사가 막는다.
            break;

        case cmdcat::CmdKind::WRITE1:
            if (req.args.empty()) { *out_msg = std::string(def->name) + ": args[0] 필요"; return false; }
            if (def->waddr >= total) { *out_msg = "대상 보드에 없는 주소"; return false; }
            // ⚠ `dpc_set_seq`(127) 만 값 검증이 붙는다 — **DPC 펌웨어는 값을 검증 없이
            //   그대로 `DPC_CTL.STATE` 에 대입한다** (`rd_map_dpcb.c:264`). FSM 중간
            //   상태(DESCEND_2 등)를 실수로 쓰면 단계를 통째로 건너뛰므로, 막는 주체가
            //   Orin 밖에 없다. `rd_sequence` 와 **같은 규칙**을 쓴다 — 두 경로가 다른
            //   값을 허용하면 한쪽으로만 사고가 난다.
            if (req.cmd == cmdcat::CMD_DPC_SET_SEQ && !dpc::IsWritableTarget(req.args[0])) {
                *out_msg = "dpc_set_seq: " + std::to_string(req.args[0]) + "(" +
                           dpc::SysStateName(req.args[0]) + ") 는 밖에서 쓸 수 없다 — "
                           "CTRL(0)/HOLD(1)/INIT(2)/ASCEND_1(6) 만 허용";
                return false;
            }
            break;

        case cmdcat::CmdKind::RAW_READ:
            if (req.data_len == 0 || req.start_addr + req.data_len > total) {
                *out_msg = "addr/len 범위 오류 (reg " + std::to_string(total) + "B)";
                return false;
            }
            if (req.data_len > kMaxRespPayload) {
                // 여기서 막지 않으면 응답이 잘려 오고, 잘린 것을 아무도 모른다.
                *out_msg = "len 이 응답 예산 초과 (" + std::to_string(req.data_len) +
                           " > " + std::to_string(kMaxRespPayload) + "B)";
                return false;
            }
            break;

        case cmdcat::CmdKind::RAW_WRITE:
            // data 가 비면 브리지 섀도 값을 보낸다 (구 INST_WRITE_REG). 그때는 data_len 이 길이다.
            if (req.data.empty()) {
                if (req.data_len == 0 || req.start_addr + req.data_len > total) {
                    *out_msg = "raw_write: data 가 비면 data_len 으로 섀도 구간을 지정해야 한다";
                    return false;
                }
            } else if (req.start_addr + req.data.size() > total) {
                *out_msg = "raw_write: 범위 초과";
                return false;
            }
            break;

        case cmdcat::CmdKind::REBOOT:
            break;
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
    const cmdcat::CmdDef* def = cmdcat::Find(req.cmd);
    if (!def) { *out_msg = "알 수 없는 cmd"; return false; }

    Slot_t& s    = slots_[idx];
    s = Slot_t{};                       // 이전 명령의 잔재를 남기지 않는다
    s.active     = true;
    s.is_auto    = false;
    s.target_id  = req.target_id;
    s.cmd        = req.cmd;
    s.read_span  = nullptr;
    s.user_data.clear();

    // **여기가 B6 의 번역이다** — 의미 단위 이름 → 실제 트랜잭션.
    switch (def->kind) {
        case cmdcat::CmdKind::READ:
            s.inst      = CmdInst::READ;
            s.read_span = def->read;
            s.start_addr = def->read->spans[0].addr;   // 로그·표시용 대표값
            s.data_len   = def->read->RespPayload();
            break;
        case cmdcat::CmdKind::WRITE1:
            s.inst       = CmdInst::WRITE_DATA;
            s.start_addr = def->waddr;
            s.data_len   = 1;
            s.user_data  = {req.args[0]};
            break;
        case cmdcat::CmdKind::REBOOT:
            s.inst = CmdInst::REBOOT;
            break;
        case cmdcat::CmdKind::RAW_READ:
            s.inst       = CmdInst::READ;
            s.start_addr = req.start_addr;
            s.data_len   = req.data_len;
            break;
        case cmdcat::CmdKind::RAW_WRITE:
            // data 가 있으면 사용자 값, 없으면 브리지 섀도 값 (구 INST_WRITE_REG).
            s.inst       = req.data.empty() ? CmdInst::WRITE_REG : CmdInst::WRITE_DATA;
            s.start_addr = req.start_addr;
            s.data_len   = req.data.empty() ? req.data_len
                                            : static_cast<uint16_t>(req.data.size());
            s.user_data  = req.data;
            break;
    }

    // REBOOT 는 본질적으로 1회성 — once 로 강제
    s.duration   = (s.inst == CmdInst::REBOOT) ? CMD_DURATION_ONCE : req.duration;
    s.start_time = steady_clock::now();
    s.err_streak = 0;

    std::ostringstream oss;
    oss << "slot " << idx << " SET: " << TargetName(s.target_id)
        << " " << def->name;
    if (s.read_span && s.read_span->count > 1) {
        oss << " segs=" << static_cast<int>(s.read_span->count);
    } else {
        oss << " addr=" << s.start_addr;
    }
    oss << " len=" << s.data_len << " dur=" << s.duration;
    *out_msg = oss.str();
    RD_INFO(log_, "RdCommand", "[Command] %s", out_msg->c_str());
    return true;
}

void RdCommand::ClearSlotLocked(int idx) {
    slots_[idx] = Slot_t{};
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

    // once: **시도 횟수**로 포기한다 (00 C2). 종전에는 벽시계 2초였다.
    //
    // 무엇이 문제였나: REBOOT 후 blackout 이 3초인데 once timeout 이 2초였다. blackout
    // 중에는 위 early-return 으로 **한 번도 시도되지 않는데** 시계는 계속 흘러,
    // REBOOT 직후 예약한 명령은 시도 0회로 죽었다. 시간이 아니라 시도를 세면
    // blackout 이 예산을 갉아먹지 않는다.
    //
    // 40회 = 5Hz 발사 기준 **실제 시도 8초치**다 (blackout 대기 시간은 제외).
    if (s.duration == CMD_DURATION_ONCE && s.attempts >= kOnceMaxAttempts) {
        RD_ERROR(log_, "RdCommand",
            "[Command] slot %d 포기: %s inst=%d addr=%u — %u회 시도 실패 (%.1fs 경과)",
            slot, TargetName(s.target_id), static_cast<int>(s.inst), s.start_addr,
            s.attempts, std::chrono::duration<double>(elapsed).count());
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
        RD_INFO(log_, "RdCommand", "[Command] slot %d 지속시간(%us) 만료 — 해제", slot, s.duration);
        ClearSlotLocked(slot);
        return false;
    }

    // 여기까지 왔으면 이번 tick 에 **실제로 발사된다** — blackout·만료로 빠진 경로는
    // 위에서 이미 return 했다. 그래서 시도 카운트는 여기서 올린다.
    s.attempts++;

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
        case CmdInst::READ: {
            if (!s.read_span) {                       // raw / auto sequence — 단일 구간
                *out_task = {s.target_id, PacketInst::READ, s.start_addr, s.data_len};
                return true;
            }
            // 의미 단위 READ (B6) — 구간은 카탈로그가 안다. 세그가 여럿이면 **한
            // 트랜잭션에 전부** 실린다 (예약 구간을 건너뛰는 것이 그래서 공짜다).
            TaskConfig_t t(s.target_id, PacketInst::READ, 0, 0);
            for (uint8_t i = 0; i < s.read_span->count; i++)
                t.segs[t.seg_count++] = Segment_t{s.read_span->spans[i].addr,
                                                  s.read_span->spans[i].len};
            t.start_addr = t.segs[0].addr;            // 로그 호환용
            t.data_len   = t.segs[0].len;
            *out_task = t;
            return true;
        }
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
        RD_INFO(log_, "RdCommand", "[Command] slot %d REBOOT OK: %s — 3초간 접근 차단",
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
                RD_INFO(log_, "RdCommand", "[Command] slot %d READ OK %s addr=%u len=%u:%s",
                            slot, TargetName(s.target_id), s.start_addr, s.data_len,
                            oss.str().c_str());
                // 07 §2 — **무엇을 덮었는지 기록한다.** 로그로만 남기면 소비자가 텍스트를
                // 정규식으로 뜯어야 하고, 그것이 C3 로 없앤 바로 그 형태다.
                last_read_ = LastRead_t{};
                last_read_.valid     = true;
                last_read_.target_id = s.target_id;
                if (s.read_span) {
                    last_read_.count = s.read_span->count;
                    for (uint8_t i = 0; i < s.read_span->count; i++)
                        last_read_.spans[i] = s.read_span->spans[i];
                } else {
                    last_read_.count    = 1;
                    last_read_.spans[0] = {s.start_addr, s.data_len};
                }
                last_read_time_ = steady_clock::now();
            } else {
                RD_INFO(log_, "RdCommand", "[Command] slot %d 성공 (RET_OK): %s inst=%d addr=%u",
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
            RD_WARN(log_, "RdCommand", "[Command] slot %d 실패 누적 %u회 (%s addr=%u)",
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
    RD_WARN(log_, "RdCommand", "[Command] 모든 슬롯 사용 중 — slot %d 일시정지 후 auto command 차용",
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
        RD_INFO(log_, "RdCommand", "[Command] slot %d 일시정지 해제 — 이전 명령 복귀", idx);
    }
}

// ===== ISlotHost — RdSequence 가 슬롯을 빌려 쓰는 통로 (02 A3) =====
//
// 시퀀스가 필요로 하는 것은 셋뿐이다: 명령 하나 보내기 / 끝났는지 묻기 / cmd_vel 정지·재개.
// 그 셋만 노출하면 시퀀스는 슬롯의 내부(우선순위·차용·복귀)를 몰라도 된다.

bool RdCommand::PostAutoWriteTo(uint8_t target, uint16_t addr, uint8_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    int idx = AcquireAutoSlotLocked();
    if (idx < 0) {
        RD_ERROR(log_, "RdCommand",
                 "[Command] 자동 슬롯 확보 실패 — target 0x%02X addr %u write 미발행",
                 target, addr);
        return false;
    }
    Slot_t& s    = slots_[idx];
    s.active     = true;
    s.is_auto    = true;
    s.target_id  = target;
    s.inst       = CmdInst::WRITE_DATA;
    s.start_addr = addr;
    s.user_data  = {value};
    s.data_len   = 1;
    s.duration   = CMD_DURATION_ONCE;
    s.attempts   = 0;
    s.start_time = steady_clock::now();
    seq_slot_     = idx;
    seq_cmd_done_ = false;
    seq_cmd_ok_   = false;
    return true;
}

bool RdCommand::AutoCommandDone(bool* ok) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!seq_cmd_done_) return false;
    if (ok) *ok = seq_cmd_ok_;
    return true;
}

bool RdCommand::DpcSysState(uint8_t* out) const {
    // 신선도 기준은 `comm.is_connected` 다 — DPC 와 **성공한 트랜잭션이 한 번이라도
    // 있었는가**. DPC 읽기 슬롯은 표가 정한 고정 구간이므로(kDpcProject20Hz) 연결됐다는 것은
    // 곧 이 필드가 갱신됐다는 뜻이다.
    // `enable_dpc_read_` 가 꺼져 있으면 트랜잭션 자체가 없어 false 로 남는다 — 의도한 대로다.
    std::lock_guard<std::mutex> lock(state_->state_mutex);
    if (!state_->dpc.comm.is_connected) return false;
    if (out) *out = state_->dpc.reg.sys.sys_state;
    return true;
}

RdCommand::LastRead_t RdCommand::LastReadSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    LastRead_t out = last_read_;
    if (out.valid) {
        out.age_s = std::chrono::duration<double>(steady_clock::now() - last_read_time_).count();
    }
    return out;
}

std::vector<RdCommand::SlotView_t> RdCommand::SlotSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SlotView_t> out;
    out.reserve(slots_.size());
    for (const auto& s : slots_) {
        SlotView_t v;
        v.active    = s.active;
        v.target_id = s.target_id;
        v.inst      = static_cast<uint8_t>(s.inst);
        v.duration  = s.duration;
        v.attempts  = s.attempts;      // 연속 실패(err_streak)가 아니라 **실제 발사 횟수**
        out.push_back(v);
    }
    return out;
}

} // namespace orin_bridge
