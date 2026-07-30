#include "orin_firmware_bridge/policy/rd_control.hpp"

#include <cmath>


namespace orin_bridge {

const char* ControlStateStr(ControlState s) {
    switch (s) {
        case ControlState::INIT:    return "INIT";
        case ControlState::IDLE:    return "IDLE";
        case ControlState::RUNNING: return "RUNNING";
        case ControlState::STREAM:  return "STREAM";
        case ControlState::LOCKED:  return "LOCKED";
    }
    return "?";
}

ControlState RdControl::State() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void RdControl::MarkInitDone() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ControlState::INIT) return;   // 재진입 방지 (INIT 은 1회성)
    state_ = ControlState::IDLE;
    RD_INFO(log_, "RdControl", "제어 FSM: INIT -> IDLE (write 0A 고정)");
}

bool RdControl::BeginProfile(uint32_t goal_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    // IDLE 에서만 수락 — RUNNING 중 새 goal 은 여기서 거부 (§3.2 동시 goal 불가).
    if (state_ != ControlState::IDLE) return false;
    state_          = ControlState::RUNNING;
    goal_id_        = goal_id;
    profile_time_   = 0.0f;
    segment_index_  = 0;
    for (int i = 0; i < 4; i++) profile_current_[i] = 0.0f;
    RD_INFO(log_, "RdControl",
                "제어 FSM: IDLE -> RUNNING (goal_id=%u)", goal_id);
    return true;
}

void RdControl::EndProfile() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ControlState::RUNNING) return;
    state_   = ControlState::IDLE;
    goal_id_ = 0;
    // 샘플 초기화 — 다음 tick 부터 SelectWrite 가 0A 를 낸다 (잔류 명령 제거).
    for (int i = 0; i < 4; i++) profile_current_[i] = 0.0f;
    profile_time_  = 0.0f;
    segment_index_ = 0;
    RD_INFO(log_, "RdControl", "제어 FSM: RUNNING -> IDLE (write 0A 복귀)");
}

void RdControl::Lock(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ControlState::LOCKED) return;   // 최초 사유 보존 (연쇄 래치 시 원인 유실 방지)
    state_       = ControlState::LOCKED;
    lock_reason_ = reason;
    goal_id_     = 0;
    for (int i = 0; i < 4; i++) profile_current_[i] = 0.0f;
    RD_ERROR(log_, "RdControl",
                 "제어 FSM: -> LOCKED (0A 래치) 사유: %s — config REARM 으로만 해제",
                 reason.c_str());
}

bool RdControl::Rearm() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ControlState::LOCKED) return false;
    state_ = ControlState::IDLE;
    RD_WARN(log_, "RdControl",
                "제어 FSM: LOCKED -> IDLE (REARM, 직전 사유: %s)", lock_reason_.c_str());
    lock_reason_.clear();
    return true;
}

std::string RdControl::LockReason() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lock_reason_;
}

bool RdControl::AcceptsConfig(bool is_rearm) const {
    std::lock_guard<std::mutex> lock(mutex_);
    switch (state_) {
        case ControlState::IDLE:    return true;         // 전부 허용
        case ControlState::LOCKED:  return is_rearm;     // REARM 만
        case ControlState::INIT:                          // 아직 루프 전
        case ControlState::RUNNING:                       // 실험 오염 방지 (abort = action cancel)
        case ControlState::STREAM:  return false;
    }
    return false;
}

void RdControl::NoteWriteErrStreak(uint64_t streak) {
    { std::lock_guard<std::mutex> lock(mutex_); last_write_streak_ = streak; }
    // RdMap 은 write 성공 시 카운터를 0 으로 되돌린다 — 즉 이 값이 곧 연속 거부 tick 수.
    if (streak < kWriteErrLockStreak) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == ControlState::LOCKED) return;
    }
    Lock("RW write 연속 거부 " + std::to_string(streak) + " tick (ECU mtr_lock/모드 확인 필요)");
}

// arm 이 켜져 있을 때만 STREAM 으로 간다 (01 §6.1.3). 스테일이면 IDLE 로 내리고
// **arm 도 함께 off** 한다 — 조작자가 손을 뗀 것을 시스템이 기억하면 안 된다.
// LOCKED 가 아니라 IDLE 인 이유: §6.1.2 재시드가 IDLE 을 안전하게 만들었고,
// "슬라이더에서 손을 뗐다" 는 **정상 조작**이라 REARM 을 요구할 이유가 없다.
void RdControl::UpdateStreamStateLocked() {
    if (!stream_armed_.load()) {
        if (state_ == ControlState::STREAM) state_ = ControlState::IDLE;
        return;
    }
    const double now   = (clock_ ? clock_ : DefaultClock())->NowEpoch();
    const bool   fresh = (stream_cmd_.stamp > 0.0) && ((now - stream_cmd_.stamp) <= stream_timeout_);

    if (fresh) {
        if (state_ == ControlState::IDLE) state_ = ControlState::STREAM;
    } else if (state_ == ControlState::STREAM) {
        state_ = ControlState::IDLE;
        stream_armed_.store(false);
        RD_WARN(log_, "RdControl",
            "스트림 스테일 (%.3fs > %.3fs) — IDLE 복귀, arm off. 재개하려면 run 을 다시 누를 것",
            now - stream_cmd_.stamp, stream_timeout_);
    }
}

bool RdControl::SetStreamArm(bool on, std::string* why) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!on) {                       // stop 은 언제나 허용 — 감속 방향이다 (01 §6.2)
        stream_armed_.store(false);
        if (state_ == ControlState::STREAM) state_ = ControlState::IDLE;
        if (why) *why = "arm off — IDLE";
        return true;
    }
    if (state_ != ControlState::IDLE) {
        if (why) *why = std::string("arm on 은 IDLE 에서만 가능 — 현재 ") + ControlStateStr(state_);
        return false;
    }
    // 스테일 값이 남아 있으면 arm 하자마자 옛 명령이 나간다. 시각을 무효화해 둔다.
    stream_cmd_.stamp = 0.0;
    stream_armed_.store(true);
    if (why) *why = "arm on — 다음 스트림 수신부터 STREAM";
    return true;
}

void RdControl::PushStreamCommand(const StreamCmd_t& c) {
    std::lock_guard<std::mutex> lock(mutex_);
    stream_cmd_ = c;
}

ControlWrite_t RdControl::SelectWrite() {
    std::lock_guard<std::mutex> lock(mutex_);
    UpdateStreamStateLocked();       // 매 tick 스테일 판정 — 상태 전이는 여기서만 일어난다
    ControlWrite_t w{};
    switch (state_) {
        case ControlState::RUNNING:
            for (int i = 0; i < 4; i++) w.current[i] = profile_current_[i];
            w.goal_id       = goal_id_;
            w.profile_time  = profile_time_;
            w.segment_index = segment_index_;
            break;
        case ControlState::STREAM: {
            // 단위는 auto_mode 가 정한다 — PrepareWrite 가 해당 레지스터에 꽂는다.
            // **DIRECT 만 예외**: 거기선 모터별 ctr_mode 가 단위를 정하므로 모터마다 다른
            // 필드를 고른다. 구 코드는 DIRECT 를 전부 current 로 취급해서, DIRECT 에서
            // position/velocity 를 스트림하면 **경고 없이 버려졌다** — FSM 은 STREAM 으로
            // 가고 모터는 안 움직여, 조작자가 "arm 이 안 먹었다" 와 구분할 수 없었다.
            const uint8_t am = auto_mode_.load();
            if (am == ecu::AUTO_MODE_DIRECT) {
                std::lock_guard<std::mutex> dl(ctr_mutex_);   // 락 순서: mutex_ -> ctr_mutex_
                for (int i = 0; i < 4; i++) {
                    w.current[i] = (ctr_mode_[i] == ecu::CTR_MODE_POSITION) ? stream_cmd_.position[i]
                                 : (ctr_mode_[i] == ecu::CTR_MODE_VELOCITY) ? stream_cmd_.velocity[i]
                                                                            : stream_cmd_.current[i];
                }
                break;
            }
            for (int i = 0; i < 4; i++) {
                w.current[i] = (am == ecu::AUTO_MODE_POSITION) ? stream_cmd_.position[i]
                             : (am == ecu::AUTO_MODE_VELOCITY) ? stream_cmd_.velocity[i]
                                                               : stream_cmd_.current[i];
            }
            break;
        }
        case ControlState::INIT:
        case ControlState::IDLE:
        case ControlState::LOCKED:
            break;   // 0A (기본값 그대로)
    }
    return w;
}

void RdControl::SetProfileSample(const float current[4], float t, uint16_t seg_idx) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ControlState::RUNNING) return;   // 취소·LOCKED 직후 잔류 샘플 무시
    for (int i = 0; i < 4; i++) profile_current_[i] = current[i];
    profile_time_  = t;
    segment_index_ = seg_idx;
}


void RdControl::SetCtrMode(int idx, uint8_t mode) {
    std::lock_guard<std::mutex> lock(ctr_mutex_);
    if (idx >= 0 && idx < 4) ctr_mode_[idx] = mode;
}

uint8_t RdControl::CtrMode(int idx) const {
    std::lock_guard<std::mutex> lock(ctr_mutex_);
    return (idx >= 0 && idx < 4) ? ctr_mode_[idx] : ecu::CTR_MODE_CURRENT;
}

// safe_stop (01 §6.2). **상태만 보고 판정하면 회전 중에 설정을 바꾸는 사고가 난다** —
// IDLE 이어도 관성으로 아직 돌 수 있으므로 속도·전류 실측 조건이 필수다.
// 판정에 쓰는 값은 직전 RW 트랜잭션의 read 스냅샷 하나뿐 — 여러 tick 을 섞지 않는다.
bool RdControl::SafeStop(std::string* why) const {
    if (!shadow_) { if (why) *why = "shadow 미주입"; return false; }

    ControlState st;
    uint64_t streak;
    { std::lock_guard<std::mutex> lock(mutex_); st = state_; streak = last_write_streak_; }

    if (st != ControlState::IDLE && st != ControlState::LOCKED) {
        if (why) *why = std::string("거부: ControlState=") + ControlStateStr(st) +
                        " (IDLE/LOCKED 에서만 설정 변경 가능)";
        return false;
    }
    if (streak != 0) {
        if (why) *why = "거부: 직전 RW write 거부 " + std::to_string(streak) + " tick 연속";
        return false;
    }

    std::lock_guard<std::mutex> lock(shadow_->state_mutex);
    const auto& md   = shadow_->ecu.reg.motor_data;
    const uint8_t mask = shadow_->ecu.reg.cmd_system.motor_mask;
    for (int i = 0; i < 4; i++) {
        if (!((mask >> i) & 0x01)) continue;          // 비활성 모터는 보지 않는다
        const float vel = md.velocity[i] * 10.0f;     // x10 [RPM]
        const float cur = md.current[i] * 0.01f;      // x0.01 [A]
        if (std::fabs(vel) >= kSafeStopVelRpm) {
            if (why) *why = "거부: M" + std::to_string(i + 1) + " fb_velocity=" +
                            std::to_string(vel) + " RPM (한계 " + std::to_string(kSafeStopVelRpm) + ")";
            return false;
        }
        if (std::fabs(cur) >= kSafeStopCurA) {
            if (why) *why = "거부: M" + std::to_string(i + 1) + " fb_current=" +
                            std::to_string(cur) + " A (한계 " + std::to_string(kSafeStopCurA) + ")";
            return false;
        }
    }
    if (why) *why = "safe_stop OK";
    return true;
}

// SET_ORIGIN 펄스 요청 (01 §6.3 C-1).
// **자동화하지 않는다** — auto_mode=POSITION 진입 시 원점을 자동으로 잡는 안은 기각됐다
// (실험마다 기준이 달라져 기록 간 비교가 깨진다). 사람이 실시간 pose 를 보고 누르는 동작이다.
bool RdControl::RequestOriginPulse(std::string* why) {
    // ctr_mode 가 bridge 소유여야 한다 = auto_mode DIRECT (01 §3).
    // CURRENT 등에서는 ECU 가 100Hz 로 ctr_mode 를 덮어써 펄스가 그대로 지워진다.
    if (auto_mode_.load() != ecu::AUTO_MODE_DIRECT) {
        if (why) *why = "거부: SET_ORIGIN 은 auto_mode=2(DIRECT) 에서만 — ctr_mode 가 "
                        "bridge 소유여야 펄스가 유지된다";
        return false;
    }
    std::string s;
    if (!SafeStop(&s)) { if (why) *why = s; return false; }

    std::lock_guard<std::mutex> lock(mutex_);
    if (pulse_ != PulseState::NONE) {          // 동시 1건 (01 §6.3 C-1 규약 5)
        if (why) *why = "거부: 다른 펄스가 진행 중";
        return false;
    }
    for (int i = 0; i < 4; i++) pulse_restore_[i] = ctr_mode_[i];
    pulse_ = PulseState::ARMED;
    if (why) *why = "SET_ORIGIN 펄스 예약 — 다음 tick 1회 전송 후 원복. "
                    "결과는 fb_position 으로 확인할 것";
    return true;
}

bool RdControl::OriginPulsePending() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pulse_ != PulseState::NONE;
}

// 구 RdNode::PrepareControlCommand — 02 A1-(c) 로 L3 에서 L2 로 이동.
// 내용은 그대로다 (동작 불변). 바뀐 것은 **누가 소유하느냐** 뿐이다.
// 구 RdNode::PrepareControlCommand — 02 A1-(c) 로 L3 에서 L2 로 이동.
//
// **안전값은 auto_mode 가 정한다** (01 §6.1.2). 초안은 이것을 "0A write 유지" 라고
// 단위째 박아 뒀는데, write 범위가 cmd_position 일 때 0 을 쓰는 것은 **"원점으로 가라"**
// 는 명령이다. 2026-07-28 실기에서 실제로 그렇게 됐다 — auto_mode=5 로 바꾸는 순간
// 모터가 41.6도에서 0도로 슬루했다 (06 §4.7).
//
//   CURRENT  0A          토크 없음
//   VELOCITY 0RPM        정지
//   POSITION fb_position **현재 자세 유지** — 매 tick 재시드 (bumpless transfer)
//   DIRECT   모터별 ctr_mode 를 따름
void RdControl::PrepareWrite() {
    if (!shadow_) return;

    const ControlWrite_t w = SelectWrite();
    const uint8_t am = auto_mode_.load();
    // 명령이 실제로 흐르는 상태인가. 아니면 아래 안전값으로 간다.
    const bool cmd_active = (State() == ControlState::RUNNING || State() == ControlState::STREAM);

    std::lock_guard<std::mutex> lock(shadow_->state_mutex);
    auto& cm       = shadow_->ecu.reg.cmd_motor;
    const auto& md = shadow_->ecu.reg.motor_data;

    // fb_position [deg] — POSITION 계열 안전값의 소스. x0.1 스케일.
    auto fb_pos = [&](int i) { return md.position[i] * 0.1f; };

    // 전류에만 전역 클램프를 적용한다. clamp_max_ 는 **[A] 단위**이므로 RPM·deg 에 걸면
    // 단위가 다른 값을 잘라내는 셈이다 (05 §2.1 이 지적한 "클램프의 단위 의존").
    auto clamp_a = [&](float v) {
        if (v >  clamp_max_) return  clamp_max_;
        if (v < -clamp_max_) return -clamp_max_;
        return v;
    };
    // 재생 중에는 **프로파일이 정한 실효 한계**로 자른다 (05 §2.4) — 단위가 mode 를 따른다.
    // 비대칭이므로 ± 한 값으로 표현할 수 없다 (position 의 관절 가동범위).
    const float plo = prof_lo_.load(), phi = prof_hi_.load();
    const bool  prof_lim = cmd_active && (plo < phi);
    auto clamp_prof = [&](float v) {
        if (!prof_lim) return v;
        if (v > phi) return phi;
        if (v < plo) return plo;
        return v;
    };

    // §2.6 원칙: **섀도에는 bridge 가 소유한 것만 쓴다** (= write 범위 안의 것만).
    //   범위 밖을 덮어쓰면 read 세그로 들어온 ECU 실값(진실 원천)을 지워버린다.
    switch (am) {
        case ecu::AUTO_MODE_CURRENT:   // write 164:16
            for (int i = 0; i < 4; i++)
                cm.cmd_current[i] = clamp_a(clamp_prof(cmd_active ? w.current[i] : 0.0f));
            break;

        case ecu::AUTO_MODE_VELOCITY:  // write 148:16
            for (int i = 0; i < 4; i++)
                cm.cmd_velocity[i] = cmd_active ? clamp_prof(w.current[i]) : 0.0f;
            break;

        case ecu::AUTO_MODE_POSITION:  // write 132:16
            // **매 tick 재시드**이지 latch 가 아니다. IDLE 중 외력으로 트랙이 밀려도 명령이
            // 따라오므로 RUNNING 진입 순간의 변위가 항상 0 이다.
            // 측정 노이즈가 드리프트를 만들지 않는 이유: IDLE 에서는 명령이 실측을 따라갈 뿐
            // 실측이 명령을 따라가지 않아 폐루프가 닫히지 않는다.
            // 이 규칙이 §3.3 "범위 진입 전 shadow 소독" 도 겸한다 — cmd_position 은 그전까지
            // 어느 프리셋에도 안 읽히던 자리라 낡은 값이 남아 있는데, 전환 직후 재시드가 덮는다.
            for (int i = 0; i < 4; i++)
                cm.cmd_position[i] = cmd_active ? clamp_prof(w.current[i]) : fb_pos(i);
            break;

        case ecu::AUTO_MODE_DIRECT: {   // write 128:52 — ctr_mode 까지 bridge 소유
            // 펄스 (01 §6.3 C-1): ARMED 면 이번 tick 만 SET_ORIGIN 을 싣고, **다음 tick 에
            // 무조건 원복**한다. write_err 여부를 보지 않는다 — 엣지 명령을 재시도하면
            // 의도가 두 번 실행된다.
            {
                std::lock_guard<std::mutex> plock(mutex_);
                if (pulse_ == PulseState::SENT) {
                    std::lock_guard<std::mutex> dl(ctr_mutex_);
                    for (int i = 0; i < 4; i++) ctr_mode_[i] = pulse_restore_[i];
                    pulse_ = PulseState::NONE;
                } else if (pulse_ == PulseState::ARMED) {
                    std::lock_guard<std::mutex> dl(ctr_mutex_);
                    for (int i = 0; i < 4; i++) ctr_mode_[i] = ecu::CTR_MODE_SET_ORIGIN;
                    pulse_ = PulseState::SENT;
                }
            }
            std::lock_guard<std::mutex> dlock(ctr_mutex_);
            for (int i = 0; i < 4; i++) {
                // **DIRECT 에서는 모터별 ctr_mode 가 단위를 정한다** (01 §6.1.2 표 마지막 행).
                // 명령 중이라고 CURRENT 로 강제하지 않는다 — 구 코드는 그렇게 해서
                // w.current[i] 를 단위와 무관하게 cmd_current 에 꽂았다. position 프로파일이면
                // **각도가 암페어로 나갔고**, clamp_prof 가 아니라 clamp_a([A]) 로 잘려
                // 90도 샘플이 클램프 상한(기본 30A)까지 갔다. AcceptsAutoMode 는 바로 이
                // 조합(DIRECT + ctr_mode=POSITION + mode:position)을 수락하므로 도달 가능했다.
                const uint8_t mode = ctr_mode_[i];
                cm.ctr_mode[i] = mode;

                // 안전값은 비-DIRECT 분기와 같은 규칙이다 (01 §6.1.2):
                //   CURRENT 0A / VELOCITY 0RPM / POSITION fb_position(현재 자세 유지)
                // ESTOP·SET_ORIGIN 등 나머지는 전 채널 0 — 펄스 tick 이 여기로 온다.
                float cur = 0.0f, vel = 0.0f, pos = 0.0f;
                switch (mode) {
                    case ecu::CTR_MODE_CURRENT:
                    case 2:  /* CURRENT_BRAKE — AcceptsAutoMode 가 current 로 인정하는 값 */
                        cur = clamp_a(clamp_prof(cmd_active ? w.current[i] : 0.0f));
                        break;
                    case ecu::CTR_MODE_VELOCITY:
                        vel = cmd_active ? clamp_prof(w.current[i]) : 0.0f;
                        break;
                    case ecu::CTR_MODE_POSITION:
                        pos = cmd_active ? clamp_prof(w.current[i]) : fb_pos(i);
                        break;
                    default:
                        break;   // ESTOP(0) / SET_ORIGIN(5) — 전 채널 0
                }
                cm.cmd_current[i]  = cur;
                cm.cmd_velocity[i] = vel;
                cm.cmd_position[i] = pos;
            }
            break;
        }

        default:
            // KINEMATIC(0) 은 project 전용, CONTROL(3) 은 ECU 미구현 — 기동 검증이 막는다.
            break;
    }
}

} // namespace orin_bridge
