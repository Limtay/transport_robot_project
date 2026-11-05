#include "orin_firmware_bridge/policy/rd_pos_frame.hpp"

#include <cmath>
#include <limits>

namespace orin_bridge {

namespace {
constexpr double kPosScale = 0.1;   // x0.1 [deg]
}

void RdPosFrame::Reset() {
    *this = RdPosFrame{};
}

void RdPosFrame::Seed(const ecu::PAGE_POS_FB_t& fb) {
    for (int i = 0; i < 4; i++) {
        last_seq_[i]      = fb.origin_seq[i];
        offset_deg_[i]    = 0.0;
        abs_deg_[i]       = fb.mtr_position[i] * kPosScale;
        origin_cnt_[i]    = 0;
        max_track_err_[i] = 0.0;
        start_abs_deg_[i] = abs_deg_[i];
        cmd_abs_deg_[i]   = std::numeric_limits<double>::quiet_NaN();
    }
    last_tick_   = fb.realtime_tick;
    held_mask_   = 0;
    initialized_ = true;
}

RdPosFrame::Verdict RdPosFrame::Update(const ecu::PAGE_POS_FB_t& fb, uint8_t active_mask,
                                       bool running, std::string* reason) {
    // 이미 좌표가 무효다. REARM 전까지 낡은 offset 으로 abs 를 계산하지 않는다.
    if (needs_reset_) return Verdict::OK;

    if (!initialized_) Seed(fb);

    // TIM5 는 free-run 이라 랩어라운드한다 — 부호 있는 차이로 본다.
    // seq 도 mod 256 이다. 255→0 은 +1 이지 감소가 아니다.
    bool rebooted = static_cast<int32_t>(fb.realtime_tick - last_tick_) < 0;
    for (int i = 0; i < 4; i++) {
        if (static_cast<int8_t>(fb.origin_seq[i] - last_seq_[i]) < 0) rebooted = true;
    }
    if (rebooted) {
        if (running) {
            needs_reset_ = true;
            if (reason) *reason = "ECU 리부트 감지 (tick 역행 또는 seq 감소) — 좌표 무효";
            return Verdict::LOCK;
        }
        Seed(fb);
        if (reason) *reason = "IDLE 중 ECU 리부트 감지 — 프레임 재초기화";
        return Verdict::REINIT;
    }

    for (int i = 0; i < 4; i++) {
        const uint8_t diff = static_cast<uint8_t>(fb.origin_seq[i] - last_seq_[i]);
        if (diff >= 2) {
            needs_reset_ = true;
            if (reason) {
                *reason = "M" + std::to_string(i + 1) + " 원점 이벤트 유실 (seq " +
                          std::to_string(last_seq_[i]) + " -> " +
                          std::to_string(fb.origin_seq[i]) + ")";
            }
            return Verdict::LOCK;
        }
    }

    const uint8_t lost = static_cast<uint8_t>((fb.origin_flag >> 4) & active_mask & 0x0F);
    frame_lost_ = (lost != 0);
    if (lost != 0) {
        needs_reset_ = true;
        if (reason) {
            *reason = "FRAME_LOST mask=0x" + std::to_string(lost) +
                      " — ECU 제동 중. POSITION 이탈 후 재진입 필요 (REARM)";
        }
        return Verdict::LOCK;
    }

    held_mask_ = 0;
    for (int i = 0; i < 4; i++) {
        const bool seq_changed = (fb.origin_seq[i] != last_seq_[i]);
        const bool pend        = (fb.origin_flag >> i) & 0x01;
        if (seq_changed) {
            offset_deg_[i] += fb.pre_origin_pose[i] * kPosScale;
            origin_cnt_[i]++;
            last_seq_[i] = fb.origin_seq[i];
        } else if (pend) {
            // 11 §9-2: 이 샘플의 fb 는 어느 좌표인지 모른다. 쓰면 abs 가 한 샘플 −3000° 로 튄다.
            held_mask_ |= static_cast<uint8_t>(1u << i);
            continue;
        }
        abs_deg_[i] = offset_deg_[i] + fb.mtr_position[i] * kPosScale;
    }
    last_tick_ = fb.realtime_tick;
    return Verdict::OK;
}

bool RdPosFrame::LatchStart(uint8_t active_mask, std::string* why) {
    if (!initialized_ || needs_reset_) {
        if (why) *why = "좌표 프레임이 유효하지 않다 (첫 응답 전이거나 REARM 필요)";
        return false;
    }
    const uint8_t pend = static_cast<uint8_t>(held_mask_ & active_mask);
    if (pend != 0) {
        if (why) *why = "활성 모터 원점 이동 중 (PEND) — 잠시 후 다시 시도할 것";
        return false;
    }
    for (int i = 0; i < 4; i++) {
        start_abs_deg_[i] = abs_deg_[i];
        max_track_err_[i] = 0.0;
    }
    return true;
}

bool RdPosFrame::CheckTrack(double limit_deg, uint8_t active_mask, std::string* why) {
    bool ok = true;
    for (int i = 0; i < 4; i++) {
        if (!((active_mask >> i) & 0x01) || ((held_mask_ >> i) & 0x01)) continue;
        if (std::isnan(cmd_abs_deg_[i])) continue;
        const double err = std::fabs(cmd_abs_deg_[i] - abs_deg_[i]);
        if (err > max_track_err_[i]) max_track_err_[i] = err;
        if (ok && err > limit_deg) {
            ok = false;
            if (why) {
                *why = "M" + std::to_string(i + 1) + " 추종 오차 " + std::to_string(err) +
                       " deg > 한계 " + std::to_string(limit_deg) + " deg";
            }
        }
    }
    return ok;
}

}  // namespace orin_bridge
