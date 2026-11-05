#ifndef ORIN_FIRMWARE_BRIDGE__RD_POS_FRAME_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_POS_FRAME_HPP_

// RdPosFrame — INST_POS 경로의 절대 좌표 추적 (redesign/12 §4, 계약은 11 §9)
//
// ECU 는 |fb| 가 3000° 에 닿으면 스스로 원점을 옮기고 `origin_seq` 를 올린다.
// Orin 이 할 일은 "seq 가 바뀐 응답에서 offset += pre_origin_pose" 뿐이다:
//
//     abs_deg = offset + mtr_position·0.1
//     cmd_rel = cmd_abs − offset
//
// 절대좌표의 0 은 이 세션의 시작 위치다. 호밍은 없다 — 프로파일이 시작점 기준 변위라 필요 없다.
//
// 스레드: 자체 락이 없다. 소유자 RdControl 의 mutex_ 안에서만 부른다.

#include <cstdint>
#include <string>

#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

namespace orin_bridge {

class RdPosFrame {
public:
    enum class Verdict : uint8_t {
        OK = 0,
        REINIT,   // IDLE 중 ECU 리부트 — 프레임을 새로 잡았다 (WARN 만)
        LOCK,     // 좌표를 믿을 수 없다 — 호출자가 LOCKED 로 래치한다
    };

    // 응답마다 1회. LOCK/REINIT 이면 reason 에 사유를 채운다.
    // running = RUNNING 중인가 (리부트의 처리가 갈린다).
    Verdict Update(const ecu::PAGE_POS_FB_t& fb, uint8_t active_mask, bool running,
                   std::string* reason);

    // 프레임을 버린다. 다음 Update 가 첫 응답으로 취급한다 (offset = 0).
    void Reset();

    // 좌표가 무효화되어 REARM 때 Reset 이 필요한가 (리부트·seq 유실·FRAME_LOST).
    bool NeedsReset() const { return needs_reset_; }
    bool Initialized() const { return initialized_; }

    // 프로파일 시작점 래치. 활성 모터가 PEND 중이거나 첫 응답 전이면 false.
    bool LatchStart(uint8_t active_mask, std::string* why);

    // RUNNING 중 |cmd_abs − abs| 검사. PEND 로 abs 가 유지된 모터는 건너뛴다.
    // 위반이면 false + why. max_track_err 는 위반 여부와 무관하게 갱신한다.
    bool CheckTrack(double limit_deg, uint8_t active_mask, std::string* why);

    void   SetCmdAbs(int i, double v) { cmd_abs_deg_[i] = v; }
    double CmdAbs(int i) const { return cmd_abs_deg_[i]; }
    double Offset(int i) const { return offset_deg_[i]; }
    double Abs(int i) const { return abs_deg_[i]; }
    double StartAbs(int i) const { return start_abs_deg_[i]; }
    double MaxTrackErr(int i) const { return max_track_err_[i]; }
    uint8_t OriginCnt(int i) const { return origin_cnt_[i]; }
    bool   AnyHeld() const { return held_mask_ != 0; }
    // 현재 FRAME_LOST 인 활성 모터가 있는가 — ECU 래치 해제 절차가 필요한지 판단한다.
    bool   FrameLost() const { return frame_lost_; }

private:
    void Seed(const ecu::PAGE_POS_FB_t& fb);

    bool     initialized_ = false;
    bool     needs_reset_ = false;
    bool     frame_lost_  = false;
    uint32_t last_tick_   = 0;
    uint8_t  held_mask_   = 0;       // 이번 응답에서 abs 를 유지한 모터
    uint8_t  last_seq_[4]      = {0, 0, 0, 0};
    uint8_t  origin_cnt_[4]    = {0, 0, 0, 0};
    double   offset_deg_[4]    = {0.0, 0.0, 0.0, 0.0};
    double   abs_deg_[4]       = {0.0, 0.0, 0.0, 0.0};
    double   start_abs_deg_[4] = {0.0, 0.0, 0.0, 0.0};
    double   cmd_abs_deg_[4]   = {0.0, 0.0, 0.0, 0.0};
    double   max_track_err_[4] = {0.0, 0.0, 0.0, 0.0};
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_POS_FRAME_HPP_
