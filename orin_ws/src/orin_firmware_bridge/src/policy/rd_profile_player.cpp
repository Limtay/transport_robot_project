#include "orin_firmware_bridge/policy/rd_profile_player.hpp"

namespace orin_bridge {

bool RdProfilePlayer::Load(const std::string& yaml, uint8_t motor_mask, std::string* err) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 클램프 상한은 프로파일 자체가 들고 있지 않다 — 호출자가 BridgeConfig 에서 넘긴다.
    // (이 클래스는 설정을 모른다: L2 는 L3 파라미터를 참조하지 않는다)
    return profile_.LoadFromYaml(yaml, motor_mask, clamp_max_, err);
}

uint32_t RdProfilePlayer::Begin() {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint32_t gid = next_goal_id_++;
    active_goal_id_ = gid;
    tick_ = 0;
    done_ = false;
    return gid;
}

void RdProfilePlayer::Activate() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;   // 이 시점부터 스케줄 tick 이 샘플을 소비한다
}

size_t RdProfilePlayer::End() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    active_goal_id_ = 0;
    return tick_;
}

void RdProfilePlayer::Tick(RdControl* control) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || done_) return;

    float c[4]; uint16_t seg = 0;
    if (profile_.SampleAt(tick_, c, &seg)) {
        control->SetProfileSample(c, static_cast<float>(tick_ * RdProfile::kDt), seg);
        tick_++;
    } else {
        done_ = true;   // 마지막 tick 소비 완료 — 액션 실행부가 정리한다
    }
}

bool RdProfilePlayer::AcceptsAutoMode(uint8_t auto_mode, const uint8_t ctr_mode[4],
                                     uint8_t active_mask, std::string* err) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.AcceptsAutoMode(auto_mode, ctr_mode, active_mask, err);
}

void RdProfilePlayer::EffectiveLimits(float* lo, float* hi) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lo) *lo = profile_.limit_lo();
    if (hi) *hi = profile_.limit_hi();
}

const char* RdProfilePlayer::ModeStr() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return RdProfile::ModeStr(profile_.mode());
}

uint8_t RdProfilePlayer::ModeNum() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.ModeNum();
}

uint64_t RdProfilePlayer::Seed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.Seed();
}

uint32_t RdProfilePlayer::SlewExemptTicks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.SlewExemptTicks();
}

void RdProfilePlayer::Progress(size_t* tick, bool* done) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tick) *tick = tick_;
    if (done) *done = done_;
}

bool RdProfilePlayer::SampleAt(size_t tick, float out[4], uint16_t* seg) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.SampleAt(tick, out, seg);
}

std::string RdProfilePlayer::Name() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.Name();
}
double RdProfilePlayer::DurationSec() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.DurationSec();
}
size_t RdProfilePlayer::TickCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.TickCount();
}
uint32_t RdProfilePlayer::ClampCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.ClampCount();
}
uint32_t RdProfilePlayer::ActiveGoalId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_goal_id_;
}

}  // namespace orin_bridge
