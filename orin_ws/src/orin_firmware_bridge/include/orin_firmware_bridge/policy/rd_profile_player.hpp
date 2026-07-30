#ifndef ORIN_FIRMWARE_BRIDGE__RD_PROFILE_PLAYER_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_PROFILE_PLAYER_HPP_

// RdProfilePlayer — 프로파일 **재생 진행** (redesign/02 A2 신설, §5.2)
//
// `RdProfile`(파싱·검증·사전 샘플링)과 나누는 이유는 **스레드가 다르기 때문**이다:
//   - 파싱·검증은 action 콜백 스레드에서 (수십 ms 걸려도 무방)
//   - 재생은 200Hz 스케줄 스레드에서 배열 인덱싱만 (수 µs)
// 같은 클래스에 두면 락 범위 설계가 어려워진다. 데이터(샘플 배열)와 진행(인덱스)을 분리한다.
//
// 왜 L3 에서 떼어냈나: `TickProfile()` 이 노드 메서드라 L1 스케줄러가 매 tick 노드를 거꾸로
// 참조했다 (A1-(c)). 재생 자체에 ROS 는 하나도 안 쓰인다.

#include <cstdint>
#include <mutex>
#include <string>

#include "orin_firmware_bridge/policy/rd_profile.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"

namespace orin_bridge {

class RdProfilePlayer {
public:
    // 클램프 상한 [A] — L3 가 BridgeConfig 에서 1회 주입한다.
    // (L2 는 설정을 직접 읽지 않는다 — 그게 A1-a 로 끊은 방향이다)
    void SetClampMax(float a) { clamp_max_ = a; }

    // [action 스레드] YAML 로드·검증. 실패 시 false + err 에 사유.
    bool Load(const std::string& yaml, uint8_t motor_mask, std::string* err);

    // [action 스레드] 재생 시작 — goal_id 채번 후 반환. 인덱스·완료 플래그 초기화.
    uint32_t Begin();
    // [action 스레드] 재생 개시 표시. 이 시점부터 스케줄 tick 이 샘플을 소비한다.
    void Activate();
    // [action 스레드] 재생 종료(완료/취소/에러 공통). 소비한 tick 수를 반환.
    size_t End();

    // [스케줄 스레드] 매 tick — 사전 샘플링 배열에서 이번 tick 값을 꺼내 FSM 에 넣는다.
    // RUNNING 이 아니면 no-op (write 소스 결정은 FSM 이 한다).
    void Tick(RdControl* control);

    // [action 스레드] 진행 상황 스냅샷 (5Hz 피드백용)
    void Progress(size_t* tick, bool* done) const;
    bool SampleAt(size_t tick, float out[4], uint16_t* seg) const;

    // 프로파일 메타 — 로드 후 불변
    std::string Name() const;
    double      DurationSec() const;
    size_t      TickCount() const;
    uint32_t    ClampCount() const;
    uint32_t    ActiveGoalId() const;

    // 05 §2.3 수락 규칙 — goal 수락 시 **1회만** 판정한다.
    // 재생 중 auto_mode 는 IDLE 에서만 바뀌므로 재검사가 불필요하다.
    bool AcceptsAutoMode(uint8_t auto_mode, const uint8_t ctr_mode[4],
                         uint8_t active_mask, std::string* err) const;
    // 재생 중 유효한 클램프 — 단위는 프로파일의 mode 가 정한다 (05 §2.4).
    void EffectiveLimits(float* lo, float* hi) const;
    const char* ModeStr() const;
    uint8_t     ModeNum() const;   // 0=current 1=velocity 2=position (05 §5.2 result.json)
    uint64_t    Seed()    const;   // 실효 시드 — 미지정이면 브리지가 정한 값
    uint32_t    SlewExemptTicks() const;   // E3 — noise 구간 길이 (slew 면제)

private:
    mutable std::mutex mutex_;
    RdProfile profile_;
    float     clamp_max_ = 30.0f;
    size_t    tick_    = 0;
    bool      active_  = false;
    bool      done_    = false;
    uint32_t  next_goal_id_   = 1;   // 세션 내 단조증가 (전역 유일성은 bag 폴더명이 담당)
    uint32_t  active_goal_id_ = 0;
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_PROFILE_PLAYER_HPP_
