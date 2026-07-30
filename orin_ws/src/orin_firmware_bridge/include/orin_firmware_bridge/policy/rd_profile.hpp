#ifndef ORIN_FIRMWARE_BRIDGE__RD_PROFILE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_PROFILE_HPP_

#include <string>
#include <vector>
#include <cstdint>

namespace orin_bridge {

// 프로파일 재생기 (testbed_spec.md §4)
//
// 역할은 "파싱 → 검증 → 사전 샘플링" 까지다. 재생 중에는 배열 인덱싱만 하고
// 파싱·수식 연산을 일절 하지 않는다 (§4.3 규칙 5 — 200Hz 예산 보호).
//
// 단위: **v1 player 는 CURRENT[A] 전용** (spec §7-0 확정, 2026-07-19 노트북 회신).
// SET_CTR_MODE(C-5)와 프로파일 재생의 모드 혼재는 TEST4 확장 예약 — 구현하지 않는다.
// RUNNING 재생은 상태와 무관하게 항상 CURRENT 로 write 한다.
class RdProfile {
public:
    static constexpr int    kTickHz       = 200;
    static constexpr double kDt           = 1.0 / kTickHz;
    static constexpr double kMaxDuration  = 3600.0;  // [s] 사전 샘플링 메모리 상한 (1시간)
    // 05 §3.2 — 세그먼트를 **펼치기 전에** 이 값으로 검사한다. 상한을 넘는 프로파일은
    // 메모리를 한 바이트도 쓰기 전에 거부된다.
    static constexpr size_t kMaxTicks     = static_cast<size_t>(kMaxDuration) * kTickHz;

    // YAML 전문을 파싱·검증하고 전 구간을 200Hz 배열로 사전 샘플링한다.
    // 실패 시 false + err 에 사유(가능하면 YAML 위치·모터·세그먼트 번호 포함).
    // 프로파일이 요구하는 제어 모드 (05 §2.2). auto_mode 수락 규칙의 좌변이 된다.
    enum class Mode : uint8_t { CURRENT = 0, VELOCITY, POSITION };
    static const char* ModeStr(Mode m);
    Mode     mode()     const { return mode_; }
    // 재생 중 유효한 클램프 [lo, hi] — **단위는 mode 가 정한다** (05 §2.4).
    // wire 직전 클램프가 cmd_current_max 를 무조건 쓰던 것을 이것으로 대체한다.
    float    limit_lo() const { return limit_lo_; }
    float    limit_hi() const { return limit_hi_; }

    // 05 §2.3 수락 규칙 — mode <-> auto_mode <-> ctr_mode 2단 검사.
    // 판정 입력은 **shadow 의 auto_mode** (01 §7 권위 모델) — read-back 세그를 새로 만들지 않는다.
    // DIRECT 일 때만 ctr_mode 까지 내려간다 (그때는 ctr_mode 가 bridge 소유라 신뢰 가능).
    bool AcceptsAutoMode(uint8_t auto_mode, const uint8_t ctr_mode[4],
                         uint8_t active_mask, std::string* err) const;

    // active_mask: §3.1 활성 모터 비트필드 / global_max_current: 노드 파라미터 cmd_current_max.
    bool LoadFromYaml(const std::string& yaml_text, uint8_t active_mask,
                      float global_max_current, std::string* err);

    // 사전 샘플링 결과 조회 — 재생 tick 에서 호출 (연산 없음).
    // tick 이 범위를 넘으면 false (= 프로파일 종료).
    bool SampleAt(size_t tick, float out_current[4], uint16_t* out_segment_index) const;

    size_t   TickCount()  const { return tick_count_; }
    double   DurationSec() const { return tick_count_ * kDt; }
    uint32_t ClampCount() const { return clamp_cnt_; }   // §4.3 규칙 3 발동 횟수
    const std::string& Name() const { return name_; }

    // **실효 시드** (05 §5.2). `seed:` 미지정이면 시각 기반으로 정해지는데, 종전에는
    // 그 값이 LoadFromYaml 의 지역변수로만 존재해 재생이 끝나면 영원히 사라졌다.
    // noise/prbs 프로파일은 그러면 **두 번 다시 같은 파형을 만들 수 없다.**
    uint64_t Seed() const { return seed_; }

    // E3 (05 §3.4) — slew 검사를 건너뛴 tick 수. `noise` 구간의 길이다.
    // result.json 에 남긴다: 그 구간은 레이트 제한이 없었다는 사실을 분석이 알아야 한다.
    uint32_t SlewExemptTicks() const { return slew_exempt_ticks_; }

    // result.json 의 `mode` 는 숫자로 나간다 (0=current 1=velocity 2=position).
    // enum 값이 그 규약과 같으므로 캐스팅이 곧 계약이다 — 어긋나면 테스트가 잡는다.
    uint8_t  ModeNum() const { return static_cast<uint8_t>(mode_); }

private:
    std::string name_;
    size_t      tick_count_ = 0;
    uint32_t    clamp_cnt_  = 0;
    uint64_t    seed_       = 0;
    uint32_t    slew_exempt_ticks_ = 0;
    Mode        mode_       = Mode::CURRENT;
    float       limit_lo_   = 0.0f;
    float       limit_hi_   = 0.0f;

    // [4][tick] — 모터별 사전 샘플. 미지정 모터는 전 구간 0 (§4.3 규칙 2).
    std::vector<float> samples_[4];
    // 피드백용 세그먼트 인덱스: 기준 모터(세그먼트 수가 가장 많은 모터, 동수면 번호 작은 쪽)의 것.
    // spec §7-0 승인 확정 (2026-07-19) — 분석의 권위 소스는 YAML+profile_time 이므로
    // 이 필드의 정밀화에 추가 공수를 쓰지 않는다 (보조 지표).
    std::vector<uint16_t> segment_index_;
};

} // namespace orin_bridge

#endif
