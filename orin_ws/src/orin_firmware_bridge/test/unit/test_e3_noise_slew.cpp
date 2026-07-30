// E3 — `noise` 는 slew 검사 면제 (redesign/05 §3.4)
//
// 무엇이 문제였나: `slew_rate` 위반은 자동 성형이 아니라 **reject** 인데(§4.3-4),
// 백색 가우시안 잡음은 정의상 tick 간 델타가 무제한이다. `std` 가 아무리 작아도 긴
// 프로파일 어딘가는 반드시 한계를 넘는다. **두 규칙을 함께 지키면 noise 세그먼트는
// 사실상 항상 거부된다** — 문서가 예고했고 2026-07-29 실기에서 그대로 재현됐다
// (`std=0.2A` 가 `53.6 A/s > 50` 으로 거부).
//
// 대신 진폭 쪽 안전장치를 둔다: `mean ± 4σ` 가 실효 클램프를 벗어나면 거부.
// 클램프가 분포의 꼬리를 잘라내면 **재생된 잡음이 YAML 이 선언한 정규분포와 달라지는데**,
// 시스템 식별 입력이 선언과 다르면 동정 결과가 틀린다.

#include <gtest/gtest.h>

#include <string>

#include "orin_firmware_bridge/policy/rd_profile.hpp"

namespace {

using orin_bridge::RdProfile;

constexpr uint8_t kAll = 0x0F;

std::string NoiseYaml(double std_dev, double slew, double max_abs) {
    return "name: e3\nmode: current\n"
           "limits: {max_abs: " + std::to_string(max_abs) +
           ", slew_rate: " + std::to_string(slew) + "}\n"
           "motors:\n  m1:\n"
           "    - {type: hold,  duration: 0.5, value: 0.0}\n"
           "    - {type: noise, duration: 2.0, mean: 0.0, std: " + std::to_string(std_dev) + "}\n"
           "    - {type: hold,  duration: 0.5, value: 0.0}\n";
}

// ★ 이것이 E3 의 전부다 — 구 코드에서는 이 프로파일이 거부됐다.
TEST(E3NoiseSlew, NoiseIsExemptFromSlewCheck) {
    RdProfile p;
    std::string err;
    // slew_rate 50 A/s 는 200Hz 에서 tick 당 0.25A — std 0.2A 잡음이 수시로 넘는다.
    ASSERT_TRUE(p.LoadFromYaml(NoiseYaml(0.2, 50.0, 5.0), kAll, 30.0f, &err)) << err;
    EXPECT_GT(p.SlewExemptTicks(), 0u) << "면제가 한 번도 안 일어났다면 검사 자체가 안 돈 것";
}

// 면제 구간의 길이는 **noise 세그먼트 길이와 같은 규모**여야 한다.
// (경계 tick 이 함께 빠지므로 정확히 같지는 않다 — 그 델타도 잡음이 만든 것이다.)
TEST(E3NoiseSlew, ExemptCountMatchesNoiseSegment) {
    RdProfile p;
    std::string err;
    ASSERT_TRUE(p.LoadFromYaml(NoiseYaml(0.2, 50.0, 5.0), kAll, 30.0f, &err)) << err;
    // noise 2.0s @200Hz = 400 tick. 진입 경계 1개가 더 빠진다.
    EXPECT_GE(p.SlewExemptTicks(), 400u);
    EXPECT_LE(p.SlewExemptTicks(), 402u);
}

// ★ 면제는 noise 에만 적용된다 — 다른 세그먼트의 slew 위반은 **여전히 거부**다.
//   면제를 전역으로 풀어 버리면 §4.3-4 가 통째로 죽는다.
TEST(E3NoiseSlew, NonNoiseSegmentsStillRejected) {
    RdProfile p;
    std::string err;
    // 0 -> 5A 를 1 tick 만에: 1000 A/s. 한계 50 A/s.
    const std::string yaml =
        "name: e3b\nmode: current\nlimits: {max_abs: 10.0, slew_rate: 50.0}\n"
        "motors:\n  m1:\n"
        "    - {type: hold, duration: 0.5, value: 0.0}\n"
        "    - {type: hold, duration: 0.5, value: 5.0}\n";
    EXPECT_FALSE(p.LoadFromYaml(yaml, kAll, 30.0f, &err));
    EXPECT_NE(err.find("slew_rate"), std::string::npos) << err;
}

// ── E3-3 진폭 가드 ────────────────────────────────────────────────────────────
// mean ± 4σ 가 실효 클램프를 벗어나면 거부. 4σ 인 이유: 720,000 tick 에서 기대 초과
// 횟수 약 45회 — 실무상 "거의 안 잘린다" 는 뜻이다.
TEST(E3NoiseSlew, AmplitudeGuardRejectsWhenTailWouldBeClipped) {
    RdProfile p;
    std::string err;
    // max_abs 1.0 인데 std 0.5 → 4σ = 2.0 이 한계를 넘는다.
    EXPECT_FALSE(p.LoadFromYaml(NoiseYaml(0.5, 50.0, 1.0), kAll, 30.0f, &err));
    EXPECT_NE(err.find("4"), std::string::npos) << err;
    EXPECT_NE(err.find("noise"), std::string::npos) << err;
}

TEST(E3NoiseSlew, AmplitudeGuardAcceptsWhenTailFits) {
    RdProfile p;
    std::string err;
    // max_abs 5.0, std 0.2 → 4σ = 0.8 ≪ 5.0
    EXPECT_TRUE(p.LoadFromYaml(NoiseYaml(0.2, 50.0, 5.0), kAll, 30.0f, &err)) << err;
}

// slew_rate 를 아예 안 준 프로파일은 면제 계산이 돌지 않는다 (검사 자체가 없다).
TEST(E3NoiseSlew, NoSlewLimitMeansNoExemptCount) {
    RdProfile p;
    std::string err;
    const std::string yaml =
        "name: e3c\nmode: current\nlimits: {max_abs: 5.0}\n"
        "motors:\n  m1:\n    - {type: noise, duration: 1.0, mean: 0.0, std: 0.2}\n";
    ASSERT_TRUE(p.LoadFromYaml(yaml, kAll, 30.0f, &err)) << err;
    EXPECT_EQ(p.SlewExemptTicks(), 0u);
}

}  // namespace
