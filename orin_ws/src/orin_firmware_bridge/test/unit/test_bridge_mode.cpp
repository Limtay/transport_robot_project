// bridge_mode 열거형 + manual (redesign/00 Q1·R1·R2 / 01 §4)
//
// 종전에는 `control_mode`·`traction_test_mode` 두 불리언이었고, 둘 다 false 인 상태가
// project 를 뜻했다. 축이 둘이면 **표현 불가능한 조합**(둘 다 true)이 생기고, 실제로
// 코드에 "동시 지정 시 traction 우선" 이라는 특례가 있었다. 열거형은 그 조합을 없앤다.
//
// manual 은 "안전장치가 없는 모드" 가 아니라 **"자동 설정이 없는 모드"** 다 (01 §4.1) —
// 그 구분을 테스트로 못박는다. 여기가 흐려지면 manual 이 "아무거나 되는 모드" 가 된다.

#include <gtest/gtest.h>

#include <string>

#include "orin_firmware_bridge/rd_config.hpp"

namespace {

using orin_bridge::BridgeMode;
using orin_bridge::BridgeConfig;
using orin_bridge::BridgeModeName;
using orin_bridge::ParseBridgeMode;

TEST(BridgeModeEnum, ParsesEveryDocumentedName) {
    BridgeMode m;
    ASSERT_TRUE(ParseBridgeMode("project", &m));  EXPECT_EQ(m, BridgeMode::PROJECT);
    ASSERT_TRUE(ParseBridgeMode("control", &m));  EXPECT_EQ(m, BridgeMode::CONTROL);
    ASSERT_TRUE(ParseBridgeMode("manual", &m));   EXPECT_EQ(m, BridgeMode::MANUAL);
}

// ★ 오타는 **거부**한다. 기본값으로 떨어뜨리면 조작자가 의도한 모드가 아닌 채로 뜬다 —
//   이 프로젝트에서 가장 비싼 실수다 (모터가 도는 모드인지 아닌지가 갈린다).
TEST(BridgeModeEnum, RejectsUnknownNameInsteadOfFallingBack) {
    BridgeMode m = BridgeMode::CONTROL;
    EXPECT_FALSE(ParseBridgeMode("traction", &m)) << "traction 은 모드가 아니라 control 의 구성이다";
    EXPECT_FALSE(ParseBridgeMode("Control", &m)) << "대소문자도 구분한다";
    EXPECT_FALSE(ParseBridgeMode("contorl", &m));
    EXPECT_FALSE(ParseBridgeMode("", &m));
    EXPECT_EQ(m, BridgeMode::CONTROL) << "실패 시 출력을 건드리지 않는다";
}

TEST(BridgeModeEnum, NameRoundTrips) {
    for (auto m : {BridgeMode::PROJECT, BridgeMode::CONTROL, BridgeMode::MANUAL}) {
        BridgeMode back;
        ASSERT_TRUE(ParseBridgeMode(BridgeModeName(m), &back)) << BridgeModeName(m);
        EXPECT_EQ(back, m);
    }
}

// 술어는 **서로 배타적**이다 — 두 불리언 시절의 "둘 다 true" 가 사라졌다는 확인.
TEST(BridgeModeEnum, PredicatesAreMutuallyExclusive) {
    for (auto m : {BridgeMode::PROJECT, BridgeMode::CONTROL, BridgeMode::MANUAL}) {
        BridgeConfig c;
        c.bridge_mode = m;
        const int n = int(c.IsProject()) + int(c.IsControl()) + int(c.IsManual());
        EXPECT_EQ(n, 1) << BridgeModeName(m) << " 에서 술어가 " << n << "개 참";
    }
}

// ★ manual 만 자동 설정을 하지 않는다 (01 §4.1).
TEST(BridgeModeEnum, OnlyManualSkipsAutoConfiguration) {
    BridgeConfig c;
    for (auto m : {BridgeMode::PROJECT, BridgeMode::CONTROL}) {
        c.bridge_mode = m;
        EXPECT_TRUE(c.AutoConfiguresEcu()) << BridgeModeName(m) << " 가 자동 설정을 건너뛴다";
    }
    c.bridge_mode = BridgeMode::MANUAL;
    EXPECT_FALSE(c.AutoConfiguresEcu());
}

// 기본값은 project — 아무 파라미터도 안 주면 종전과 같은 동작이어야 한다.
TEST(BridgeModeEnum, DefaultIsProject) {
    BridgeConfig c;
    EXPECT_EQ(c.bridge_mode, BridgeMode::PROJECT);
    EXPECT_TRUE(c.IsProject());
    EXPECT_TRUE(c.bridge_mode_valid);
}

}  // namespace
