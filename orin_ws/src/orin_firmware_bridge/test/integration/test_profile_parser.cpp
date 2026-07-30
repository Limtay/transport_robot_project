// C-4a 프로파일 파서·검증·사전 샘플링 (testbed_spec.md §4)
// 막는 것: 잘못된 YAML 이 조용히 수락돼 엉뚱한 파형이 모터로 나가는 것, 파형 수식 오류,
//          클램프/slew 규칙 누락, 그리고 사용자가 고칠 수 없는 불친절한 오류 메시지.
#include "rd_test_common.hpp"
#include "orin_firmware_bridge/policy/rd_profile.hpp"
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"
#include <cmath>

using namespace orin_bridge;

namespace {
bool Load(RdProfile& p, const std::string& y, std::string* err,
          uint8_t mask = 0x0F, float gmax = 30.0f) {
    return p.LoadFromYaml(y, mask, gmax, err);
}
// 거부를 기대하는 케이스 — 사유에 기대 키워드가 들어있는지까지 본다 (메시지 품질도 계약)
void ExpectReject(const std::string& y, const char* needle, uint8_t mask = 0x0F) {
    RdProfile p; std::string err;
    ASSERT_FALSE(p.LoadFromYaml(y, mask, 30.0f, &err)) << "거부돼야 하는데 수락됨";
    EXPECT_NE(err.find(needle), std::string::npos)
        << "사유에 '" << needle << "' 가 없다 — 실제: " << err;
}
}

TEST(ProfileParser, SpecExampleHysteresisRamp) {
    const std::string y = R"(
name: hysteresis_ramp_v1
seed: 42
limits: {max_current: 25.0}
motors:
  m2:
    - {type: hold, duration: 3.0, value: 0}
    - {type: ramp, duration: 20.0, from: 0, to: 10}
    - {type: hold, duration: 3.0, value: 10}
    - {type: ramp, duration: 20.0, from: 10, to: 0}
  m3:
    - {type: hold, duration: 46.0, value: 0}
)";
    RdProfile p; std::string err;
    ASSERT_TRUE(Load(p, y, &err)) << err;
    EXPECT_EQ(p.Name(), "hysteresis_ramp_v1");
    EXPECT_EQ(p.TickCount(), 46u * 200);
    EXPECT_NEAR(p.DurationSec(), 46.0, 1e-9);

    float c[4]; uint16_t seg;
    p.SampleAt(0, c, &seg);          EXPECT_EQ(seg, 0);   EXPECT_FLOAT_EQ(c[1], 0.0f);
    p.SampleAt(3 * 200, c, &seg);    EXPECT_EQ(seg, 1) << "3s 에 ramp 진입";
    p.SampleAt(23 * 200 - 1, c, &seg);
    EXPECT_NEAR(c[1], 10.0f, 1e-3) << "ramp 끝값이 정확히 to — 히스테리시스 상승/하강 접합점";
    p.SampleAt(13 * 200, c, &seg);   EXPECT_NEAR(c[1], 5.0f, 0.05f) << "ramp 선형성";
    p.SampleAt(9199, c, &seg);       EXPECT_NEAR(c[1], 0.0f, 1e-3);
    EXPECT_FLOAT_EQ(c[0], 0.0f) << "미지정 활성 모터는 전 구간 0A (§4.3-2)";
    EXPECT_FLOAT_EQ(c[3], 0.0f);
    EXPECT_FALSE(p.SampleAt(p.TickCount(), c, &seg)) << "범위 밖 = 종료 신호";
}

TEST(ProfileParser, SegmentTypeWaveforms) {
    float c[4]; uint16_t s;
    {   RdProfile p; std::string e;
        ASSERT_TRUE(Load(p, "motors: {m1: [{type: stair, values: [1,2,3], step_duration: 1.0}]}", &e)) << e;
        EXPECT_EQ(p.TickCount(), 600u);
        p.SampleAt(0, c, &s);   EXPECT_FLOAT_EQ(c[0], 1.0f);
        p.SampleAt(250, c, &s); EXPECT_FLOAT_EQ(c[0], 2.0f);
        p.SampleAt(550, c, &s); EXPECT_FLOAT_EQ(c[0], 3.0f);
    }
    {   RdProfile p; std::string e;
        ASSERT_TRUE(Load(p, "motors: {m1: [{type: step, duration: 2.0, from: 1, to: 5, t_step: 1.0}]}", &e)) << e;
        p.SampleAt(199, c, &s); EXPECT_FLOAT_EQ(c[0], 1.0f);
        p.SampleAt(200, c, &s); EXPECT_FLOAT_EQ(c[0], 5.0f) << "t_step 경계";
    }
    {   RdProfile p; std::string e;
        ASSERT_TRUE(Load(p, "motors: {m1: [{type: sine, duration: 2.0, amp: 5, freq: 1.0, offset: 2}]}", &e)) << e;
        p.SampleAt(0, c, &s);   EXPECT_NEAR(c[0], 2.0f, 1e-4) << "t=0 은 offset";
        p.SampleAt(50, c, &s);  EXPECT_NEAR(c[0], 7.0f, 1e-2) << "1/4 주기 = offset+amp";
        p.SampleAt(150, c, &s); EXPECT_NEAR(c[0], -3.0f, 1e-2) << "3/4 주기 = offset-amp";
    }
    {   RdProfile p; std::string e;
        ASSERT_TRUE(Load(p, "motors: {m1: [{type: chirp, duration: 5.0, amp: 3, f0: 1, f1: 10}]}", &e)) << e;
        EXPECT_EQ(p.TickCount(), 1000u);
        p.SampleAt(0, c, &s); EXPECT_NEAR(c[0], 0.0f, 1e-4);
        for (size_t t = 0; t < p.TickCount(); t++) {
            p.SampleAt(t, c, &s);
            ASSERT_LE(std::fabs(c[0]), 3.001f) << "chirp 이 진폭을 넘으면 안 된다 (tick " << t << ")";
        }
    }
    {   RdProfile p; std::string e;
        ASSERT_TRUE(Load(p, "motors: {m1: [{type: custom, samples: [1,2,3,4], rate: 200}]}", &e)) << e;
        EXPECT_EQ(p.TickCount(), 4u);
        p.SampleAt(2, c, &s); EXPECT_FLOAT_EQ(c[0], 3.0f) << "custom 샘플 순서 보존";
        RdProfile p2; std::string e2;
        ASSERT_TRUE(Load(p2, "motors: {m1: [{type: custom, samples: [0,10], rate: 100}]}", &e2)) << e2;
        EXPECT_EQ(p2.TickCount(), 4u) << "rate 100 -> 200Hz 리샘플 (웹 드로잉 경로)";
    }
}

TEST(ProfileParser, SeedReproducibility) {
    const char* y7  = "seed: 7\nmotors: {m1: [{type: prbs, duration: 2.0, low: -2, high: 4, bit_duration: 0.1}]}";
    const char* y99 = "seed: 99\nmotors: {m1: [{type: prbs, duration: 2.0, low: -2, high: 4, bit_duration: 0.1}]}";
    RdProfile a, b, c99; std::string e;
    ASSERT_TRUE(Load(a, y7, &e)); ASSERT_TRUE(Load(b, y7, &e)); ASSERT_TRUE(Load(c99, y99, &e));

    float x[4], y[4]; uint16_t s;
    bool same = true, diff = false;
    for (size_t t = 0; t < a.TickCount(); t++) {
        a.SampleAt(t, x, &s);
        ASSERT_TRUE(x[0] == -2.0f || x[0] == 4.0f) << "prbs 는 low/high 이진";
        b.SampleAt(t, y, &s);   if (x[0] != y[0]) same = false;
        c99.SampleAt(t, y, &s); if (x[0] != y[0]) diff = true;
    }
    EXPECT_TRUE(same) << "같은 seed 는 같은 파형이어야 실험 재현이 된다";
    EXPECT_TRUE(diff) << "다른 seed 는 다른 파형";
}

TEST(ProfileParser, ValidationRules) {
    // 규칙 1 — 지정 모터 ⊆ active_motors
    ExpectReject("motors: {m3: [{type: hold, duration: 1.0, value: 0}]}", "active_motors", 0x03);

    // 규칙 2 — 길이 불일치 시 짧은 쪽 0A 패딩
    {   RdProfile p; std::string e; float c[4]; uint16_t s;
        ASSERT_TRUE(Load(p, "motors: {m1: [{type: hold, duration: 5.0, value: 3}],"
                            " m2: [{type: hold, duration: 1.0, value: 7}]}", &e)) << e;
        EXPECT_EQ(p.TickCount(), 1000u) << "전체 길이 = 가장 긴 모터";
        p.SampleAt(999, c, &s);
        EXPECT_FLOAT_EQ(c[0], 3.0f);
        EXPECT_FLOAT_EQ(c[1], 0.0f) << "짧은 쪽은 0A 로 패딩";
    }
    // 규칙 3 — 클램프는 거부가 아니라 값 제한 + 횟수 기록
    {   RdProfile p; std::string e; float c[4]; uint16_t s;
        ASSERT_TRUE(Load(p, "limits: {max_current: 5.0}\nmotors: {m1: [{type: hold, duration: 1.0, value: 50}]}", &e));
        p.SampleAt(0, c, &s); EXPECT_FLOAT_EQ(c[0], 5.0f);
        EXPECT_EQ(p.ClampCount(), 200u);

        RdProfile p2; std::string e2;
        ASSERT_TRUE(Load(p2, "limits: {max_current: 50.0}\nmotors: {m1: [{type: hold, duration: 1.0, value: 40}]}",
                         &e2, 0x0F, 30.0f));
        p2.SampleAt(0, c, &s); EXPECT_FLOAT_EQ(c[0], 30.0f) << "전역 cmd_current_max 가 더 작으면 그쪽";

        RdProfile p3; std::string e3;
        ASSERT_TRUE(Load(p3, "limits: {max_current: 5.0}\nmotors: {m1: [{type: hold, duration: 1.0, value: -50}]}", &e3));
        p3.SampleAt(0, c, &s); EXPECT_FLOAT_EQ(c[0], -5.0f) << "음수도 대칭 클램프";
    }
    // 규칙 4 — slew 위반은 성형이 아니라 거부 (프로파일 = 실험 기록이므로 몰래 수정 금지)
    ExpectReject("limits: {slew_rate: 1.0}\nmotors: {m1: [{type: step, duration: 2.0, from: 0, to: 20, t_step: 1.0}]}",
                 "slew_rate");
    {   RdProfile p; std::string e;
        EXPECT_TRUE(Load(p, "limits: {slew_rate: 100.0}\nmotors: {m1: [{type: ramp, duration: 10.0, from: 0, to: 10}]}", &e));
    }
}

TEST(ProfileParser, ErrorMessagesLocateTheProblem) {
    ExpectReject("motors: {m1: [{type: ramp, duration: 1.0, from: 0}]}", "to");
    ExpectReject("motors: {m1: [{type: bogus, duration: 1.0}]}", "bogus");
    ExpectReject("motors: {m1: [{type: hold, duration: -1.0, value: 0}]}", "duration");
    ExpectReject("motors: {m5: [{type: hold, duration: 1.0, value: 0}]}", "1~4");
    ExpectReject("motors: {x1: [{type: hold, duration: 1.0, value: 0}]}", "형식 오류");
    ExpectReject("name: only", "motors");
    ExpectReject("[1,2,3]", "맵이 아님");
    ExpectReject("motors: {m1: [{type: hold, duration: 0.001, value: 1}]}", "0 tick");
    ExpectReject("motors: {m1: [{type: custom, samples: []}]}", "samples 길이");
    ExpectReject("motors: {m1: [{type: hold duration: bad", "YAML");
    ExpectReject("motors: {m1: [{type: hold, duration: 4000, value: 0}]}", "상한");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RegisterRclcppEnv();
    return RUN_ALL_TESTS();
}

// ===== mode: 키 + 모드 의존 limits (05 §2.2~§2.4, 2026-07-28) =====

namespace {
orin_bridge::RdProfile LoadOk(const std::string& yaml, float gmax = 30.0f) {
    orin_bridge::RdProfile p;
    std::string err;
    EXPECT_TRUE(p.LoadFromYaml(yaml, 0x0F, gmax, &err)) << err;
    return p;
}
bool LoadFails(const std::string& yaml, std::string* err, float gmax = 30.0f) {
    orin_bridge::RdProfile p;
    return !p.LoadFromYaml(yaml, 0x0F, gmax, err);
}
constexpr const char* kSeg = "motors: {m1: [{type: hold, duration: 1.0, value: 1}]}\n";
}  // namespace

TEST(ProfileMode, DefaultsToCurrentSoOldProfilesStillLoad) {
    auto p = LoadOk(std::string(kSeg));
    EXPECT_EQ(p.mode(), orin_bridge::RdProfile::Mode::CURRENT);
}

TEST(ProfileMode, UnknownModeRejected) {
    std::string err;
    EXPECT_TRUE(LoadFails("mode: torque\n" + std::string(kSeg), &err));
    EXPECT_NE(err.find("current|velocity|position"), std::string::npos) << err;
}

// 05 §2.4 — velocity/position 에 전역 기본값을 주지 않는다.
// **모르는 값에 기본값을 주는 것보다 프로파일이 매번 명시하게 강제하는 쪽이 안전하다.**
TEST(ProfileMode, VelocityRequiresMaxAbs) {
    std::string err;
    EXPECT_TRUE(LoadFails("mode: velocity\n" + std::string(kSeg), &err));
    EXPECT_NE(err.find("max_abs"), std::string::npos) << err;
}

TEST(ProfileMode, PositionRequiresRange) {
    std::string err;
    EXPECT_TRUE(LoadFails("mode: position\nlimits: {max_abs: 90}\n" + std::string(kSeg), &err));
    EXPECT_NE(err.find("range"), std::string::npos)
        << "관절 가동범위는 비대칭이라 max_abs 로 표현할 수 없다. err=" << err;
}

// 비대칭 클램프 — position 의 핵심 (05 §2.4).
TEST(ProfileMode, AsymmetricRangeClampsBothSidesIndependently) {
    auto p = LoadOk("mode: position\nlimits: {range: [-10, 100]}\n"
                    "motors: {m1: [{type: hold, duration: 0.1, value: 500}]}\n");
    EXPECT_FLOAT_EQ(p.limit_lo(), -10.0f);
    EXPECT_FLOAT_EQ(p.limit_hi(), 100.0f);
    float out[4]; uint16_t seg;
    ASSERT_TRUE(p.SampleAt(0, out, &seg));
    EXPECT_FLOAT_EQ(out[0], 100.0f) << "상한으로 잘려야 한다";
    EXPECT_GT(p.ClampCount(), 0u);
}

// max_current 는 deprecated 별칭 — current 전용 (05 §2.4).
TEST(ProfileMode, DeprecatedMaxCurrentRejectedOutsideCurrentMode) {
    std::string err;
    EXPECT_TRUE(LoadFails("mode: velocity\nlimits: {max_current: 5}\n" + std::string(kSeg), &err));
    EXPECT_NE(err.find("deprecated"), std::string::npos) << err;
}

// current 는 전역 클램프와 **더 좁은 쪽**을 취한다 (현행 유지).
TEST(ProfileMode, CurrentIntersectsGlobalClamp) {
    auto p = LoadOk("mode: current\nlimits: {max_abs: 50}\n" + std::string(kSeg), 12.0f);
    EXPECT_FLOAT_EQ(p.limit_hi(), 12.0f) << "전역 12A 가 프로파일 50A 보다 좁다";
}

// 05 §2.3 수락 규칙 — DIRECT 일 때만 ctr_mode 까지 내려간다.
TEST(ProfileMode, AcceptanceRaisesGuardToAutoMode) {
    namespace ecu = ::orin_bridge::ecu;
    auto p = LoadOk("mode: velocity\nlimits: {max_abs: 100}\n" + std::string(kSeg));
    const uint8_t all_vel[4] = {3, 3, 3, 3};
    const uint8_t all_cur[4] = {1, 1, 1, 1};
    std::string err;

    EXPECT_TRUE(p.AcceptsAutoMode(ecu::AUTO_MODE_VELOCITY, all_cur, 0x0F, &err))
        << "native auto_mode 면 ctr_mode 를 보지 않는다 (ECU 자가치유). err=" << err;
    EXPECT_FALSE(p.AcceptsAutoMode(ecu::AUTO_MODE_CURRENT, all_vel, 0x0F, &err));
    EXPECT_NE(err.find("auto_mode"), std::string::npos) << err;

    EXPECT_TRUE(p.AcceptsAutoMode(ecu::AUTO_MODE_DIRECT, all_vel, 0x0F, &err)) << err;
    EXPECT_FALSE(p.AcceptsAutoMode(ecu::AUTO_MODE_DIRECT, all_cur, 0x0F, &err))
        << "DIRECT 에서는 ctr_mode 가 단위를 정하므로 확인해야 한다";
    // 비활성 모터는 보지 않는다
    const uint8_t mixed[4] = {3, 1, 1, 1};
    EXPECT_TRUE(p.AcceptsAutoMode(ecu::AUTO_MODE_DIRECT, mixed, 0x01, &err)) << err;
}

// ===== 05 §3.3 타입별 파라미터 제약 (확정표) =====
//
// "이 표가 rd_profile.cpp 와 lint_profiles.py 양쪽의 정본이다" — 두 구현이 갈라지지 않도록
// 여기서 고정한다. 각 제약은 **조용한 오해**를 막는다: 제약이 없으면 사용자가 그렸다고
// 믿는 것과 실제 재생되는 것이 달라진다.
TEST(ProfileConstraints, TypeParameterTable) {
    // step: t_step 이 구간 밖이면 step 이 아니라 hold 다
    ExpectReject("motors: {m1: [{type: step, duration: 1.0, from: 0, to: 1, t_step: 2.0}]}", "t_step");
    ExpectReject("motors: {m1: [{type: step, duration: 1.0, from: 0, to: 1, t_step: 0}]}", "t_step");

    // sine/chirp: 25Hz = 200Hz 체인에서 주기당 8샘플. 넘으면 앨리어싱으로 **다른 파형**이 나온다
    ExpectReject("motors: {m1: [{type: sine, duration: 1.0, amp: 1, freq: 40}]}", "25");
    ExpectReject("motors: {m1: [{type: sine, duration: 1.0, amp: 1, freq: 0}]}", "25");
    ExpectReject("motors: {m1: [{type: chirp, duration: 1.0, amp: 1, f0: 1, f1: 99}]}", "25");

    // prbs: 1 tick 미만은 비트가 표현되지 않는다 / low==high 는 hold 다
    ExpectReject("motors: {m1: [{type: prbs, duration: 1.0, low: 0, high: 1, bit_duration: 0.001}]}",
                 "tick");
    ExpectReject("motors: {m1: [{type: prbs, duration: 1.0, low: 1, high: 1, bit_duration: 0.01}]}",
                 "hold");

    // noise: std<=0 이면 기록에는 noise 로 남는데 실제로는 상수다
    ExpectReject("motors: {m1: [{type: noise, duration: 1.0, mean: 0, std: 0}]}", "std");

    // stair / custom 길이 상한
    ExpectReject("motors: {m1: [{type: stair, step_duration: 0.01, values: []}]}", "values 길이");

    // custom rate: 200Hz 초과는 **물리적으로 재생 불가**. 조용히 다운샘플하면
    // 재생된 것과 기록된 것이 달라진다 (§4.3-4 가 금지하는 것).
    ExpectReject("motors: {m1: [{type: custom, rate: 500, samples: [0,1,0]}]}", "다운샘플");
    ExpectReject("motors: {m1: [{type: custom, rate: 0.5, samples: [0,1,0]}]}", "1~200");
    ExpectReject("motors: {m1: [{type: custom, interp: cubic, samples: [0,1,0]}]}", "linear|nearest");
}

// 05 §4.3 — 기본 보간이 선형. floor 는 rate 가 낮을수록 큰 계단을 만들고,
// slew_rate 와 함께 쓰면 **사용자가 그리지도 않은 이유로 reject** 된다.
TEST(ProfileConstraints, CustomDefaultsToLinearInterp) {
    RdProfile p; std::string err;
    // 100Hz 로 [0, 10] → 200Hz 재생이면 중간 tick 이 5 여야 한다 (선형)
    ASSERT_TRUE(Load(p, "motors: {m1: [{type: custom, rate: 100, samples: [0, 10]}]}", &err)) << err;
    float out[4]; uint16_t seg;
    ASSERT_TRUE(p.SampleAt(1, out, &seg));
    EXPECT_NEAR(out[0], 5.0f, 0.01f) << "선형 보간이 아니다 (floor 면 0 이 나온다)";

    RdProfile q;
    ASSERT_TRUE(Load(q, "motors: {m1: [{type: custom, rate: 100, interp: nearest, samples: [0, 10]}]}",
                     &err)) << err;
    ASSERT_TRUE(q.SampleAt(1, out, &seg));
    EXPECT_FLOAT_EQ(out[0], 0.0f) << "nearest 는 현행(floor) 동작이어야 한다";
}

// 05 §4.3-3 — rate==200 이면 두 방식이 완전히 같다. **기존 기록이 안 바뀐다.**
TEST(ProfileConstraints, NativeRateUnaffectedByInterpChange) {
    RdProfile lin, nea; std::string err;
    const char* y = "motors: {m1: [{type: custom, rate: 200, samples: [0, 3, 7, 2]}]}";
    ASSERT_TRUE(Load(lin, y, &err)) << err;
    ASSERT_TRUE(Load(nea, std::string(y).substr(0, 34) + "interp: nearest, " +
                         std::string(y).substr(34), &err)) << err;
    float a[4], b[4]; uint16_t s1, s2;
    for (size_t t = 0; t < lin.TickCount(); t++) {
        ASSERT_TRUE(lin.SampleAt(t, a, &s1));
        ASSERT_TRUE(nea.SampleAt(t, b, &s2));
        EXPECT_FLOAT_EQ(a[0], b[0]) << "tick " << t << " 에서 갈렸다 — 기존 기록이 바뀐다";
    }
}
