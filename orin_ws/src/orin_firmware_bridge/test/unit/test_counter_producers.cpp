// 카운터에 **생산자가 있는가** (redesign/06 §9.5)
//
// ## 왜 이 테스트가 필요한가
//
// `irregular_tick_cnt` 는 배관이 완벽했다 — 필드가 `RdTelemetry` 에 있고, 스냅샷 람다가
// 읽고, `RdControlApi` 가 런 구간 차분을 떠서 action Result 에 싣고, `record.py` 가
// result.json 에 적었다. 그런데 **`OnIrregularTick()` 을 부르는 곳이 하나도 없었다.**
// 그래서 값은 늘 0 이었고, 0 은 "사고가 없었다" 와 구분되지 않아 **아무도 몰랐다.**
// (`missing_tick_cnt` 도 같은 상태였다 — 2026-07-29 발견.)
//
// 값을 보는 테스트로는 이걸 못 잡는다. 관측값이 기본값과 같으면 판별이 불가능하기 때문이다.
// 그래서 **소스에 호출부가 존재하는지**를 본다.
//
// ## 두 방향을 다 봐야 한다
//
//   ① ITelemetrySink 의 통지는 **L1(rd_schedule.cpp)에 호출부**가 있어야 한다.
//      없으면 "인터페이스에는 있는데 아무도 안 알린다" — 이번 사고가 정확히 이것이다.
//   ② RdTelemetry 의 `On*` 는 **어딘가에 호출부**가 있어야 한다.
//      없으면 "구현은 있는데 인터페이스에 안 꽂혔다" — 사고 직전 상태다.
//
// ①만 있으면 sink 를 안 거치는 카운터를 놓치고, ②만 있으면 RdNode 의 위임만 보고
// 통과해 버린다. 둘을 같이 걸어야 사슬 전체가 닫힌다.

// 소스 텍스트만 읽는다 — 노드도 통신도 필요 없다 (rclcpp 링크 없음).
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

namespace {

std::string Slurp(const std::filesystem::path& p) {
    std::ifstream f(p);
    EXPECT_TRUE(f.good()) << "파일을 못 읽었다: " << p;
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// `//` 주석을 지운다 — 주석에 적힌 메서드 이름을 호출부로 세면 안 된다.
// (설명 주석에 `sink_->OnLateTick()` 이라고 써두면 테스트가 통과해 버린다.)
std::string StripLineComments(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    bool in_comment = false;
    for (size_t i = 0; i < src.size(); i++) {
        if (!in_comment && src[i] == '/' && i + 1 < src.size() && src[i + 1] == '/') in_comment = true;
        if (src[i] == '\n') in_comment = false;
        if (!in_comment) out += src[i];
    }
    return out;
}

std::set<std::string> MatchAll(const std::string& text, const std::string& pattern) {
    std::set<std::string> out;
    const std::regex re(pattern);
    for (auto it = std::sregex_iterator(text.begin(), text.end(), re);
         it != std::sregex_iterator(); ++it) {
        out.insert((*it)[1].str());
    }
    return out;
}

std::string Src(const char* rel) { return Slurp(std::filesystem::path(RD_SRC_DIR) / rel); }

}  // namespace

// ① 인터페이스에 선언된 통지는 전부 스케줄 루프에 호출부가 있어야 한다.
TEST(CounterProducers, EverySinkNotificationIsFiredBySchedule) {
    const auto declared = MatchAll(StripLineComments(
        Src("include/orin_firmware_bridge/sched/rd_telemetry_sink.hpp")),
        R"(virtual\s+void\s+(On\w+)\s*\()");
    ASSERT_GE(declared.size(), 6u) << "ITelemetrySink 의 On* 선언을 못 찾았다 — 정규식이 낡았다";

    const auto fired = MatchAll(StripLineComments(Src("src/sched/rd_schedule.cpp")),
                                R"(sink_->(On\w+)\s*\()");

    for (const auto& name : declared) {
        EXPECT_TRUE(fired.count(name) > 0)
            << "ITelemetrySink::" << name << " 을 rd_schedule.cpp 에서 부르는 곳이 없다.\n"
            << "  배관만 있고 생산자가 없으면 그 카운터는 **영원히 0** 이고, 0 은 "
               "'사고가 없었다' 와 구분되지 않아 아무도 알아채지 못한다.\n"
               "  (irregular_tick_cnt 가 정확히 이 상태였다 — 06 §9.5)";
    }
}

// ② 구현 쪽 통지도 호출부가 있어야 한다 — 인터페이스에 안 꽂힌 채 방치되는 것을 막는다.
TEST(CounterProducers, EveryTelemetryNotifierHasACaller) {
    const auto declared = MatchAll(StripLineComments(
        Src("include/orin_firmware_bridge/ros/rd_telemetry.hpp")),
        R"(void\s+(On\w+)\s*\()");
    ASSERT_GE(declared.size(), 5u) << "RdTelemetry 의 On* 선언을 못 찾았다 — 정규식이 낡았다";

    // 선언 파일 자신은 빼고 본다 (선언이 곧 호출부로 세어지면 안 된다).
    std::string haystack;
    for (const char* f : {"src/sched/rd_schedule.cpp", "src/ros/rd_node.cpp", "src/ros/rd_telemetry.cpp",
                          "src/ros/rd_control_api.cpp", "src/ros/rd_carrier_api.cpp",
                          "include/orin_firmware_bridge/ros/rd_node.hpp"}) {
        haystack += StripLineComments(Src(f));
    }
    const auto called = MatchAll(haystack, R"(->(On\w+)\s*\()");

    for (const auto& name : declared) {
        EXPECT_TRUE(called.count(name) > 0)
            << "RdTelemetry::" << name << " 을 부르는 곳이 없다 — "
               "카운터는 있는데 아무도 올려주지 않는다는 뜻이다";
    }
}

// 셋이 서로 다른 사고를 세는지 — 이름이 겹치면 분석이 구분할 수 없다.
TEST(CounterProducers, TheThreeTickCountersAreDistinctFields) {
    const auto tele = Src("include/orin_firmware_bridge/ros/rd_telemetry.hpp");
    EXPECT_NE(tele.find("irregular_tick_cnt_"), std::string::npos);
    EXPECT_NE(tele.find("late_tick_cnt_"), std::string::npos)
        << "late_tick_cnt 가 없다 — 주기 초과로 시간축이 밀린 양을 아는 유일한 수단이다";

    // 종전 이름이 남아 있으면 안 된다. "빠진 tick" 은 이 루프에서 원리적으로 생기지 않는다
    // (주기를 넘겨도 tick 번호는 안 건너뛰고 시간축이 밀린다) — 06 §9.5.
    for (const char* f : {"include/orin_firmware_bridge/ros/rd_telemetry.hpp",
                          "include/orin_firmware_bridge/ros/rd_control_api.hpp",
                          "src/ros/rd_control_api.cpp"}) {
        EXPECT_EQ(Src(f).find("missing_tick"), std::string::npos)
            << f << " 에 missing_tick 이 남아 있다 — late_tick_cnt 로 개명됐다";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 센서 필드에도 **생산자가 있는가** (08 이 놓친 것 / 06 §10 B1)
//
// 카운터와 같은 사고가 센서 쪽에도 있었다. `imu_quat`·`imu_gyro`·`imu_acc`·`link_angle`
// 은 배관이 전부 있었다 — 메시지에 필드가 있고, 웹이 계열로 뽑아 그리고, bag 에 담긴다.
// 그런데 **`NaN` 을 쓰는 4줄이 그 필드에 대한 유일한 쓰기**였다. 디코더가 아예 없었다.
//
// 이쪽이 카운터보다 더 안 보인다. 카운터의 "늘 0" 은 그래도 "사고 없음" 으로 읽히지만,
// 센서의 "늘 NaN" 은 **미판독 규약과 완전히 같은 모양**이라(03 §5.3) 정상 동작처럼 보인다.
// 실제로 06 §9.9 는 "엔코더를 읽는 프리셋이 없다" 를 원인으로 지목했고, 04 §2.3 으로
// 프리셋을 고친 뒤에도 값은 여전히 NaN 이었다 — 원인이 둘이었는데 하나만 고쳤기 때문이다.
//
// 그래서 "레지스터 섀도를 실제로 읽는가" 를 본다. 값 비교로는 판별할 수 없다.
//
// ⚠ **검사 범위를 `OnFeedback()` 안으로 좁혀야 한다.** 파일 전체를 보면 `reg.imu.` 가
//   `PublishStatus()` 의 `ecu_reg.imu.state`(NodeStatus 용)에 부분 일치해서, 디코더가
//   전혀 없어도 통과한다 — 이 테스트를 처음 쓸 때 실제로 그렇게 통과했다.
//   "무엇을 검사하는가" 보다 "무엇에 대고 검사하는가" 가 먼저다.
namespace {
// `void RdTelemetry::OnFeedback() {` 부터 다음 멤버 정의 직전까지.
std::string OnFeedbackBody() {
    const auto src = StripLineComments(Src("src/ros/rd_telemetry.cpp"));
    const std::string head = "void RdTelemetry::OnFeedback()";
    const size_t b = src.find(head);
    EXPECT_NE(b, std::string::npos) << "OnFeedback() 을 못 찾았다 — 개명됐다면 이 테스트도 고칠 것";
    if (b == std::string::npos) return "";
    const size_t e = src.find("void RdTelemetry::", b + head.size());
    return src.substr(b, (e == std::string::npos ? src.size() : e) - b);
}
}  // namespace

TEST(CounterProducers, EverySensorBlockIsActuallyDecodedIntoFeedback) {
    const auto body = OnFeedbackBody();

    struct Block { const char* needle; const char* fields; const char* note; };
    for (const Block& b : {
             Block{"reg.imu.quat_x",         "imu_quat/imu_gyro/imu_acc", "IMU(48:22)"},
             Block{"reg.encoder.encoder[",   "link_angle",                "엔코더(70:16)"},
             Block{"reg.loadcell.avg",       "loadcell_raw",              "로드셀(42:6)"},
             Block{"reg.motor_data.current", "fb_current",                "모터(88:40)"},
         }) {
        EXPECT_NE(body.find(b.needle), std::string::npos)
            << b.note << " 를 ControlFeedback 으로 디코딩하는 코드가 없다 "
            << "(`" << b.needle << "` 가 OnFeedback() 안에 없다).\n"
            << "  그러면 " << b.fields << " 는 **영원히 NaN** 이고, NaN 은 미판독 규약과\n"
            << "  같은 모양이라(03 §5.3) 아무도 고장으로 인식하지 못한다.";
    }
}

// 채널별 stale 판정을 하고 있는가 — 06 §10 B1 이 만든 요구사항이다.
//
// AS5600 5채널은 I2C MUX 로 순차 취득하므로 **일부만 죽는 것이 정상적인 고장 양상**이다.
// 그때 raw 값은 12bit 범위 안의 그럴듯한 잔값으로 남아 있었다. 블록 단위로만 판정하면
// (프리셋이 읽었으니 전 채널 유효) 죽은 4채널이 정상 각도로 나간다.
// project 계약 토픽 `/carrier_imu` 도 같은 판정을 해야 한다 (2026-07-30 실기).
//
// 여기가 더 위험하다: `ControlFeedback` 의 NaN 은 "미판독" 규약이 있지만,
// `sensor_msgs/Imu` 의 **크기 0인 사원수 `(0,0,0,0)`** 은 규약이 아니라 그냥 거짓이다.
// 정규화하는 소비자는 0으로 나눈다.
//
// 종전 가드는 `state.bits.lifecycle == LS_OFFLINE` 하나였는데, 실기에서 IMU 채널이
// `lc=1/hs=0`("정상") 을 보고하면서 데이터는 하나도 안 오는 상태가 관측돼 걸리지 않았다.
//
// ★ 그리고 `Reads()` 가 반드시 함께 있어야 한다. 섀도는 0으로 초기화되므로
//   **한 번도 안 읽은 블록은 `delta_tick == 0`("지연 0ms = 신선함")** 으로 보인다.
//   `control_test` 프리셋(견인)이 IMU 를 안 읽는데, delta_tick 만 보면 그 0이 통과해
//   전부 0인 자세가 발행된다 — 고친 뒤 실기에서 실제로 그 순서로 드러났다.
TEST(CounterProducers, ImuTopicIsGatedByBothPresetAndStaleness) {
    const auto src = StripLineComments(Src("src/ros/rd_telemetry.cpp"));
    const size_t b = src.find("void RdTelemetry::PublishMotorFeedback()");
    ASSERT_NE(b, std::string::npos);
    const size_t e = src.find("void RdTelemetry::", b + 40);
    const auto body = src.substr(b, (e == std::string::npos ? src.size() : e) - b);

    EXPECT_NE(body.find("Reads(ecu::REG_IMU_OFFSET"), std::string::npos)
        << "/carrier_imu 가 **프리셋 판정 없이** 발행된다.\n"
        << "  섀도 0 초기화 때문에 안 읽은 블록이 delta_tick=0(신선함)으로 보인다 —\n"
        << "  전부 0인 사원수가 유효한 자세로 나간다.";
    EXPECT_NE(body.find("DELTA_STALE"), std::string::npos)
        << "/carrier_imu 가 취득 시각(delta_tick)을 안 본다 — 채널 상태 보고만으로는 부족하다\n"
        << "  (실기에서 lc=1/hs=0 인데 데이터가 0인 상태가 나왔다).";
    // 정렬 공백에 걸리지 않게 두 조각으로 본다 (`covariance[0]  = -1.0;` 처럼 띄어 쓴다).
    EXPECT_NE(body.find("orientation_covariance[0]"), std::string::npos)
        << "REP-145 의 '추정 없음' 표식(covariance[0] = -1)이 없다.";
    EXPECT_NE(body.find("-1.0"), std::string::npos)
        << "covariance 를 -1 로 두는 곳이 없다 — REP-145 소비자가 미판독을 구분하지 못한다.";
}

TEST(CounterProducers, EncoderStalenessIsJudgedPerChannel) {
    EXPECT_NE(OnFeedbackBody().find("reg.encoder.delta_tick[i]"), std::string::npos)
        << "엔코더 stale 판정이 채널별이 아니다.\n"
        << "  `delta_tick[i]` 를 채널마다 봐야 한다 — 값의 그럴듯함은 판정 기준이 아니다\n"
        << "  (2026-07-29 실기: ch0 만 살아 있었고 ch1~4 raw 는 전부 12bit 범위 안이었다).";
}
