// GET_STATUS 정형 JSON (redesign/04 §4 — C3)
//
// 막는 것: **소비자가 텍스트를 정규식으로 뜯게 되는 것.** 종전 응답은 한국어 문장이었고
// `control_web` 이 실제로 그러고 있었다 — 그래서 웹이 알 수 있는 값이 넷뿐이었고,
// 그 위에 조작 UI 를 올릴 수 없었다 (06 §9.1).
//
// 스키마가 계약이므로 **값이 아니라 키의 존재와 타입**을 고정한다.
// rclcpp 없이 돈다 (A5) — 조립을 자유 함수로 뺀 이유가 이것이다.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "orin_firmware_bridge/policy/rd_status.hpp"

namespace {

using orin_bridge::StatusSnapshot_t;
using orin_bridge::StatusSlot_t;
using orin_bridge::StatusToJson;

// 의존성을 늘리지 않으려고 키 존재만 문자열로 확인한다. 값 파싱이 필요한 검사는
// Python 쪽(control_cli 테스트)에서 진짜 json 파서로 한다.
bool HasKey(const std::string& j, const std::string& key) {
    return j.find("\"" + key + "\":") != std::string::npos;
}

StatusSnapshot_t Basic() {
    StatusSnapshot_t s;
    s.bridge_mode = "control";
    s.control_state = "IDLE";
    s.ctr_mode = {"current", "current", "current", "current"};
    s.active_motors = {1};
    s.motor_mask = 1;
    return s;
}

// ★ 04 §4 의 규칙 1 — 모든 키가 항상 존재한다.
TEST(StatusJson, EveryKeyAlwaysPresent) {
    // 아무것도 안 채운 스냅샷이어도 키는 전부 있어야 한다.
    const std::string j = StatusToJson(StatusSnapshot_t{});
    for (const char* k : {"bridge_mode", "control_state", "write_source", "ecu_mode",
                          "ecu_sys_state", "auto_mode", "write_span", "read_preset",
                          "active_motors", "motor_mask", "ctr_mode",
                          "goal_id", "profile_time",
                          "safe_stop", "safe_stop_detail",
                          "stamp_valid", "stamp_quality_ms", "drift_ppm", "rtt_ms",
                          "rw_err", "drop_cnt", "lock_reason", "slots", "nodes",
                          "sequence"}) {
        EXPECT_TRUE(HasKey(j, k)) << "키 누락: " << k << "\n" << j;
    }
}

// 09 §4.2 — 보드 3개는 **항상 세 개 다** 나오고, 각각 키 3개를 갖는다.
// 안 켠 보드를 통째로 빼면 소비자가 "키가 없다 = 꺼짐" 으로 분기하게 되고, 그건 04 §4 가
// 금지한 "형태로 분기하는 파싱" 이다.
TEST(StatusJson, EveryNodeAlwaysPresentWithEveryField) {
    const std::string j = StatusToJson(StatusSnapshot_t{});
    for (const char* n : {"ecu", "dpc", "pcu"}) {
        EXPECT_TRUE(HasKey(j, n)) << "보드 누락: " << n << "\n" << j;
    }
    for (const char* k : {"enabled", "connected", "fail_streak"}) {
        // 세 보드가 같은 모양이어야 하므로 3회 등장한다.
        size_t n = 0, pos = 0;
        const std::string needle = std::string("\"") + k + "\":";
        while ((pos = j.find(needle, pos)) != std::string::npos) { n++; pos += needle.size(); }
        EXPECT_EQ(n, 3u) << k << " 가 3개 보드 전부에 있지 않다\n" << j;
    }
}

// ★ **`enabled` 와 `connected` 는 독립이다.** 합쳐 내면 "안 켰다" 와 "켰는데 무응답" 이
//   같은 화면이 되는데, 조작자에게 그 둘은 완전히 다른 상황이다 (09 §4.2).
TEST(StatusJson, EnabledAndConnectedAreIndependent) {
    StatusSnapshot_t s;
    s.node_ecu = {true,  true,  0};    // 켰고 붙었다
    s.node_dpc = {true,  false, 42};   // 켰는데 무응답 — 배선/전원을 봐야 한다
    s.node_pcu = {false, false, 0};    // 아예 안 켰다 — 설정 문제
    const std::string j = StatusToJson(s);

    EXPECT_NE(j.find("\"ecu\":{\"enabled\":true,\"connected\":true,\"fail_streak\":0}"),
              std::string::npos) << j;
    EXPECT_NE(j.find("\"dpc\":{\"enabled\":true,\"connected\":false,\"fail_streak\":42}"),
              std::string::npos) << "켰는데 무응답 상태가 표현되지 않는다\n" << j;
    EXPECT_NE(j.find("\"pcu\":{\"enabled\":false,\"connected\":false,\"fail_streak\":0}"),
              std::string::npos) << j;
}

// 값이 없으면 **키를 빼는 게 아니라 null** 이다 — 키 유무로 분기하게 만들지 않는다.
TEST(StatusJson, UnknownValuesAreNullNotMissing) {
    const std::string j = StatusToJson(StatusSnapshot_t{});
    EXPECT_NE(j.find("\"safe_stop_detail\":null"), std::string::npos);
    EXPECT_NE(j.find("\"lock_reason\":null"), std::string::npos);
    // stamp_valid=false 면 품질·드리프트는 의미가 없다 (04 §4 규칙 4)
    EXPECT_NE(j.find("\"stamp_quality_ms\":null"), std::string::npos);
    EXPECT_NE(j.find("\"drift_ppm\":null"), std::string::npos);
}

// enum 은 문자열. 정수 원본이 필요하면 별도 키(ecu_sys_state).
TEST(StatusJson, EnumsAreStringsAndRawIntKept) {
    StatusSnapshot_t s = Basic();
    s.auto_mode = "direct";
    s.ecu_mode = "auto";
    s.ecu_sys_state = 2;
    const std::string j = StatusToJson(s);
    EXPECT_NE(j.find("\"auto_mode\":\"direct\""), std::string::npos);
    EXPECT_NE(j.find("\"ecu_mode\":\"auto\""), std::string::npos);
    EXPECT_NE(j.find("\"ecu_sys_state\":2"), std::string::npos) << "정수 원본은 따옴표 없이";
}

// ★ 거부 사유에는 한글과 따옴표가 섞여 들어온다. 이스케이프를 빼면
//   **사유 하나가 응답 전체를 깨뜨린다.**
TEST(StatusJson, ReasonStringsAreEscaped) {
    StatusSnapshot_t s = Basic();
    s.safe_stop = false;
    s.safe_stop_detail = "M1 fb_velocity=-10.0 RPM (한계 5.0) \"경고\"\n둘째 줄";
    s.lock_reason = "RW write 연속 거부 50 tick\t재확인";
    const std::string j = StatusToJson(s);
    // 원문의 생 따옴표·개행·탭이 그대로 남아 있으면 안 된다.
    EXPECT_EQ(j.find("\"경고\""), std::string::npos) << "따옴표가 이스케이프되지 않았다";
    EXPECT_NE(j.find("\\\"경고\\\""), std::string::npos);
    EXPECT_NE(j.find("\\n"), std::string::npos);
    EXPECT_NE(j.find("\\t"), std::string::npos);
    // 한글 자체는 UTF-8 로 그대로 통과한다 (\u 로 부풀리지 않는다)
    EXPECT_NE(j.find("한계"), std::string::npos);
}

// safe_stop=false 일 때만 detail 이 문자열이다 (04 §4 규칙 3).
TEST(StatusJson, SafeStopDetailOnlyWhenFalse) {
    StatusSnapshot_t ok = Basic();
    ok.safe_stop = true;
    EXPECT_NE(StatusToJson(ok).find("\"safe_stop\":true"), std::string::npos);
    EXPECT_NE(StatusToJson(ok).find("\"safe_stop_detail\":null"), std::string::npos);
}

// NaN 은 JSON 에 없다 — null 이 맞다. (드리프트 추정이 아직 없을 때 실제로 나온다)
TEST(StatusJson, NonFiniteBecomesNull) {
    StatusSnapshot_t s = Basic();
    s.stamp_valid = true;
    s.drift_ppm = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(StatusToJson(s).find("\"drift_ppm\":null"), std::string::npos);
}

// 비활성 슬롯도 키를 갖는다 — 형태로 분기하게 만들지 않는다.
TEST(StatusJson, InactiveSlotStillCarriesEveryKey) {
    StatusSnapshot_t s = Basic();
    StatusSlot_t idle; idle.index = 0; idle.active = false;
    StatusSlot_t busy; busy.index = 1; busy.active = true;
    busy.cmd = "read"; busy.target = "ecu"; busy.duration = 0; busy.attempts = 128;
    s.slots = {idle, busy};
    const std::string j = StatusToJson(s);
    EXPECT_NE(j.find("{\"index\":0,\"active\":false,\"cmd\":null,\"target\":null,"
                     "\"duration\":null,\"attempts\":null}"), std::string::npos) << j;
    EXPECT_NE(j.find("\"cmd\":\"read\""), std::string::npos);
    EXPECT_NE(j.find("\"attempts\":128"), std::string::npos);
}

// 소수점이 로케일을 따라가면 JSON 이 깨진다 ("0,000").
TEST(StatusJson, NumbersUseDotRegardlessOfLocale) {
    StatusSnapshot_t s = Basic();
    s.profile_time = 1.5;
    const std::string j = StatusToJson(s);
    EXPECT_NE(j.find("\"profile_time\":1.500"), std::string::npos) << j;
    EXPECT_EQ(j.find(",500"), std::string::npos) << "로케일 소수점이 새어 나왔다";
}

}  // namespace

// 09 §5.3 ④ (U12) — jeongae 시퀀스 블록.
//
// **`busy` 를 따로 내는 것이 요점이다.** 웹이 `state != "IDLE"` 로 판정하게 두면 단계
// 이름이 하나 늘 때마다 웹이 같이 바뀌어야 하고, 안 바꾸면 새 단계에서 수동 127 입력이
// 열린 채로 남는다 — 시퀀스가 자기가 안 쓴 상태 전이를 보고 Abort 하는 경로다.
TEST(StatusJson, SequenceBlockCarriesItsOwnBusyFlag) {
    StatusSnapshot_t s;
    s.seq_state      = "DPC_WAIT_CAMERA";
    s.seq_wait_ticks = 42;
    s.seq_wait_max   = 150;
    s.seq_busy       = true;
    s.seq_locked     = true;
    const std::string j = StatusToJson(s);

    EXPECT_NE(j.find("\"sequence\":{"), std::string::npos) << j;
    EXPECT_NE(j.find("\"state\":\"DPC_WAIT_CAMERA\""), std::string::npos) << j;
    EXPECT_NE(j.find("\"wait_ticks\":42"), std::string::npos) << j;
    EXPECT_NE(j.find("\"wait_max\":150"), std::string::npos) << j;
    EXPECT_NE(j.find("\"busy\":true"), std::string::npos) << j;
    EXPECT_NE(j.find("\"locked\":true"), std::string::npos) << j;
}

// 기본값(IDLE)에서도 **키가 다 있어야 한다** — 04 §4: 소비자가 키 유무로 분기하게
// 만들지 않는다. 없으면 웹이 `sequence` 를 못 찾아 조용히 잠금을 안 건다.
TEST(StatusJson, SequenceKeysExistWhenIdle) {
    const std::string j = StatusToJson(StatusSnapshot_t{});
    EXPECT_NE(j.find("\"state\":\"IDLE\""), std::string::npos) << j;
    EXPECT_NE(j.find("\"busy\":false"), std::string::npos) << j;
    EXPECT_NE(j.find("\"locked\":false"), std::string::npos) << j;
    EXPECT_NE(j.find("\"wait_ticks\":0"), std::string::npos) << j;
}

// 2026-08-07 — cmd_vel **0 수렴 스킵**은 상태로 나가야 한다 (웹 버튼이 이 값을 그린다).
//
// **기본이 false 인 것이 안전 요건이다.** 켜면 cmd_vel 이 오래 0 일 때 브리지가 쓰기를
// 멈추는데, 경사에서는 ECU 가 100ms 뒤 `AUTO_TIMEOUT` 으로 명령을 무효화한다 —
// "0 을 유지하라" 가 "명령이 없다" 가 된다.
TEST(StatusJson, CmdVelZeroSkipDefaultsToOffAndIsReported) {
    const std::string j = StatusToJson(StatusSnapshot_t{});
    EXPECT_NE(j.find("\"cmd_vel_zero_skip\":false"), std::string::npos)
        << "기본이 off 가 아니다 — 경사에서 위험한 쪽이 기본값이 되면 안 된다\n" << j;
    EXPECT_NE(j.find("\"cmd_vel_zero_timeout_s\":"), std::string::npos) << j;
}

TEST(StatusJson, CmdVelZeroSkipOnIsReported) {
    StatusSnapshot_t s;
    s.cmd_vel_zero_skip = true;
    s.cmd_vel_zero_timeout_s = 30.0;
    const std::string j = StatusToJson(s);
    EXPECT_NE(j.find("\"cmd_vel_zero_skip\":true"), std::string::npos) << j;
}
