// 죽은 코드가 **다시 자라지 않는가** (redesign/08 §2.1 ①, §2.2 ⑤)
//
// ## 왜 소스 텍스트를 읽는가
//
// 여기서 막으려는 것들은 **컴파일러가 절대 잡지 못하는 부류**다:
//
//   ① `RdSchedule::ThreadStart()` — 정의는 살아 있고 호출부는 주석 처리돼 있었다.
//      링커는 "정의만 있고 아무도 안 부르는 public 멤버" 를 오류로 보지 않는다.
//   ② `HW_BIT_UART4` — 매크로 이름이 실제 슬롯(uart6)과 달랐다. **사용처가 0** 이라
//      값을 보는 테스트도, 컴파일러도, 실기도 영원히 못 잡는다.
//   ③ `hw_connected_` / `hw_error_msg_` — 세터를 부르는 곳도, 두 필드를 읽는 곳도
//      없었다. 06 §9.5 "배관은 있는데 생산자가 없다" 의 극단형이다 (소비자까지 없다).
//   ④ `RdNode` 에 남은 L3 이관 잔재 — 선언만 있고 정의가 없는 멤버 함수는
//      **부르지 않는 한** 링크 에러가 안 난다. 그래서 "이 노드가 아직 config 서비스를
//      다루나?" 를 읽는 사람이 매번 확인해야 했다.
//
// 공통점: **잘못된 상태와 정상 상태가 관측적으로 구분되지 않는다.** 그래서 소스를 본다.
// (같은 이유로 만든 것이 test_counter_producers.cpp 다 — 그쪽과 함께 읽을 것.)

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
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

std::string Src(const char* rel) { return Slurp(std::filesystem::path(RD_SRC_DIR) / rel); }

// 주석에 이름이 적혀 있는 것은 세지 않는다 — 이 파일들의 주석은 **왜 지웠는지**를
// 적어 두고 있으므로, 주석을 세면 그 설명 때문에 테스트가 실패한다.
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

void ExpectAbsent(const char* file, const char* needle, const char* why) {
    EXPECT_EQ(StripLineComments(Src(file)).find(needle), std::string::npos)
        << file << " 에 `" << needle << "` 가 다시 생겼다.\n  " << why;
}

}  // namespace

// ① 루프 입구는 하나여야 한다.
TEST(NoDeadCode, SchedulerHasExactlyOneLoopEntry) {
    for (const char* f : {"include/orin_firmware_bridge/sched/rd_schedule.hpp",
                          "src/sched/rd_schedule.cpp", "src/main.cpp"}) {
        ExpectAbsent(f, "ThreadStart",
                     "루프는 MainLoopStart 로만 들어간다. 입구가 둘이면 SCHED_FIFO 를 "
                     "어느 스레드에 걸었는지가 호출부마다 달라진다");
    }
    // 남은 입구가 실제로 있는지 — 위 검사만 있으면 둘 다 지워도 통과한다.
    EXPECT_NE(Src("src/main.cpp").find("MainLoopStart"), std::string::npos)
        << "main 이 루프를 시작하지 않는다";
}

// ② 이름이 실제 하드웨어 슬롯과 맞는가.
TEST(NoDeadCode, HardwareBitNamesMatchTheirSlots) {
    const auto common = Src("include/orin_firmware_bridge/core/rd_common.hpp");
    EXPECT_NE(common.find("HW_BIT_UART6"), std::string::npos)
        << "bit2 는 uart6(IMU) 슬롯이다 — 03 §3.1";
    ExpectAbsent("include/orin_firmware_bridge/core/rd_common.hpp", "define HW_BIT_UART4",
                 "슬롯이 uart4 -> uart6 으로 옮겨졌는데 매크로 이름만 남은 것이다 "
                 "(06 §7 이 고쳤다고 적어 두고 실제로는 안 고쳤다 — 08 §2.1 ①)");
}

// ③ 세터도 게터도 없는 필드가 남지 않는가.
TEST(NoDeadCode, TelemetryHasNoUnreachableHardwareStatus) {
    for (const char* f : {"include/orin_firmware_bridge/ros/rd_telemetry.hpp",
                          "src/ros/rd_telemetry.cpp",
                          "include/orin_firmware_bridge/ros/rd_node.hpp"}) {
        ExpectAbsent(f, "SetHardwareStatus",
                     "부르는 곳이 없었다. 연결 상태의 출처는 NodeStatus.connected 하나다 — "
                     "두 번째 경로를 만들면 어느 쪽이 참인지 모르게 된다");
        ExpectAbsent(f, "hw_connected_",
                     "쓰는 곳도 읽는 곳도 없던 필드다");
    }
}

// ④ 노드 껍데기에 L3 소유물이 되돌아오지 않는가.
//    선언만 남으면 링크 에러가 안 나므로, 이름으로 막는다.
TEST(NoDeadCode, BridgeNodeDoesNotOwnLayer3Members) {
    struct Case { const char* needle; const char* owner; };
    for (const Case& c : {
             Case{"sub_vel_",            "RdCarrierApi"},
             Case{"sub_jeongae_",        "RdCarrierApi"},
             Case{"srv_command_",        "RdCarrierApi"},
             Case{"srv_jeongae_lock_",   "RdCarrierApi"},
             Case{"srv_control_config_", "RdControlApi"},
             Case{"cb_group_config_",    "RdControlApi"},
             Case{"CallbackControlConfig", "RdControlApi"},
             Case{"act_run_profile_",    "RdControlApi"},
             Case{"HandleProfileGoal",   "RdControlApi"},
             Case{"ExecuteProfile",      "RdControlApi"},
             Case{"DoOutOfSpanWrite",    "RdControlApi"},
             Case{"DoSetAutoMode",       "RdControlApi"},
             Case{"kQuatScale",          "RdTelemetry"},
             Case{"kCurrentFilterAlpha", "RdTelemetry"},
             Case{"InputData",           "RdCarrierApi"},
         }) {
        EXPECT_EQ(StripLineComments(
                      Src("include/orin_firmware_bridge/ros/rd_node.hpp")).find(c.needle),
                  std::string::npos)
            << "rd_node.hpp 에 `" << c.needle << "` 가 있다 — 이것은 " << c.owner
            << " 의 소유다 (02 A2).\n"
            << "  선언만 남아도 링크 에러가 안 나므로, 남으면 **두 곳에 적힌 상태**가 되고\n"
            << "  읽는 사람은 어느 쪽이 실제로 동작하는지 매번 대조해야 한다.";
    }
}

// 상수가 **한 곳에만** 있는가 — 종전 IMU 환산 상수가 rd_node.hpp 와 rd_telemetry.hpp
// 양쪽에 같은 값으로 있었고, 쓰이는 것은 후자뿐이었다. 전자를 고치면 조용히 무효가 된다.
TEST(NoDeadCode, UnitScalesAreDefinedExactlyOnce) {
    int count = 0;
    for (const char* f : {"include/orin_firmware_bridge/ros/rd_node.hpp",
                          "include/orin_firmware_bridge/ros/rd_telemetry.hpp"}) {
        if (StripLineComments(Src(f)).find("kQuatScale") != std::string::npos) count++;
    }
    EXPECT_EQ(count, 1) << "IMU 환산 상수가 " << count << " 곳에 있다 — "
                           "발행 시점에만 쓰이므로 RdTelemetry 하나가 제자리다";
}
