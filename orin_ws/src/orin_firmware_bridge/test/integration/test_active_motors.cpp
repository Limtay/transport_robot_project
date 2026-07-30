// C-3 기동 파라미터 검증 (testbed_spec.md §3.1)
// 막는 것: 오타 난 active_motors/auto_mode 로 엉뚱한 트랙이 돌거나, 하드웨어와 무관한 설정 오류인데
//          "Waiting for USB..." 만 반복하며 매달리는 것. 계약은 **프로세스 종료 코드**라
//          in-process 가 아니라 실제 노드를 띄워 확인한다.
#include "rd_test_common.hpp"
#include <cstdlib>
#include <string>

#ifndef COMM_TEST_NODE_PATH
#error "COMM_TEST_NODE_PATH 미정의 — CMakeLists 의 target_compile_definitions 확인"
#endif

namespace {
// 노드를 인자와 함께 띄우고 종료 코드를 돌려준다.
// 파라미터가 유효하면 USB 대기로 진입해 끝나지 않으므로 timeout 이 걸린다(124).
int RunNode(const std::string& args, int timeout_s = 6) {
    const std::string cmd = "timeout " + std::to_string(timeout_s) + " " +
                            std::string(COMM_TEST_NODE_PATH) +
                            " --ros-args -p bridge_mode:=control " + args +
                            " >/dev/null 2>&1";
    const int rc = std::system(cmd.c_str());
    return WEXITSTATUS(rc);
}
constexpr int kRejected = 1;    // 파라미터 거부 → 통신 시도 없이 종료
constexpr int kTimedOut = 124;  // 유효 → USB 대기 중 timeout 이 끊음
}

TEST(ActiveMotors, ValidParamsProceedToUsbWait) {
    EXPECT_EQ(RunNode(""), kTimedOut) << "파라미터 없이 기동 = 기본 [1,2,3,4] 동작해야 한다";
    EXPECT_EQ(RunNode("-p active_motors:=\"[2,3]\""), kTimedOut) << "부분 트랙 지정";
    EXPECT_EQ(RunNode("-p active_motors:=\"[1,2,3,4]\""), kTimedOut);
}

TEST(ActiveMotors, OutOfRangeRejectedFastWithoutSerial) {
    EXPECT_EQ(RunNode("-p active_motors:=\"[1,5]\""), kRejected)
        << "5 는 범위 밖 — 오타로 엉뚱한 트랙이 도는 것을 막아야 한다";
    EXPECT_EQ(RunNode("-p active_motors:=\"[0]\""), kRejected);
    EXPECT_EQ(RunNode("-p active_motors:=\"[-1]\""), kRejected);
}

// **단어형이 정본이다** (01 §4.2) — 로그·CLI 에서 `auto_mode=1` 보다 `current` 가 사고를 줄인다.
TEST(AutoModeParam, WordFormAccepted) {
    EXPECT_EQ(RunNode("-p auto_mode:=current"),  kTimedOut);
    EXPECT_EQ(RunNode("-p auto_mode:=direct"),   kTimedOut);
    EXPECT_EQ(RunNode("-p auto_mode:=velocity"), kTimedOut) << "01 §3.2 신규";
    EXPECT_EQ(RunNode("-p auto_mode:=position"), kTimedOut) << "01 §3.2 신규";
    // none = 아무것도 쓰지 않는다 (READ 전용). 구 traction_test_mode 가 이것이다.
    EXPECT_EQ(RunNode("-p auto_mode:=none"), kTimedOut);
}

TEST(AutoModeParam, IntegerFormStillAccepted) {
    EXPECT_EQ(RunNode("-p auto_mode:=1"), kTimedOut) << "1=current";
    EXPECT_EQ(RunNode("-p auto_mode:=2"), kTimedOut) << "2=direct";
}

TEST(AutoModeParam, ForbiddenModesRejectedAtStartup) {
    EXPECT_EQ(RunNode("-p auto_mode:=kinematic"), kRejected)
        << "KINEMATIC 은 ECU 가 ctr_mode 를 덮어써 전류 실험이 성립하지 않는다 (§2.6)";
    EXPECT_EQ(RunNode("-p auto_mode:=control"), kRejected) << "CONTROL 은 ECU 미구현";
    EXPECT_EQ(RunNode("-p auto_mode:=0"), kRejected);
    EXPECT_EQ(RunNode("-p auto_mode:=3"), kRejected);
    EXPECT_EQ(RunNode("-p auto_mode:=oops"), kRejected) << "오타는 기본값 폴백이 아니라 거부다";
}

// bridge_mode·read_preset 도 오타면 **기동을 거부**한다 — 엉뚱한 모드로 뜨는 것이
// 이 프로젝트에서 가장 비싼 실수다 (모터가 도는 모드인지 아닌지가 갈린다).
TEST(BridgeModeParam, TyposRejectedAtStartup) {
    const std::string node = std::string(COMM_TEST_NODE_PATH);
    auto Run = [&](const std::string& a) {
        return WEXITSTATUS(std::system(("timeout 6 " + node + " --ros-args " + a +
                                        " >/dev/null 2>&1").c_str()));
    };
    EXPECT_EQ(Run("-p bridge_mode:=control"),  kTimedOut);
    EXPECT_EQ(Run("-p bridge_mode:=manual"),   kTimedOut);
    EXPECT_EQ(Run("-p bridge_mode:=project"),  kTimedOut);
    EXPECT_EQ(Run("-p bridge_mode:=traction"), kRejected)
        << "traction 은 모드가 아니라 control 의 구성이다 (auto_mode:=none)";
    EXPECT_EQ(Run("-p bridge_mode:=contorl"),  kRejected);
    EXPECT_EQ(Run("-p bridge_mode:=control -p read_preset:=nope"), kRejected);
    EXPECT_EQ(Run("-p bridge_mode:=control -p read_preset:=control_test"), kTimedOut);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
