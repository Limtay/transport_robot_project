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

// ★★ **빈칸의 의미가 뒤집혔고, 표기가 `""` 다** (2026-08-05, memo_260731 / 09 §4.3).
//
// 종전: 빈 값 = "구동할 모터가 0개" → **기동 거부**.
// 현재: 빈 값 = **"건드리지 않는다"** — motor_mask WRITE 를 건너뛰고 ECU 의 현재 값을
//       INIT 이 READ 해서 채택한다. 웹 기동 패널의 **빈 입력칸**이 이 경로로 온다.
//
// ⚠ **빈칸은 `""` 이고 `[]` 가 아니다.** `[]` 는 원소 타입 추론이 안 돼 override 가
//    `NOT_SET` 으로 들어오고, ROS 2 Humble 이 **노드 생성 단계에서** terminate 한다
//    (파라미터 이름과 무관 — 선언하지도 않은 이름으로도 죽는다). 그래서 `[]` 케이스는
//    **테스트로 만들 수도 없다**: 노드가 죽는 것이지 우리 코드가 거부하는 것이 아니다.
//
// ⚠ 범위 밖 값(위 테스트)은 **여전히 거부**다. 둘을 같이 두는 이유는 "빈칸 허용" 이
//    "아무 값이나 허용" 으로 번지지 않게 못박기 위해서다 — 빈칸은 의도 표명이고
//    `[0]`·`[5]` 는 오타다.
// ⚠ **셸 인용에 주의.** `-p active_motors:=""` 를 셸에 그대로 주면 셸이 따옴표를 벗겨
//    `-p active_motors:=` 가 되고, rcl 이 *"Couldn't parse parameter override rule"* 로
//    **노드를 죽인다**(SIGABRT). 빈 문자열은 리터럴로 도달해야 하므로 홑따옴표로 감싼다.
//    (웹 감독자는 `subprocess` 에 리스트로 넘기므로 셸을 거치지 않아 이 문제가 없다.)
TEST(ActiveMotors, EmptyStringMeansLeaveEcuValueAlone) {
    EXPECT_EQ(RunNode("-p 'active_motors:=\"\"'"), kTimedOut)
        << "빈 문자열은 기동 거부가 아니다 — motor_mask 를 쓰지 않고 ECU 값을 채택한다";
}

// 빈칸이 "아무 문자열이나 허용" 으로 번지지 않게. 오타는 여전히 기동 거부다.
TEST(ActiveMotors, NonEmptyStringIsStillATypo) {
    EXPECT_EQ(RunNode("-p active_motors:=\"none\""), kRejected)
        << "빈칸은 빈 문자열뿐이다 — 'none' 같은 단어형은 만들지 않았다";
    EXPECT_EQ(RunNode("-p active_motors:=\"1,2\""), kRejected)
        << "리스트는 \"[1,2]\" 형태여야 한다";
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
