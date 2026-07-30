// 디렉터리가 곧 계층인가 (redesign/02 §4, A3)
//
// ## 왜 CMake 분리만으로는 부족한가
//
// 02 §4.1 의 2분할(`rd_core_lib` 는 rclcpp 를 링크하지 않는다)이 **실질 안전장치**이고
// 그건 이미 적용돼 있다. 누군가 `policy/` 에서 `RCLCPP_INFO` 를 쓰면 링크 에러가 난다.
//
// 그런데 링커가 못 잡는 것이 둘 있다:
//
//   ① **헤더 전용 위반.** `#include <rclcpp/rclcpp.hpp>` 만 하고 심볼을 안 쓰면
//      링크는 통과한다. 그 순간 L0~L2 는 ROS 없이 컴파일할 수 없게 되고,
//      "노드 없이 도는 유닛 테스트" 라는 A5 의 성과가 조용히 사라진다.
//   ② **역방향 include.** `core/` 가 `ros/` 를 include 해도 두 헤더가 같은
//      include 경로 아래 있으므로 컴파일된다. 의존 방향은 컴파일러의 관심사가 아니다.
//
// 02 §4.1 은 이것을 "CI 보조 검사 (선택)" 로 적어 뒀다. 선택으로 두면 아무도 안 돌리므로
// 테스트로 옮긴다 — 디렉터리를 나눈 값어치는 **경계가 지켜질 때만** 생긴다.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string Slurp(const fs::path& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// 하위 계층 3개. 상위(`ros/`)와 루트 공용 헤더는 대상이 아니다.
const std::vector<std::string> kLowerLayers = {"core", "sched", "policy"};

std::vector<fs::path> LowerLayerFiles() {
    std::vector<fs::path> out;
    const fs::path root(RD_SRC_DIR);
    for (const auto& lay : kLowerLayers) {
        for (const auto& dir : {root / "src" / lay,
                                root / "include" / "orin_firmware_bridge" / lay}) {
            if (!fs::exists(dir)) continue;
            for (const auto& e : fs::directory_iterator(dir)) {
                if (e.is_regular_file()) out.push_back(e.path());
            }
        }
    }
    return out;
}

}  // namespace

// 디렉터리가 실제로 있는지 — 아래 검사들은 파일이 0개여도 통과하므로 먼저 못박는다.
TEST(LayerBoundary, TheFourLayerDirectoriesExist) {
    const fs::path inc = fs::path(RD_SRC_DIR) / "include" / "orin_firmware_bridge";
    for (const char* lay : {"core", "sched", "policy", "ros"}) {
        EXPECT_TRUE(fs::is_directory(inc / lay))
            << "include/orin_firmware_bridge/" << lay << " 가 없다 — 02 §4 의 배치가 아니다";
    }
    EXPECT_GE(LowerLayerFiles().size(), 20u)
        << "하위 계층 파일이 너무 적다 — 경로 상수(RD_SRC_DIR)가 틀렸을 수 있다";
}

// ① L0~L2 는 rclcpp 를 **이름으로도** 몰라야 한다.
TEST(LayerBoundary, LowerLayersDoNotMentionRclcpp) {
    for (const auto& p : LowerLayerFiles()) {
        const auto src = Slurp(p);
        // 주석에 "rclcpp 를 링크하지 않는다" 처럼 적는 것은 허용해야 한다.
        // 그래서 **include 문과 심볼 사용**만 본다.
        EXPECT_EQ(src.find("#include <rclcpp"), std::string::npos)
            << p.string() << " 가 rclcpp 를 include 한다.\n"
            << "  링크 에러는 안 나지만, 이 순간 L0~L2 는 ROS 없이 컴파일할 수 없게 된다 —\n"
            << "  노드를 띄우지 않는 유닛 테스트(test/unit)가 전부 ROS 의존이 된다.";
        EXPECT_EQ(src.find("#include \"rclcpp"), std::string::npos) << p.string();
        EXPECT_EQ(src.find("RCLCPP_"), std::string::npos)
            << p.string() << " 가 RCLCPP_* 로그 매크로를 쓴다 — 로깅은 ILogger(A5) 로 한다";
        EXPECT_EQ(src.find("rclcpp::"), std::string::npos)
            << p.string() << " 가 rclcpp:: 심볼을 쓴다";
    }
}

// ② 의존 방향 — 하위가 상위(`ros/`)를 include 하면 A1-(c) 역방향 의존이 되살아난다.
//    그것을 없애려고 BridgeConfig(02 §5.4)와 ITelemetrySink(A1-b)를 만든 것이다.
TEST(LayerBoundary, LowerLayersDoNotIncludeTheRosLayer) {
    for (const auto& p : LowerLayerFiles()) {
        EXPECT_EQ(Slurp(p).find("orin_firmware_bridge/ros/"), std::string::npos)
            << p.string() << " 가 ros/ 계층을 include 한다.\n"
            << "  설정이 필요하면 BridgeConfig 를 const& 로 받고(02 §5.4),\n"
            << "  상위에 알릴 것이 있으면 ITelemetrySink 로 통지한다(A1-b).\n"
            << "  상위 객체를 직접 잡는 것이 재설계 전 코드의 문제였다.";
    }
}

// ③ 계층 순서 — 아래만 본다.
//
// ⚠ **실제 방향은 `sched → policy` 다** (스케줄 루프가 매 tick 정책 객체를 호출한다).
//    호출하는 쪽이 의존하는 쪽이므로 policy 가 sched 보다 아래다.
//
//    02 문서가 이 지점에서 자기 모순이다:
//      §3   "의존 방향: L3 → L2 → L1 → L0 단방향"  (L1=sched, L2=policy → policy→sched 를 뜻함)
//      §4.2 "`L1→L2` 는 정상 방향" ×2                (sched→policy — **코드가 이쪽이다**)
//    확인 결과 `policy/` 는 `sched/` 를 **한 번도** include 하지 않고, `sched/rd_schedule.hpp`
//    가 policy 3개를 include 한다. §4.2 와 코드가 일치하므로 §3 의 화살표가 낡은 것이다.
//    (문서 정정 대상 — 여기서는 코드가 하는 대로 고정한다.)
TEST(LayerBoundary, EachLayerOnlyLooksDownward) {
    struct Rule { const char* layer; std::vector<const char*> forbidden; };
    for (const Rule& r : {Rule{"core",   {"sched/", "policy/"}},
                          Rule{"policy", {"sched/"}}}) {
        const fs::path root(RD_SRC_DIR);
        for (const auto& dir : {root / "src" / r.layer,
                                root / "include" / "orin_firmware_bridge" / r.layer}) {
            if (!fs::exists(dir)) continue;
            for (const auto& e : fs::directory_iterator(dir)) {
                if (!e.is_regular_file()) continue;
                const auto src = Slurp(e.path());
                for (const char* f : r.forbidden) {
                    EXPECT_EQ(src.find(std::string("orin_firmware_bridge/") + f), std::string::npos)
                        << e.path().string() << " (" << r.layer << ") 가 " << f
                        << " 를 include 한다 — 계층이 아래만 본다는 규칙을 깬다";
                }
            }
        }
    }
}
