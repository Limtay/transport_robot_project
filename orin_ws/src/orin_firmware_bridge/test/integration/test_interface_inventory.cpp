// 인터페이스 인벤토리 (redesign/06 §2.4)
//
// 왜 5단계 **직전에** 만드는가: `RdNode` 를 4개로 쪼갤 때 토픽·서비스·액션이 하나라도
// 사라지면 **실기에서야 발견된다.** 골든 바이트 테스트는 wire 만 보므로 ROS 표면은 못 본다.
// 분할 전에 목록을 고정해두면, 분할 후 같은 테스트가 "빠진 게 없다" 를 말해준다.
//
// ⚠ 이 테스트는 기대값을 **여기 하드코딩**한다. 노드에서 읽어와 비교하면 노드가 바뀔 때
//   기대값도 같이 바뀌어 아무것도 증명하지 못한다 (06 §2.1 의 함정).

#include "rd_test_common.hpp"
#include "orin_firmware_bridge/ros/rd_node.hpp"
#include "orin_firmware_bridge/policy/rd_command.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

namespace {

// 노드가 실제로 광고 중인 이름을 모은다. graph API 는 discovery 지연이 있어 재시도한다.
std::set<std::string> Names(rclcpp::Node* probe, const std::string& node_name,
                            char kind) {  // 'p'=publisher 'sub' 'srv'
    std::set<std::string> out;
    for (int attempt = 0; attempt < 20 && out.empty(); attempt++) {
        auto by_node = (kind == 'p') ? probe->get_node_graph_interface()->get_publisher_names_and_types_by_node(node_name, "/")
                     : (kind == 's') ? probe->get_node_graph_interface()->get_subscriber_names_and_types_by_node(node_name, "/")
                                     : probe->get_node_graph_interface()->get_service_names_and_types_by_node(node_name, "/");
        for (const auto& kv : by_node) out.insert(kv.first);
        if (out.empty()) {
            rclcpp::spin_some(probe->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return out;
}

void ExpectHas(const std::set<std::string>& got, const std::vector<std::string>& want,
               const char* what) {
    for (const auto& w : want) {
        EXPECT_TRUE(got.count(w) > 0) << what << " 에 '" << w << "' 가 없다 — 분할 중 유실됐다";
    }
}

class InventoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = std::make_shared<orin_bridge::RobotState_t>();
        node_  = std::make_shared<orin_bridge::RdNode>(state_.get());
        // 조립 순서를 main.cpp 와 맞춘다 — command_set/jeongae_lock 서비스는
        // AttachCommand 안에서 생성되므로, 이걸 빠뜨리면 "원래 없는 것" 으로 오판한다.
        command_ = std::make_unique<orin_bridge::RdCommand>(state_.get());
        node_->AttachCommand(command_.get());
        node_->StartProfileServer();   // action 서버는 INIT 통과 후에 열린다 (실기 대역)
        probe_ = std::make_shared<rclcpp::Node>("inventory_probe");

        // ⚠ node_->Start() 를 쓰면 안 된다 — 그 spin 스레드는 rclcpp::shutdown() 까지
        //   끝나지 않아 소멸자의 join 이 영구 블록된다. 기존 하네스와 같은 방식으로 돈다.
        exec_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        exec_->add_node(node_);
        exec_->add_node(probe_);
        spinning_ = true;
        spin_ = std::thread([this]{ while (spinning_ && rclcpp::ok()) exec_->spin_once(5ms); });
        std::this_thread::sleep_for(500ms);   // discovery
    }
    void TearDown() override {
        spinning_ = false;
        if (spin_.joinable()) spin_.join();
        exec_.reset(); probe_.reset(); node_.reset(); command_.reset(); state_.reset();
    }

    std::shared_ptr<orin_bridge::RobotState_t> state_;
    std::shared_ptr<orin_bridge::RdNode>     node_;
    std::shared_ptr<rclcpp::Node>              probe_;
    std::unique_ptr<orin_bridge::RdCommand>   command_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
    std::atomic<bool> spinning_{false};
    std::thread spin_;
};

// project 계약 (03 §5.2) — 기본 모드에서 항상 있어야 하는 것들
TEST_F(InventoryTest, ProjectPublishers) {
    auto got = Names(probe_.get(), "firmware_bridge_node", 'p');
    ASSERT_FALSE(got.empty()) << "노드의 publisher 를 하나도 못 찾았다 (discovery 실패?)";
    // 03 §5.2 — project 토픽 47개가 6개로 접혔다. 이 목록이 계약의 전부다.
    ExpectHas(got, {
        "/carrier_battery",          // 계약명 고정
        "/carrier_imu",              // 계약명 고정
        "/carrier/ecu/status",       // NodeStatus — 구 8개 대체
        "/carrier/dpc/status",
        "/carrier/pcu/status",
        "/carrier/ecu/motor",        // MotorStatus — 구 10개 대체
    }, "publishers");

    // 삭제된 것이 실제로 사라졌는지도 본다 — 개명은 "새 이름이 생겼다" 만으로는 부족하다.
    for (const char* gone : {"/carrier/status", "/carrier/battery/soc", "/carrier/ecu/imu",
                             "/carrier/ecu/connected", "/carrier/ecu/fsm",
                             "/carrier/ecu/motor/pose", "/carrier/ecu/lc/motor",
                             "/carrier/ecu/error/hw_error/can",
                             "/carrier/ecu/sensor/linkage_angle",
                             "/carrier/testbed/feedback"}) {
        EXPECT_EQ(got.count(gone), 0u) << gone << " 가 아직 남아 있다 — 구 계약 잔재";
    }
}

TEST_F(InventoryTest, ProjectSubscriptions) {
    auto got = Names(probe_.get(), "firmware_bridge_node", 's');
    // ⚠ 실제 이름은 `/carrier_cmd_vel`(밑줄) 과 `/jeongae`(네임스페이스 없음) 이다.
    //   03 §5.2 의 명명 규칙과 어긋나지만 **지금 고치면 동작 변경**이라 7단계 개명에서 다룬다.
    ExpectHas(got, {"/carrier_cmd_vel", "/jeongae"}, "subscriptions");
}

TEST_F(InventoryTest, Services) {
    auto got = Names(probe_.get(), "firmware_bridge_node", 'v');
    ExpectHas(got, {
        "/carrier/command_set",
        "/carrier/jeongae_lock",
        "/carrier/control/config",
    }, "services");
}

// control 계약 토픽은 **기본 모드에 없어야 한다** (03 §5.2 — control 전용).
// 구 TestbedFeedback 은 모드와 무관하게 만들어졌다. 7단계에서 계약대로 조건부가 됐다.
TEST_F(InventoryTest, ControlTopicsAbsentInDefaultMode) {
    auto got = Names(probe_.get(), "firmware_bridge_node", 'p');
    EXPECT_EQ(got.count("/carrier/control/feedback"), 0u)
        << "control 전용 토픽이 기본 모드에 있다";
    EXPECT_EQ(got.count("/carrier/control/comm_diag"), 0u);
}

// §2.5 계측 토픽은 **control/traction 모드에서만** 생긴다 (조건부 publisher).
//
// 이 케이스가 없어서 2026-07-28 회귀를 놓쳤다: 5단계에서 publisher 생성이 RdTelemetry
// 생성자로 옮겨졌는데 그 생성자가 **파라미터 파싱보다 먼저** 불렸다. cfg_.IsControl() 가
// 아직 false 라 계측 publisher 가 아예 안 만들어졌고, OnTxn 이 그 널 체크로
// early-return 하면서 rtt·clock_offset 갱신까지 통째로 죽었다. 기준런 bag 에서
// rtt 가 전부 0.000ms 로 나와서야 발견했다 — 골든도 기존 인벤토리도 못 보는 구멍이었다.
TEST(InventoryModes, CommDiagExistsOnlyInControlMode) {
    auto state = std::make_shared<orin_bridge::RobotState_t>();

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("bridge_mode", std::string("control")),
                              rclcpp::Parameter("comm_diag_enable", true)});
    auto node = std::make_shared<orin_bridge::RdNode>(state.get(), opts);
    auto probe = std::make_shared<rclcpp::Node>("inventory_probe_ctrl");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(probe);
    std::atomic<bool> spinning{true};
    std::thread spin([&]{ while (spinning && rclcpp::ok()) exec.spin_once(5ms); });
    std::this_thread::sleep_for(500ms);

    auto got = Names(probe.get(), "firmware_bridge_node", 'p');
    EXPECT_TRUE(got.count("/carrier/control/feedback") > 0)
        << "bridge_mode=control 인데 200Hz 피드백 토픽이 없다";
    EXPECT_TRUE(got.count("/carrier/control/comm_diag") > 0)
        << "bridge_mode=control 인데 §2.5 계측 토픽이 없다 — publisher 가 파라미터 파싱 전에 "
           "만들어졌을 가능성 (2026-07-28 회귀와 동일 증상)";

    spinning = false;
    if (spin.joinable()) spin.join();
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int rc = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return rc;
}
