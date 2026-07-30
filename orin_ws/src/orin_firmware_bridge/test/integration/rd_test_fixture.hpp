// 노드 + 가짜 ECU + tick 모사를 묶은 공용 픽스처.
// 스케줄 루프가 실제로 하는 순서(out-of-span 대체 → TickProfile → PrepareControlCommand)를
// 그대로 재현한다. 시리얼만 가짜이고 그 위 로직은 전부 실제 코드.
#ifndef RD_TEST_FIXTURE_HPP_
#define RD_TEST_FIXTURE_HPP_

#include "rd_test_common.hpp"
#include "orin_firmware_bridge/ros/rd_node.hpp"
#include <mutex>
#include <vector>
#include <string>

using Cfg = mgs_tp_msgs::srv::ControlConfig;
using RunProfileAct = mgs_tp_msgs::action::RunProfile;

class BridgeFixture : public ::testing::Test {
protected:
    void SetUp() override {
        // 기동 시점의 현실을 맞춰 둔다: INIT(auto_mode=1) 직후 ECU 는 ctr_mode 를 CURRENT 로
        // 강제하므로 read 세그가 실어오는 값도 CURRENT 다. 0(ESTOP)으로 두면 ticker 가
        // 첫 echo 를 하기 전에 나간 goal 이 §2.6-3 가드에 걸려 테스트가 타이밍에 흔들린다.
        for (int i = 0; i < 4; i++)
            state_.ecu.reg.cmd_motor.ctr_mode[i] = orin_bridge::ecu::CTR_MODE_CURRENT;

        node_ = std::make_shared<orin_bridge::RdNode>(&state_);
        node_->Control().MarkInitDone();      // INIT 플로우(시리얼 필요)의 대역
        node_->StartProfileServer();
        cli_node_ = std::make_shared<rclcpp::Node>("rd_test_client");
        cfg_cli_  = cli_node_->create_client<Cfg>("/carrier/control/config");
        act_cli_  = rclcpp_action::create_client<RunProfileAct>(
                        cli_node_, "/carrier/control/run_profile");

        exec_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        exec_->add_node(node_);
        exec_->add_node(cli_node_);
        spinning_ = true;
        spin_ = std::thread([this]{ while (spinning_ && rclcpp::ok()) exec_->spin_once(5ms); });

        ticking_ = true;
        ticker_ = std::thread([this]{ TickLoop(); });
        ASSERT_TRUE(cfg_cli_->wait_for_service(5s)) << "config service 미발견";
        ASSERT_TRUE(act_cli_->wait_for_action_server(5s)) << "action 서버 미발견";
    }

    void TearDown() override {
        ticking_ = false;  if (ticker_.joinable()) ticker_.join();
        spinning_ = false; if (spin_.joinable())   spin_.join();
        exec_.reset(); act_cli_.reset(); cfg_cli_.reset(); cli_node_.reset(); node_.reset();
    }

    // 스케줄 루프 모사 (실시간보다 빠른 1ms — tick 당 1회라는 계약은 그대로)
    void TickLoop() {
        while (ticking_ && rclcpp::ok()) {
            orin_bridge::OosStep_t step;
            if (!stall_ && node_->Oos().TakeStep(&step)) {
                HandleOos(step);
            } else if (!stall_) {
                node_->Player().Tick(&node_->Control());
                node_->Control().PrepareWrite();
                // 실제 RW 응답의 read 세그 {128,4} 가 하는 일: ECU 실값으로 섀도를 덮는다.
                // 이걸 모사해야 §2.6-3 가드가 보는 값이 실제와 같아진다.
                if (echo_ctr_mode_) {
                    std::lock_guard<std::mutex> l(state_.state_mutex);
                    for (int i = 0; i < 4; i++)
                        state_.ecu.reg.cmd_motor.ctr_mode[i] = ecu_ctr_mode_[i];
                }
            }
            std::this_thread::sleep_for(1ms);
        }
    }

    void HandleOos(const orin_bridge::OosStep_t& step) {
        if (step.phase == orin_bridge::OosPhase::WRITE) {
            { std::lock_guard<std::mutex> l(trace_mtx_);
              trace_.push_back("W" + std::to_string(step.addr) + "=" + std::to_string(step.value)); }
            OnOosWrite(step);
            if (!fail_write_) ecu_reg_[step.addr] = corrupt_ ? uint8_t(step.value ^ 0xFF) : step.value;
            node_->Oos().ReportResult(!fail_write_, 0);
        } else {
            { std::lock_guard<std::mutex> l(trace_mtx_);
              trace_.push_back("R" + std::to_string(step.addr)); }
            node_->Oos().ReportResult(true, ecu_reg_[step.addr]);
        }
    }
    // 파생 테스트가 WRITE 시점 상태를 관측하고 싶을 때 훅
    virtual void OnOosWrite(const orin_bridge::OosStep_t&) {}

    std::pair<bool, std::string> Config(uint8_t op, std::vector<uint8_t> motors, int32_t value) {
        auto req = std::make_shared<Cfg::Request>();
        req->op = op; req->motors = motors; req->value = value;
        auto fut = cfg_cli_->async_send_request(req);
        if (fut.wait_for(5s) != std::future_status::ready) return {false, "<timeout>"};
        auto r = fut.get();
        return {r->ok, r->message};
    }

    // goal 을 보내고 수락 여부만 본다 (수락되면 즉시 취소해 정리)
    bool SendGoalAccepted(const std::string& yaml, const std::string& name = "t") {
        RunProfileAct::Goal g; g.name = name; g.profile_yaml = yaml;
        auto f = act_cli_->async_send_goal(g);
        if (f.wait_for(5s) != std::future_status::ready) return false;
        auto gh = f.get();
        if (!gh) return false;
        act_cli_->async_cancel_goal(gh);
        act_cli_->async_get_result(gh).wait_for(5s);
        return true;
    }

    void SetEcuCtrMode(int idx, uint8_t v) {
        ecu_ctr_mode_[idx] = v;
        std::lock_guard<std::mutex> l(state_.state_mutex);
        state_.ecu.reg.cmd_motor.ctr_mode[idx] = v;
    }

    std::vector<std::string> Trace() {
        std::lock_guard<std::mutex> l(trace_mtx_);
        return trace_;
    }
    void ClearTrace() { std::lock_guard<std::mutex> l(trace_mtx_); trace_.clear(); }

    orin_bridge::RobotState_t state_{};
    std::shared_ptr<orin_bridge::RdNode> node_;
    std::shared_ptr<rclcpp::Node> cli_node_;
    rclcpp::Client<Cfg>::SharedPtr cfg_cli_;
    rclcpp_action::Client<RunProfileAct>::SharedPtr act_cli_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
    std::thread spin_, ticker_;
    std::atomic<bool> spinning_{false}, ticking_{false};

    uint8_t ecu_reg_[256] = {0};
    uint8_t ecu_ctr_mode_[4] = {1, 1, 1, 1};   // 가짜 ECU 의 실제 ctr_mode
    std::atomic<bool> echo_ctr_mode_{true};    // read 세그 모사 on/off
    std::atomic<bool> fail_write_{false}, corrupt_{false}, stall_{false};
    std::mutex trace_mtx_;
    std::vector<std::string> trace_;
};

#endif
