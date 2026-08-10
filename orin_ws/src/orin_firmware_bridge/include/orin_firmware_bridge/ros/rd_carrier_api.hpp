#ifndef ORIN_FIRMWARE_BRIDGE__RD_CARRIER_API_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_CARRIER_API_HPP_

// RdCarrierApi — project 계약의 입출구 (redesign/02 A2, §5.1)
//
//   sub  /carrier_cmd_vel, /jeongae
//   srv  /carrier/command_set, /carrier/jeongae_lock
//
// **상태를 갖지 않는 것이 규칙이다** (02 §5.1). 요청을 받아 L2(`RdCommand`)에 위임하고
// 응답만 만든다. 예외는 cmd_vel 워치독이 보는 입력 신선도(`inputs_`) 하나인데, 이것은
// "마지막으로 언제 왔는가" 라는 **ROS 표면의 사실**이라 여기가 제자리다.
//
// RdTelemetry 와 같은 이유로 자체 rclcpp::Node 를 만들지 않는다 — ROS 그래프 불변.

#include <atomic>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mgs01_base_msgs/msg/jeon_gae.hpp>
#include <mgs_tp_msgs/srv/command_set.hpp>

#include "orin_firmware_bridge/policy/rd_command.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"
#include "orin_firmware_bridge/rd_config.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"

namespace orin_bridge {

// cmd_vel / jeongae 입력 스냅샷 (구 RdNode::RosInputs_t)
struct RosInputs_t {
    double linear_x = 0.0, angular_z = 0.0;
    bool   jeongae_open = false;
    rclcpp::Time last_cmd_time, last_topic_time, last_nonzero_time;
};

class RdCarrierApi {
public:
    RdCarrierApi(rclcpp::Node* node, RobotState_t* state, const BridgeConfig& cfg);

    // command_set / jeongae_lock 서비스는 커맨드 매니저가 붙은 뒤에야 의미가 있다.
    void AttachCommand(RdCommand* command);
    // B6 게이트가 safe_stop 을 물어야 한다 (01 §6.2). 없으면 WRITE 계열이 전부 거부된다 —
    // "모르면 막는다" 가 이 방향에서는 안전측이다.
    void AttachControl(RdControl* control) { control_ = control; }

    // 매 tick 전 스케줄러가 묻는다: 이번 tick 의 cmd_vel WRITE 를 건너뛸까?
    bool ShouldSkipCmdWrite();
    // 100Hz 타이머 — 토픽 입력을 shadow 로 옮긴다
    void GetRosInputs();

    // 런타임 토글 (ros2 param set) — 02 §5.4 에서 BridgeConfig 에 넣지 않은 이유는 rd_config.hpp 참조
    void SetGuardEnable(bool on) { guard_enable_.store(on); }
    // 09 §? (2026-08-07) — **0 수렴 스킵**만 따로 끄고 켠다. 기본 off.
    // 토픽 미수신(0.1s) 스킵과 **다른 축**이다: 그쪽은 "상위가 죽었다" 이고
    // 이쪽은 "살아 있는데 0 을 계속 준다" 라, 경사에서는 후자를 스킵하면 안 된다.
    void SetZeroSkipEnable(bool on) { zero_skip_enable_.store(on); }
    bool ZeroSkipEnabled() const    { return zero_skip_enable_.load(); }

private:
    rclcpp::Node*       node_;
    RobotState_t*       state_;
    const BridgeConfig& cfg_;
    RdCommand*          command_ = nullptr;
    RdControl*          control_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     sub_vel_;
    rclcpp::Subscription<mgs01_base_msgs::msg::JeonGae>::SharedPtr sub_jeongae_;
    rclcpp::Service<mgs_tp_msgs::srv::CommandSet>::SharedPtr   srv_command_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr             srv_jeongae_lock_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr             srv_zero_skip_;

    std::mutex        data_mutex_;
    RosInputs_t       inputs_;
    std::atomic<bool> guard_enable_{true};
    std::atomic<bool> zero_skip_enable_{false};   // 기본 off (경사 안전)

    void CallbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg);
    void CallbackCommandSet(const std::shared_ptr<mgs_tp_msgs::srv::CommandSet::Request> req,
                            std::shared_ptr<mgs_tp_msgs::srv::CommandSet::Response> res);
    void CallbackJeongaeLock(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void CallbackZeroSkip(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    static constexpr double kZeroEps = 1e-6;
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_CARRIER_API_HPP_
