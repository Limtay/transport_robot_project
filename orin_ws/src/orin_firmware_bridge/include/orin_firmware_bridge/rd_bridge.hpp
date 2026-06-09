#ifndef ORIN_FIRMWARE_BRIDGE__RD_BRIDGE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include <array>
#include "orin_firmware_bridge/rd_map.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "mgs01_base_msgs/msg/jeon_gae.hpp"

namespace orin_bridge {

class RdBridge : public rclcpp::Node {
public:
    explicit RdBridge(RobotState_t* robot_state);
    virtual ~RdBridge();

    void Start();
    void SetHardwareStatus(bool is_connected, const std::string& error_msg = "");
    void GetRosInputs();
    void PublishMotorFeedback();  // 100Hz
    void PublishStatus();         // 10Hz

private:
    RobotState_t* state_;

    // === Subscribers ===
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_vel_;
    rclcpp::Subscription<mgs01_base_msgs::msg::JeonGae>::SharedPtr sub_jeongae_;

    // === Publishers (기존) ===
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr          pub_battery_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_ecu_connected_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_dpc_connected_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_pcu_connected_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr           pub_ecu_fsm_state_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr         pub_ecu_alive_time_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_current_raw_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_current_filtered_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_pose_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_speed_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr  pub_motor_temp_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr  pub_motor_error_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_linkage_angle_;

    // === Publishers (신규: 에러 채널별 분리) ===
    // 채널 순서/비트 순서 공통: idx 0=uart1, 1=uart2, 2=uart4, 3=can, 4=i2c (HW_BIT_* 와 정렬)
    static constexpr int kNumErrCh = 5;
    static constexpr std::array<const char*, kNumErrCh> kErrCh =
        {"uart1", "uart2", "uart4", "can", "i2c"};

    std::array<rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr, kNumErrCh> pub_degraded_cnt_; // 오염도 %
    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr,  kNumErrCh> pub_hw_reset_;     // 비트
    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr,  kNumErrCh> pub_hw_error_;     // 비트
    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr,  kNumErrCh> pub_hw_fatal_;     // 비트

    // STATE_t 를 lc(lifecycle)/hs(health) 로 분리 — motor / encoder / rc
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_lc_motor_,   pub_hs_motor_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_lc_encoder_, pub_hs_encoder_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_lc_rc_,      pub_hs_rc_;

    // 모터별 comm_err (2bit×4 packed): 0=OK / 1=RX / 2=TX / 3=BOTH
    std::array<rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr, 4> pub_motor_comm_err_;

    // === EMA 필터 ===
    static constexpr float kCurrentFilterAlpha = 0.05f;
    float current_filtered_[4]       = {0.0f, 0.0f, 0.0f, 0.0f};
    bool  current_filter_initialized_ = false;

    // === Callbacks ===
    void CallbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg);

    // === Input 저장소 ===
    struct InputData {
        double linear_x     = 0.0;
        double angular_z    = 0.0;
        bool   jeongae_open = false;
        rclcpp::Time last_cmd_time;
    } inputs_;

    std::thread spin_thread_;
    std::mutex  data_mutex_;

    // publish 전용 타이머 (spin_thread_ 위에서 실행, UART 루프와 완전 분리)
    rclcpp::TimerBase::SharedPtr publish_timer_fast_;  // 100Hz: 모터 피드백
    rclcpp::TimerBase::SharedPtr publish_timer_slow_;  // 10Hz: 에러·상태
};

} // namespace orin_bridge

#endif
