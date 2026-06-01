#ifndef ORIN_FIRMWARE_BRIDGE__RD_BRIDGE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_BRIDGE_HPP_

// ROS 2 System
#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include "orin_firmware_bridge/rd_map.hpp"

// Messages
#include "geometry_msgs/msg/twist.hpp"              // cmd_vel message
#include "std_msgs/msg/u_int32.hpp"                 // battery message
#include "std_msgs/msg/u_int8.hpp"                  // fsm state message
#include "std_msgs/msg/u_int8_multi_array.hpp"      // motor health/err message
#include "std_msgs/msg/int8_multi_array.hpp"        // motor temp/error message
#include "std_msgs/msg/float32.hpp"                 // alive time message
#include "std_msgs/msg/float32_multi_array.hpp"     // motor feedback message
#include "std_msgs/msg/string.hpp"                  // status message
#include "std_msgs/msg/bool.hpp"                    // board connection status
#include "mgs01_base_msgs/msg/jeon_gae.hpp"         // jeongae message


namespace orin_bridge {

class RdBridge : public rclcpp::Node {
public:
    RdBridge(RobotState_t* robot_state);
    virtual ~RdBridge();

    // --- [Main Loop를 위한 인터페이스] ---
    // 이 함수들은 Main(Controller)에서 호출하여 데이터를 가져가거나 넣습니다.

    void Start(); // Main Loop 시작 (필요시 구현)

    // 메인 루프에서 UART/Comm 상태를 업데이트 해주는 함수
    void SetHardwareStatus(bool is_connected, const std::string& error_msg = "");

    /**
     * @brief 현재 저장된 ROS 입력값들을 한 번에 가져옵니다. (Thread-Safe)
     * @param linear_x (output) 선속도
     * @param angular_z (output) 각속도
     * @param is_open   (output) 전개 장치 상태
     */
    void GetRosInputs();

    /**
     * @brief 배터리 데이터를 받아 ROS 토픽으로 발행합니다.
     * @param battery_level 배터리 잔량 (Raw Data or Voltage)
     */
    void PublishTelemetry();

private:
    RobotState_t* state_;

    // --- [ROS 2 Interface] ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<mgs01_base_msgs::msg::JeonGae>::SharedPtr sub_jeongae_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_battery_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ecu_connected_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dpc_connected_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pcu_connected_;
    // ECU state
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_ecu_fsm_state_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_ecu_alive_time_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ecu_hw_can_err_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ecu_hw_i2c_err_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ecu_hw_uart1_err_;
    // Motor state
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_motor_health_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_motor_tx_err_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_motor_rx_err_;
    // Motor feedback
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_current_raw_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_current_filtered_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_pose_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_speed_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_motor_temp_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_motor_error_;
    // Linkage angle
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_linkage_angle_;

    // --- [Filter State] ---
    static constexpr float kCurrentFilterAlpha = 0.05f;
    float current_filtered_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool  current_filter_initialized_ = false;

    // --- [Callbacks] ---
    void CallbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg);

    // --- [Data Storage] ---
    // ROS 콜백(비동기)과 Main Loop(동기) 간의 데이터 보호를 위한 저장소
    struct InputData {
        double linear_x     = 0.0;
        double angular_z    = 0.0;
        bool   jeongae_open = false;
        
        // Watchdog: 데이터가 끊겼는지 확인하기 위한 시간 기록
        rclcpp::Time last_cmd_time; 
    } inputs_;

    std::thread spin_thread_; // 멤버 변수로 관리
    std::mutex data_mutex_; // 데이터 충돌 방지용 자물쇠
};

} // namespace orin_bridge

#endif