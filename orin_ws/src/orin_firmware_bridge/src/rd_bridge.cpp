#include "orin_firmware_bridge/rd_bridge.hpp"

namespace orin_bridge {

RdBridge::RdBridge(RobotState_t* robot_state) : Node("firmware_bridge_node"), state_(robot_state) {
    // 1. Subscribers
    // (1) 속도 명령 (/carrier_cmd_vel)
    sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/carrier_cmd_vel", 10,
        std::bind(&RdBridge::CallbackCmdVel, this, std::placeholders::_1));

    // (2) 전개 명령 (/jeongae)
    sub_jeongae_ = this->create_subscription<mgs01_base_msgs::msg::JeonGae>(
        "/jeongae", 10,
        std::bind(&RdBridge::CallbackJeongae, this, std::placeholders::_1));

    // 2. Publishers
    // (1) 배터리 (/carrier_battery)
    pub_battery_ = this->create_publisher<std_msgs::msg::UInt32>("/carrier/battery/soc", 10);
    pub_status_  = this->create_publisher<std_msgs::msg::String>("/carrier/status", 10);

    // (2) 보드 연결 상태
    pub_ecu_connected_ = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/connected", 10);
    pub_dpc_connected_ = this->create_publisher<std_msgs::msg::Bool>("/carrier/dpc/connected", 10);
    pub_pcu_connected_ = this->create_publisher<std_msgs::msg::Bool>("/carrier/pcu/connected", 10);

    // (3) ECU 상태
    pub_ecu_fsm_state_   = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/fsm", 10);
    pub_ecu_alive_time_  = this->create_publisher<std_msgs::msg::Float32>("/carrier/ecu/alive_time", 10);
    pub_ecu_hw_can_err_  = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/error/can", 10);
    pub_ecu_hw_i2c_err_  = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/error/i2c", 10);
    pub_ecu_hw_uart1_err_= this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/error/uart1", 10);

    // (4) 모터 상태 (4채널)
    pub_motor_health_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/carrier/ecu/motor/health", 10);
    pub_motor_tx_err_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/carrier/ecu/motor/tx_err", 10);
    pub_motor_rx_err_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/carrier/ecu/motor/rx_err", 10);

    // (5) 모터 피드백 (4채널)
    pub_motor_current_raw_      = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/current/raw", 10);
    pub_motor_current_filtered_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/current/filtered", 10);
    pub_motor_pose_    = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/pose", 10);
    pub_motor_speed_   = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/speed", 10);
    pub_motor_temp_    = this->create_publisher<std_msgs::msg::Int8MultiArray>("/carrier/ecu/motor/temp", 10);
    pub_motor_error_   = this->create_publisher<std_msgs::msg::Int8MultiArray>("/carrier/ecu/motor/error", 10);
    pub_linkage_angle_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/sensor/linkage_angle", 10);

    // Watchdog 초기화
    inputs_.last_cmd_time = this->now();

    RCLCPP_INFO(this->get_logger(), "RdBridge Initialized (Independent Mode)");
}

RdBridge::~RdBridge() {
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

// --- [Main Loop Interface Implementation] ---

void RdBridge::Start() {
    if (!spin_thread_.joinable()) {
        spin_thread_ = std::thread([this]() {
            rclcpp::spin(this->get_node_base_interface());
        });
    }
}

void RdBridge::SetHardwareStatus(bool is_connected, const std::string& error_msg) {
    // 1. ROS 로그로 출력 (RQT Console에서 볼 수 있음)
    if (!is_connected) {
        // 평소엔 조용하다가 에러 날 때만 빨간 줄 출력
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Hardware Error: %s", error_msg.c_str());
    }

    // 2. 상태 메시지 발행 (선택 사항: 모니터링 툴용)
    std_msgs::msg::String msg;
    if (is_connected) {
        msg.data = "OK";
    } else {
        msg.data = "ERROR: " + error_msg;
    }
    pub_status_->publish(msg);
}

void RdBridge::GetRosInputs() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Watchdog Check (Safety)
    auto duration = this->now() - inputs_.last_cmd_time;
    if (duration.seconds() > 0.5) {
        state_->ecu.cmd_linear_x  = 0.0;
        state_->ecu.cmd_angular_z = 0.0;
        state_->dpc.cmd_jeongae   = inputs_.jeongae_open; // 마지막 상태 유지
    } else {
        state_->ecu.cmd_linear_x  = static_cast<int16_t>(inputs_.linear_x*1000.0);
        state_->ecu.cmd_angular_z = static_cast<int16_t>(inputs_.angular_z*1000.0);
        state_->dpc.cmd_jeongae   = inputs_.jeongae_open;
    }
}

void RdBridge::PublishTelemetry() {
    std::lock_guard<std::mutex> lock(state_->state_mutex);

    // (1) 배터리 전압
    auto bat_msg = std_msgs::msg::UInt32();
    bat_msg.data = state_->pcu.battery_voltage;
    pub_battery_->publish(bat_msg);

    // (2) 보드별 연결 상태
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = state_->ecu.comm.is_connected;
        pub_ecu_connected_->publish(msg);
        msg.data = state_->dpc.comm.is_connected;
        pub_dpc_connected_->publish(msg);
        msg.data = state_->pcu.comm.is_connected;
        pub_pcu_connected_->publish(msg);
    }

    // (3) ECU 상태
    {
        auto fsm_msg = std_msgs::msg::UInt8();
        fsm_msg.data = static_cast<uint8_t>(state_->ecu.fsm_state);
        pub_ecu_fsm_state_->publish(fsm_msg);

        auto alive_msg = std_msgs::msg::Float32();
        alive_msg.data = static_cast<float>(state_->ecu.comm.alive_time) / 10.0f;
        pub_ecu_alive_time_->publish(alive_msg);

        auto hw_msg = std_msgs::msg::Bool();
        hw_msg.data = state_->ecu.hw_err.ecu_can;
        pub_ecu_hw_can_err_->publish(hw_msg);
        hw_msg.data = state_->ecu.hw_err.ecu_i2c;
        pub_ecu_hw_i2c_err_->publish(hw_msg);
        hw_msg.data = state_->ecu.hw_err.ecu_uart1;
        pub_ecu_hw_uart1_err_->publish(hw_msg);
    }

    // (4) 모터 상태 (4채널)
    {
        auto health_msg = std_msgs::msg::UInt8MultiArray();
        auto tx_msg     = std_msgs::msg::UInt8MultiArray();
        auto rx_msg     = std_msgs::msg::UInt8MultiArray();
        health_msg.data.resize(4);
        tx_msg.data.resize(4);
        rx_msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            health_msg.data[i] = static_cast<uint8_t>(state_->ecu.motor_state[i]);
            tx_msg.data[i]     = state_->ecu.motor_tx_err_cnt[i];
            rx_msg.data[i]     = state_->ecu.motor_rx_err_cnt[i];
        }
        pub_motor_health_->publish(health_msg);
        pub_motor_tx_err_->publish(tx_msg);
        pub_motor_rx_err_->publish(rx_msg);
    }

    // (5) 모터 피드백 (4채널)
    {
        auto raw_msg      = std_msgs::msg::Float32MultiArray();
        auto filtered_msg = std_msgs::msg::Float32MultiArray();
        auto pose_msg     = std_msgs::msg::Float32MultiArray();
        auto speed_msg    = std_msgs::msg::Float32MultiArray();
        auto temp_msg     = std_msgs::msg::Int8MultiArray();
        auto err_msg      = std_msgs::msg::Int8MultiArray();
        raw_msg.data.resize(4);
        filtered_msg.data.resize(4);
        pose_msg.data.resize(4);
        speed_msg.data.resize(4);
        temp_msg.data.resize(4);
        err_msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            const float raw = state_->ecu.motor_current[i];
            // 첫 수신 시 필터 상태를 raw 값으로 초기화 (초기 튀김 방지)
            if (!current_filter_initialized_) {
                current_filtered_[i] = raw;
            } else {
                current_filtered_[i] = kCurrentFilterAlpha * raw
                                     + (1.0f - kCurrentFilterAlpha) * current_filtered_[i];
            }
            raw_msg.data[i]      = raw;
            filtered_msg.data[i] = current_filtered_[i];
            pose_msg.data[i]     = state_->ecu.motor_pose[i];
            speed_msg.data[i]    = state_->ecu.motor_speed[i];
            temp_msg.data[i]     = state_->ecu.motor_temp[i];
            err_msg.data[i]      = state_->ecu.motor_error[i];
        }
        current_filter_initialized_ = true;
        pub_motor_current_raw_->publish(raw_msg);
        pub_motor_current_filtered_->publish(filtered_msg);
        pub_motor_pose_->publish(pose_msg);
        pub_motor_speed_->publish(speed_msg);
        pub_motor_temp_->publish(temp_msg);
        pub_motor_error_->publish(err_msg);
    }

    // (6) Linkage angle (5채널, 0~4096 → 0~360 degree)
    {
        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.resize(5);
        for (int i = 0; i < 5; i++) {
            angle_msg.data[i] = static_cast<float>(state_->ecu.linkage_encoder[i]) * 360.0f / 4096.0f;
        }
        pub_linkage_angle_->publish(angle_msg);
    }
}

// --- [ROS Callbacks] ---

void RdBridge::CallbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    inputs_.linear_x = msg->linear.x;
    inputs_.angular_z = msg->angular.z;
    inputs_.last_cmd_time = this->now(); // 시간 갱신
}

void RdBridge::CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    inputs_.jeongae_open = msg->open;
}

} // namespace orin_bridge