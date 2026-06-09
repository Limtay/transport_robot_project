#include "orin_firmware_bridge/rd_bridge.hpp"

namespace orin_bridge {

RdBridge::RdBridge(RobotState_t* robot_state) : Node("firmware_bridge_node"), state_(robot_state) {
    // === Subscribers ===
    sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/carrier_cmd_vel", 10,
        std::bind(&RdBridge::CallbackCmdVel, this, std::placeholders::_1));

    sub_jeongae_ = this->create_subscription<mgs01_base_msgs::msg::JeonGae>(
        "/jeongae", 10,
        std::bind(&RdBridge::CallbackJeongae, this, std::placeholders::_1));

    // === Publishers ===
    pub_battery_      = this->create_publisher<std_msgs::msg::UInt32>("/carrier/battery/soc", 10);
    pub_status_       = this->create_publisher<std_msgs::msg::String>("/carrier/status", 10);
    pub_ecu_connected_ = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/connected", 10);
    pub_dpc_connected_ = this->create_publisher<std_msgs::msg::Bool>("/carrier/dpc/connected", 10);
    pub_pcu_connected_ = this->create_publisher<std_msgs::msg::Bool>("/carrier/pcu/connected", 10);

    pub_ecu_fsm_state_   = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/fsm", 10);
    pub_ecu_alive_time_  = this->create_publisher<std_msgs::msg::Float32>("/carrier/ecu/alive_time", 10);

    pub_motor_current_raw_      = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/current/raw", 10);
    pub_motor_current_filtered_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/current/filtered", 10);
    pub_motor_pose_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/pose", 10);
    pub_motor_speed_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/motor/speed", 10);
    pub_motor_temp_  = this->create_publisher<std_msgs::msg::Int8MultiArray>("/carrier/ecu/motor/temp", 10);
    pub_motor_error_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/carrier/ecu/motor/error", 10);
    pub_linkage_angle_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/carrier/ecu/sensor/linkage_angle", 10);

    // === 신규: 에러 채널별 토픽 (degraded_cnt % / hw_reset·hw_error·hw_fatal 비트) ===
    for (int i = 0; i < kNumErrCh; ++i) {
        const std::string ch = kErrCh[i];
        pub_degraded_cnt_[i] = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/error/degraded_cnt/" + ch, 10);
        pub_hw_reset_[i]     = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/error/hw_reset/" + ch, 10);
        pub_hw_error_[i]     = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/error/hw_error/" + ch, 10);
        pub_hw_fatal_[i]     = this->create_publisher<std_msgs::msg::Bool>("/carrier/ecu/error/hw_fatal/" + ch, 10);
    }

    // === 신규: STATE_t lc/hs 분리 (motor / encoder / rc) ===
    pub_lc_motor_   = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/lc/motor", 10);
    pub_hs_motor_   = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/hs/motor", 10);
    pub_lc_encoder_ = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/lc/encoder", 10);
    pub_hs_encoder_ = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/hs/encoder", 10);
    pub_lc_rc_      = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/lc/rc", 10);
    pub_hs_rc_      = this->create_publisher<std_msgs::msg::UInt8>("/carrier/ecu/hs/rc", 10);

    // === 신규: 모터별 comm_err (0~3) ===
    for (int i = 0; i < 4; ++i) {
        pub_motor_comm_err_[i] = this->create_publisher<std_msgs::msg::UInt8>(
            "/carrier/ecu/motor/comm_err/" + std::to_string(i), 10);
    }

    inputs_.last_cmd_time = this->now();

    // publish 타이머 — spin_thread_ 위에서 실행, UART 루프와 완전 분리
    // 100Hz: 제어 입력 갱신 + 모터 피드백(고속 데이터)
    publish_timer_fast_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
            GetRosInputs();
            PublishMotorFeedback();
        });
    // 10Hz: 배터리/연결/시스템/에러/상태(저속 변화 데이터)
    publish_timer_slow_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { PublishStatus(); });

    RCLCPP_INFO(this->get_logger(), "RdBridge Initialized");
}

RdBridge::~RdBridge() {
    if (spin_thread_.joinable()) spin_thread_.join();
}

void RdBridge::Start() {
    if (!spin_thread_.joinable()) {
        spin_thread_ = std::thread([this]() {
            rclcpp::spin(this->get_node_base_interface());
        });
    }
}

void RdBridge::SetHardwareStatus(bool is_connected, const std::string& error_msg) {
    if (!is_connected) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Hardware Error: %s", error_msg.c_str());
    }
    std_msgs::msg::String msg;
    msg.data = is_connected ? "OK" : ("ERROR: " + error_msg);
    pub_status_->publish(msg);
}

void RdBridge::GetRosInputs() {
    // Step 1: inputs_ 복사 (data_mutex_ 짧게 유지)
    float lin, ang;
    bool  jeongae;
    bool  watchdog_triggered;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto duration       = this->now() - inputs_.last_cmd_time;
        watchdog_triggered  = duration.seconds() > 0.5;
        lin                 = static_cast<float>(inputs_.linear_x);
        ang                 = static_cast<float>(inputs_.angular_z);
        jeongae             = inputs_.jeongae_open;
    }

    // Step 2: state_ 쓰기 (state_mutex_ 보호 — map_->Encode/Decode와 경합)
    {
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        if (watchdog_triggered) {
            state_->ecu.reg.cmd_system.cmd_lin_vel = 0.0f;
            state_->ecu.reg.cmd_system.cmd_ang_vel = 0.0f;
        } else {
            state_->ecu.reg.cmd_system.cmd_lin_vel = lin;
            state_->ecu.reg.cmd_system.cmd_ang_vel = ang;
        }
        state_->dpc.reg.cmd.cmd_jeongae = jeongae;
    }
}

// 10Hz — 배터리/연결/시스템/에러/링키지/상태 (저속 변화 데이터)
void RdBridge::PublishStatus() {
    std::lock_guard<std::mutex> lock(state_->state_mutex);

    // (1) 배터리
    auto bat_msg = std_msgs::msg::UInt32();
    bat_msg.data = state_->pcu.reg.power.battery_voltage;
    pub_battery_->publish(bat_msg);

    // (2) 보드 연결 상태
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = state_->ecu.comm.is_connected;
        pub_ecu_connected_->publish(msg);
        msg.data = state_->dpc.comm.is_connected;
        pub_dpc_connected_->publish(msg);
        msg.data = state_->pcu.comm.is_connected;
        pub_pcu_connected_->publish(msg);
    }

    // (3) ECU 시스템 상태
    {
        auto fsm_msg = std_msgs::msg::UInt8();
        fsm_msg.data = state_->ecu.reg.sys.sys_state;
        pub_ecu_fsm_state_->publish(fsm_msg);

        auto alive_msg = std_msgs::msg::Float32();
        alive_msg.data = static_cast<float>(state_->ecu.reg.sys.realtime_tick) / 1000.0f; // ms → s
        pub_ecu_alive_time_->publish(alive_msg);
    }

    // (3-1) 에러 채널별: degraded_cnt(%) + hw_reset/hw_error/hw_fatal(비트)
    //       채널 idx 0=uart1,1=uart2,2=uart4,3=can,4=i2c → HW_BIT_*(1<<idx) 와 정렬
    {
        const uint8_t hw_reset = state_->ecu.reg.sys.hw_reset;
        const uint8_t hw_error = state_->ecu.reg.sys.hw_error;
        const uint8_t hw_fatal = state_->ecu.reg.sys.hw_fatal;
        auto u8 = std_msgs::msg::UInt8();
        auto b  = std_msgs::msg::Bool();
        for (int i = 0; i < kNumErrCh; ++i) {
            u8.data = state_->ecu.reg.sys.degraded_cnt[i];
            pub_degraded_cnt_[i]->publish(u8);
            b.data = (hw_reset >> i) & 0x01; pub_hw_reset_[i]->publish(b);
            b.data = (hw_error >> i) & 0x01; pub_hw_error_[i]->publish(b);
            b.data = (hw_fatal >> i) & 0x01; pub_hw_fatal_[i]->publish(b);
        }
    }

    // (4) 모터별 comm_err (2bit×4 packed, LSB=M1): 0=OK / 1=RX / 2=TX / 3=BOTH
    {
        auto u8 = std_msgs::msg::UInt8();
        for (int i = 0; i < 4; i++) {
            u8.data = (state_->ecu.reg.motor_data.comm_err >> (i * 2)) & 0x03;
            pub_motor_comm_err_[i]->publish(u8);
        }
    }

    // (6) 링키지 각도 (5채널, 0~360°)
    {
        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.resize(5);
        for (int i = 0; i < 5; i++) {
            angle_msg.data[i] = (state_->ecu.reg.encoder.encoder[i] & 0x0FFF) * 360.0f / 4096.0f;
        }
        pub_linkage_angle_->publish(angle_msg);
    }

    // (7) STATE_t lc(lifecycle)/hs(health) 분리 — motor / encoder / rc
    {
        auto lc = std_msgs::msg::UInt8();
        auto hs = std_msgs::msg::UInt8();
        const auto& motor_st = state_->ecu.reg.motor_data.state;
        const auto& enc_st   = state_->ecu.reg.encoder.state;
        const auto& rc_st    = state_->ecu.reg.rc.state;

        lc.data = motor_st.bits.lifecycle; pub_lc_motor_->publish(lc);
        hs.data = motor_st.bits.health;    pub_hs_motor_->publish(hs);
        lc.data = enc_st.bits.lifecycle;   pub_lc_encoder_->publish(lc);
        hs.data = enc_st.bits.health;      pub_hs_encoder_->publish(hs);
        lc.data = rc_st.bits.lifecycle;    pub_lc_rc_->publish(lc);
        hs.data = rc_st.bits.health;       pub_hs_rc_->publish(hs);
    }
}

// 100Hz — 모터 피드백 (motor_data 가 100Hz 로 갱신되는 고속 데이터)
void RdBridge::PublishMotorFeedback() {
    std::lock_guard<std::mutex> lock(state_->state_mutex);

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
        const float raw_cur = state_->ecu.reg.motor_data.current[i] * 0.01f; // ×0.01 [A]
        if (!current_filter_initialized_) {
            current_filtered_[i] = raw_cur;
        } else {
            current_filtered_[i] = kCurrentFilterAlpha * raw_cur
                                 + (1.0f - kCurrentFilterAlpha) * current_filtered_[i];
        }
        raw_msg.data[i]      = raw_cur;
        filtered_msg.data[i] = current_filtered_[i];
        pose_msg.data[i]     = state_->ecu.reg.motor_data.position[i] * 0.1f;   // ×0.1 [deg]
        speed_msg.data[i]    = state_->ecu.reg.motor_data.velocity[i] * 10.0f;  // ×10 [RPM]
        temp_msg.data[i]     = state_->ecu.reg.motor_data.temp[i];
        err_msg.data[i]      = static_cast<int8_t>(
                                 (state_->ecu.reg.motor_data.error_code >> (i * 4)) & 0x0F);
    }
    current_filter_initialized_ = true;

    pub_motor_current_raw_->publish(raw_msg);
    pub_motor_current_filtered_->publish(filtered_msg);
    pub_motor_pose_->publish(pose_msg);
    pub_motor_speed_->publish(speed_msg);
    pub_motor_temp_->publish(temp_msg);
    pub_motor_error_->publish(err_msg);
}

void RdBridge::CallbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    inputs_.linear_x      = msg->linear.x;
    inputs_.angular_z     = msg->angular.z;
    inputs_.last_cmd_time = this->now();
}

void RdBridge::CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    inputs_.jeongae_open = msg->open;
}

} // namespace orin_bridge
