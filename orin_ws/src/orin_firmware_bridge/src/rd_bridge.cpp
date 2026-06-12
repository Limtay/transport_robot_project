#include "orin_firmware_bridge/rd_bridge.hpp"
#include <cmath>

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
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/carrier/ecu/imu", 10);

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
            "/carrier/ecu/motor/comm_err/m" + std::to_string(i), 10);
    }

    inputs_.last_cmd_time     = this->now();
    inputs_.last_topic_time   = this->now();
    inputs_.last_nonzero_time = this->now();

    // === cmd_vel 안전장치 파라미터 (Code_modify.md: on/off 가능) ===
    guard_enable_.store(this->declare_parameter<bool>("cmd_vel_guard_enable", true));
    guard_topic_timeout_ = this->declare_parameter<double>("cmd_vel_topic_timeout", 0.1);
    guard_zero_timeout_  = this->declare_parameter<double>("cmd_vel_zero_timeout", 3.0);
    imu_frame_id_        = this->declare_parameter<std::string>("imu_frame_id", "imu_link");
    // 런타임 토글: ros2 param set /firmware_bridge_node cmd_vel_guard_enable false
    param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& p : params) {
                if (p.get_name() == "cmd_vel_guard_enable") {
                    guard_enable_.store(p.as_bool());
                    RCLCPP_INFO(this->get_logger(), "cmd_vel guard %s",
                                p.as_bool() ? "ON" : "OFF");
                }
            }
            return result;
        });

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

void RdBridge::AttachCommand(RdCommand* command) {
    command_ = command;

    srv_command_ = this->create_service<mgs01_base_msgs::srv::CommandSet>(
        "/carrier/command_set",
        std::bind(&RdBridge::CallbackCommandSet, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_jeongae_lock_ = this->create_service<std_srvs::srv::SetBool>(
        "/carrier/jeongae_lock",
        std::bind(&RdBridge::CallbackJeongaeLock, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Command services ready: /carrier/command_set, /carrier/jeongae_lock");
}

void RdBridge::CallbackCommandSet(
    const std::shared_ptr<mgs01_base_msgs::srv::CommandSet::Request> req,
    std::shared_ptr<mgs01_base_msgs::srv::CommandSet::Response> res) {
    if (!command_) {
        res->accepted = false;
        res->message  = "command manager 미연결";
        return;
    }
    CommandRequest_t creq;
    creq.slot       = req->slot;
    creq.action     = req->action;
    creq.target_id  = req->target_id;
    creq.inst       = req->inst;
    creq.start_addr = req->start_addr;
    creq.data_len   = req->data_len;
    creq.data       = req->data;
    creq.duration   = req->duration;
    res->accepted = command_->HandleRequest(creq, &res->message);
}

void RdBridge::CallbackJeongaeLock(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (!command_) {
        res->success = false;
        res->message = "command manager 미연결";
        return;
    }
    command_->SetJeongaeLock(req->data);
    res->success = true;
    res->message = std::string("jeongae lock ") + (req->data ? "ON" : "OFF");
}

// cmd_vel 안전장치: 50Hz WRITE 직전 스케줄러가 호출
bool RdBridge::ShouldSkipCmdWrite() {
    if (!guard_enable_.load()) return false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto now = this->now();
    // (1) jeongae 포함 명령 토픽이 100ms 내 들어오지 않음 → skip
    if ((now - inputs_.last_topic_time).seconds() > guard_topic_timeout_) return true;
    // (2) cmd_vel 이 3초 이상 0 에 수렴 → skip
    if ((now - inputs_.last_nonzero_time).seconds() > guard_zero_timeout_) return true;
    return false;
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
    // state_mutex 는 스냅샷만 잡고 즉시 해제 — publish()/DDS 직렬화는 lock 밖에서 수행.
    ecu::REGISTER_t ecu_reg;
    uint32_t battery_voltage;
    bool ecu_conn, dpc_conn, pcu_conn;
    {
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        ecu_reg         = state_->ecu.reg;                       // 256B 스냅샷
        battery_voltage = state_->pcu.reg.power.battery_voltage;
        ecu_conn        = state_->ecu.comm.is_connected;
        dpc_conn        = state_->dpc.comm.is_connected;
        pcu_conn        = state_->pcu.comm.is_connected;
    }

    // (1) 배터리
    auto bat_msg = std_msgs::msg::UInt32();
    bat_msg.data = battery_voltage;
    pub_battery_->publish(bat_msg);

    // (2) 보드 연결 상태
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = ecu_conn;
        pub_ecu_connected_->publish(msg);
        msg.data = dpc_conn;
        pub_dpc_connected_->publish(msg);
        msg.data = pcu_conn;
        pub_pcu_connected_->publish(msg);
    }

    // (3) ECU 시스템 상태
    {
        auto fsm_msg = std_msgs::msg::UInt8();
        fsm_msg.data = ecu_reg.sys.sys_state;
        pub_ecu_fsm_state_->publish(fsm_msg);

        auto alive_msg = std_msgs::msg::Float32();
        alive_msg.data = static_cast<float>(ecu_reg.sys.realtime_tick) / 1000.0f; // ms → s
        pub_ecu_alive_time_->publish(alive_msg);
    }

    // (3-1) 에러 채널별: degraded_cnt(%) + hw_reset/hw_error/hw_fatal(비트)
    //       채널 idx 0=uart1,1=uart2,2=uart4,3=can,4=i2c → HW_BIT_*(1<<idx) 와 정렬
    {
        const uint8_t hw_reset = ecu_reg.sys.hw_reset;
        const uint8_t hw_error = ecu_reg.sys.hw_error;
        const uint8_t hw_fatal = ecu_reg.sys.hw_fatal;

        // ECU reg54(hw_reset) 플래그 감지 → RCLCPP_ERROR 로 리셋 요청 알림 (Code_modify.md)
        // 처리: CLI 에서 'macro hw_reset <ch>' → ECU reg5 에 해당 비트 WRITE
        if (hw_reset != 0) {
            std::string chs;
            for (int i = 0; i < kNumErrCh; ++i) {
                if ((hw_reset >> i) & 0x01) {
                    if (!chs.empty()) chs += ", ";
                    chs += kErrCh[i];
                }
            }
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "ECU hw_reset 요청 (reg54=0x%02X): [%s] — 'macro hw_reset <ch>' 로 reg5 write 필요",
                hw_reset, chs.c_str());
        }

        auto u8 = std_msgs::msg::UInt8();
        auto b  = std_msgs::msg::Bool();
        for (int i = 0; i < kNumErrCh; ++i) {
            u8.data = ecu_reg.sys.degraded_cnt[i];
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
            u8.data = (ecu_reg.motor_data.comm_err >> (i * 2)) & 0x03;
            pub_motor_comm_err_[i]->publish(u8);
        }
    }

    // (6) 링키지 각도 (5채널, 0~360°)
    {
        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.resize(5);
        for (int i = 0; i < 5; i++) {
            angle_msg.data[i] = (ecu_reg.encoder.encoder[i] & 0x0FFF) * 360.0f / 4096.0f;
        }
        pub_linkage_angle_->publish(angle_msg);
    }

    // (7) STATE_t lc(lifecycle)/hs(health) 분리 — motor / encoder / rc
    {
        auto lc = std_msgs::msg::UInt8();
        auto hs = std_msgs::msg::UInt8();
        const auto& motor_st = ecu_reg.motor_data.state;
        const auto& enc_st   = ecu_reg.encoder.state;
        const auto& rc_st    = ecu_reg.rc.state;

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
    // state_mutex 는 스냅샷만 잡고 즉시 해제 — publish()/DDS 직렬화는 lock 밖에서 수행해
    // 200Hz comm 루프(Encode/Decode)의 state_mutex 대기를 없앤다.
    ecu::DATA_MOTOR_t md;
    ecu::DATA_IMU_t   imu;
    {
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        md  = state_->ecu.reg.motor_data;
        imu = state_->ecu.reg.imu;
    }

    // === IMU: STM raw → SI 단위 변환 후 sensor_msgs/Imu 발행 ===
    // quat z,y,x,w(×0.0001 무단위) / gyro x,y,z(×0.1 deg/s → rad/s) / acc x,y,z(×0.001 g → m/s²)
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp    = this->now();
        imu_msg.header.frame_id = imu_frame_id_;

        imu_msg.orientation.x = imu.quat_x * kQuatScale;
        imu_msg.orientation.y = imu.quat_y * kQuatScale;
        imu_msg.orientation.z = imu.quat_z * kQuatScale;
        imu_msg.orientation.w = imu.quat_w * kQuatScale;

        imu_msg.angular_velocity.x = imu.gyro_x * kGyroToRads;
        imu_msg.angular_velocity.y = imu.gyro_y * kGyroToRads;
        imu_msg.angular_velocity.z = imu.gyro_z * kGyroToRads;

        imu_msg.linear_acceleration.x = imu.acc_x * kAccToMs2;
        imu_msg.linear_acceleration.y = imu.acc_y * kAccToMs2;
        imu_msg.linear_acceleration.z = imu.acc_z * kAccToMs2;

        // 공분산 미추정 → REP-145 의 "unknown" 표기(0 행렬). 단, IMU 가 OFFLINE 이면
        // orientation_covariance[0] = -1 로 "orientation 없음" 통보.
        if (imu.state.bits.lifecycle == LS_OFFLINE) {
            imu_msg.orientation_covariance[0] = -1.0;
        }
        pub_imu_->publish(imu_msg);
    }

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
        const float raw_cur = md.current[i] * 0.01f; // ×0.01 [A]
        if (!current_filter_initialized_) {
            current_filtered_[i] = raw_cur;
        } else {
            current_filtered_[i] = kCurrentFilterAlpha * raw_cur
                                 + (1.0f - kCurrentFilterAlpha) * current_filtered_[i];
        }
        raw_msg.data[i]      = raw_cur;
        filtered_msg.data[i] = current_filtered_[i];
        pose_msg.data[i]     = md.position[i] * 0.1f;   // ×0.1 [deg]
        speed_msg.data[i]    = md.velocity[i] * 10.0f;  // ×10 [RPM]
        temp_msg.data[i]     = md.temp[i];
        err_msg.data[i]      = static_cast<int8_t>(
                                 (md.error_code >> (i * 4)) & 0x0F);
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
    inputs_.linear_x        = msg->linear.x;
    inputs_.angular_z       = msg->angular.z;
    auto now                = this->now();
    inputs_.last_cmd_time   = now;
    inputs_.last_topic_time = now;
    // 안전장치 (2): 0 이 아닌 명령이 들어온 마지막 시각 기록
    if (std::abs(msg->linear.x) > kZeroEps || std::abs(msg->angular.z) > kZeroEps) {
        inputs_.last_nonzero_time = now;
    }
}

void RdBridge::CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        inputs_.jeongae_open    = msg->open;
        inputs_.last_topic_time = this->now();  // 안전장치 (1): jeongae 도 명령 토픽에 포함
    }
    // jeongae 자동 전개 시퀀스 트리거 (§3) — lock 검사는 FSM 쪽에서 수행
    if (msg->open && command_) {
        command_->TriggerJeongae();
    }
}

} // namespace orin_bridge
