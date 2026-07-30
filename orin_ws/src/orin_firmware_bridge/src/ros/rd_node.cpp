#include "orin_firmware_bridge/ros/rd_node.hpp"
#include <cmath>

namespace orin_bridge {

RdNode::RdNode(RobotState_t* robot_state, const rclcpp::NodeOptions& opts)
    : Node("firmware_bridge_node", opts), state_(robot_state) {
    // === Subscribers ===

    // === cmd_vel 안전장치 파라미터 (Code_modify.md: on/off 가능) ===
    config_.cmd_vel_guard_enable_default = this->declare_parameter<bool>("cmd_vel_guard_enable", true);
    config_.cmd_vel_topic_timeout = this->declare_parameter<double>("cmd_vel_topic_timeout", 0.1);
    config_.cmd_vel_zero_timeout  = this->declare_parameter<double>("cmd_vel_zero_timeout", 3.0);
    config_.imu_frame_id        = this->declare_parameter<std::string>("imu_frame_id", "imu_link");
    // ── 동작 모드 (00 Q1 / 01 §4.2) ──
    // **project | control | manual 셋뿐이다.** 구 `control_mode`/`traction_test_mode`
    // 불리언은 삭제했다 — 축이 둘이면 "둘 다 true" 같은 표현 불가능한 조합이 생기고,
    // 실제로 "동시 지정 시 traction 우선" 이라는 특례가 코드에 있었다.
    //
    // 구 traction_test_mode 는 별도 모드가 아니라 control 의 한 **구성**이다:
    //     -p bridge_mode:=control -p auto_mode:=none -p read_preset:=control_test
    {
        const std::string bm = this->declare_parameter<std::string>("bridge_mode", "project");
        if (!ParseBridgeMode(bm, &config_.bridge_mode)) {
            RCLCPP_ERROR(this->get_logger(),
                "bridge_mode='%s' 는 알 수 없음 — project|control|manual 중 하나. "
                "(구 traction_test_mode 는 `control` + `auto_mode:=none` + "
                "`read_preset:=control_test` 로 대체됐다) "
                "오타로 엉뚱한 모드가 뜨는 것을 막기 위해 기동 거부", bm.c_str());
            config_.bridge_mode_valid = false;
        }
    }

    // 01 §4.2 — 읽기 프리셋도 기동 파라미터다. 런타임 교체는 config service(op 6).
    {
        const std::string rp = this->declare_parameter<std::string>("read_preset", "control");
        if (!ecu::ParseReadPreset(rp, &config_.read_preset_param)) {
            RCLCPP_ERROR(this->get_logger(),
                "read_preset='%s' 는 알 수 없음 — control|diag|control_test 중 하나", rp.c_str());
            config_.read_preset_valid = false;
        }
    }

    config_.comm_diag_enable = this->declare_parameter<bool>("comm_diag_enable", false);
    // 04 §6 — 보드 미장착 시 timeout 폭주를 막으려 기본 off. DPC 는 레지스터가 확정돼
    // 있으므로 보드를 붙였으면 켠다 (`-p enable_dpc_read:=true`).
    config_.enable_dpc_read  = this->declare_parameter<bool>("enable_dpc_read", false);
    config_.enable_pcu_read  = this->declare_parameter<bool>("enable_pcu_read", false);
    config_.stream_timeout   = this->declare_parameter<double>("stream_timeout", 0.1);
    config_.cmd_current_max  = this->declare_parameter<double>("cmd_current_max", 30.0);

    // §3.1 active_motors — 제어에서 단일/부분 트랙만 구동할 때 지정. 기본 [1,2,3,4].
    // 여기선 검증·마스크 계산만 하고, 실제 WRITE+검증은 스케줄러 INIT 플로우가 수행한다.
    {
        std::vector<int64_t> motors;
        try {
            motors = this->declare_parameter<std::vector<int64_t>>(
                "active_motors", std::vector<int64_t>{1, 2, 3, 4});
        } catch (const std::exception& e) {
            // CLI 의 `-p active_motors:="[]"` 는 타입 추론이 안 돼 여기로 온다
            // (rclcpp 가 Type/Value 중 어느 예외를 던질지는 입력 형태에 따라 갈린다).
            // 그냥 두면 std::terminate 로 죽어 원인이 안 보이므로 사유를 남기고 기동 거부.
            RCLCPP_ERROR(this->get_logger(),
                "active_motors 파라미터 타입 오류 (%s) — 정수 리스트로 지정할 것 (예: -p active_motors:=\"[2,3]\")",
                e.what());
            config_.active_motors_valid = false;
        }
        config_.active_motor_mask   = 0;
        // 타입 오류(위 catch)로 이미 무효면 그 사유를 유지하고 빈 리스트 검사는 건너뛴다.
        if (config_.active_motors_valid && motors.empty()) {
            RCLCPP_ERROR(this->get_logger(), "active_motors 가 비어 있음 — 구동할 모터가 없다");
            config_.active_motors_valid = false;
        }
        for (int64_t m : motors) {
            if (m < 1 || m > 4) {
                RCLCPP_ERROR(this->get_logger(),
                    "active_motors 값 %ld 는 범위 밖 (1~4) — 오타로 엉뚱한 트랙이 도는 것을 막기 위해 기동 거부", m);
                config_.active_motors_valid = false;
                continue;
            }
            config_.active_motor_mask |= static_cast<uint8_t>(1u << (m - 1));
        }
        if (config_.IsControl() && config_.active_motors_valid) {
            RCLCPP_WARN(this->get_logger(), "active_motors mask=0x%02X (bit0~3 = M1~M4)", config_.active_motor_mask);
        }
    }

    // §2.6 auto_mode — write 범위가 여기서 파생된다. **단어형이 정본이다** (01 §4.2):
    // 로그·CLI 에서 `auto_mode=1` 보다 `auto_mode=current` 가 사고를 줄인다. 정수도 받는다.
    //
    //   none      아무것도 쓰지 않는다 (READ 전용) — 구 traction_test_mode
    //   current   ECU 가 ctr_mode 를 CURRENT 로 강제 → cmd_current(164:16)만 쓴다
    //   direct    ECU 미개입 → bridge 가 ctr_mode 까지 소유 (128:52)
    //   velocity  / position  — CURRENT 와 대칭 (01 §3.2, 2026-07-28 실기 확인)
    //
    // 금지 2개: kinematic 은 ECU 가 100Hz 로 ctr_mode 를 덮어써 bridge write 와 경쟁하고,
    //           control 은 ECU 미구현(motor off)이다.
    {
        // **dynamic typing** — 단어형이 정본이지만 정수도 받는다 (01 §4.2).
        // 이걸 빼면 문자열로 선언한 파라미터에 `-p auto_mode:=2` 를 주는 순간 타입 불일치
        // 예외로 노드가 죽는다 (exit 134). "정수도 병행 허용" 이 곧 dynamic typing 이다.
        rcl_interfaces::msg::ParameterDescriptor am_desc;
        am_desc.dynamic_typing = true;
        const rclcpp::ParameterValue amv = this->declare_parameter(
            "auto_mode", rclcpp::ParameterValue(std::string("current")), am_desc);
        std::string am_name;
        if (amv.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            const int64_t n = amv.get<int64_t>();
            am_name = (n == ecu::AUTO_MODE_KINEMATIC) ? "kinematic"
                    : (n == ecu::AUTO_MODE_CURRENT)   ? "current"
                    : (n == ecu::AUTO_MODE_DIRECT)    ? "direct"
                    : (n == ecu::AUTO_MODE_CONTROL)   ? "control"
                    : (n == ecu::AUTO_MODE_VELOCITY)  ? "velocity"
                    : (n == ecu::AUTO_MODE_POSITION)  ? "position" : "?";
        } else {
            am_name = amv.get<std::string>();
        }

        if (am_name == "none") {
            // READ 전용 구성. INIT 의 ECU 설정 write 를 건너뛰고 200Hz tick 도 READ 를 낸다.
            config_.auto_mode_param = AUTO_MODE_NONE;
        } else if (am_name == "current")  { config_.auto_mode_param = ecu::AUTO_MODE_CURRENT;
        } else if (am_name == "direct")   { config_.auto_mode_param = ecu::AUTO_MODE_DIRECT;
        } else if (am_name == "velocity") { config_.auto_mode_param = ecu::AUTO_MODE_VELOCITY;
        } else if (am_name == "position") { config_.auto_mode_param = ecu::AUTO_MODE_POSITION;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "auto_mode='%s' 는 control 에서 금지 — none|current|direct|velocity|position 만 허용. %s",
                am_name.c_str(),
                am_name == "kinematic"
                    ? "kinematic 은 ECU 가 ctr_mode 를 VELOCITY 로 덮어써 실험이 성립하지 않는다"
                : am_name == "control"
                    ? "control 은 ECU 미구현"
                    : "알 수 없는 값이다");
            config_.auto_mode_valid = false;
        }
        if (config_.auto_mode_valid && config_.auto_mode_param != AUTO_MODE_NONE) {
            control_.SetAutoMode(config_.auto_mode_param);
        }
    }

    if (config_.IsReadOnlyControl()) {
        RCLCPP_WARN(this->get_logger(),
            "CONTROL MODE / auto_mode=none — **200Hz 배치 READ 전용, WRITE 없음.** "
            "read_preset=%s. 구 traction_test_mode 와 같은 배치다",
            ecu::kPresets[config_.read_preset_param]->name);
    } else if (config_.IsControl()) {
        RCLCPP_WARN(this->get_logger(),
            "CONTROL MODE — 200Hz RW (cmd_motor WRITE + 배치 READ). "
            "write 소스 = 제어 FSM (기동 IDLE = 0A 고정), 클램프 ±%.1fA, "
            "auto_mode=%u(%s) → write %s, read_preset=%s",
            config_.cmd_current_max, config_.auto_mode_param,
            ecu::AutoModeName(config_.auto_mode_param),
            ecu::AutoModeWriteSpan(config_.auto_mode_param),
            ecu::kPresets[config_.read_preset_param]->name);
    }
    // L2 주입 (02 A1-a) — L2 는 BridgeConfig 를 직접 읽지 않으므로 값만 건네준다.
    // ⚠ 이걸 빠뜨리면 PrepareWrite 가 shadow_ == nullptr 로 **조용히 no-op** 이 된다.
    control_.Bind(state_, static_cast<float>(config_.cmd_current_max));
    control_.SetStreamTimeout(config_.stream_timeout);
    player_.SetClampMax(static_cast<float>(config_.cmd_current_max));

    // === 02 A2 — L3 3분할 조립. 셋 다 **이 노드 위에** 엔티티를 만든다
    //     (ROS 그래프는 노드 1개 그대로 — 나뉜 것은 C++ 책임이다). ===
    // ⚠ **반드시 파라미터 파싱이 끝난 뒤에** 만든다. RdTelemetry 는 생성자에서
    //   `cfg_.IsControl()` / `cfg_.comm_diag_enable` 을 보고 control 계약 publisher 를
    //   만들지 말지 결정하고, RdCarrierApi 는 cmd_vel_guard_enable_default 를 읽는다.
    //   앞에 두면 둘 다 기본값(false/true)으로 굳어 §2.5 계측이 통째로 죽는다 —
    //   2026-07-28 기준런에서 rtt·clock_offset 이 전부 0 으로 나와 발견했다.
    telemetry_   = std::make_unique<RdTelemetry>(this, state_, config_, &control_);
    carrier_api_ = std::make_unique<RdCarrierApi>(this, state_, config_);
    control_api_ = std::make_unique<RdControlApi>(
        this, state_, config_, &control_, &player_, &oos_,
        // 05 §5.3 — result.json 이 요구하는 값들의 스냅샷. **한 번에 모아서** 넘긴다:
        // 필드마다 따로 읽으면 tick 경계를 가로질러 서로 다른 시점의 값이 섞인다.
        [this]{
            RunCounters_t c;
            c.write_err_total    = telemetry_->WriteErrTotal();
            c.irregular_tick_cnt = telemetry_->IrregularTickCount();
            c.late_tick_cnt      = telemetry_->LateTickCount();
            c.drop_cnt           = telemetry_->DropCount();
            c.clock_converged    = telemetry_->ClockConverged();
            c.drift_ppm          = telemetry_->DriftPpm();
            c.stamp_quality_ms   = telemetry_->StampQuality();
            c.rtt_ms             = telemetry_->LastRtt();
            c.rw_err             = telemetry_->RwErr();
            c.read_preset_name   = telemetry_->ReadPresetPtr()->name;
            return c;
        });


    // 런타임 토글: ros2 param set /firmware_bridge_node cmd_vel_guard_enable false
    param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& p : params) {
                if (p.get_name() == "cmd_vel_guard_enable") {
                    carrier_api_->SetGuardEnable(p.as_bool());
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

    RCLCPP_INFO(this->get_logger(), "RdNode Initialized");
}

RdNode::~RdNode() {
    if (telemetry_) telemetry_->Stop();
    if (spin_thread_.joinable()) spin_thread_.join();
}

void RdNode::Start() {
    telemetry_->Start();
    if (!spin_thread_.joinable()) {
        spin_thread_ = std::thread([this]() {
            // 기동 직후 종료(§3.1 파라미터 오류 등)와 경합하면 이미 shutdown 된 컨텍스트에서
            // spin 이 guard condition 을 만들다 RCLError 로 죽어 프로세스가 SIGABRT 로 끝난다.
            // 종료 경합은 정상 경로이므로 조용히 빠져나온다 (ok() 검사만으로는 창이 남아 try 병용).
            if (!rclcpp::ok()) return;
            try {
                // MultiThreaded: config service 가 read-back 검증으로 최대 50ms 블록되는 동안
                // action 피드백·취소가 멈추면 안 된다. 기본 콜백 그룹은 MutuallyExclusive 라
                // 기존 콜백들끼리의 직렬성은 그대로 유지된다 (새 경합 없음) — 전용 그룹을
                // 가진 config service 만 병렬로 돈다.
                rclcpp::executors::MultiThreadedExecutor exec;
                exec.add_node(this->get_node_base_interface());
                exec.spin();
            } catch (const rclcpp::exceptions::RCLError& e) {
                RCLCPP_DEBUG(this->get_logger(), "spin 종료 경합 (무시): %s", e.what());
            }
        });
    }
}

void RdNode::AttachReadPresetSetter(std::function<bool(uint8_t, std::string*)> fn) {
    if (control_api_) control_api_->SetReadPresetSetter(std::move(fn));
}

} // namespace orin_bridge
