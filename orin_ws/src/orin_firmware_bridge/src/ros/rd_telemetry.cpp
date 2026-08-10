#include "orin_firmware_bridge/ros/rd_telemetry.hpp"

#include <chrono>
#include <cmath>

namespace orin_bridge {

namespace {
// 진단 배열 인덱스 규약 (03 §3.1) — hw_error/fatal/reset 비트 위치와 **동일**하다.
//   0=uart1/RC  1=uart2/RS485  2=uart6/IMU  3=can1/모터  4=i2c1/엔코더  5=adc/로드셀  6,7=RSVD
constexpr int kIdxRc       = 0;
constexpr int kIdxRs485    = 1;
constexpr int kIdxImu      = 2;
constexpr int kIdxMotor    = 3;
constexpr int kIdxEncoder  = 4;
constexpr int kIdxLoadcell = 5;
}  // namespace

RdTelemetry::RdTelemetry(rclcpp::Node* node, RobotState_t* state, const BridgeConfig& cfg,
                         RdControl* control)
    : node_(node), state_(state), cfg_(cfg), control_(control) {
    // ── project 계약 (03 §5.2) — 계약명 고정 2개 ──
    pub_battery_ = node_->create_publisher<std_msgs::msg::UInt8>("/carrier_battery", 10);
    pub_imu_     = node_->create_publisher<sensor_msgs::msg::Imu>("/carrier_imu", 10);

    // ── 진단 (우리 팀 전용 네임스페이스) ──
    pub_ecu_status_   = node_->create_publisher<mgs_tp_msgs::msg::NodeStatus>("/carrier/ecu/status", 10);
    pub_dpc_status_   = node_->create_publisher<mgs_tp_msgs::msg::NodeStatus>("/carrier/dpc/status", 10);
    pub_pcu_status_   = node_->create_publisher<mgs_tp_msgs::msg::NodeStatus>("/carrier/pcu/status", 10);
    pub_motor_status_ = node_->create_publisher<mgs_tp_msgs::msg::MotorStatus>("/carrier/ecu/motor", 10);

    // ── control 계약 — 토픽 1개 (+디버그 1) ──
    // ⚠ 반드시 파라미터 파싱 뒤에 생성돼야 한다. 앞에 두면 cfg_ 가 기본값이라
    //   조건부 publisher 가 안 만들어진다 (06 §4.10 회귀).
    if (cfg_.IsControl()) {
        pub_control_feedback_ = node_->create_publisher<mgs_tp_msgs::msg::ControlFeedback>(
            "/carrier/control/feedback", 100);
        if (cfg_.comm_diag_enable) {
            pub_comm_diag_ = node_->create_publisher<mgs_tp_msgs::msg::CommDiag>(
                "/carrier/control/comm_diag", 10);
        }
    }
}

RdTelemetry::~RdTelemetry() { Stop(); }

void RdTelemetry::Start() {
    // 200Hz 발행 스레드 (02 A4) — 놀고 있으면 1ms sleep 이라 비용이 없다. 항상 띄운다
    // (모드별 분기를 두면 "모드에 따라 발행 경로가 다르다" 는 규칙이 하나 더 생긴다).
    if (!tele_thread_.joinable()) {
        tele_running_.store(true);
        tele_thread_ = std::thread(&RdTelemetry::TelemetryLoop, this);
    }
}

void RdTelemetry::Stop() {
    tele_running_.store(false);
    if (tele_thread_.joinable()) tele_thread_.join();
}

// ===== 10Hz — NodeStatus x3 + MotorStatus + battery =====
//
// 구 토픽 26개(status, connected x3, fsm, alive_time, error 20, lc/hs 6, motor 10)가
// 여기 4개로 접힌다. 03 §4.1.
void RdTelemetry::PublishStatus() {
    ecu::REGISTER_t ecu_reg;
    dpc::REGISTER_t dpc_reg;
    uint32_t battery_voltage;
    bool ecu_conn, dpc_conn, pcu_conn;
    {
        // state_mutex 는 스냅샷만 잡고 즉시 해제 — publish()/DDS 직렬화는 lock 밖에서.
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        ecu_reg         = state_->ecu.reg;                       // 256B 스냅샷
        dpc_reg         = state_->dpc.reg;
        battery_voltage = state_->pcu.reg.power.battery_voltage;
        ecu_conn        = state_->ecu.comm.is_connected;
        dpc_conn        = state_->dpc.comm.is_connected;
        pcu_conn        = state_->pcu.comm.is_connected;
    }

    const auto stamp = node_->now();

    // (1) 배터리 — 03 §5.2 계약명 고정
    // TODO: 구 토픽명은 `/carrier/battery/soc` 인데 싣는 값은 battery_voltage 였다.
    //       이름과 내용이 원래부터 어긋나 있다. PCU 레지스터 확정 시 정리 필요.
    {
        std_msgs::msg::UInt8 m;
        m.data = static_cast<uint8_t>(battery_voltage > 255 ? 255 : battery_voltage);
        pub_battery_->publish(m);
    }

    // (2) NodeStatus — ECU
    {
        mgs_tp_msgs::msg::NodeStatus m;
        m.header.stamp    = stamp;
        m.header.frame_id = "ecu";
        m.connected  = ecu_conn;
        m.fsm        = ecu_reg.sys.sys_state;
        m.alive_time = static_cast<float>(ecu_reg.sys.realtime_tick) / 10000.0f;  // x0.1ms → s

        // lc/hs 는 채널별 STATE_t 에서 모은다. 안 읽은 채널은 kUnread 로 남는다.
        m.lc.fill(kUnread);
        m.hs.fill(kUnread);
        auto put = [&](int idx, STATE_t st) {
            m.lc[idx] = st.bits.lifecycle;
            m.hs[idx] = st.bits.health;
        };
        put(kIdxRc,       ecu_reg.rc.state);
        put(kIdxRs485,    ecu_reg.uart2.state);
        put(kIdxImu,      ecu_reg.imu.state);
        put(kIdxMotor,    ecu_reg.motor_data.state);
        put(kIdxEncoder,  ecu_reg.encoder.state);
        put(kIdxLoadcell, ecu_reg.loadcell.state);

        for (size_t i = 0; i < m.degraded_cnt.size(); i++) m.degraded_cnt[i] = ecu_reg.sys.degraded_cnt[i];
        m.hw_error = ecu_reg.sys.hw_error;
        m.hw_fatal = ecu_reg.sys.hw_fatal;
        m.hw_reset = ecu_reg.sys.hw_reset;
        pub_ecu_status_->publish(m);
    }

    // (3) NodeStatus — PCU. 레지스터 미확정이라 connected 만 유효하고
    //     나머지는 kUnread 다 (04 §6). "0=정상" 으로 보이게 두지 않는다.
    auto publish_stub = [&](auto& pub, bool conn, const char* frame) {
        mgs_tp_msgs::msg::NodeStatus m;
        m.header.stamp    = stamp;
        m.header.frame_id = frame;
        m.connected = conn;
        m.fsm       = kUnread;
        m.alive_time = NaNf();
        m.lc.fill(kUnread);
        m.hs.fill(kUnread);
        m.degraded_cnt.fill(kUnread);
        m.hw_error = kUnread;
        m.hw_fatal = kUnread;
        m.hw_reset = kUnread;
        pub->publish(m);
    };
    publish_stub(pub_pcu_status_, pcu_conn, "pcu");

    // (3b) NodeStatus — DPC. 레지스터가 확정돼(2026-07-30) 실값을 싣는다.
    //
    // ⚠ **ECU 와 공용 매핑을 쓸 수 없다** (04 §6.2 C5 미채택 — rd_register_dpc.hpp 표 참조):
    //   SYS 위치가 46:16 이고, hw_* 순서가 역순이며, tick 단위가 ms 다(ECU 는 ×0.1ms).
    //   보드별로 적는 것이 그 결정의 대가다.
    {
        mgs_tp_msgs::msg::NodeStatus m;
        m.header.stamp    = stamp;
        m.header.frame_id = "dpc";
        m.connected = dpc_conn;

        // 읽은 적이 없으면 **전부 미판독으로 낸다.** DPC 섀도는 0으로 초기화되는데
        // `sys_state == 0` 은 CTRL(정상값)이라, 값만 실으면 "안 읽은 것" 과 "정상 대기" 가
        // 구분되지 않는다 (rd_sequence.hpp 의 DpcSysState 와 같은 이유).
        m.lc.fill(kUnread);
        m.hs.fill(kUnread);
        m.degraded_cnt.fill(kUnread);
        if (dpc_conn) {
            m.fsm        = dpc_reg.sys.sys_state;                      // DPCB_STATE_e
            m.alive_time = static_cast<float>(dpc_reg.sys.realtime_tick) / 1000.0f;  // ms → s
            m.hw_error   = dpc_reg.sys.hw_error;
            m.hw_fatal   = dpc_reg.sys.hw_fatal;
            m.hw_reset   = dpc_reg.sys.hw_reset;
            // 채널 인덱스는 **보드마다 다르다**: DPC 는 0=uart2 1=uart4 2=uart6 3=i2c1.
            // (ECU 는 0=uart1 1=uart2 2=uart6 3=can1 4=i2c1 — 03 §3.1)
            for (size_t i = 0; i < 8 && i < m.degraded_cnt.size(); i++)
                m.degraded_cnt[i] = dpc_reg.sys.degraded_cnt[i];
            // ⚠ lc/hs 는 아직 미판독이다. DPC 의 채널별 STATE_t 는 SYS(46:16) **밖**에
            //   흩어져 있어(uart4=64 / uart2=65 / panel=73 / uart6=110) 지금 읽기 세그로는
            //   안 온다. 세그를 넓히기 전까지 0 으로 채우지 않는다.
        } else {
            m.fsm        = kUnread;
            m.alive_time = NaNf();
            m.hw_error   = kUnread;
            m.hw_fatal   = kUnread;
            m.hw_reset   = kUnread;
        }
        pub_dpc_status_->publish(m);
    }

    // (4) MotorStatus — 구 motor/* 6개 + comm_err 4개를 대체
    {
        mgs_tp_msgs::msg::MotorStatus m;
        m.header.stamp    = stamp;
        m.header.frame_id = "ecu";
        m.motor_mask = ecu_reg.cmd_system.motor_mask;
        for (int i = 0; i < 4; i++) {
            const float raw_cur = ecu_reg.motor_data.current[i] * 0.01f;   // x0.01 [A]
            m.position[i] = ecu_reg.motor_data.position[i] * 0.1f;         // x0.1 [deg]
            m.velocity[i] = ecu_reg.motor_data.velocity[i] * 10.0f;        // x10 [RPM]
            m.current[i]  = raw_cur;
            m.temp[i]     = ecu_reg.motor_data.temp[i];

            if (!current_filter_initialized_) {
                current_filtered_[i] = raw_cur;
            } else {
                current_filtered_[i] = kCurrentFilterAlpha * raw_cur
                                     + (1.0f - kCurrentFilterAlpha) * current_filtered_[i];
            }
            m.current_filtered[i] = current_filtered_[i];
        }
        current_filter_initialized_ = true;
        m.error_code = ecu_reg.motor_data.error_code;
        m.comm_err   = ecu_reg.motor_data.comm_err;
        m.state      = ecu_reg.motor_data.state.raw;
        pub_motor_status_->publish(m);
    }
}

// ===== 100Hz — IMU =====
void RdTelemetry::PublishMotorFeedback() {
    ecu::DATA_IMU_t imu;
    {
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        imu = state_->ecu.reg.imu;
    }

    auto m = sensor_msgs::msg::Imu();
    m.header.stamp    = node_->now();
    m.header.frame_id = cfg_.imu_frame_id;

    // ⚠ **판정 기준은 `delta_tick` 이지 `state` 가 아니다** (2026-07-30 실기).
    //
    // 종전에는 `state.bits.lifecycle == LS_OFFLINE` 만 봤는데, 실기에서 IMU 채널이
    // `lc=1 / hs=0`("정상")을 보고하면서 **데이터는 하나도 안 오는** 상태가 관측됐다.
    // 그래서 이 가드가 안 걸렸고, 전부 0인 섀도가 그대로 변환돼
    // **`orientation = (0,0,0,0)` 이 유효한 자세처럼 발행됐다** — 크기가 0인 사원수는
    // 물리적으로 존재할 수 없고, 정규화하는 소비자는 0으로 나눈다.
    //
    // 엔코더에서 배운 것과 같다(06 §10 B1): **채널 상태 보고보다 취득 시각이 믿을 만하다.**
    // 둘 중 하나라도 "없다" 고 하면 없는 것으로 낸다.
    //
    // ⚠ 그리고 `Reads()` 가 **맨 앞에** 와야 한다. 섀도는 0으로 초기화되므로
    //   **한 번도 안 읽은 블록은 `delta_tick == 0`, 즉 "지연 0ms = 아주 신선함"** 으로 보인다.
    //   `control_test` 프리셋(견인 실험)은 IMU 를 안 읽는데, 그 상태에서 이 가드를
    //   delta_tick 만으로 두면 초기값 0이 통과해 **전부 0인 자세가 그대로 발행된다**
    //   (2026-07-30 실기에서 실제로 그랬다). 미판독과 정상 0 을 섞지 않는다는 규칙이
    //   섀도 초기화 층에서 한 번 더 필요하다 — 03 §5.3.
    const bool imu_live = Reads(ecu::REG_IMU_OFFSET, ecu::REG_IMU_SIZE) &&
                          (imu.delta_tick != ecu::DELTA_STALE) &&
                          (imu.state.bits.lifecycle != LS_OFFLINE);

    if (imu_live) {
        m.orientation.x = imu.quat_x * kQuatScale;
        m.orientation.y = imu.quat_y * kQuatScale;
        m.orientation.z = imu.quat_z * kQuatScale;
        m.orientation.w = imu.quat_w * kQuatScale;

        m.angular_velocity.x = imu.gyro_x * kGyroToRads;
        m.angular_velocity.y = imu.gyro_y * kGyroToRads;
        m.angular_velocity.z = imu.gyro_z * kGyroToRads;

        m.linear_acceleration.x = imu.acc_x * kAccToMs2;
        m.linear_acceleration.y = imu.acc_y * kAccToMs2;
        m.linear_acceleration.z = imu.acc_z * kAccToMs2;
    } else {
        // 두 가지를 **같이** 한다. REP-145 를 지키는 소비자는 covariance 로 걸러내고,
        // 안 지키는 소비자도 NaN 을 만나 결과가 NaN 으로 번지므로 조용히 틀리지 않는다.
        // (0 으로 두면 "정지한 정상값" 과 구분되지 않는다 — 03 §5.3 과 같은 규칙이다.)
        const double nan = std::numeric_limits<double>::quiet_NaN();
        m.orientation.x = m.orientation.y = m.orientation.z = m.orientation.w = nan;
        m.angular_velocity.x = m.angular_velocity.y = m.angular_velocity.z = nan;
        m.linear_acceleration.x = m.linear_acceleration.y = m.linear_acceleration.z = nan;
        m.orientation_covariance[0]         = -1.0;   // REP-145: 추정 없음
        m.angular_velocity_covariance[0]    = -1.0;
        m.linear_acceleration_covariance[0] = -1.0;
    }
    pub_imu_->publish(m);
}

// ===== 200Hz — ControlFeedback (1 RW 트랜잭션 = 1 메시지) =====
void RdTelemetry::OnFeedback() {
    if (!pub_control_feedback_) return;

    mgs_tp_msgs::msg::ControlFeedback m;

    ecu::REGISTER_t reg;
    {
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        reg = state_->ecu.reg;
    }

    // ── 시간축 ──
    // stamp = ECU 취득 기준시각을 Orin 시간축으로 변환한 값 (03 §2.4).
    // 추정기가 아직 수렴 전이면 Orin 수신 시각으로 fallback 하고 stamp_valid=false 로 알린다.
    m.ecu_tick       = reg.sys.realtime_tick;
    m.stamp_valid    = last_clock_valid_;
    m.stamp_quality  = last_quality_;
    m.drift_ppm      = last_drift_ppm_;
    if (last_clock_valid_) {
        const double t = reg.sys.realtime_tick * 1e-4 + last_clock_offset_;
        m.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    } else {
        m.header.stamp = node_->now();
    }
    m.header.frame_id = "ecu";

    // 센서별 취득 지연. **읽지 않은 구간은 NaN** — 0 으로 두면 "지연 없음" 으로 오독된다.
    // 무엇을 읽고 있는지는 활성 프리셋이 정한다 (04 §2.4.2).
    const bool motor_fresh    = Reads(ecu::REG_MOTOR_DATA_OFFSET, 36);
    const bool loadcell_fresh = Reads(ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE);
    const bool imu_fresh      = Reads(ecu::REG_IMU_OFFSET, ecu::REG_IMU_SIZE);
    const bool enc_fresh      = Reads(ecu::REG_ENCODER_OFFSET, ecu::REG_ENCODER_SIZE);

    m.dt_imu      = imu_fresh ? DtSec(reg.imu.delta_tick) : NaNf();
    m.dt_loadcell = loadcell_fresh ? DtSec(reg.loadcell.delta_tick) : NaNf();
    for (size_t i = 0; i < m.dt_encoder.size(); i++)
        m.dt_encoder[i] = enc_fresh ? DtSec(reg.encoder.delta_tick[i]) : NaNf();
    for (int i = 0; i < 4; i++) {
        m.dt_motor[i]     = motor_fresh ? DtSec(reg.motor_data.delta_tick[i])     : NaNf();
        m.dt_motor_cmd[i] = motor_fresh ? DtSec(reg.motor_data.cmd_delta_tick[i]) : NaNf();
    }

    // ── 진단 ──
    // **활성 읽기 프리셋이 SYS 블록({16,17})을 읽을 때만 실값이다** (04 §2.4.2).
    // 04 §2.3 이후로 **전 프리셋이 {16,17} 을 읽으므로 실값이 정상이다.** 종전 control
    // 프리셋은 {27,5} 로 시작해 이 구간을 건너뛰었고, 그래서 엔코더 버스가 100% degraded 인
    // 것을 몇 주 동안 아무도 못 봤다 (06 §10 B1). 그 사고가 §2.3 을 만든 이유다.
    // 미판독일 때 0 이 아니라 kUnread 로 두는 이유: 0 은 "고장 없음" 이라는 유효한 관측값이다.
    m.lc.fill(kUnread);
    m.hs.fill(kUnread);
    m.degraded_cnt.fill(kUnread);
    if (Reads(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE)) {
        m.hw_error = reg.sys.hw_error;
        m.hw_fatal = reg.sys.hw_fatal;
        m.hw_reset = reg.sys.hw_reset;
        for (size_t i = 0; i < m.degraded_cnt.size(); i++)
            m.degraded_cnt[i] = reg.sys.degraded_cnt[i];
    } else {
        m.hw_error = kUnread;
        m.hw_fatal = kUnread;
        m.hw_reset = kUnread;
    }

    // ── 상태 ──
    const ControlWrite_t w = control_->SelectWrite();
    const auto st = control_->State();
    m.ecu_state     = reg.sys.sys_state;
    m.control_state = static_cast<uint8_t>(st);
    // write_source: 0=NONE 1=CMD_VEL 2=PROFILE 3=STREAM
    m.write_source  = (st == ControlState::RUNNING) ? 2u
                    : (st == ControlState::STREAM)  ? 3u : 0u;
    m.auto_mode     = control_->AutoMode();
    m.motor_mask    = reg.cmd_system.motor_mask;
    m.goal_id       = w.goal_id;
    m.profile_time  = w.profile_time;
    m.segment_index = w.segment_index;

    // ── 명령·피드백 ──
    // `cmd` 는 **auto_mode 가 정한 write 범위의 값**이어야 한다 (03: 단위 = A / RPM / deg).
    // cmd_current 를 고정으로 실으면 POSITION·VELOCITY 에서 피드백이 거짓말을 한다 —
    // 명령이 나가고 있는데 0 으로 보인다.
    //
    // ⚠ **이 필드의 의미는 프리셋에 달려 있다** (09 §1.2 ②, 2026-08-04). 소스는 `cmd_motor`
    //   섀도인데, 섀도는 **동시에 write 버퍼**다 (`RdControl::PrepareWrite`).
    //
    //     control      — `{164,16}` read-back 이 없다 → **브리지가 마지막으로 보낸 값**
    //     control_test — `{164,16}` 을 읽고 브리지는 아무것도 안 쓴다 (auto_mode:none)
    //                    → **ECU 실값**. 견인 실험에서 STM RC 램프가 만든 전류 명령이다
    //     diag         — 모터 cmd 구간을 안 읽는다 → 마지막으로 보낸 값
    //
    //   같은 필드가 프리셋에 따라 다른 것을 뜻하는 것이 09 §1.2 결정의 대가다. 분석할 때
    //   `read_preset` 을 같이 봐야 한다 (bag 의 `node_params` 에 남는다).
    const uint8_t am = m.auto_mode;
    for (int i = 0; i < 4; i++) {
        m.cmd[i]         = (am == ecu::AUTO_MODE_POSITION) ? reg.cmd_motor.cmd_position[i]
                         : (am == ecu::AUTO_MODE_VELOCITY) ? reg.cmd_motor.cmd_velocity[i]
                                                           : reg.cmd_motor.cmd_current[i];
        // 프리셋이 모터 블록을 안 읽으면 섀도가 갱신되지 않는다 — **낡은 값을 신선한 값처럼
        // 내보내지 않는다.** 0 이 아니라 NaN 인 이유: 0 은 "정지" 라는 유효한 관측값이다.
        m.fb_current[i]  = motor_fresh ? reg.motor_data.current[i]  * 0.01f : NaNf();
        m.fb_velocity[i] = motor_fresh ? reg.motor_data.velocity[i] * 10.0f : NaNf();
        m.fb_position[i] = motor_fresh ? reg.motor_data.position[i] * 0.1f  : NaNf();
    }
    // ⚠ 09 §1.3 — 종전에는 `loadcell_fresh` 를 계산해 놓고 `dt_loadcell` 에만 쓰고, 값은
    //   게이트 없이 섀도에서 꺼냈다. control 이 로드셀을 읽던 동안에는 안 드러났지만
    //   09 §1.1 에서 `{42,6}` 을 빼면서 **낡은 값이 신선한 값처럼 나가는 상태**가 됐다.
    //   int32 라 NaN 을 못 쓰므로 `kUnreadI32` 를 쓴다 (0 은 "하중 없음" 이라 안 된다).
    m.loadcell_raw[0] = loadcell_fresh ? reg.loadcell.avg[0] : kUnreadI32;
    m.loadcell_raw[1] = loadcell_fresh ? reg.loadcell.avg[1] : kUnreadI32;

    // ── IMU (48:22) ──
    // 프레임 대표 delta_tick 1개라 채널별 판정이 없다 — stale 이면 6축 전부 NaN 이다.
    // 순서는 메시지 정의(x,y,z,w) 이고 레지스터 순서(z,y,x,w)와 **다르다** — 03 §3.1 이
    // 그렇게 정했다. 여기서 뒤집는 것이 유일한 변환 지점이므로 인덱스를 명시한다.
    const bool imu_live = imu_fresh && reg.imu.delta_tick != ecu::DELTA_STALE;
    m.imu_quat[0] = imu_live ? reg.imu.quat_x * kQuatScale : NaNf();
    m.imu_quat[1] = imu_live ? reg.imu.quat_y * kQuatScale : NaNf();
    m.imu_quat[2] = imu_live ? reg.imu.quat_z * kQuatScale : NaNf();
    m.imu_quat[3] = imu_live ? reg.imu.quat_w * kQuatScale : NaNf();
    m.imu_gyro[0] = imu_live ? reg.imu.gyro_x * kGyroToRads : NaNf();
    m.imu_gyro[1] = imu_live ? reg.imu.gyro_y * kGyroToRads : NaNf();
    m.imu_gyro[2] = imu_live ? reg.imu.gyro_z * kGyroToRads : NaNf();
    m.imu_acc[0]  = imu_live ? reg.imu.acc_x * kAccToMs2 : NaNf();
    m.imu_acc[1]  = imu_live ? reg.imu.acc_y * kAccToMs2 : NaNf();
    m.imu_acc[2]  = imu_live ? reg.imu.acc_z * kAccToMs2 : NaNf();

    // ── 링크 각 (엔코더 70:16) ──
    // ⚠ **채널별로 판정한다.** AS5600 5채널은 I2C MUX 로 순차 취득하므로 일부만 죽는 것이
    //   정상적인 고장 양상이고, 실제로 2026-07-29 진단에서 ch0 만 살아 있었다 (06 §10 B1).
    //   그때 raw 값은 ch1~4 도 12bit 범위 안의 **그럴듯한 잔값**이었다 — 값의 그럴듯함으로
    //   판정하면 죽은 채널이 정상 각도로 나간다. 기준은 `delta_tick` 하나뿐이다.
    for (size_t i = 0; i < m.link_angle.size(); i++) {
        const bool ch_live = enc_fresh && reg.encoder.delta_tick[i] != ecu::DELTA_STALE;
        m.link_angle[i] = ch_live ? reg.encoder.encoder[i] * kEncToDeg : NaNf();
    }

    m.rw_err             = rw_err_.load();
    m.drop_cnt           = static_cast<uint32_t>(tele_q_.DropCount());
    m.irregular_tick_cnt = irregular_tick_cnt_.load();
    m.late_tick_cnt      = late_tick_cnt_.load();

    // ⚠ 메시지 **구성은 여기(스케줄 스레드)에서 끝낸다.** 발행 시점에 shadow 를 읽으면
    //   tick 시점의 값이 아닌 것이 나간다. 넘기는 것은 publish() 뿐이다 (02 §6.3).
    TeleItem_t item;
    item.has_feedback = true;
    item.feedback     = m;
    tele_q_.TryPush(item);   // 가득 차면 드롭 + drop_cnt++ — 절대 블록하지 않는다
}

// ===== 200Hz — 시계 추정기 갱신. 원자료(CommDiag) 발행은 5Hz 로 솎는다 =====
void RdTelemetry::OnTxn(const TxnTiming_t& txn) {
    uint32_t ecu_tick;
    uint8_t  proc_raw;
    {
        std::lock_guard<std::mutex> lock(state_->state_mutex);
        const auto& reg = state_->ecu.reg;
        ecu_tick = reg.sys.realtime_tick;
        proc_raw = reg.sys.rs485_proc_delta;   // 2026-07-27 diag(228) → sys(32) 이동
    }
    const double proc_prev = (proc_raw == ecu::DELTA_STALE) ? -1.0 : proc_raw * 1e-4;

    // **추정기는 항상 돈다** — comm_diag 가 꺼져 있어도 ControlFeedback.header.stamp 가
    // 이 결과를 쓴다. 발행만 가리는 것이지 계산을 가리는 것이 아니다.
    auto est = clock_sync_.Update(txn, ecu_tick, proc_prev);
    last_clock_offset_ = est.offset;
    last_clock_valid_  = est.valid;
    last_rtt_          = txn.t_resp - txn.t_req;
    last_quality_      = static_cast<float>(est.quality);
    last_drift_ppm_    = static_cast<float>(est.drift_ppm);
    clock_sample_cnt_++;

    if (est.valid && !clock_sync_announced_) {
        clock_sync_announced_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "시계 동기 수렴: offset %.6f s, drift %+.2f ppm (ECU tick -> ROS time)",
            est.offset, est.drift_ppm);
    }

    if (!pub_comm_diag_) return;
    if (++comm_diag_tick_ < kCommDiagDecim) return;
    comm_diag_tick_ = 0;

    mgs_tp_msgs::msg::CommDiag d;
    d.header.stamp    = node_->now();
    d.header.frame_id = "ecu";
    d.t_req        = txn.t_req;
    d.t_resp       = txn.t_resp;
    d.rtt          = static_cast<float>(last_rtt_);
    d.wire_up      = static_cast<float>(est.wire_up);
    d.wire_down    = static_cast<float>(est.wire_down);
    d.proc_delta   = static_cast<float>(proc_prev);
    d.residual     = static_cast<float>(last_rtt_ - est.wire_up - est.wire_down -
                                        (proc_prev < 0 ? 0.0 : proc_prev));
    d.clock_offset = est.offset;
    d.tick_period  = static_cast<float>(1e-4 * (1.0 + est.drift_ppm * 1e-6));
    d.drift_ppm    = last_drift_ppm_;
    d.quality      = last_quality_;
    d.sample_cnt   = clock_sample_cnt_;
    d.drop_cnt     = static_cast<uint32_t>(tele_q_.DropCount());

    TeleItem_t item;
    item.has_diag = true;
    item.diag     = d;
    tele_q_.TryPush(item);
}

// 발행 스레드 — 여기서 DDS 가 블록돼도 200Hz tick 은 안 밀린다 (02 §6.3).
void RdTelemetry::TelemetryLoop() {
    TeleItem_t item;
    // ⚠ rclcpp::ok() 를 반드시 같이 본다. executor 가 관리하지 않는 생 스레드라,
    //   SIGTERM 으로 rclcpp::shutdown() 이 먼저 돌면 파괴 중인 컨텍스트에 publish 하게 된다
    //   (5단계에서 실기 SIGSEGV 로 드러났다 — 06 §3 참조).
    while (tele_running_.load() && rclcpp::ok()) {
        bool any = false;
        while (tele_q_.Pop(&item)) {
            any = true;
            if (item.has_diag     && pub_comm_diag_)        pub_comm_diag_->publish(item.diag);
            if (item.has_feedback && pub_control_feedback_) pub_control_feedback_->publish(item.feedback);
        }
        if (!any) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // 종료 시 남은 것을 비운다. 컨텍스트가 죽었으면 버린다 — 크래시보다 낫다.
    while (rclcpp::ok() && tele_q_.Pop(&item)) {
        if (item.has_diag     && pub_comm_diag_)        pub_comm_diag_->publish(item.diag);
        if (item.has_feedback && pub_control_feedback_) pub_control_feedback_->publish(item.feedback);
    }
}

}  // namespace orin_bridge
