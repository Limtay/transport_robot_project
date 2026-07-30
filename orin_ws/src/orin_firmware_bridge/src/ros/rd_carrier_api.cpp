#include "orin_firmware_bridge/ros/rd_carrier_api.hpp"

#include <cmath>

namespace orin_bridge {

RdCarrierApi::RdCarrierApi(rclcpp::Node* node, RobotState_t* state, const BridgeConfig& cfg)
    : node_(node), state_(state), cfg_(cfg) {
    sub_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/carrier_cmd_vel", 10,
        std::bind(&RdCarrierApi::CallbackCmdVel, this, std::placeholders::_1));
    sub_jeongae_ = node_->create_subscription<mgs01_base_msgs::msg::JeonGae>(
        "/jeongae", 10,
        std::bind(&RdCarrierApi::CallbackJeongae, this, std::placeholders::_1));

    guard_enable_.store(cfg_.cmd_vel_guard_enable_default);
    inputs_.last_cmd_time     = node_->now();
    inputs_.last_topic_time   = node_->now();
    inputs_.last_nonzero_time = node_->now();
}

void RdCarrierApi::AttachCommand(RdCommand* command) {
    command_ = command;

    srv_command_ = node_->create_service<mgs_tp_msgs::srv::CommandSet>(
        "/carrier/command_set",
        std::bind(&RdCarrierApi::CallbackCommandSet, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_jeongae_lock_ = node_->create_service<std_srvs::srv::SetBool>(
        "/carrier/jeongae_lock",
        std::bind(&RdCarrierApi::CallbackJeongaeLock, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(),
                "Command services ready: /carrier/command_set, /carrier/jeongae_lock");
}

void RdCarrierApi::CallbackCommandSet(
    const std::shared_ptr<mgs_tp_msgs::srv::CommandSet::Request> req,
    std::shared_ptr<mgs_tp_msgs::srv::CommandSet::Response> res) {
    if (!command_) {
        res->accepted = false;
        res->message  = "command manager 미연결";
        return;
    }
    // B6 — 게이트. **판정은 카탈로그의 순수 함수가 한다** (rd_command_catalog.hpp) —
    // 여기서 조건을 다시 적으면 표와 갈라진다. 이 층이 하는 일은 컨텍스트를 모으는 것뿐이다.
    //
    // **게이트를 CLI 에만 두지 않는 이유**: 클라이언트에만 있는 가드는 가드가 아니다.
    // 서비스는 누구나 부를 수 있다.
    //
    // RESET(슬롯 비우기)은 감속 방향이라 어느 모드에서든 허용한다 — 잘못 넣은 명령을
    // 빼는 수단까지 잠그면 게이트의 목적과 정반대가 된다 (arm off 와 같은 취지, 01 §6.2).
    if (req->action != mgs_tp_msgs::srv::CommandSet::Request::ACTION_RESET) {
        std::string why_stop;
        const bool safe = control_ ? control_->SafeStop(&why_stop) : false;
        const uint8_t arg0 = req->args.empty() ? 0 : req->args[0];
        const auto g = cmdcat::Gate(req->cmd, cfg_.IsManual(), safe, arg0);
        if (!g.ok) {
            res->accepted = false;
            res->message  = std::string("거부: ") + g.why +
                            " (cmd=" + cmdcat::CmdName(req->cmd) +
                            ", bridge_mode=" + BridgeModeName(cfg_.bridge_mode) + ")";
            if (!safe && !why_stop.empty()) res->message += " — " + why_stop;
            return;
        }
    }
    CommandRequest_t creq;
    creq.slot       = req->slot;
    creq.action     = req->action;
    creq.target_id  = req->target_id;
    creq.cmd        = req->cmd;
    creq.args       = req->args;
    creq.start_addr = req->start_addr;
    creq.data_len   = req->data_len;
    creq.data       = req->data;
    creq.duration   = req->duration;
    res->accepted = command_->HandleRequest(creq, &res->message);
}

void RdCarrierApi::CallbackJeongaeLock(
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

bool RdCarrierApi::ShouldSkipCmdWrite() {
    if (!guard_enable_.load()) return false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto now = node_->now();
    // (1) jeongae 포함 명령 토픽이 100ms 내 들어오지 않음 → skip
    if ((now - inputs_.last_topic_time).seconds() > cfg_.cmd_vel_topic_timeout) return true;
    // (2) cmd_vel 이 3초 이상 0 에 수렴 → skip
    if ((now - inputs_.last_nonzero_time).seconds() > cfg_.cmd_vel_zero_timeout) return true;
    return false;
}

void RdCarrierApi::GetRosInputs() {
    // Step 1: inputs_ 복사 (data_mutex_ 짧게 유지)
    float lin, ang;
    bool  jeongae;
    bool  watchdog_triggered;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto duration       = node_->now() - inputs_.last_cmd_time;
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
        // ⚠ 종전 여기서 `state_->dpc.reg.cmd.cmd_jeongae = jeongae;` 를 했다. 그 필드는
        //   레지스터 미확정 시절의 **골격 스텁**이었고, 2026-07-30 실제 맵을 반영하면서
        //   사라졌다. 실제 맵에 "jeongae" 라는 필드는 없다 — 전개는 `sys_state_target`(127)
        //   으로 FSM 을 몰고, 잠금은 `servo_cmd`(125) 로 한다.
        //
        //   그 write 는 **어차피 아무 데도 가지 않았다**: 프레임에 DPC WRITE 슬롯이 없고
        //   (`DpcRd` 만 있다) `enable_dpc_read_` 도 false 였다. 여기서 되살리면 어느 필드에
        //   매핑할지가 곧 액추에이터 동작을 정하는 결정이 되므로, **확인 전까지 쓰지 않는다.**
        //   `/jeongae` 토픽의 실제 경로는 `RdSequence`(전개 FSM) 다 — 그쪽이 정본이다.
        (void)jeongae;
    }
}

// 10Hz — 배터리/연결/시스템/에러/링키지/상태 (저속 변화 데이터)
void RdCarrierApi::CallbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    inputs_.linear_x        = msg->linear.x;
    inputs_.angular_z       = msg->angular.z;
    auto now                = node_->now();
    inputs_.last_cmd_time   = now;
    inputs_.last_topic_time = now;
    // 안전장치 (2): 0 이 아닌 명령이 들어온 마지막 시각 기록
    if (std::abs(msg->linear.x) > kZeroEps || std::abs(msg->angular.z) > kZeroEps) {
        inputs_.last_nonzero_time = now;
    }
}

void RdCarrierApi::CallbackJeongae(const mgs01_base_msgs::msg::JeonGae::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        inputs_.jeongae_open    = msg->open;
        inputs_.last_topic_time = node_->now();  // 안전장치 (1): jeongae 도 명령 토픽에 포함
    }
    // jeongae 자동 전개 시퀀스 트리거 (§3) — lock 검사는 FSM 쪽에서 수행
    if (msg->open && command_) {
        command_->TriggerJeongae();
    }
}

// out-of-span 구현은 rd_oos.cpp (L2) 로 이동 — 02 A2

// §2.6 제약 2 — auto_mode 전환. **순서가 안전의 핵심**이다.
//   1→2 (범위 확장): ① shadow 소독 → ② WRITE+검증 → ③ 범위 확장
//      소독을 먼저 하지 않고 범위를 넓히면, 그동안 read 로 유입됐거나 과거에 남은
//      shadow 의 ctr_mode/pos/vel 잔재가 그대로 ECU 로 나간다 (128:52 를 쓰기 시작하므로).
//   2→1 (범위 축소): ① WRITE+검증 → ② 범위 축소
//      축소는 쓰는 바이트가 줄어드는 방향이라 잔재가 나갈 창이 없다.

}  // namespace orin_bridge
