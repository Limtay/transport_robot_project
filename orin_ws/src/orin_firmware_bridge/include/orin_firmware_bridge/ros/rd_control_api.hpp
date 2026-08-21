#ifndef ORIN_FIRMWARE_BRIDGE__RD_CONTROL_API_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_CONTROL_API_HPP_

// RdControlApi — control 계약의 입출구 (redesign/02 A2, §5.1)
//
//   action  /carrier/control/run_profile
//   srv     /carrier/control/config
//   (구 이름은 `/carrier/testbed/*` 였다 — 01 §8.3 에서 개명됐다.)
//
// **상태 없음** (02 §5.1): 서버 핸들만 갖고, 실제 상태는 전부 L2 가 갖는다
// (FSM=`RdControl`, 재생=`RdProfilePlayer`, 단발 write=`RdOos`).
// 이 규칙이 지켜지면 웹(`control_web`)이 붙을 때 같은 L2 를 그대로 재사용한다.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mgs_tp_msgs/action/run_profile.hpp>
#include <mgs_tp_msgs/msg/cmd_motor.hpp>
#include <mgs_tp_msgs/srv/control_config.hpp>

#include "orin_firmware_bridge/policy/rd_command.hpp"
#include "orin_firmware_bridge/rd_config.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/policy/rd_oos.hpp"
#include "orin_firmware_bridge/policy/rd_profile_player.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"

namespace orin_bridge {

// 브리지만 아는 런 카운터 (05 §5.3). CLI 는 이 값들을 만들어낼 수 없으므로
// action Result 로 실어 보내야 하고, L3(RdNode)가 스냅샷을 주입한다.
// **L2 에서 RdTelemetry 를 직접 참조하지 않기 위한 경계**다 — 종전 write_err_total
// 콜백 하나가 하던 일을 그대로 넓힌 것이다.
struct RunCounters_t {
    uint64_t write_err_total    = 0;
    uint32_t irregular_tick_cnt = 0;
    uint32_t late_tick_cnt      = 0;
    uint64_t drop_cnt           = 0;
    bool     clock_converged    = false;
    float    drift_ppm          = 0.0f;
    // GET_STATUS(04 §4) 가 추가로 요구하는 계측값. **한 스냅샷으로 모아 오는 것이 중요하다** —
    // 필드마다 따로 읽으면 tick 경계를 가로질러 서로 다른 시점의 값이 섞인다.
    float       stamp_quality_ms = 0.0f;
    double      rtt_ms           = 0.0;
    uint8_t     rw_err           = 0;
    const char* read_preset_name = "control";
};

class RdControlApi {
public:
    using RunProfile = mgs_tp_msgs::action::RunProfile;
    using GoalHandle = rclcpp_action::ServerGoalHandle<RunProfile>;

    // ⚠ cfg 가 **비 const** 인 이유: config service 의 SET_ACTIVE_MOTORS 가
    //   `active_motor_mask` 를 런타임에 바꾼다. BridgeConfig 의 "기동 후 불변" 약속에서
    //   유일하게 벗어나는 필드이며, 그 사실을 타입으로 드러내 둔다 (rd_config.hpp 참조).
    RdControlApi(rclcpp::Node* node, RobotState_t* state, BridgeConfig& cfg,
                 RdControl* control, RdProfilePlayer* player, RdOos* oos,
                 std::function<RunCounters_t()> counters,
                 std::function<bool(uint8_t, std::string*)> set_read_preset = nullptr);

    void SetReadPresetSetter(std::function<bool(uint8_t, std::string*)> fn) {
        set_read_preset_ = std::move(fn);
    }
    // OP_GET_REGISTERS 의 신선도 판정에 필요한 둘 (07 §2 Tab3).
    // 없으면 `fresh` 가 비고, UI 는 전 구간을 회색으로 그린다 — **모르면 회색이 안전측이다**
    // (06 §4.11 에서 "모르면 관대하게" 가 낡은 값을 신선하게 보이게 만든 적이 있다).
    void SetReadPresetPtr(std::function<const ReadPreset*()> fn) { preset_ptr_ = std::move(fn); }
    void SetLastRead(std::function<RdCommand::LastRead_t()> fn) { last_read_ = std::move(fn); }

    void SetSlotSnapshot(std::function<std::vector<RdCommand::SlotView_t>()> fn) {
        slot_snapshot_ = std::move(fn);
    }
    // 09 §5.3 ④ (U12) — jeongae 시퀀스 단계. 슬롯과 같은 이유로 스냅샷만 받는다.
    void SetSeqSnapshot(std::function<RdSequence::Snapshot_t()> fn) {
        seq_snapshot_ = std::move(fn);
    }
    // 2026-08-07 — cmd_vel 0 수렴 스킵 상태. 소유자는 `RdCarrierApi` 라 조회만 받는다.
    void SetZeroSkipGetter(std::function<bool()> fn) { zero_skip_ = std::move(fn); }

    // ── 두 서버를 **따로** 연다 (2026-07-30 실기에서 드러난 결함) ──
    //
    // 종전에는 하나의 `StartProfileServer()` 가 둘을 같이 만들었고, 그것을 부르는 곳이
    // "INIT 검증을 통과한 control 구성" 하나뿐이었다. 그래서 `auto_mode: none`
    // (= 견인 실험, `./run.sh traction`)에서는 **config 서비스까지 통째로 안 열렸다** —
    // `cli status` 가 아예 안 되고, 웹 Tab3(레지스터 맵)이 죽고, 프리셋 교체도 못 했다.
    //
    // 조작 입구(config)와 재생 서버(action)는 열릴 조건이 다르다:
    //   · config  : control 이면 **항상** — 상태를 못 읽으면 조작자가 눈을 잃는다
    //   · profile : **명령을 쓰는 구성에서만** — write 가 없는데 goal 을 수락하면
    //               "재생은 됐다는데 아무것도 안 움직인다" 가 된다 (거부보다 나쁘다)
    void StartConfigService();
    // INIT 검증 통과 후에만 연다 — 통과 전에 goal 이 들어오면 안 되기 때문.
    void StartProfileServer();

private:
    rclcpp::Node*       node_;
    RobotState_t*       state_;
    BridgeConfig&       cfg_;
    RdControl*          control_;
    RdProfilePlayer*    player_;
    RdOos*              oos_;
    std::function<RunCounters_t()> counters_;
    // 04 §2.4.2 프리셋 교체 — L1(RdSchedule)이 소유한 동작이라 콜백으로 받는다.
    std::function<bool(uint8_t, std::string*)> set_read_preset_;
    // GET_STATUS 의 slots[] — RdCommand 는 이 클래스가 모르는 객체라 스냅샷만 받는다.
    std::function<std::vector<RdCommand::SlotView_t>()> slot_snapshot_;
    std::function<RdSequence::Snapshot_t()>             seq_snapshot_;
    std::function<bool()>                               zero_skip_;
    std::function<const ReadPreset*()>       preset_ptr_;
    std::function<RdCommand::LastRead_t()>   last_read_;

    rclcpp::Service<mgs_tp_msgs::srv::ControlConfig>::SharedPtr srv_control_config_;
    rclcpp::CallbackGroup::SharedPtr cb_group_config_;   // 검증 대기(최대 50ms) 동안 다른 콜백을 막지 않기 위해 분리
    rclcpp_action::Server<RunProfile>::SharedPtr action_server_;
    // 웹 슬라이더 + MPC 공용 스트림 입구 (03 §5.2). arm=on 일 때만 소비된다.
    rclcpp::Subscription<mgs_tp_msgs::msg::CmdMotor>::SharedPtr sub_cmd_motor_;
    void CallbackCmdMotor(const mgs_tp_msgs::msg::CmdMotor::SharedPtr msg);

    std::mutex data_mutex_;   // ctr_mode 조회 등 짧은 임계구역

    // CmdMotor.ctr_mode(예약 필드)를 채워 보내는 발행자에게 1회만 경고 (CmdMotor.msg 참조).
    // 200Hz 로 들어오므로 반복 경고는 로그를 덮는다.
    bool warned_ctr_mode_ignored_ = false;

    void CallbackControlConfig(
        const std::shared_ptr<mgs_tp_msgs::srv::ControlConfig::Request> req,
        std::shared_ptr<mgs_tp_msgs::srv::ControlConfig::Response> res);

    rclcpp_action::GoalResponse   HandleProfileGoal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RunProfile::Goal> goal);
    rclcpp_action::CancelResponse HandleProfileCancel(const std::shared_ptr<GoalHandle> gh);
    void HandleProfileAccepted(const std::shared_ptr<GoalHandle> gh);
    void ExecuteProfile(const std::shared_ptr<GoalHandle> gh);

    bool DoOutOfSpanWrite(uint16_t addr, uint8_t value, std::string* msg) {
        return oos_->Request(addr, value, msg);
    }
    bool DoSetAutoMode(uint8_t mode, std::string* msg);
    bool DoInSpanCtrMode(const std::vector<uint8_t>& motors, uint8_t mode, std::string* msg);
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_CONTROL_API_HPP_
