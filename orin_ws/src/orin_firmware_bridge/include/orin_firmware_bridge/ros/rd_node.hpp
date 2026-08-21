#ifndef ORIN_FIRMWARE_BRIDGE__RD_NODE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_NODE_HPP_

// ⚠ 여기에 메시지·서비스·액션 헤더를 **추가하지 않는다.** 이 노드는 조립과 수명만
//   소유하고, 토픽/서비스/액션은 전부 L3 3개(RdTelemetry·RdCarrierApi·RdControlApi)가
//   갖는다. 종전 여기에 twist·imu·set_bool·command_set 등 13개가 있었는데, 그 멤버들이
//   L3 로 옮겨간 뒤에도 헤더만 남아 "이 노드가 아직 그것들을 다룬다" 처럼 보였다.
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "orin_firmware_bridge/ros/rd_carrier_api.hpp"
#include "orin_firmware_bridge/policy/rd_command.hpp"
#include "orin_firmware_bridge/rd_config.hpp"
#include "orin_firmware_bridge/ros/rd_control_api.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/policy/rd_oos.hpp"
#include "orin_firmware_bridge/policy/rd_profile_player.hpp"
#include "orin_firmware_bridge/ros/rd_telemetry.hpp"
#include "orin_firmware_bridge/sched/rd_telemetry_sink.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"

namespace orin_bridge {

class RdNode : public rclcpp::Node, public ITelemetrySink {
public:
    // opts 는 테스트가 파라미터를 주입하기 위한 통로 — 기본값은 기존 동작 그대로.
    explicit RdNode(RobotState_t* robot_state,
                      const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
    virtual ~RdNode();

    void Start();
    // 커맨드 매니저 연결 + 서비스 서버 오픈 (/carrier/command_set, /carrier/jeongae_lock)
    void AttachCommand(RdCommand* command) {
        command_ = command;
        carrier_api_->AttachCommand(command);
        // B6 게이트가 safe_stop 을 물어야 한다. 여기서 같이 꽂아 두면 "서비스는 열렸는데
        // 게이트가 판정을 못 해 전부 거부" 하는 상태가 생기지 않는다.
        carrier_api_->AttachControl(&control_);
        // GET_STATUS 의 slots[] 출처 (04 §4). 여기서 함께 꽂아야 "커맨드는 붙었는데
        // 상태에는 안 보이는" 어긋남이 생기지 않는다.
        control_api_->SetSlotSnapshot([command]{ return command->SlotSnapshot(); });
        // 09 §5.3 ④ (U12) — 같은 자리에서 시퀀스도 꽂는다. 빠뜨리면 웹의 배타 잠금이
        // 영원히 "IDLE" 로 보여 **전개 중에도 수동 127 입력이 열려 있게** 된다.
        control_api_->SetSeqSnapshot([command]{ return command->Sequence().SnapshotState(); });
        // 2026-08-12 — CAMERA_ACTION 이 쓰는 카메라 서비스 클라이언트는 RdCarrierApi 가
        // 들고 있다(rclcpp 접근이 필요해서). RdCommand(L2)는 콜백으로만 받는다.
        command->SetCameraHost(
            [this]{ return carrier_api_->TriggerCameraCapture(); },
            [this](bool* ok){ return carrier_api_->CameraCaptureDone(ok); });
        // 2026-08-07 — 0 수렴 스킵은 `RdCarrierApi` 가 소유한다. 상태만 끌어온다.
        control_api_->SetZeroSkipGetter([this]{ return carrier_api_->ZeroSkipEnabled(); });
        // OP_GET_REGISTERS 의 신선도 판정 (07 §2 Tab3) — 여기서 같이 꽂는다.
        control_api_->SetLastRead([command]{ return command->LastReadSnapshot(); });
        control_api_->SetReadPresetPtr([this]{ return telemetry_->ReadPresetPtr(); });
    }
    void GetRosInputs() { carrier_api_->GetRosInputs(); }
    void PublishMotorFeedback() { telemetry_->PublishMotorFeedback(); }  // 100Hz
    void PublishStatus()        { telemetry_->PublishStatus(); }         // 10Hz

    // cmd_vel 안전장치 (Code_modify.md: 토픽 100ms 미수신 / 0 수렴 3초 → 50Hz WRITE skip)
    // cmd_vel_guard_enable 파라미터로 on/off (런타임 변경 가능)
    bool ShouldSkipCmdWrite() { return carrier_api_->ShouldSkipCmdWrite(); }

    // === 기동 설정 — 파싱은 생성자에서 끝나고 이후 불변 (02 §5.4) ===
    // A1-a: 하위 계층은 이 묶음을 const& 로 받는다 — 노드를 거꾸로 참조하지 않는다.
    const BridgeConfig& Config() const { return config_; }
    // INIT 이 `active_motor_mask` 를 채택할 때만 필요하다 (09 §4.3, rd_schedule.hpp cfg_ 주석).
    // 나머지 소비자는 위 const 접근자를 쓴다 — 기동 후 불변이라는 계약(02 §5.4)은 유지된다.
    BridgeConfig& ConfigMut() { return config_; }

    // ── ITelemetrySink (02 A1-b) — 구현은 전부 RdTelemetry 로 위임한다 (02 A2 분할).
    //    노드가 sink 를 계속 구현하는 이유: 스케줄러는 인터페이스 하나만 알면 되고,
    //    조립(누가 실제로 발행하는가)은 rd_node 의 관심사이기 때문이다.
    void OnTxn(const TxnTiming_t& txn) override        { telemetry_->OnTxn(txn); }
    void OnFeedback() override                         { telemetry_->OnFeedback(); }
    void OnRwErr(uint8_t rw_err) override              { telemetry_->OnRwErr(rw_err); }
    void OnWriteErrTotal(uint64_t total) override      { telemetry_->OnWriteErrTotal(total); }
    void OnIrregularTick() override                    { telemetry_->OnIrregularTick(); }
    void OnLateTick() override                         { telemetry_->OnLateTick(); }
    void SetReadPreset(const ReadPreset* p) override   { telemetry_->SetReadPreset(p); }

    // 프리셋 교체 동작은 L1(RdSchedule)이 소유하는데 스케줄러는 이 노드보다 **나중에**
    // 생성된다 (main.cpp). 그래서 AttachCommand 와 같은 방식으로 나중에 꽂는다.
    void AttachReadPresetSetter(std::function<bool(uint8_t, std::string*)> fn);

    // === 제어 FSM (01 §6) ===
    // control 모드의 write 소스는 이 FSM 이 **단독** 결정 — PrepareWrite 가 소비한다.
    // service/action 서버와 INIT 플로우도 이 객체를 통해서만 상태를 바꾼다.
    RdControl& Control() { return control_; }

    // === §2.6 auto_mode ===
    // 기동 파라미터 (기본 1=CURRENT). 0=KINEMATIC 은 기동 거부 — ECU 가 ctr_mode 를
    // VELOCITY 로 덮어써 전류 실험이 성립하지 않는다.
    uint8_t AutoModeParam() const { return config_.auto_mode_param; }
    bool    AutoModeValid() const { return config_.auto_mode_valid; }
    // 현재 선택된 값 — write 범위가 여기서 파생된다 (atomic 읽기).
    uint8_t AutoMode() const { return control_.AutoMode(); }

    // === §3.2 프로파일 재생 (run_profile action) ===
    // 스케줄 루프가 매 tick(200Hz) PrepareControlCommand 직전에 호출 — 사전 샘플링된
    // 배열에서 한 tick 을 꺼내 FSM 에 넣는다. 배열 인덱싱뿐, 연산 없음 (§4.3-5).
    RdProfilePlayer& Player() { return player_; }

    // 스케줄러가 INIT 을 끝낸 뒤 부른다. `with_profiles` 는 **명령을 쓰는 구성인가** —
    // false 면 조작 입구(config)만 열고 재생 서버는 열지 않는다 (rd_control_api.hpp 참조).
    void StartServers(bool with_profiles) {
        if (with_profiles) control_api_->StartProfileServer();   // config 도 같이 연다
        else               control_api_->StartConfigService();
    }
    // 테스트·기존 호출부 호환 — 둘 다 여는 쪽.
    void StartProfileServer() { control_api_->StartProfileServer(); }

    // === §2 out-of-span 단발 write (config service) ===
    // RW write 범위(128~179) 밖 레지스터(motor_mask 192 / mode 190)는 패킷 삽입 대신
    // **RW 1 tick 을 일반 WRITE/READ 패킷으로 대체**해 처리한다. IDLE 은 write 값이 0A 라
    // 1~2 tick 대체가 무해하다는 것이 근거 (§2 표).
    //
    // 제약 (노트북 회신 §6.2):
    //   1) IDLE 에서만, 동시 1건 — in-flight 중 새 요청은 즉시 거부 (큐잉 없음)
    //   2) 대체된 tick 은 read 스냅샷이 없다 → 그 tick 의 피드백 발행 skip (보간 금지)
    //   3) 대체 WRITE 실패 시 재시도 없이 다음 tick 정상 RW 복귀 (재시도는 호출자 몫)
    //   4) 검증은 WRITE tick + READ tick, 최대 2 tick 소모
    // 구현은 L2 `RdOos` 로 이동 (02 A2). 스케줄러는 이 객체를 **직접** 잡는다 —
    // 노드를 거쳐 갈 이유가 없었고, 그 경유가 곧 A1-(c) 역방향 의존이었다.
    RdOos& Oos() { return oos_; }

private:
    // 기동 1회 확정 후 불변 (02 §5.4). 파싱은 생성자에서 끝난다.
    BridgeConfig config_;

    RobotState_t* state_;
    RdCommand*    command_ = nullptr;

    // ⚠ **선언 순서가 곧 소멸 순서의 역순**이다. 아래 L3 3개가 이 L2 셋을 생 포인터로
    //   잡고 있으므로 L2 가 먼저 선언돼야 L3 가 먼저 사라진다. 순서를 바꾸면
    //   종료 시 이미 죽은 객체를 가리키게 된다 — 컴파일러는 알려주지 않는다.
    RdControl       control_;   // 01 §6 제어 FSM — 자체 mutex 보유
    RdProfilePlayer player_;    // 재생 상태 소유 (02 A2) — 락도 이쪽
    RdOos           oos_;       // out-of-span 단발 WRITE (02 A2 — 상태·검증 규칙 전부 이쪽)

    // 02 A2 — L3 3분할. 노드는 **조립과 수명만** 소유한다.
    // 구독·서비스·액션·발행자는 전부 이 셋이 갖는다 (여기 두면 소유가 두 곳으로 갈라진다).
    std::unique_ptr<RdTelemetry>  telemetry_;
    std::unique_ptr<RdCarrierApi> carrier_api_;
    std::unique_ptr<RdControlApi> control_api_;

    // ROS 콜백 전용 스레드 — 200Hz UART 루프(main 스레드)와 완전히 분리된다.
    std::thread spin_thread_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::TimerBase::SharedPtr publish_timer_fast_;  // 100Hz: 입력 갱신 + 모터 피드백
    rclcpp::TimerBase::SharedPtr publish_timer_slow_;  // 10Hz: 배터리·연결·상태·에러
};

} // namespace orin_bridge

#endif
