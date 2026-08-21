#ifndef ORIN_FIRMWARE_BRIDGE__RD_TELEMETRY_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_TELEMETRY_HPP_

// RdTelemetry — shadow·TxnResult → ROS 메시지 **변환·발행 전담** (redesign/02 A2, §5.1)
//
// 7단계에서 발행 계약이 전면 교체됐다 (03 §4):
//   project 47 토픽 → 6   /   control 47+2 → 1 (+디버그 1)
//   진단 26개 → NodeStatus x3 + MotorStatus
//   TestbedFeedback + CommLatency → ControlFeedback (+ CommDiag, 기본 off)
//
// **자체 rclcpp::Node 를 만들지 않는다.** 노드를 쪼개면 `ros2 node list` 와 그래프가
// 바뀌어 외부에서 관측 가능한 변화가 생긴다. 나뉘는 것은 **C++ 책임**이지 ROS 노드가 아니다.
//
// 미판독 표현 (03 §5.3):
//   진단 배열 uint8 → **255(0xFF)**   /   float → **NaN**
//   "안 읽음" 과 "정상(0)" 을 절대 섞지 않는다. 프리셋이 그 구간을 안 읽으면 그렇게 나간다.

#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mgs_tp_msgs/msg/comm_diag.hpp>
#include <mgs_tp_msgs/msg/control_feedback.hpp>
#include <mgs_tp_msgs/msg/motor_status.hpp>
#include <mgs_tp_msgs/msg/node_status.hpp>

#include "orin_firmware_bridge/core/rd_clock_sync.hpp"
#include "orin_firmware_bridge/rd_config.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_read_preset.hpp"
#include "orin_firmware_bridge/policy/rd_control.hpp"
#include "orin_firmware_bridge/sched/rd_txn_queue.hpp"

namespace orin_bridge {

class RdTelemetry {
public:
    RdTelemetry(rclcpp::Node* node, RobotState_t* state, const BridgeConfig& cfg,
                RdControl* control);
    ~RdTelemetry();

    void Start();   // 발행 스레드 기동
    void Stop();

    // ── 스케줄 스레드(200Hz)에서 불린다. 블록 금지 ──
    void OnTxn(const TxnTiming_t& txn);
    void OnFeedback();
    void OnRwErr(uint8_t rw_err)          { rw_err_ = rw_err; }
    void OnWriteErrTotal(uint64_t total)  { write_err_total_ = total; }
    // out-of-span 등으로 정규 RW 가 대체된 tick (03 ControlFeedback.irregular_tick_cnt)
    void OnIrregularTick()                { irregular_tick_cnt_++; }
    // 주기를 넘겨 위상이 리셋된 tick — "늘어난 시간" 의 양 (rd_telemetry_sink.hpp 참조)
    void OnLateTick()                     { late_tick_cnt_++; }

    // ── spin 스레드 타이머에서 불린다 ──
    void PublishMotorFeedback();  // 100Hz — IMU
    void PublishStatus();         // 10Hz  — NodeStatus x3 + MotorStatus + battery

    uint64_t DropCount() const     { return tele_q_.DropCount(); }
    uint64_t WriteErrTotal() const { return write_err_total_.load(); }

    // 05 §5.3 — result.json 이 요구하는 "브리지만 아는 값" 들.
    // CLI 는 이것들을 만들어낼 방법이 없으므로 action Result 로 실어 보낸다.
    uint32_t IrregularTickCount() const { return irregular_tick_cnt_.load(); }
    uint32_t LateTickCount() const      { return late_tick_cnt_.load(); }

    // 04 §2.4.2 — 지금 어떤 구간을 읽고 있는가. 발행 시 **안 읽는 블록은 미판독 센티넬**로
    // 나간다. 프리셋을 바꿔 놓고 이걸 안 알리면 낡은 섀도 값이 신선한 값처럼 발행된다.
    void SetReadPreset(const ReadPreset* p) { read_preset_.store(p, std::memory_order_relaxed); }
    const ReadPreset* ReadPresetPtr() const { return read_preset_.load(std::memory_order_relaxed); }
    bool Reads(uint16_t addr, uint16_t len) const {
        // **널일 때 true 를 돌려주면 안 된다.** 그건 "안 읽는 것까지 읽었다고 주장" 하는 쪽이라,
        // 이 함수가 막으려던 바로 그 일(낡은 섀도를 신선한 값처럼 발행)을 하게 된다.
        // 2026-07-29 실기에서 실제로 그렇게 나왔다 — 기동 직후 hw_error 가 미판독(255)이 아니라
        // **0("정상")** 으로 발행됐다. 실제 값은 16(미연결 엔코더)이었다.
        // 기본값을 control 프리셋으로 두어 첫 tick 부터 사실과 맞춘다.
        return read_preset_.load(std::memory_order_relaxed)->Covers(addr, len);
    }
    // 미수렴이면 header.stamp 가 Orin 수신 시각으로 fallback 한다 —
    // **시간축 정밀도 등급이 다른 런**이라 분석이 섞으면 안 된다 (05 §5.2 B1).
    bool     ClockConverged() const { return last_clock_valid_; }
    float    DriftPpm() const       { return last_drift_ppm_; }
    float    StampQuality() const   { return last_quality_; }   // [s]
    double   LastRtt() const        { return last_rtt_; }       // [s]
    uint8_t  RwErr() const          { return rw_err_.load(); }

    // 03 §5.3 미판독 sentinel
    static constexpr uint8_t kUnread = 0xFF;
    static float NaNf() { return std::numeric_limits<float>::quiet_NaN(); }
    // int32 계열(loadcell_raw)의 미판독. NaN 이 없는 타입이라 별도 값이 필요하다.
    // **0 을 쓸 수 없다** — 로드셀 raw 0 은 "하중 없음" 이라는 유효한 관측값이다.
    // 실측 raw 는 ADC 범위 안이라 INT32_MIN 과 충돌하지 않는다 (09 §1.3).
    static constexpr int32_t kUnreadI32 = std::numeric_limits<int32_t>::min();

private:
    rclcpp::Node*       node_;
    RobotState_t*       state_;
    const BridgeConfig& cfg_;
    RdControl*          control_;

    std::atomic<uint8_t>  rw_err_{0};
    std::atomic<uint64_t> write_err_total_{0};
    std::atomic<uint32_t> irregular_tick_cnt_{0};
    std::atomic<uint32_t> late_tick_cnt_{0};
    // 기본은 실제 기본 프리셋이다 (nullptr 아님 — 위 Reads() 주석 참조).
    std::atomic<const ReadPreset*> read_preset_{&ecu::kPresetControl};

    // ⚠ 종전 여기에 `hw_connected_`/`hw_error_msg_` 가 있었다. **쓰는 곳도 읽는 곳도
    //   없었다** — `SetHardwareStatus()` 를 부르는 곳이 하나도 없고, 두 필드를 보는
    //   발행 경로도 없었다. 연결 상태는 `NodeStatus.connected`(state_ 에서 온다)가
    //   유일한 출처다. 06 §9.5 의 "배관은 있는데 생산자가 없다" 와 같은 부류이고,
    //   이쪽은 소비자까지 없어 컴파일러도 테스트도 영원히 잡지 못했다.

    // ── 발행자 (03 §5.2 — 이것이 전부다) ──
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr           pub_battery_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr          pub_imu_;
    rclcpp::Publisher<mgs_tp_msgs::msg::NodeStatus>::SharedPtr   pub_ecu_status_;
    rclcpp::Publisher<mgs_tp_msgs::msg::NodeStatus>::SharedPtr   pub_dpc_status_;
    rclcpp::Publisher<mgs_tp_msgs::msg::NodeStatus>::SharedPtr   pub_pcu_status_;
    rclcpp::Publisher<mgs_tp_msgs::msg::MotorStatus>::SharedPtr  pub_motor_status_;
    rclcpp::Publisher<mgs_tp_msgs::msg::ControlFeedback>::SharedPtr pub_control_feedback_;
    rclcpp::Publisher<mgs_tp_msgs::msg::CommDiag>::SharedPtr     pub_comm_diag_;

    // 시계 동기 — 추정기는 **항상** 돈다. 원자료 발행만 comm_diag_enable 로 가린다.
    RdClockSync clock_sync_;
    bool     clock_sync_announced_ = false;
    double   last_clock_offset_ = 0.0;
    bool     last_clock_valid_  = false;
    double   last_rtt_          = 0.0;
    float    last_quality_      = 0.0f;
    float    last_drift_ppm_    = 0.0f;
    uint32_t clock_sample_cnt_  = 0;

    // EMA 전류 필터 (MotorStatus.current_filtered)
    static constexpr float kCurrentFilterAlpha = 0.05f;
    float current_filtered_[4]        = {0.0f, 0.0f, 0.0f, 0.0f};
    bool  current_filter_initialized_ = false;

    // 단위 환산 — 발행 시점에만 쓰이므로 여기가 제자리다
    static constexpr float kQuatScale  = 0.0001f;
    static constexpr float kGyroToRads = 0.1f * 0.01745329252f;
    static constexpr float kAccToMs2   = 0.001f * 9.81f;
    // AS5600 12bit(0~4095) → deg. 한 바퀴가 4096 카운트다.
    static constexpr float kEncToDeg   = 360.0f / 4096.0f;

    // CommDiag 5Hz 다운샘플 (200Hz 중 40 tick 마다 1개)
    static constexpr int kCommDiagDecim = 40;
    int comm_diag_tick_ = 0;

    // ── 200Hz 발행 분리 (02 §6.3 / A4) ──
    struct TeleItem_t {
        bool has_diag     = false;
        bool has_feedback = false;
        mgs_tp_msgs::msg::CommDiag        diag;
        mgs_tp_msgs::msg::ControlFeedback feedback;
    };
    RdTxnQueue<TeleItem_t, 256> tele_q_;
    std::thread       tele_thread_;
    std::atomic<bool> tele_running_{false};
    void TelemetryLoop();

    // delta_tick(x0.1ms, 0xFF=stale) → 초. stale 이면 NaN.
    static float DtSec(uint8_t raw) {
        return (raw == ecu::DELTA_STALE) ? NaNf() : raw * 1e-4f;
    }
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_TELEMETRY_HPP_
