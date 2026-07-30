#ifndef ORIN_FIRMWARE_BRIDGE__RD_CONFIG_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_CONFIG_HPP_

// BridgeConfig — 기동 1회 확정 후 **불변**인 설정 (redesign/02 §5.4, A1-a)
//
// 왜: L1(`rd_schedule`)이 설정을 읽으려고 L3(`RdNode`)를 거꾸로 참조하고 있었다
// (`bridge_node_->IsControlMode()` 등 **10곳**). 값이 필요한 것뿐인데 상위 객체 전체에
// 의존하게 되어 계층 방향이 깨지고, 단위 테스트가 노드를 띄워야만 돌아간다.
//
// 처방: 파라미터 파싱 결과를 **값 묶음**으로 뽑아 하위 계층에 `const&` 로 내려보낸다.
// 기동 후 바뀌지 않으므로 락도 atomic 도 필요 없다 (Q2).
//
// ⚠ 여기 들어가면 안 되는 것: **런타임에 변하는 값**. `auto_mode` 현재값(config service 로
//   바뀐다)·FSM 상태·tick 카운터는 설정이 아니다. `auto_mode_param` 은 "기동 시 지정된 목표값"
//   이라 불변이고, 현재값 `AutoMode()` 와는 다른 것이다 — 이 둘을 섞지 말 것.

#include <cstdint>
#include <string>

#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

namespace orin_bridge {

// 브리지 동작 모드 (00 Q1 / 01 §4.2 / 02 §5.4). 기동 시 고정 — 런타임 토글 없다.
//
// **`project|control|manual` 3개뿐이다.** 구 `traction_test_mode` 는 별도 모드가 아니라
// control 의 한 **구성**이다:
//
//     bridge_mode: control  +  auto_mode: none  +  read_preset: control_test
//
// (`auto_mode: none` = 아무것도 쓰지 않는다 / `control_test` = 로드셀 포함 READ 배치.)
//
// ⚠ 종전 여기에 *"두 프리셋의 세그를 비교하면 traction 은 control 에서 ctr_mode read-back 만
//    뺀 것이다"* 라고 적혀 있었다. **2026-07-30 04 §2.3 적용으로 더 이상 맞지 않는다** —
//    control 은 42~127 을 연속으로 덮고(엔코더·IMU 포함) control_test 는 로드셀+모터만 본다.
//    지금 둘의 관계는 "control_test = control 에서 ctr_mode·IMU·엔코더를 뺀 것" 이다.
//    정확한 배치는 rd_read_preset.hpp 가 소유하고, 골든이 세그를 그 표에서 가져온다.
//
// 모드를 축 하나로 두는 이유: 불리언 두 개는 **표현 불가능한 조합**(둘 다 true)을 만들고,
// 실제로 "동시 지정 시 traction 우선" 이라는 특례가 코드에 있었다.
enum class BridgeMode : uint8_t {
    PROJECT = 0,   // 기본 — cmd_vel 주행, 50ms 프레임
    CONTROL,       // 200Hz ECU (auto_mode 가 RW 인지 READ 인지 정한다)
    MANUAL,        // 자동 설정이 **전무한** 백지 모드 (01 §4.1 R1·R2)
};

inline const char* BridgeModeName(BridgeMode m) {
    switch (m) {
        case BridgeMode::PROJECT: return "project";
        case BridgeMode::CONTROL: return "control";
        case BridgeMode::MANUAL:  return "manual";
    }
    return "unknown";
}

// 문자열 -> enum. 실패하면 false (호출자가 기동을 거부한다 — 오타로 엉뚱한 모드가
// 뜨는 것보다 안 뜨는 것이 낫다).
inline bool ParseBridgeMode(const std::string& s, BridgeMode* out) {
    if (s == "project") { *out = BridgeMode::PROJECT; return true; }
    if (s == "control") { *out = BridgeMode::CONTROL; return true; }
    if (s == "manual")  { *out = BridgeMode::MANUAL;  return true; }
    return false;
}

// `auto_mode: none` — **아무것도 쓰지 않는다** (01 §4.2 파라미터 표).
// ECU 레지스터 값이 아니라 **브리지 쪽 표식**이다. 레지스터 값(0~5)과 겹치지 않게 0xFF.
// 이 값이면: INIT 의 ECU 설정 write 를 건너뛰고, 200Hz tick 은 RW 가 아니라 READ 를 낸다.
constexpr uint8_t AUTO_MODE_NONE = 0xFF;

struct BridgeConfig {
    // ── 동작 모드 (기동 시 고정, 런타임 토글 없음) ──
    BridgeMode bridge_mode = BridgeMode::PROJECT;
    bool bridge_mode_valid = true;

    // 기동 시 지정된 읽기 프리셋 (01 §4.2). 런타임 교체는 config service 가 한다.
    uint8_t read_preset_param = 0;      // 0 = control
    bool    read_preset_valid = true;

    // 편의 술어 — 호출부가 enum 비교를 흩뿌리지 않도록.
    bool IsControl() const { return bridge_mode == BridgeMode::CONTROL; }
    bool IsManual()  const { return bridge_mode == BridgeMode::MANUAL; }
    bool IsProject() const { return bridge_mode == BridgeMode::PROJECT; }

    // **명령을 쓰는가.** `auto_mode: none` 이면 control 이어도 READ 전용이다 —
    // 구 traction_test_mode 가 하던 일이 정확히 이것이다.
    bool WritesCommands() const {
        return IsControl() && auto_mode_param != AUTO_MODE_NONE;
    }
    // control 이면서 아무것도 안 쓰는 구성 = 견인 실험 (구 traction_test_mode).
    bool IsReadOnlyControl() const {
        return IsControl() && auto_mode_param == AUTO_MODE_NONE;
    }

    // manual 은 **자동 설정을 하나도 하지 않는다** (01 §4.1). INIT 의 motor_mask/auto_mode/
    // mode=AUTO write 를 전부 건너뛰고 범위 스캔만 한 뒤 IDLE 로 간다.
    // "안전장치가 없는 모드" 가 아니라 "자동 설정이 없는 모드" 다 — safe_stop·LOCKED·
    // active_motors 불변식은 그대로 산다.
    bool AutoConfiguresEcu() const { return bridge_mode != BridgeMode::MANUAL; }

    // ── §3.1 기동 파라미터 검증 결과 ──
    // valid=false 면 스케줄러가 **통신 시도 없이** exit≠0 으로 끝낸다.
    // ⚠ **이 구조체에서 유일하게 런타임에 바뀌는 필드다.** config service 의
    //   SET_ACTIVE_MOTORS 가 read-back 검증 후 갱신한다(RdControlApi).
    //   service 스레드가 쓰고 200Hz 스케줄 스레드가 읽으므로 **동기화 없는 경합**이 있다 —
    //   재설계 이전부터 있던 것이라 "동작 불변" 원칙상 여기서 고치지 않고 드러내기만 한다.
    //   1바이트라 찢어진 값이 나오진 않지만, 정식으로는 atomic 이어야 한다. (별도 결정 필요)
    uint8_t active_motor_mask   = 0x0F;   // bit0~3 = M1~M4
    bool    active_motors_valid = true;
    uint8_t auto_mode_param     = ecu::AUTO_MODE_CURRENT;  // INIT 이 ECU 에 쓸 목표값
    bool    auto_mode_valid     = true;

    // ── 제어 한계 ──
    double cmd_current_max = 30.0;   // [A] 전역 클램프

    // ── cmd_vel 워치독 (project 모드) ──
    // ⚠ 02 §5.4 는 `cmd_vel_guard_enable` 도 여기 넣었지만 **제외했다** — 실제 코드는
    //   `ros2 param set` 콜백으로 **런타임에 토글**한다(rd_node.cpp 의 param_cb_).
    //   불변 구조체에 넣으면 "기동 후 불변" 이라는 이 구조체의 약속이 거짓이 된다.
    // 여기 있는 것은 기동 시 파라미터로 읽은 **초기값**뿐이다. 이후 토글은
    // `RdCarrierApi::guard_enable_`(atomic) 이 소유하고, `param_cb_` 가 그것을 민다.
    bool   cmd_vel_guard_enable_default = true;
    double cmd_vel_topic_timeout = 0.1;   // [s] 토픽 미수신 한계
    double cmd_vel_zero_timeout  = 3.0;   // [s] 0 수렴 지속 한계

    // STREAM 스테일 한계 [s] (01 §6.1.3). cmd_vel_topic_timeout 과 같은 기본값 —
    // 둘 다 "상위 발행자가 죽었다" 를 같은 시간 척도로 본다.
    double stream_timeout = 0.1;

    // 지연 원자료(CommDiag) 발행 — 기본 off (03 §4.2).
    // 클럭 추정기 자체는 항상 돈다. 발행만 가린다.
    bool comm_diag_enable = false;

    // ── DPC / PCU 슬롯 활성화 (04 §6) ──
    // **기본 off 인 이유는 보드 미장착 시 timeout 폭주를 막기 위함이다.** 슬롯이 skip 되면
    // 그 tick 은 커맨드에 양보되므로 나머지 동작에 영향이 없다 (04 §2.4).
    //
    // DPC 는 2026-07-30 에 레지스터가 확정됐다(rd_register_dpc.hpp). 보드를 붙였으면
    // `-p enable_dpc_read:=true` 로 켠다. PCU 는 아직 레지스터 미확정이다.
    bool enable_dpc_read = false;
    bool enable_pcu_read = false;

    // ── 기타 ──
    std::string imu_frame_id = "imu_link";
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_CONFIG_HPP_
