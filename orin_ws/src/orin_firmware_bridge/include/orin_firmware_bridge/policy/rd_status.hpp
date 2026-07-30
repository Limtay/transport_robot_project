#ifndef ORIN_FIRMWARE_BRIDGE__RD_STATUS_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_STATUS_HPP_

// GET_STATUS 정형 JSON (redesign/04 §4 — C3)
//
// **텍스트 정규식 파싱을 시키지 않는다.** 종전 응답은
//   "state=IDLE motor_mask=0x01 ctr_mode=[4,1,1,1] auto_mode=2(DIRECT) write=128:52"
// 라는 한국어 섞인 문장이었고, `control_web` 은 그걸 정규식으로 뜯어 쓰고 있었다.
// 조작 UI 를 그 위에 올릴 수 없었던 이유가 이것이다 (06 §9.1).
//
// ## 규칙 (04 §4)
//
//  - **모든 키가 항상 존재한다.** 값이 없으면 `null` — 키 유무로 분기하게 만들지 않는다.
//  - enum 은 **문자열**이다. 정수 원본이 필요하면 별도 키를 둔다 (`ecu_sys_state`).
//  - `safe_stop_detail` 은 `safe_stop=false` 일 때만 문자열.
//  - `stamp_valid=false` 면 `stamp_quality_ms`·`drift_ppm` 은 `null`.
//
// ## 왜 자유 함수인가
//
// 조립을 rclcpp 밖에 두면 **노드 없이 유닛 테스트할 수 있다** (A5 의 성과).
// 스키마는 계약이므로 값이 아니라 **키의 존재와 타입**을 테스트로 고정한다.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orin_bridge {

// 커맨드 슬롯 1칸의 외부 표현. 비활성 슬롯은 index/active 만 의미가 있다.
struct StatusSlot_t {
    int                        index    = 0;
    bool                       active   = false;
    std::optional<std::string> cmd;        // "READ" / "WRITE" / "REBOOT" ...
    std::optional<std::string> target;     // "ecu" / "dpc" / "pcu"
    std::optional<int>         duration;   // 0=forever / 1=once / 2~100 [s]
    std::optional<uint32_t>    attempts;
};

struct StatusSnapshot_t {
    std::string bridge_mode   = "project";
    std::string control_state = "INIT";
    std::string write_source  = "none";
    std::string ecu_mode      = "manual";
    int         ecu_sys_state = 0;
    std::string auto_mode     = "current";
    std::string write_span    = "-";
    std::string read_preset   = "control";

    std::vector<int>         active_motors;
    int                      motor_mask = 0;
    std::vector<std::string> ctr_mode;      // 4개 — "estop"/"current"/...

    uint32_t goal_id      = 0;
    double   profile_time = 0.0;

    bool                       safe_stop = false;
    std::optional<std::string> safe_stop_detail;   // safe_stop=false 일 때만

    bool                  stamp_valid = false;
    std::optional<double> stamp_quality_ms;        // stamp_valid=false 면 null
    std::optional<double> drift_ppm;               //  "
    std::optional<double> rtt_ms;
    int                   rw_err   = 0;
    uint32_t              drop_cnt = 0;

    std::optional<std::string> lock_reason;        // LOCKED 아니면 null
    std::vector<StatusSlot_t>  slots;
};

// 04 §4 스키마 그대로. 키 순서는 문서 순서를 따른다 (사람이 대조하기 쉽도록).
std::string StatusToJson(const StatusSnapshot_t& s);

// ── enum → 문자열 (스키마가 요구하는 소문자 표기) ─────────────────────────────
const char* CtrModeName(uint8_t ctr_mode);      // 0=estop 1=current 2=current_brake ...
const char* AutoModeJsonName(uint8_t auto_mode);// AutoModeName 의 소문자판
const char* WriteSourceName(uint8_t src);       // 0=none 1=cmd_vel 2=profile 3=stream
const char* EcuModeName(uint8_t mode);          // 0=manual 1=auto
const char* TargetName(uint8_t target_id);      // ecu/dpc/pcu

// JSON 문자열 이스케이프. lock_reason·safe_stop_detail 에는 한글과 따옴표가 섞여 들어온다 —
// 이스케이프를 빼면 **거부 사유 하나가 응답 전체를 깨뜨린다.**
std::string JsonEscape(const std::string& in);

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_STATUS_HPP_
