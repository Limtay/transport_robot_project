#include "orin_firmware_bridge/policy/rd_status.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

#include "orin_firmware_bridge/core/rd_map.hpp"          // TARGET::{ECU,DPC,PCU}
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"
#include "orin_firmware_bridge/rd_config.hpp"            // AUTO_MODE_NONE (브리지 쪽 표식)

namespace orin_bridge {

namespace {

// 숫자는 로케일에 흔들리지 않게 직접 찍는다. `std::to_string(double)` 은 소수점이
// 로케일을 따라가 ',' 가 나올 수 있고, 그러면 JSON 이 깨진다.
std::string Num(double v, int prec) {
    if (!std::isfinite(v)) return "null";      // NaN/inf 는 JSON 에 없다 — null 이 맞다
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
}

std::string OptStr(const std::optional<std::string>& v) {
    return v ? ("\"" + JsonEscape(*v) + "\"") : "null";
}
std::string OptNum(const std::optional<double>& v, int prec) {
    return v ? Num(*v, prec) : "null";
}
std::string OptInt(const std::optional<int>& v) {
    return v ? std::to_string(*v) : "null";
}
std::string OptU32(const std::optional<uint32_t>& v) {
    return v ? std::to_string(*v) : "null";
}

}  // namespace

std::string JsonEscape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (unsigned char c : in) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                // 제어문자만 \u 이스케이프. UTF-8 본문(한글)은 그대로 통과시킨다 —
                // JSON 은 UTF-8 을 허용하므로 굳이 \u 로 부풀릴 이유가 없다.
                if (c < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

const char* CtrModeName(uint8_t m) {
    switch (m) {
        case ecu::CTR_MODE_ESTOP:      return "estop";
        case ecu::CTR_MODE_CURRENT:    return "current";
        case 2:                        return "current_brake";
        case ecu::CTR_MODE_VELOCITY:   return "velocity";
        case ecu::CTR_MODE_POSITION:   return "position";
        case ecu::CTR_MODE_SET_ORIGIN: return "set_origin";
        default:                       return "unknown";
    }
}

const char* AutoModeJsonName(uint8_t am) {
    switch (am) {
        case ecu::AUTO_MODE_KINEMATIC: return "kinematic";
        case ecu::AUTO_MODE_CURRENT:   return "current";
        case ecu::AUTO_MODE_DIRECT:    return "direct";
        case ecu::AUTO_MODE_CONTROL:   return "control";
        case ecu::AUTO_MODE_VELOCITY:  return "velocity";
        case ecu::AUTO_MODE_POSITION:  return "position";
        // `none` 은 **유효한 선택지다** (01 §4.2 — 아무것도 쓰지 않는 구성, 견인 실험).
        // 표에 없어서 "unknown" 으로 나가고 있었다: 조작자에게는 고장처럼 보이고,
        // result.json 의 `mode` 에도 그대로 남아 분석이 그 런을 분류하지 못한다.
        case AUTO_MODE_NONE:           return "none";
        default:                       return "unknown";
    }
}

const char* WriteSourceName(uint8_t s) {
    switch (s) {
        case 0:  return "none";
        case 1:  return "cmd_vel";
        case 2:  return "profile";
        case 3:  return "stream";
        default: return "unknown";
    }
}

const char* EcuModeName(uint8_t m) {
    switch (m) {
        case ecu::MODE_MANUAL: return "manual";
        case ecu::MODE_AUTO:   return "auto";
        default:               return "unknown";
    }
}

const char* TargetName(uint8_t t) {
    switch (t) {
        case TARGET::ECU: return "ecu";
        case TARGET::DPC: return "dpc";
        case TARGET::PCU: return "pcu";
        default:          return "unknown";
    }
}

std::string StatusToJson(const StatusSnapshot_t& s) {
    std::ostringstream o;
    o << "{";
    o << "\"bridge_mode\":\""   << JsonEscape(s.bridge_mode)   << "\",";
    o << "\"control_state\":\"" << JsonEscape(s.control_state) << "\",";
    o << "\"write_source\":\""  << JsonEscape(s.write_source)  << "\",";
    o << "\"ecu_mode\":\""      << JsonEscape(s.ecu_mode)      << "\",";
    o << "\"ecu_sys_state\":"   << s.ecu_sys_state             << ",";
    o << "\"auto_mode\":\""     << JsonEscape(s.auto_mode)     << "\",";
    o << "\"write_span\":\""    << JsonEscape(s.write_span)    << "\",";
    o << "\"read_preset\":\""   << JsonEscape(s.read_preset)   << "\",";

    o << "\"active_motors\":[";
    for (size_t i = 0; i < s.active_motors.size(); i++)
        o << (i ? "," : "") << s.active_motors[i];
    o << "],";
    o << "\"motor_mask\":" << s.motor_mask << ",";

    o << "\"ctr_mode\":[";
    for (size_t i = 0; i < s.ctr_mode.size(); i++)
        o << (i ? "," : "") << "\"" << JsonEscape(s.ctr_mode[i]) << "\"";
    o << "],";

    o << "\"goal_id\":"      << s.goal_id                  << ",";
    o << "\"profile_time\":" << Num(s.profile_time, 3)     << ",";

    o << "\"safe_stop\":"        << (s.safe_stop ? "true" : "false") << ",";
    o << "\"safe_stop_detail\":" << OptStr(s.safe_stop_detail)       << ",";

    o << "\"stamp_valid\":"      << (s.stamp_valid ? "true" : "false") << ",";
    o << "\"stamp_quality_ms\":" << OptNum(s.stamp_quality_ms, 3)      << ",";
    o << "\"drift_ppm\":"        << OptNum(s.drift_ppm, 1)             << ",";
    o << "\"rtt_ms\":"           << OptNum(s.rtt_ms, 3)                << ",";
    o << "\"rw_err\":"           << s.rw_err                           << ",";
    o << "\"drop_cnt\":"         << s.drop_cnt                         << ",";

    o << "\"lock_reason\":" << OptStr(s.lock_reason) << ",";

    // 09 §4.2 — 보드별 상태. `enabled`(설정) 와 `connected`(관측) 를 나눠 낸다.
    auto node_obj = [](const StatusNode_t& n) {
        std::ostringstream j;
        j << "{\"enabled\":"     << (n.enabled   ? "true" : "false")
          << ",\"connected\":"   << (n.connected ? "true" : "false")
          << ",\"fail_streak\":" << n.fail_streak << "}";
        return j.str();
    };
    o << "\"nodes\":{"
      << "\"ecu\":" << node_obj(s.node_ecu) << ","
      << "\"dpc\":" << node_obj(s.node_dpc) << ","
      << "\"pcu\":" << node_obj(s.node_pcu) << "},";

    // 09 §5.3 ④ — jeongae 시퀀스 (U12). `busy` 를 따로 내는 이유: 웹이 `state != "IDLE"`
    // 로 판정하게 두면 단계 이름이 하나 늘 때마다 웹이 같이 바뀌어야 한다. **잠글지 말지의
    // 판단은 브리지가 내려 준다.**
    o << "\"cmd_vel_zero_skip\":" << (s.cmd_vel_zero_skip ? "true" : "false") << ","
      << "\"cmd_vel_zero_timeout_s\":" << s.cmd_vel_zero_timeout_s << ",";

    o << "\"sequence\":{"
      << "\"state\":\""     << s.seq_state << "\","
      << "\"wait_ticks\":"  << s.seq_wait_ticks << ","
      << "\"wait_max\":"    << s.seq_wait_max << ","
      << "\"busy\":"        << (s.seq_busy ? "true" : "false") << ","
      << "\"locked\":"      << (s.seq_locked ? "true" : "false") << "},";

    o << "\"slots\":[";
    for (size_t i = 0; i < s.slots.size(); i++) {
        const auto& sl = s.slots[i];
        o << (i ? "," : "") << "{";
        o << "\"index\":"    << sl.index << ",";
        o << "\"active\":"   << (sl.active ? "true" : "false") << ",";
        // 비활성 슬롯도 **키를 뺀 채로 두지 않는다** — 소비자가 키 유무로 분기하게 되면
        // 그것이 곧 04 §4 가 금지한 "형태로 분기하는 파싱" 이다.
        o << "\"cmd\":"      << OptStr(sl.cmd)      << ",";
        o << "\"target\":"   << OptStr(sl.target)   << ",";
        o << "\"duration\":" << OptInt(sl.duration) << ",";
        o << "\"attempts\":" << OptU32(sl.attempts);
        o << "}";
    }
    o << "]";
    o << "}";
    return o.str();
}

}  // namespace orin_bridge
