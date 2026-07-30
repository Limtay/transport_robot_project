#ifndef ORIN_FIRMWARE_BRIDGE__RD_ROS_ADAPTERS_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_ROS_ADAPTERS_HPP_

// L3 전용 — ILogger/IClock/IRunGate 의 rclcpp 구현체 (redesign/02 A5)
//
// **이 헤더를 L0/L1/L2 에서 include 하면 안 된다.** 여기가 rclcpp 를 아는 유일한 지점이고,
// 6단계에서 `rd_core_lib`(rclcpp 링크 안 함)와 `rd_ros_lib` 로 갈릴 때 이 파일은 후자에 남는다.

#include <rclcpp/rclcpp.hpp>

#include "orin_firmware_bridge/rd_clock.hpp"
#include "orin_firmware_bridge/rd_logger.hpp"

namespace orin_bridge {

// 기존 `RCLCPP_*(rclcpp::get_logger(name), ...)` 와 **동일한 출력**이 나오도록 이름을 그대로 넘긴다.
class RclcppLogger : public ILogger {
public:
    void Log(LogLevel level, const char* name, const char* msg) override {
        auto lg = rclcpp::get_logger(name);
        switch (level) {
            case LogLevel::DEBUG: RCLCPP_DEBUG(lg, "%s", msg); break;
            case LogLevel::INFO:  RCLCPP_INFO(lg, "%s", msg);  break;
            case LogLevel::WARN:  RCLCPP_WARN(lg, "%s", msg);  break;
            case LogLevel::ERROR: RCLCPP_ERROR(lg, "%s", msg); break;
            case LogLevel::FATAL: RCLCPP_FATAL(lg, "%s", msg); break;
        }
    }
};

// 시각은 기존과 같은 소스를 쓴다 — steady_clock / system_clock.
// (rclcpp::Clock 으로 바꾸면 use_sim_time 여부에 따라 §2.5 시간축이 달라지므로 건드리지 않는다.)
using RosClock = SystemClock;

class RclcppRunGate : public IRunGate {
public:
    bool Ok() const override { return rclcpp::ok(); }
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_ROS_ADAPTERS_HPP_
