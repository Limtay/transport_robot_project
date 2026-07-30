#ifndef ORIN_FIRMWARE_BRIDGE__RD_LOGGER_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_LOGGER_HPP_

// ILogger — 전 계층 공용 로깅 추상화 (redesign/02 A5)
//
// 왜: L0/L1/L2 가 `RCLCPP_*` 를 직접 부르면 rclcpp 없이는 빌드도 테스트도 안 된다.
// 인터페이스로 한 겹 덮어 **L3 만 rclcpp 를 안다**. 구현체는 rd_ros_adapters.hpp (L3).
//
// 호출부는 기존 RCLCPP_* 와 형태를 맞췄다 — 로거 핸들이 첫 인자, printf 포맷:
//     RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "tick %lu 밀림", n);
//     RD_WARN     (log_,                "RdSchedule", "tick %lu 밀림", n);
//
// 포맷 문자열은 컴파일 타임 검사된다(__attribute__((format))) — RCLCPP_* 와 동일한 안전망.

#include <cstdarg>
#include <cstdint>

namespace orin_bridge {

enum class LogLevel : uint8_t { DEBUG = 0, INFO, WARN, ERROR, FATAL };

const char* LogLevelStr(LogLevel l);

class ILogger {
public:
    virtual ~ILogger() = default;
    // name = 기존 get_logger("...") 의 이름. 이미 포맷이 끝난 문자열을 받는다.
    virtual void Log(LogLevel level, const char* name, const char* msg) = 0;
};

// 주입 전/단위 테스트용 기본 구현 — stderr 로 떨어진다. 절대 nullptr 을 돌려주지 않는다.
ILogger* DefaultLogger();

// 포맷 후 ILogger::Log 로 넘긴다. lg == nullptr 이면 DefaultLogger() 로 폴백하므로
// 주입을 깜빡해도 로그가 사라지지 않는다 (조용한 유실이 제일 나쁘다).
void RdLogf(ILogger* lg, LogLevel level, const char* name, const char* fmt, ...)
    __attribute__((format(printf, 4, 5)));

}  // namespace orin_bridge

#define RD_DEBUG(lg, name, ...) \
    ::orin_bridge::RdLogf((lg), ::orin_bridge::LogLevel::DEBUG, (name), __VA_ARGS__)
#define RD_INFO(lg, name, ...) \
    ::orin_bridge::RdLogf((lg), ::orin_bridge::LogLevel::INFO, (name), __VA_ARGS__)
#define RD_WARN(lg, name, ...) \
    ::orin_bridge::RdLogf((lg), ::orin_bridge::LogLevel::WARN, (name), __VA_ARGS__)
#define RD_ERROR(lg, name, ...) \
    ::orin_bridge::RdLogf((lg), ::orin_bridge::LogLevel::ERROR, (name), __VA_ARGS__)
#define RD_FATAL(lg, name, ...) \
    ::orin_bridge::RdLogf((lg), ::orin_bridge::LogLevel::FATAL, (name), __VA_ARGS__)

#endif  // ORIN_FIRMWARE_BRIDGE__RD_LOGGER_HPP_
