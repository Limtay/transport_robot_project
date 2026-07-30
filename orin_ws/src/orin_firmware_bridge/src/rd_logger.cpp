#include "orin_firmware_bridge/rd_logger.hpp"

#include <cstdio>

#include "orin_firmware_bridge/rd_clock.hpp"

namespace orin_bridge {

const char* LogLevelStr(LogLevel l) {
    switch (l) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default:              return "?";
    }
}

namespace {

// rclcpp 없이 도는 경로(단위 테스트·tools/ 계측기)의 기본 싱크.
// 형식은 ROS 로거와 비슷하게 맞춰 로그를 눈으로 대조하기 쉽게 했다.
class StderrLogger : public ILogger {
public:
    void Log(LogLevel level, const char* name, const char* msg) override {
        std::fprintf(stderr, "[%s] [%s]: %s\n", LogLevelStr(level), name, msg);
    }
};

}  // namespace

ILogger* DefaultLogger() {
    static StderrLogger inst;
    return &inst;
}

IClock* DefaultClock() {
    static SystemClock inst;
    return &inst;
}

IRunGate* DefaultRunGate() {
    static AlwaysOkGate inst;
    return &inst;
}

void RdLogf(ILogger* lg, LogLevel level, const char* name, const char* fmt, ...) {
    // 200Hz 루프에서도 불리므로 힙 할당을 피한다. 넘치면 잘린다 — 로그가 잘리는 것이
    // 실시간 루프에서 malloc 이 끼어드는 것보다 낫다.
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    (lg ? lg : DefaultLogger())->Log(level, name, buf);
}

}  // namespace orin_bridge
