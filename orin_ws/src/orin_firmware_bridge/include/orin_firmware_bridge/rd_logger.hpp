#ifndef ORIN_FIRMWARE_BRIDGE__RD_LOGGER_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_LOGGER_HPP_

#include <string>
#include <fstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "orin_firmware_bridge/rd_map.hpp" 

namespace orin_bridge {

class RdLogger {
public:
    RdLogger(RobotState_t* state, const std::string& folder_name);
    ~RdLogger();

    void Start();
    void Stop();

private:
    void LogLoop();

    RobotState_t* state_;
    std::string file_path_;
    std::ofstream csv_file_;

    std::thread log_thread_;
    std::atomic<bool> is_logging_;
};

} // namespace orin_bridge

#endif