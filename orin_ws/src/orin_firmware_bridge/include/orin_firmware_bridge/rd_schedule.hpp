#ifndef ORIN_FIRMWARE_BRIDGE__RD_SCHEDULE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_SCHEDULE_HPP_

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include "orin_firmware_bridge/rd_comm.hpp"
#include "orin_firmware_bridge/rd_map.hpp"
#include "orin_firmware_bridge/rd_bridge.hpp"

namespace orin_bridge {

class RdSchedule {
public:
    RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state, std::shared_ptr<RdBridge> bridge_node);
    ~RdSchedule();

    void ThreadStart();
    void MainLoopStart();
    void Stop();

    // 외부에서 커스텀 명령을 밀어넣기 (10번 슬롯 차례에 1회 발사)
    bool PushCustomCommand(const TaskConfig_t& task);

private:
    RdComm* comm_;
    RdMap*  map_;
    RobotState_t* robot_state_;
    std::shared_ptr<RdBridge> bridge_node_;

    void SupervisorLoop();
    RD_RET RunLoop();

    // 스케줄링 슬롯
    TaskConfig_t task_100hz_;
    TaskConfig_t tasks_10hz_[10];

    // 커스텀 명령 슬롯 (slot 9)
    std::atomic<bool> is_custom_ready_;
    TaskConfig_t      custom_task_;
    std::mutex        custom_mutex_;

    PACKET_comm_t packet_obj_;
    uint64_t tick_count_;
    uint64_t rx_count = 0;
    uint64_t tx_count = 0;

    std::thread      sched_thread_;
    std::atomic<bool> is_running_;

    RD_RET Initialize();
    RD_RET ExecuteTask(const TaskConfig_t& config);
};

} // namespace orin_bridge

#endif
