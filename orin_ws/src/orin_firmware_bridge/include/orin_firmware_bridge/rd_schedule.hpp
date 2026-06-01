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

// 각 스케줄 슬롯에 들어갈 패킷 정보 구조체
struct TaskConfig_t {
    uint8_t target_id;
    uint8_t func_code;
    uint8_t idx;
};

class RdSchedule {
public:
    // 생성자: 외부에서 관리하는 로봇 상태(상태 변수)의 포인터를 받아옵니다.
    RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state, std::shared_ptr<RdBridge> bridge_node);
    ~RdSchedule();

    void ThreadStart();
    void MainLoopStart();
    void Stop();

    // [ROS 2 Service 연결용 API] 
    // 나중에 /send_custom_cmd 서비스 콜백 함수 안에서 이 함수를 호출하게 됩니다.
    bool PushCustomCommand(uint8_t target_id, uint8_t func_code, uint8_t idx);

private:
    RdComm* comm_;
    RdMap* map_;
    RobotState_t* robot_state_; // 외부(Bridge Node)와 공유하는 로봇 상태
    std::shared_ptr<RdBridge> bridge_node_;

    // 메인 200Hz 루프 (main.cpp의 while 문을 대체)
    void SupervisorLoop();
    RD_RET RunLoop();

    // --- 스케줄링 슬롯 정의 ---
    TaskConfig_t task_100hz_;         // 100Hz 메인 태스크
    TaskConfig_t tasks_10hz_[10];     // 10Hz 서브 태스크 10개
    
    // --- 10번 슬롯 (커스텀 명령) 제어용 ---
    std::atomic<bool> is_custom_ready_; 
    TaskConfig_t custom_task_;
    std::mutex custom_mutex_;         // 스레드 안전성 보장

    // --- 내부 통신 및 상태 변수 ---
    PACKET_comm_t packet_obj_;
    uint64_t tick_count_;
    uint64_t rx_count = 0;
    uint64_t tx_count = 0;
    uint16_t latency_count = 0;

    std::thread sched_thread_;
    std::atomic<bool> is_running_;

    // 공통 실행 로직
    RD_RET ret;
    RD_RET Initialize(); 
    RD_RET ExecuteTask(const TaskConfig_t& config);
};

} // namespace orin_bridge

#endif