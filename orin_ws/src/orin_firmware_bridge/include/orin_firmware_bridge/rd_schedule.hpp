#ifndef ORIN_FIRMWARE_BRIDGE__RD_SCHEDULE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_SCHEDULE_HPP_

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include "orin_firmware_bridge/rd_comm.hpp"
#include "orin_firmware_bridge/rd_map.hpp"
#include "orin_firmware_bridge/rd_bridge.hpp"
#include "orin_firmware_bridge/rd_command.hpp"

namespace orin_bridge {

// ===== 스케줄 프레임 (Code_modify.md §1) =====
// 200Hz tick (5ms), 프레임 = 40 tick (200ms)
//   짝수 tick           : 100Hz | ECU READ 62~127 (66B, IMU+ENC+UART2+RC+MOTOR)
//   홀수 tick odd_idx 짝 :  50Hz | ECU WRITE 180~187 (8B, cmd_lin/ang_vel)
//   홀수 tick odd_idx 홀 : 서브 슬롯 10개 순환 (200ms 1회전)
//       [E10, PCU, DPC, C1, C2, E10, PCU, DPC, C3, C4]
//       E10 = ECU READ 46~61 (10Hz) / PCU·DPC READ (10Hz, 레지스터 미정 TODO)
//       C1~C4 = 커맨드 슬롯 0~3 (각 5Hz)

class RdSchedule {
public:
    RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state,
               std::shared_ptr<RdBridge> bridge_node, RdCommand* command);
    ~RdSchedule();

    void ThreadStart();
    void MainLoopStart();
    void Stop();

private:
    RdComm*  comm_;
    RdMap*   map_;
    RobotState_t* robot_state_;
    std::shared_ptr<RdBridge> bridge_node_;
    RdCommand* command_;

    void SupervisorLoop();
    RD_RET RunLoop();

    // 스케줄링 슬롯
    static constexpr uint64_t FRAME_TICKS = 40;   // (5Hz)200ms @ (200Hz)5ms tick
    TaskConfig_t task_100hz_;       // ECU READ 62~127
    TaskConfig_t task_50hz_write_;  // ECU WRITE 180~187
    TaskConfig_t task_10hz_ecu_;    // ECU READ 46~61
    TaskConfig_t task_10hz_pcu_;    // TODO: PCU READ 미정 (SOC/SOH 등)
    TaskConfig_t task_10hz_dpc_;    // TODO: DPC READ 미정 (전개 FSM state 등)

    // TODO: PCU/DPC 레지스터 확정 전까지 기본 비활성 (활성화 시 보드 미장착이면 timeout 폭주)
    bool enable_pcu_read_ = false;
    bool enable_dpc_read_ = false;

    PACKET_comm_t packet_obj_;
    uint64_t tick_count_;
    uint64_t rx_count = 0;
    uint64_t tx_count = 0;

    // 실시간 스케줄링: SCHED_FIFO + CPU 코어 고정 (마우스/GUI 선점 방지)
    // Orin 12코어 기준 마지막 코어(11)를 제어 전용으로 분리.
    // sudo 없이 실패하면 WARN 로그 출력 후 일반 우선순위로 계속.
    static constexpr int kRtPriority = 80;   // SCHED_FIFO 1~99
    static constexpr int kCpuCore    = 11;   // Orin: 0~11

    std::thread       sched_thread_;
    std::atomic<bool> is_running_;

    RD_RET Initialize();
    RD_RET ExecuteSubSlot(int sub);      // 서브 슬롯 0~9 디스패치
    RD_RET ExecuteTask(const TaskConfig_t& config, RD_RET* tx_result = nullptr);
};

} // namespace orin_bridge

#endif
