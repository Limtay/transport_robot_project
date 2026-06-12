#include "orin_firmware_bridge/rd_schedule.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <pthread.h>
#include <sched.h>

using namespace std::chrono_literals;

namespace orin_bridge {

RdSchedule::RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state,
                       std::shared_ptr<RdBridge> bridge_node, RdCommand* command)
    : comm_(comm), map_(map), robot_state_(state), bridge_node_(bridge_node),
      command_(command), tick_count_(0), is_running_(false)
{
    // 100Hz: 모터 피드백 + 센서 일괄 READ (62~127, 66B)
    task_100hz_      = {TARGET::ECU, PacketInst::READ,  ecu::REG_IMU_OFFSET,
                        static_cast<size_t>(ecu::REG_IMU_SIZE + ecu::REG_ENCODER_SIZE +
                                            ecu::REG_UART2_SIZE + ecu::REG_SENSOR_RC_SIZE +
                                            ecu::REG_MOTOR_DATA_SIZE)};
    // 50Hz: cmd_lin_vel / cmd_ang_vel WRITE (180~187, 8B)
    task_50hz_write_ = {TARGET::ECU, PacketInst::WRITE, ecu::REG_CMD_SYSTEM_OFFSET, 8};
    // 10Hz: 시스템 상태 READ (46~61, 16B)
    task_10hz_ecu_   = {TARGET::ECU, PacketInst::READ,  ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE};
    // TODO(§1): PCU READ 레지스터 미정 (SOC/SOH 등) — 임시로 POWER 영역, enable 플래그로 차단
    task_10hz_pcu_   = {TARGET::PCU, PacketInst::READ,  pcu::REG_DATA_POWER_OFFSET, pcu::REG_DATA_POWER_SIZE};
    // TODO(§1): DPC READ 레지스터 미정 (전개 FSM state 등) — 임시로 SYS 영역, enable 플래그로 차단
    task_10hz_dpc_   = {TARGET::DPC, PacketInst::READ,  dpc::REG_SYS_OFFSET, dpc::REG_SYS_SIZE};
}

RdSchedule::~RdSchedule() {
    Stop();
}

void RdSchedule::MainLoopStart() {
    is_running_ = true;
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler MainLoop Started.");
    SupervisorLoop();
}

void RdSchedule::ThreadStart() {
    if (is_running_) return;
    is_running_ = true;
    sched_thread_ = std::thread(&RdSchedule::SupervisorLoop, this);

    // SCHED_FIFO: 우선순위 낮은 프로세스(GUI 등)에게 선점당하지 않음
    sched_param sp;
    sp.sched_priority = kRtPriority;
    if (pthread_setschedparam(sched_thread_.native_handle(), SCHED_FIFO, &sp) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"),
            "SCHED_FIFO 설정 실패 (권한 부족) — sudo 실행 또는 /etc/security/limits.conf 설정 필요");
    }

    // CPU affinity: kCpuCore 에만 고정 → GUI/시스템 프로세스와 물리적 격리
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(kCpuCore, &cpuset);
    if (pthread_setaffinity_np(sched_thread_.native_handle(), sizeof(cpu_set_t), &cpuset) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "CPU affinity 설정 실패");
    }

    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"),
        "Scheduler Thread Started (SCHED_FIFO pri=%d, CPU=%d).", kRtPriority, kCpuCore);
}

void RdSchedule::Stop() {
    is_running_ = false;
    if (sched_thread_.joinable()) {
        sched_thread_.join();
        RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler Thread joined.");
    }
}

void RdSchedule::SupervisorLoop() {
    while (is_running_ && rclcpp::ok()) {
        if (RunLoop() == RD_OK) break;
        {
            std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
            robot_state_->ecu.comm.is_connected = false;
            robot_state_->dpc.comm.is_connected = false;
            robot_state_->pcu.comm.is_connected = false;
        }
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "Hardware Error! Restarting in 1 second...");
        std::this_thread::sleep_for(1s);
    }
}

RD_RET RdSchedule::RunLoop() {
    auto period        = 5ms;
    auto initial_delay = 1s;

    if (Initialize() != RD_OK) return RD_FATAL;

    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"),
                "Main Control Loop Started (200Hz tick / 100Hz RD / 50Hz WR / 10Hz sys / 5Hz cmd x4)");

    auto next_cycle = std::chrono::steady_clock::now() + initial_delay;
    RD_RET ret_val  = RD_OK;

    while (is_running_ && rclcpp::ok()) {
        next_cycle += period;
        std::this_thread::sleep_until(next_cycle);

        // jeongae 자동 시퀀스 FSM (통신 없음, 상태 전이만)
        command_->TickAutoSequence();

        ret_val = RD_OK;
        if (tick_count_ % 2 == 0) {
            // ===== 100Hz: ECU READ 62~127 =====
            if (!command_->IsTargetBlackedOut(TARGET::ECU)) {
                ret_val = ExecuteTask(task_100hz_);
            }
        } else {
            int odd_idx = static_cast<int>((tick_count_ % FRAME_TICKS) / 2);  // 0~19
            if (odd_idx % 2 == 0) {
                // ===== 50Hz: ECU WRITE 180~187 (cmd_vel) =====
                // skip 조건: 보드 blackout / jeongae soft-ESTOP / cmd_vel 안전장치
                // (CMD 영역은 unlock 불필요 — 잠금은 DEFINE 1~15 만 적용)
                bool skip = command_->IsTargetBlackedOut(TARGET::ECU) ||
                            command_->IsCmdVelPaused() ||
                            bridge_node_->ShouldSkipCmdWrite();
                if (!skip) {
                    ret_val = ExecuteTask(task_50hz_write_);
                }
            } else {
                ret_val = ExecuteSubSlot((odd_idx - 1) / 2);  // 0~9
            }
        }
        tick_count_++;

        if (ret_val == RD_FATAL) {
            RCLCPP_ERROR(rclcpp::get_logger("RdSchedule"), "Hardware Disconnected! Returning to Supervisor.");
            comm_->Clear();
            return RD_FATAL;
        }

        auto time_elapsed = std::chrono::steady_clock::now() - next_cycle;
        if (time_elapsed > 2 * period) {
            RCLCPP_FATAL(rclcpp::get_logger("RdSchedule"), "Processing time exceeded 2x period!");
            return RD_FATAL;
        } else if (time_elapsed > period) {
            next_cycle = std::chrono::steady_clock::now() + period;
            RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "Processing time exceeded period!");
        }

        if (tick_count_ % 400 == 55) {
            auto exec_us    = std::chrono::duration_cast<std::chrono::microseconds>(time_elapsed).count();
            uint64_t loss   = (tx_count > rx_count) ? (tx_count - rx_count) : 0;
            bool ecu_on, dpc_on, pcu_on;
            {
                std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
                ecu_on = robot_state_->ecu.comm.is_connected;
                dpc_on = robot_state_->dpc.comm.is_connected;
                pcu_on = robot_state_->pcu.comm.is_connected;
            }
            const char* ecu = ecu_on ? "ON" : "OFF";
            const char* dpc = dpc_on ? "ON" : "OFF";
            const char* pcu = pcu_on ? "ON" : "OFF";
            std::printf("\n====== [RdSchedule Heartbeat] ======\n");
            std::printf(" Ticks : %lu \t| Exec Time: %ld us\n", tick_count_, exec_us);
            std::printf(" Comm  : Tx %lu / Rx %lu (Loss: %lu)\n", tx_count, rx_count, loss);
            std::printf(" Nodes : ECU[%s]  DPC[%s]  PCU[%s]\n", ecu, dpc, pcu);
            std::printf("====================================\n");
        }
    }
    return RD_OK;
}

// 서브 슬롯 패턴 (200ms 1회전): [E10, PCU, DPC, C1, C2, E10, PCU, DPC, C3, C4]
RD_RET RdSchedule::ExecuteSubSlot(int sub) {
    switch (sub) {
        case 0: case 5:
            if (command_->IsTargetBlackedOut(TARGET::ECU)) return RD_OK;
            return ExecuteTask(task_10hz_ecu_);

        case 1: case 6:
            // TODO(§1): PCU 레지스터 미확정 — enable_pcu_read_ 활성화 전까지 idle slot
            if (!enable_pcu_read_ || command_->IsTargetBlackedOut(TARGET::PCU)) return RD_OK;
            return ExecuteTask(task_10hz_pcu_);

        case 2: case 7:
            // TODO(§1): DPC 레지스터 미확정 — enable_dpc_read_ 활성화 전까지 idle slot
            if (!enable_dpc_read_ || command_->IsTargetBlackedOut(TARGET::DPC)) return RD_OK;
            return ExecuteTask(task_10hz_dpc_);

        case 3: case 4: case 8: case 9: {
            // 커맨드 슬롯 0~3 — 각 5Hz
            int slot = (sub == 3) ? 0 : (sub == 4) ? 1 : (sub == 8) ? 2 : 3;
            TaskConfig_t task;
            if (!command_->GetSlotTask(slot, &task)) return RD_OK;
            RD_RET tx_res = RD_ERROR;
            RD_RET ret = ExecuteTask(task, &tx_res);
            command_->ReportResult(slot, tx_res);
            return ret;
        }
    }
    return RD_OK;
}

RD_RET RdSchedule::Initialize() {
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "RdBridge Comm Node Starting");
    // 복구 재진입 시 깨진 포트/에러 카운터를 강제로 닫아 Init() 이 실제 재오픈하도록 한다.
    // (최초 기동 시엔 포트가 닫혀 있어 사실상 no-op)
    comm_->Stop();
    while (rclcpp::ok()) {
        if (comm_->Init(&packet_obj_) == RD_OK) break;
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "Waiting for USB...");
        std::this_thread::sleep_for(1s);
    }
    if (!rclcpp::ok()) {
        std::cerr << "[RdSchedule Fatal] ROS shutdown during init" << std::endl;
        return RD_FATAL;
    }
    // 잠금은 DEFINE(1~15) 영역만 — CMD 영역 쓰기는 unlock 불필요.
    // DEFINE 수정이 필요할 땐 CLI 'macro unlock on' → write → 'macro unlock off'.
    return RD_OK;
}

RD_RET RdSchedule::ExecuteTask(const TaskConfig_t& config, RD_RET* tx_result) {
    if (tx_result) *tx_result = RD_ERROR;

    size_t data_len = 0;
    RD_RET ret = map_->Encode(config, robot_state_, &packet_obj_, &data_len);
    if (ret != RD_OK) return ret;

    // [핵심·유일한 flush 지점] Write 직전 RX 버퍼 flush.
    // Orin 마스터 req→resp 구조에서, 이전 사이클의 늦은(stale) 응답이나
    // 잔여 바이트를 이번 요청과 섞이지 않게 매번 깨끗이 비운다.
    // 정상 사이클에선 비울 게 없어 사실상 no-op, 에러(sync/length/body/CRC) 후엔
    // 여기서 1사이클 내 자가복구. → RdComm::Read 내부엔 별도 Flush 를 두지 않는다.
    comm_->Clear();

    ret = comm_->Write(&packet_obj_, data_len);
    if (ret == RD_OK) tx_count++;
    else if (ret == RD_FATAL) return ret;

    // 헤더 2ms + 바디 2ms = 최악 4ms < 5ms 주기 (1ms 마진)
    ret = comm_->Read(&packet_obj_, 2, 2);
    if (ret == RD_OK) {
        rx_count++;
        ret = map_->Decode(&packet_obj_, config, robot_state_);
        if (ret != RD_OK) return ret;  // 패킷 에러 (Data[0] != NONE 포함)
        if (tx_result) *tx_result = RD_OK;
    } else if (ret == RD_FATAL) {
        return ret;
    } else {
        // RD_ERROR / RD_TIMEOUT: Read 내부에서 이미 flush 완료, 다음 사이클 Write 전 flush 가 재보장
        if (tx_result) *tx_result = ret;
    }

    return RD_OK;
}

} // namespace orin_bridge
