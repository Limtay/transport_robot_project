#include "orin_firmware_bridge/rd_schedule.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

using namespace std::chrono_literals;

namespace orin_bridge {

RdSchedule::RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state, std::shared_ptr<RdBridge> bridge_node)
    : comm_(comm), map_(map), robot_state_(state), bridge_node_(bridge_node),
      is_custom_ready_(false), tick_count_(0), is_running_(false)
{
    // 100Hz 태스크 (짝수 틱): 모터 피드백 전체 읽기
    task_100hz_    = {TARGET::ECU, PacketInst::READ,  ecu::REG_MOTOR_DATA_OFFSET, sizeof(ecu::DATA_MOTOR_t)};
    // 10Hz 서브 태스크 (홀수 틱 순환, slot 0~8)
    tasks_10hz_[0] = {TARGET::ECU, PacketInst::READ,  ecu::REG_SYS_OFFSET,        sizeof(ecu::DATA_SYSTEM_t)};
    tasks_10hz_[1] = {TARGET::ECU, PacketInst::READ,  ecu::REG_ENCODER_OFFSET,    sizeof(ecu::DATA_ENCODER_t)};
    tasks_10hz_[2] = {TARGET::ECU, PacketInst::READ,  ecu::REG_CMD_SYSTEM_OFFSET, sizeof(ecu::CMD_SYSTEM_t)};
    tasks_10hz_[3] = {TARGET::ECU, PacketInst::READ,  ecu::REG_CMD_MOTOR_OFFSET,  sizeof(ecu::CMD_MOTOR_t)};
    tasks_10hz_[4] = {TARGET::ECU, PacketInst::READ,  ecu::REG_SYS_OFFSET,        sizeof(ecu::DATA_SYSTEM_t)};
    tasks_10hz_[5] = {TARGET::ECU, PacketInst::READ,  ecu::REG_ENCODER_OFFSET,    sizeof(ecu::DATA_ENCODER_t)};
    tasks_10hz_[6] = {TARGET::ECU, PacketInst::READ,  ecu::REG_CMD_SYSTEM_OFFSET, sizeof(ecu::CMD_SYSTEM_t)};
    tasks_10hz_[7] = {TARGET::ECU, PacketInst::READ,  ecu::REG_CMD_MOTOR_OFFSET,  sizeof(ecu::CMD_MOTOR_t)};
    tasks_10hz_[8] = {TARGET::ECU, PacketInst::READ,  ecu::REG_SYS_OFFSET,        sizeof(ecu::DATA_SYSTEM_t)};

    // slot 9: 커스텀 명령용 빈자리 (기본값: 모터 피드백 재읽기)
    tasks_10hz_[9] = {TARGET::ECU, PacketInst::READ,  ecu::REG_MOTOR_DATA_OFFSET, sizeof(ecu::DATA_MOTOR_t)};
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
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler Thread Started.");
}

void RdSchedule::Stop() {
    is_running_ = false;
    if (sched_thread_.joinable()) {
        sched_thread_.join();
        RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler Thread joined.");
    }
}

bool RdSchedule::PushCustomCommand(const TaskConfig_t& task) {
    std::lock_guard<std::mutex> lock(custom_mutex_);
    custom_task_ = task;
    is_custom_ready_.store(true);
    return true;
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

    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Main Control Loop Started (200Hz)");

    auto next_cycle = std::chrono::steady_clock::now() + initial_delay;
    RD_RET ret_val  = RD_OK;

    while (is_running_ && rclcpp::ok()) {
        next_cycle += period;
        std::this_thread::sleep_until(next_cycle);

        if (tick_count_ % 2 == 0) {
            ret_val = ExecuteTask(task_100hz_);
        } else {
            int slot = (tick_count_ % 20) / 2;
            if (slot == 9) {
                if (is_custom_ready_.load()) {
                    std::lock_guard<std::mutex> lock(custom_mutex_);
                    ret_val = ExecuteTask(custom_task_);
                    is_custom_ready_.store(false);
                    std::cout << "[Schedule] Custom Task addr=0x"
                              << std::hex << custom_task_.start_addr << std::dec << std::endl;
                } else {
                    ret_val = ExecuteTask(tasks_10hz_[9]);
                }
            } else {
                ret_val = ExecuteTask(tasks_10hz_[slot]);
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
    return RD_OK;
}

RD_RET RdSchedule::ExecuteTask(const TaskConfig_t& config) {
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
        if (ret != RD_OK) return ret;
    } else if (ret == RD_FATAL) {
        return ret;
    }
    // RD_ERROR / RD_TIMEOUT: Read 내부에서 이미 flush 완료, 다음 사이클 Write 전 flush 가 재보장

    return RD_OK;
}

} // namespace orin_bridge
