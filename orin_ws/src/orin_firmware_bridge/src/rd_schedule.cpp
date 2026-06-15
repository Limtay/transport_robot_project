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
    ApplyRtScheduling(pthread_self());  // 메인 스레드에서 루프를 돌리므로 현재 스레드에 적용
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler MainLoop Started.");
    SupervisorLoop();
}

// SCHED_FIFO 우선순위 + CPU 코어 고정을 주어진 스레드에 적용.
// 권한(rtprio/CAP_SYS_NICE) 없으면 WARN 후 일반 우선순위로 계속.
void RdSchedule::ApplyRtScheduling(pthread_t thread) {
    sched_param sp;
    sp.sched_priority = kRtPriority;
    if (pthread_setschedparam(thread, SCHED_FIFO, &sp) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"),
            "SCHED_FIFO 설정 실패 (권한 부족) — limits.conf rtprio 설정 또는 sudo 실행 필요. "
            "일반 우선순위로 계속 (주기 밀림 발생 가능).");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RdSchedule"),
            "SCHED_FIFO pri=%d 적용됨.", kRtPriority);
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(kCpuCore, &cpuset);
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "CPU affinity(core %d) 설정 실패.", kCpuCore);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "CPU affinity core %d 고정됨.", kCpuCore);
    }
}

void RdSchedule::ThreadStart() {
    if (is_running_) return;
    is_running_ = true;
    sched_thread_ = std::thread(&RdSchedule::SupervisorLoop, this);

    ApplyRtScheduling(sched_thread_.native_handle());

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

        // [계측] 깨어난 시점 — next_cycle 대비 오버슛이 순수 wake latency(스케줄 지연).
        auto wake_time = std::chrono::steady_clock::now();
        int64_t wake_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              wake_time - next_cycle).count();
        if (wake_us > stat_wake_max_) stat_wake_max_ = wake_us;

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

        // time_elapsed = wake latency(sleep_until 오버슛) + 이번 tick 처리시간.
        auto time_elapsed = std::chrono::steady_clock::now() - next_cycle;
        int64_t elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(time_elapsed).count();

        // 매 tick 구간 통계 누적 (평균/최대/초과 횟수).
        stat_sum_us_ += (elapsed_us > 0 ? static_cast<uint64_t>(elapsed_us) : 0);
        stat_cnt_++;
        if (elapsed_us > stat_max_us_) stat_max_us_ = elapsed_us;
        // 처리시간 = 전체 - wake. 스파이크가 wake(스케줄) 인지 proc(I/O) 인지 분리.
        int64_t proc_us = elapsed_us - wake_us;
        if (proc_us > stat_proc_max_) stat_proc_max_ = proc_us;

        if (time_elapsed > 5 * period) {
            RCLCPP_FATAL(rclcpp::get_logger("RdSchedule"),
                         "Processing time exceeded 5x period! (%ld us)", elapsed_us);
            return RD_FATAL;
        } else if (time_elapsed > period) {
            next_cycle = std::chrono::steady_clock::now() + period;
            // RT 루프 안에서는 카운트만 — per-tick 동기 콘솔 로깅이 그 자체로 지터를
            // 유발(다음 사이클까지 밀림)하므로 제거하고 헤더비트에서 요약 출력한다.
            exceeded_cnt_++;
        }

        if (tick_count_ % 400 == 55) {
            uint64_t loss   = (tx_count > rx_count) ? (tx_count - rx_count) : 0;
            double   avg_us = stat_cnt_ ? static_cast<double>(stat_sum_us_) / stat_cnt_ : 0.0;
            double   over_pct = stat_cnt_ ? 100.0 * exceeded_cnt_ / stat_cnt_ : 0.0;
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
            std::printf(" Ticks : %lu\n", tick_count_);
            std::printf(" Timing: avg %.0f us / max %ld us | over-period %lu/%lu (%.1f%%)\n",
                        avg_us, stat_max_us_, exceeded_cnt_, stat_cnt_, over_pct);
            std::printf(" Spike : wake_max %ld us / proc_max %ld us  <- 스파이크 출처\n",
                        stat_wake_max_, stat_proc_max_);
            std::printf(" I/O   : clear_max %ld / write_max %ld / read_max %ld us\n",
                        stat_clear_max_, stat_write_max_, stat_read_max_);
            std::printf(" Comm  : Tx %lu / Rx %lu (Loss: %lu)\n", tx_count, rx_count, loss);
            std::printf(" Nodes : ECU[%s]  DPC[%s]  PCU[%s]\n", ecu, dpc, pcu);
            std::printf("====================================\n");

            // 평균 처리시간이 예산(4ms)을 넘으면 tail spike 가 아니라 전형적 케이스가
            // 무거운 것 → 구조적 조치 필요. 구간당 1회만 경고(로깅 지터 최소화).
            if (avg_us > static_cast<double>(kBudgetUs)) {
                RCLCPP_WARN(rclcpp::get_logger("RdSchedule"),
                    "평균 처리시간 %.0f us > 예산 %ld us — 주기 예산 초과(전형 케이스). "
                    "Read 타임아웃/슬롯 부하/코어 격리 점검 필요.", avg_us, kBudgetUs);
            }

            stat_sum_us_ = 0; stat_cnt_ = 0; stat_max_us_ = 0; exceeded_cnt_ = 0;
            stat_wake_max_ = 0; stat_proc_max_ = 0;
            stat_clear_max_ = 0; stat_write_max_ = 0; stat_read_max_ = 0;
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
    auto _t0 = std::chrono::steady_clock::now();
    comm_->Clear();
    auto _t1 = std::chrono::steady_clock::now();

    ret = comm_->Write(&packet_obj_, data_len);
    auto _t2 = std::chrono::steady_clock::now();
    if (ret == RD_OK) tx_count++;
    else if (ret == RD_FATAL) return ret;

    // 헤더 2ms + 바디 2ms = 최악 4ms < 5ms 주기 (1ms 마진)
    ret = comm_->Read(&packet_obj_, 2, 2);
    auto _t3 = std::chrono::steady_clock::now();
    {
        using us = std::chrono::microseconds;
        int64_t c = std::chrono::duration_cast<us>(_t1 - _t0).count();
        int64_t w = std::chrono::duration_cast<us>(_t2 - _t1).count();
        int64_t r = std::chrono::duration_cast<us>(_t3 - _t2).count();
        if (c > stat_clear_max_) stat_clear_max_ = c;
        if (w > stat_write_max_) stat_write_max_ = w;
        if (r > stat_read_max_)  stat_read_max_  = r;
    }
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
