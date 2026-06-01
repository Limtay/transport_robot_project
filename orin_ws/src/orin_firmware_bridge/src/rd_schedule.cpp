#include "orin_firmware_bridge/rd_schedule.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

using namespace std::chrono_literals;

namespace orin_bridge {

RdSchedule::RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state, std::shared_ptr<RdBridge> bridge_node) 
    : comm_(comm), map_(map), robot_state_(state), bridge_node_(bridge_node),
      is_custom_ready_(false), tick_count_(0), is_running_(false) {
    
    // =========================================================
    // [스케줄러 슬롯 초기화]
    // =========================================================

    // 1. [100Hz 태스크] 짝수 틱마다 실행 
    task_100hz_    = {TARGET::ECU, FUNC::RQ, 130}; // 전류 요청

    // 2. [10Hz 태스크] 홀수 틱마다 1개씩 번갈아 실행
    tasks_10hz_[0] = {TARGET::ECU, FUNC::RQ, 128}; // 모터 위치 요청
    tasks_10hz_[1] = {TARGET::ECU, FUNC::RQ, 129}; // RPM 요청
    tasks_10hz_[2] = {TARGET::ECU, FUNC::RQ, 131}; // 모터 state 요청
    tasks_10hz_[3] = {TARGET::ECU, FUNC::RQ, 152}; // 모터 링키지 각도 요청 
    tasks_10hz_[4] = {TARGET::ECU, FUNC::RQ,   0}; // 
    tasks_10hz_[5] = {TARGET::ECU, FUNC::RQ, 128}; // 
    tasks_10hz_[6] = {TARGET::ECU, FUNC::RQ, 129}; // 
    tasks_10hz_[7] = {TARGET::ECU, FUNC::RQ, 131}; //
    tasks_10hz_[8] = {TARGET::ECU, FUNC::RQ, 152}; // 

    // 10번째 슬롯(Index 9)은 커스텀 명령용 빈자리 (기본값: ECU 상태 요청)
    tasks_10hz_[9] = {TARGET::ECU, FUNC::RQ, 0}; 
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
    if (is_running_) return; // 이미 실행 중이면 중복 실행 방지
    is_running_ = true;

    // 자기 자신(this)의 RunLoop 멤버 함수를 스레드로 실행
    sched_thread_ = std::thread(&RdSchedule::SupervisorLoop, this);
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler Thread Started.");

}

void RdSchedule::Stop() {
    is_running_ = false; // RunLoop 안의 while문을 빠져나오게 만듦
    
    if (sched_thread_.joinable()) {
        sched_thread_.join(); // 스레드가 루프를 끝내고 완전히 종료될 때까지 대기
        RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Scheduler Thread joined successfully.");
    }
}

// =========================================================
// [ROS 2 Service 연결용 인터페이스]
// 외부에서 명령을 밀어넣으면, 10번 슬롯 차례가 왔을 때 1회 발사됩니다.
// =========================================================
bool RdSchedule::PushCustomCommand(uint8_t target_id, uint8_t func_code, uint8_t idx) {
    // 락(Lock)을 걸어서 메인 루프가 데이터를 읽고 있을 때 덮어쓰지 않도록 보호합니다.
    std::lock_guard<std::mutex> lock(custom_mutex_);
    
    custom_task_.target_id = target_id;
    custom_task_.func_code = func_code;
    custom_task_.idx = idx;
    
    is_custom_ready_.store(true); // 다음 10번 슬롯에서 발사되도록 플래그 ON!
    return true;
}

// =========================================================
// [메인 200Hz 스케줄러 루프]
// =========================================================
void RdSchedule::SupervisorLoop() {
    // 이전 대화에서 main() 함수에 넣었던 그 로직을 여기로 가져옵니다.
    while (is_running_ && rclcpp::ok()) {
        if (RunLoop() == RD_OK) break;
        // USB 연결 끊김 — 로거가 stale 값을 기록하지 않도록 즉시 초기화
        {
            std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
            robot_state_->ecu.comm.is_connected = false;
            robot_state_->dpc.comm.is_connected = false;
            robot_state_->pcu.comm.is_connected = false;
        }
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "Hardware Error! Restarting RunLoop in 1 second...");
        std::this_thread::sleep_for(1s);
    }
}

RD_RET RdSchedule::RunLoop() {
    auto period = 5ms; // 5ms = 200Hz
    auto Initial_delay = 1s; 

    if (Initialize() != RD_OK) return RD_FATAL;
    
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "Main Control Loop Started (200Hz)");

    auto next_cycle = std::chrono::steady_clock::now() + Initial_delay;    
    RD_RET ret_val = RD_OK;

    while (is_running_ && rclcpp::ok()) {
        next_cycle += period;
        std::this_thread::sleep_until(next_cycle);

        // --- 틱(Tick) 분배 알고리즘 ---
        if (tick_count_ % 2 == 0) ret_val = ExecuteTask(task_100hz_);
        else {
            int slot = (tick_count_ % 20) / 2;
            if (slot == 9) {// 커스텀 명령
                if (is_custom_ready_.load()) {
                    std::lock_guard<std::mutex> lock(custom_mutex_);
                    ret_val = ExecuteTask(custom_task_);
                    
                    is_custom_ready_.store(false); // 1회 전송 후 플래그 해제
                    std::cout << "[Schedule] Custom Task Executed! (IDX: " << (int)custom_task_.idx << ")" << std::endl;
                } else ret_val = ExecuteTask(tasks_10hz_[9]);
            } else ret_val = ExecuteTask(tasks_10hz_[slot]);
        }
        tick_count_++;

        // [핵심] 하드웨어 에러 감지 시 즉시 루프 탈출
        if (ret_val == RD_FATAL) {
            RCLCPP_ERROR(rclcpp::get_logger("RdSchedule"), "Hardware Disconnected! Returning to Supervisor.");
            comm_->Clear(); // 버퍼 찌꺼기 청소
            return RD_FATAL; // main으로 돌아가서 재시작 유도
        }

        // ----------- Erroes 및 Warnings 체크 -----------------//
        auto time_elapsed = std::chrono::steady_clock::now() - next_cycle;
        if (time_elapsed > period) {
            latency_count++;
            if (latency_count > 5) { // 5회 연속으로 주기 초과 시 치명적 에러로 간주
                RCLCPP_FATAL(rclcpp::get_logger("RdSchedule"), "Consistent Latency Issues Detected!"); 
                return RD_FATAL;
            }
            next_cycle = std::chrono::steady_clock::now() + period; // 다음 주기 보정
            RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "Processing time exceeded period!");
        }else latency_count = 0;
        //------------- 사이클 report 출력 & Latency Check-------------
        if (tick_count_ % 400 == 55) {
            // 1. 실행 시간 계산 (microsecond 단위로 안전하게 변환)
            auto exec_time_us = std::chrono::duration_cast<std::chrono::microseconds>(time_elapsed).count();
            // 2. 패킷 통계 계산
            uint64_t packet_loss = (tx_count > rx_count) ? (tx_count - rx_count) : 0;
            // 3. 연결 상태 문자열로 변환
            const char* ecu_sts = robot_state_->ecu.comm.is_connected ? "ON" : "OFF";
            const char* dpc_sts = robot_state_->dpc.comm.is_connected ? "ON" : "OFF";
            const char* pcu_sts = robot_state_->pcu.comm.is_connected ? "ON" : "OFF";
            // 4. 대시보드 출력
            std::printf("\n====== [RdSchedule Heartbeat] ======\n");
            std::printf(" Ticks : %lu \t| Exec Time: %ld us\n", tick_count_, exec_time_us);
            std::printf(" Comm  : Tx %lu / Rx %lu (Loss: %lu)\n", tx_count, rx_count, packet_loss);
            std::printf(" Nodes : ECU[%s]  DPC[%s]  PCU[%s]\n", ecu_sts, dpc_sts, pcu_sts);
            std::printf("====================================\n");
        }
    }

    return RD_OK;
}

// =========================================================
// [공통 패킷 처리 로직] (Encode -> Write -> Read -> Decode)
// =========================================================
RD_RET RdSchedule::Initialize() {
    RCLCPP_INFO(rclcpp::get_logger("RdSchedule"), "RdBridge Comm Test Node Starting ");
    
    bool init_success = false;
    while (rclcpp::ok()) { // USB Port detect loop
        if (comm_->Init(&packet_obj_) == RD_OK) {
            init_success = true;
            break;
        }
        RCLCPP_WARN(rclcpp::get_logger("RdSchedule"), "Waiting for USB...");
        std::this_thread::sleep_for(1s);
    }
    if (!init_success) {
        std::cerr<<"[RdSchedule Fatal] ROS Connecting error"<<std::endl;
        return RD_FATAL; // if rclcpp::ok() fail 
    }
    return RD_OK;
}

RD_RET RdSchedule::ExecuteTask(const TaskConfig_t& config) {
    // 1. Encode: 로봇 상태 변수 -> 송신 패킷 조립
    bridge_node_->GetRosInputs();
    bridge_node_->PublishTelemetry();

    ret = map_->Encode(config.target_id, config.func_code, config.idx, robot_state_, &packet_obj_);
    if (ret != RD_OK) return ret;

    // 2. Write: 하드웨어(UART) 송신
    ret = comm_->Write(&packet_obj_);
    if (ret == RD_OK) tx_count++;
    else if (ret == RD_FATAL) return ret;

    // 3. Read: 하드웨어 수신 (타임아웃 3ms 설정)
    ret = comm_->Read(&packet_obj_, 3);
    if (ret == RD_OK) {
        rx_count++;
        // 4. Decode: 수신 패킷 -> 로봇 상태 변수 업데이트
        ret = map_->Decode(&packet_obj_, robot_state_);
        if (ret != RD_OK) return ret;
    }else if (ret == RD_FATAL) return ret;
    else comm_->Clear();

    return RD_OK;
}

} // namespace orin_bridge