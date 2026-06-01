#include "orin_firmware_bridge/rd_logger.hpp"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <cstdlib>
#include <filesystem>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace orin_bridge {

RdLogger::RdLogger(RobotState_t* state, const std::string& folder_name)
    : state_(state), is_logging_(false) {
    
    // 1. 운영체제에서 HOME 환경변수 가져오기
    const char* home_dir = std::getenv("HOME");
    fs::path base_dir;
    
    if (home_dir != nullptr) {
        base_dir = fs::path(home_dir) / folder_name; // 예: /home/swarm/motor_logging
    } else {
        base_dir = fs::path(".") / folder_name;      // 예: ./motor_logging
    }

    // 2. 폴더 자동 생성
    if (!fs::exists(base_dir)) {
        fs::create_directories(base_dir);
    }

    // 3. 현재 시간 구하기 
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "log_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    // 4. 최종 파일 경로 완성 및 내부 변수에 저장
    fs::path full_path = base_dir / oss.str();
    file_path_ = full_path.string();

    RCLCPP_INFO(rclcpp::get_logger("RdLogger"), "Logger Ready. File: %s", file_path_.c_str());
}

RdLogger::~RdLogger() {
    Stop();
}

void RdLogger::Start() {
    if (is_logging_) return;

    // CSV 파일 열기 (덮어쓰기 모드, 이어쓰려면 std::ios::app 추가)
    csv_file_.open(file_path_);
    if (!csv_file_.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("RdLogger"), "Failed to open CSV file!");
        return;
    }
    // --- CSV 헤더 작성 ---
    csv_file_ << "Timestamp(ms)";
    // 연결 상태
    csv_file_ << ",ECU_Connected,DPC_Connected,PCU_Connected";
    // EcuStatePkg
    csv_file_ << ",FSM_State";
    csv_file_ << ",HW_CAN,HW_I2C,HW_UART1";
    for(int i=1; i<=4; i++) csv_file_ << ",MotorHealth_M" << i;
    for(int i=1; i<=4; i++) csv_file_ << ",TxErr_M" << i;
    for(int i=1; i<=4; i++) csv_file_ << ",RxErr_M" << i;
    csv_file_ << ",AliveTime(s)";
    // Motor feedback
    for(int i=1; i<=4; i++) csv_file_ << ",Pose_M" << i;
    for(int i=1; i<=4; i++) csv_file_ << ",Speed_M" << i;
    for(int i=1; i<=4; i++) csv_file_ << ",Current_M" << i;
    for(int i=1; i<=4; i++) csv_file_ << ",Temp_M" << i;
    for(int i=1; i<=4; i++) csv_file_ << ",Error_M" << i;
    csv_file_ << "\n";

    is_logging_ = true;
    log_thread_ = std::thread(&RdLogger::LogLoop, this);
    RCLCPP_INFO(rclcpp::get_logger("RdLogger"), "CSV Logger Started.");
}

void RdLogger::Stop() {
    is_logging_ = false;
    if (log_thread_.joinable()) {
        log_thread_.join();
    }
    if (csv_file_.is_open()) {
        csv_file_.close();
        RCLCPP_INFO(rclcpp::get_logger("RdLogger"), "CSV File Closed Safely.");
    }
}

void RdLogger::LogLoop() {
    auto start_time = std::chrono::steady_clock::now();
    auto period = 10ms; // 10Hz 기록
    auto next_cycle = start_time;

    while (is_logging_ && rclcpp::ok()) {
        next_cycle += period;
        std::this_thread::sleep_until(next_cycle);

        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        EcuState_t local_ecu;
        bool ecu_conn, dpc_conn, pcu_conn;
        {
            std::lock_guard<std::mutex> lock(state_->state_mutex);
            local_ecu = state_->ecu;
            ecu_conn  = state_->ecu.comm.is_connected;
            dpc_conn  = state_->dpc.comm.is_connected;
            pcu_conn  = state_->pcu.comm.is_connected;
        }
        // 연결이 끊긴 경우 stale 데이터 대신 0으로 기록                     
        if (!ecu_conn) local_ecu = EcuState_t{};      

        csv_file_ << elapsed_ms;
        // 연결 상태
        csv_file_ << "," << ecu_conn << "," << dpc_conn << "," << pcu_conn;
        // EcuStatePkg
        csv_file_ << "," << static_cast<int>(local_ecu.fsm_state);
        csv_file_ << "," << local_ecu.hw_err.ecu_can
                  << "," << local_ecu.hw_err.ecu_i2c
                  << "," << local_ecu.hw_err.ecu_uart1;
        for(int i=0; i<4; i++) csv_file_ << "," << static_cast<int>(local_ecu.motor_state[i]);
        for(int i=0; i<4; i++) csv_file_ << "," << static_cast<int>(local_ecu.motor_tx_err_cnt[i]);
        for(int i=0; i<4; i++) csv_file_ << "," << static_cast<int>(local_ecu.motor_rx_err_cnt[i]);
        csv_file_ << "," << std::fixed << std::setprecision(1)
                  << static_cast<float>(local_ecu.comm.alive_time) / 10.0f;
        // Motor feedback
        for(int i=0; i<4; i++) csv_file_ << "," << local_ecu.motor_pose[i];
        for(int i=0; i<4; i++) csv_file_ << "," << local_ecu.motor_speed[i];
        for(int i=0; i<4; i++) csv_file_ << "," << local_ecu.motor_current[i];
        for(int i=0; i<4; i++) csv_file_ << "," << static_cast<int>(local_ecu.motor_temp[i]);
        for(int i=0; i<4; i++) csv_file_ << "," << static_cast<int>(local_ecu.motor_error[i]);

        csv_file_ << "\n";
    }

}

}// namespace orin_bridge