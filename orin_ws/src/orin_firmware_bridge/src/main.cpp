#include <iostream>
// #include <chrono>
// #include <thread>
#include <cmath>
#include <cstring>
#include "orin_firmware_bridge/rd_uart.hpp"
#include "orin_firmware_bridge/rd_comm.hpp"
#include "orin_firmware_bridge/rd_map.hpp"
#include "orin_firmware_bridge/rd_schedule.hpp"
#include "orin_firmware_bridge/rd_bridge.hpp"

using namespace orin_bridge;
using namespace std::chrono_literals;

/* USB-Serial Latency Timer 설정 (리눅스 전용) ------------
// Latency Timer 확인
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// Latency Timer 설정 (1ms로 설정 권장)
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
------------------------------------------------------ */

/* ROS2 실행 전 설정 예시 -------------------------------
0. 패키지 설치
    sudo apt update
    sudo apt install libserial-dev -y
1. 작업공간 생성
    colcon build
    colcon build --packages-select orin_firmware_bridge --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
2. 환경 설정
    source install/setup.bash
3. 노드 실행
    ros2 run orin_firmware_bridge comm_test_node
------------------------------------------------------ */

/* ros2_test_publisher 노드 실행 --------------------------
cd ~/orin_ws
rm -rf build/mgs01_base_msgs install/mgs01_base_msgs
colcon build --packages-select mgs01_base_msgs ros2_test_publisher
source install/setup.bash
ros2 run ros2_test_publisher test_driver

// topic list 확인
source /opt/ros/humble/setup.bash
ros2 topic list
// topic 주기 확인
ros2 topic hz /carrier_cmd_vel
// topic 메시지 확인
ros2 topic echo /carrier_cmd_vel
---------------------------------------------*/

/* USB 포트 점유 프로세스 확인 및 권한 부여 -------------
# 0. 시리얼(UART) 포트 권한 부여
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER

# 1. 현재 연결된 USB 포트 확인 (ttyUSB0가 있는지 확인)
ls -l /dev/ttyUSB*

# ttyUSB0를 누가 잡고 있는지 확인
sudo lsof /dev/ttyUSB0

# 범인 강제 종료
sudo kill -9 [PID번호]

# 2. 권한 부여 (비밀번호 입력)
sudo chmod 666 /dev/ttyUSB0
*/

/*  코드 삭제 방법 -----------------------------
# 1. 워크스페이스로 이동
cd ~/orin_ws

# 2. 소스 폴더 삭제 (주의: 복구 안 됨!)
# rm -rf: 묻지 않고 강제로(f) 폴더째(r) 삭제하라는 뜻
rm -rf src/ros2_test_publisher

# build, install, log 폴더를 싹 지웁니다.
# (걱정 마세요! colcon build 하면 다시 깨끗하게 생성됩니다.)
rm -rf build install log

# 1. 다시 빌드하기
colcon build

# 2. 환경 설정 다시 불러오기 (필수!)
source install/setup.bash

ros2 pkg list | grep test_publisher
-------------------------------------------------------*/


int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
     // ROS 2 초기화
    rclcpp::init(argc, argv);

    // 객체 생성
    RdUart uart("/dev/ttyUSB0");

    RdComm comm(&uart);

    RobotState_t robot_state{};
    // 섀도 기본값: soft ESTOP 해제 — ECU 기본값(RELEASE)과 일치시켜
    // WRITE_REG 로 cmd_system 영역 전송 시 의도치 않은 ESTOP 을 방지
    robot_state.ecu.reg.cmd_system.soft_estop = ecu::SOFT_ESTOP_RELEASE;

    RdMap map;

    // Bridge node start
    auto bridge_node = std::make_shared<RdBridge>(&robot_state);

    // 커맨드 매니저 (슬롯 4개 + jeongae 자동 시퀀스) — bridge 서비스와 스케줄러가 공유
    RdCommand command(&robot_state);
    bridge_node->AttachCommand(&command);

    RdSchedule scheduler(&comm, &map, &robot_state, bridge_node, &command);

    bridge_node->Start();

    // scheduler.ThreadStart(); // 쓰레드 생성해서 루프로 돔
    scheduler.MainLoopStart();

    std::cout << "Test Finished Safely." << std::endl;
    rclcpp::shutdown(); 
    return 0; 
}

/*
// 1. 스케줄러 객체 생성
orin_bridge::RdSchedule scheduler(&comm, &map, &robot_state);

// 2. ROS 2 서비스 콜백 함수 정의
auto handle_custom_cmd = [&scheduler](
    const std::shared_ptr<custom_interfaces::srv::SendCmd::Request> request,
    std::shared_ptr<custom_interfaces::srv::SendCmd::Response> response) 
{
    // 서비스로 들어온 요청을 스케줄러에 밀어넣기!
    bool success = scheduler.PushCustomCommand(request->target_id, request->func_code, request->idx);
    response->success = success;
};

// 3. 서비스 서버 오픈
auto service = node->create_service<custom_interfaces::srv::SendCmd>("/send_custom_cmd", handle_custom_cmd);

// 4. 스케줄러 시작 (ROS 루프와 별도의 스레드로 돌리는 것이 유리합니다)
scheduler.Start();
std::thread sched_thread(&orin_bridge::RdSchedule::RunLoop, &scheduler);

rclcpp::spin(node); // ROS 콜백 처리
*/
// RCLCPP 로그 레벨 설명:
// INFO:   그냥 안내  ("시동 걸림")
// WARN:   경고   ("배터리 좀 부족한데?")
// ERROR:  에러   ("데이터가 깨졌어")
// FATAL:  치명적 ("엔진 터짐, 더 이상 못 움직임")