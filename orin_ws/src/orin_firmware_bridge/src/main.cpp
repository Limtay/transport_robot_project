#include <iostream>
// #include <chrono>
// #include <thread>
#include <cmath>
#include <cstring>
#include "orin_firmware_bridge/core/rd_uart.hpp"
#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/sched/rd_schedule.hpp"
#include "orin_firmware_bridge/ros/rd_node.hpp"
#include "orin_firmware_bridge/ros/rd_ros_adapters.hpp"

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
    auto bridge_node = std::make_shared<RdNode>(&robot_state);

    // 커맨드 매니저 (슬롯 4개 + jeongae 자동 시퀀스) — bridge 서비스와 스케줄러가 공유
    RdCommand command(&robot_state);
    bridge_node->AttachCommand(&command);

    // A5 — rclcpp 구현체는 여기(L3)서만 만들어 하위 계층에 주입한다.
    // 수명: main 스코프 전체 > scheduler/bridge 수명이므로 주소 전달이 안전하다.
    static RclcppLogger  ros_logger;
    static RosClock      ros_clock;
    static RclcppRunGate ros_gate;
    command.SetLogger(&ros_logger);
    bridge_node->Control().SetLogger(&ros_logger);

    RdSchedule scheduler(&comm, &map, &robot_state, bridge_node.get(), &command,
                         // ConfigMut: INIT 이 active_motor_mask 를 채택할 수 있어야 한다
                         // (active_motors 빈칸 → ECU 값 READ 채택, 09 §4.3).
                         bridge_node->ConfigMut(), &bridge_node->Oos(),
                         &bridge_node->Player(), &bridge_node->Control(),
                         // INIT 완료 통지. 인자는 "명령을 쓰는 구성인가" — false 면
                         // 조작 입구(config)만 열고 재생 서버는 열지 않는다 (rd_control_api.hpp).
                         [bridge_node](bool with_profiles){ bridge_node->StartServers(with_profiles); },
                         [bridge_node]{ return bridge_node->ShouldSkipCmdWrite(); },
                         &ros_logger, &ros_clock, &ros_gate);

    // 04 §2.4.2 — 프리셋 교체 경로 연결. 스케줄러가 이 시점에야 존재한다.
    bridge_node->AttachReadPresetSetter(
        [&scheduler](uint8_t id, std::string* why) { return scheduler.SetReadPreset(id, why); });

    bridge_node->Start();

    // 여기서부터 **이 스레드가 200Hz 실시간 루프**가 된다 (SCHED_FIFO 는 안에서 걸린다).
    // ROS 콜백은 `bridge_node->Start()` 가 띄운 spin 스레드에서 따로 돈다.
    // §3.1: 제어 INIT 검증 실패 시 종료 코드 ≠ 0 (실험 스크립트가 기동 실패를 감지해야 함)
    const int exit_code = scheduler.MainLoopStart();

    if (exit_code == 0) std::cout << "Test Finished Safely." << std::endl;
    else                std::cerr << "Node exited with error (INIT 검증 실패)." << std::endl;
    rclcpp::shutdown();
    return exit_code;
}
