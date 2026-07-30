// 제어 하네스 공통 유틸 (testbed_spec.md 검증)
//
// 이 디렉터리의 테스트는 **ECU 하드웨어 없이** bridge 를 검증한다:
// 노드를 직접 구성하고, 스케줄 루프가 하는 일(TickProfile / PrepareControlCommand /
// TakeOutOfSpanStep)을 테스트 스레드가 대신 호출해 200Hz tick 을 모사한다.
// 시리얼 구간만 가짜 ECU 로 치환되고, 그 위의 로직은 실제 코드가 그대로 돈다.
#ifndef RD_TEST_COMMON_HPP_
#define RD_TEST_COMMON_HPP_

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <string>
#include <unistd.h>

using namespace std::chrono_literals;

// 이 테스트들은 실제 bridge 와 **같은 토픽·서비스 이름**을 쓴다. colcon test 는 패키지를
// 병렬로 돌리므로 도메인을 분리하지 않으면 다른 테스트 프로세스(특히 control_cli 의 가짜
// bridge)와 서로를 발견해 엉뚱한 상대에게 요청이 간다 — 실제로 전체 실행 시 재현됐다.
// PID 기반으로 도메인을 갈라 프로세스마다 독립 DDS 그래프를 갖게 한다.
inline void IsolateDdsDomain() {
    const int domain = static_cast<int>(getpid() % 90) + 10;   // 10~99
    setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);                      // 같은 LAN 의 로봇과도 격리
}

// rclcpp 컨텍스트는 실행파일당 1회만 초기화한다 (테스트 간 재init 은 DDS 를 흔든다).
class RclcppEnv : public ::testing::Environment {
public:
    void SetUp() override    { IsolateDdsDomain(); if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
    void TearDown() override { if (rclcpp::ok()) rclcpp::shutdown(); }
};
inline void RegisterRclcppEnv() {
    static bool once = [] {
        ::testing::AddGlobalTestEnvironment(new RclcppEnv());
        return true;
    }();
    (void)once;
}

// 조건이 참이 될 때까지 최대 timeout 만큼 대기 (폴링). 타이밍 의존 테스트의 flaky 방지용 —
// 고정 sleep 은 느린 머신에서 깨지고 빠른 머신에서 시간을 낭비한다.
template <typename Fn>
bool WaitFor(Fn&& cond, std::chrono::milliseconds timeout = 3000ms,
             std::chrono::milliseconds poll = 5ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (cond()) return true;
        std::this_thread::sleep_for(poll);
    }
    return cond();
}

#endif
