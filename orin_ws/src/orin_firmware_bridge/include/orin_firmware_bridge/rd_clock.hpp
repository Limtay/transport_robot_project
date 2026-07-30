#ifndef ORIN_FIRMWARE_BRIDGE__RD_CLOCK_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_CLOCK_HPP_

// IClock / IRunGate — 시계와 실행 게이트 추상화 (redesign/02 A5)
//
// 왜 두 개인가: L1/L2 가 rclcpp 에 남아 있던 고리는 사실 두 종류다.
//   1) 시각      — `steady_clock`(주기 제어) / `system_clock`(§2.5 t_req·t_resp, ROS time 축)
//   2) 실행 여부 — `rclcpp::ok()` (Ctrl-C 등 ROS context 종료 감지)
// 성격이 다르므로 섞지 않는다. 시계는 테스트에서 가짜 시간을 넣기 위한 것이고,
// 게이트는 "언제 멈추는가" 라는 수명 관심사다.

#include <chrono>

namespace orin_bridge {

class IClock {
public:
    virtual ~IClock() = default;

    // 단조 시계 — 주기 제어·경과시간 측정용. 벽시계 점프에 영향받지 않는다.
    virtual std::chrono::steady_clock::time_point NowSteady() const = 0;

    // 벽시계 epoch [s] — ROS time 과 **같은 축**이어야 한다 (testbed_spec §2.5 시간동기).
    virtual double NowEpoch() const = 0;
};

// 실행 지속 여부. L3 가 ROS context 를 물려준다 (`rclcpp::ok()`).
class IRunGate {
public:
    virtual ~IRunGate() = default;
    virtual bool Ok() const = 0;
};

// ── 기본 구현 (rclcpp 없음) — 단위 테스트·비 ROS 실행 경로용 ──

class SystemClock : public IClock {
public:
    std::chrono::steady_clock::time_point NowSteady() const override {
        return std::chrono::steady_clock::now();
    }
    double NowEpoch() const override {
        return std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch()).count();
    }
};

// 항상 실행 — ROS 없이 도는 경로(계측기·단위 테스트)에서 쓴다.
class AlwaysOkGate : public IRunGate {
public:
    bool Ok() const override { return true; }
};

// 주입을 깜빡해도 널 역참조로 죽지 않도록 하는 폴백. 절대 nullptr 을 돌려주지 않는다.
IClock*   DefaultClock();
IRunGate* DefaultRunGate();

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_CLOCK_HPP_
