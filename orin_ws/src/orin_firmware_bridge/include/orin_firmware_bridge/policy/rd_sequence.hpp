#ifndef ORIN_FIRMWARE_BRIDGE__RD_SEQUENCE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_SEQUENCE_HPP_

// RdSequence — jeongae 자동 전개 FSM (redesign/02 A3)
//
// `rd_command` 에서 분리한다. 그 파일은 **두 가지 일**을 하고 있었다:
//   ① 커맨드 슬롯 관리 (SET/RESET·만료·blackout·우선순위 차용)
//   ② jeongae 전개 시퀀스 (ESTOP → DPC 전개 → 카메라 → 회수 → ESTOP 해제)
// ②는 ①의 **사용자**이지 일부가 아니다. 섞여 있으면 슬롯 로직을 고칠 때 시퀀스가
// 딸려 오고, 시퀀스를 테스트하려면 슬롯 전체를 띄워야 한다.
//
// ## 경계 — 시퀀스는 슬롯을 "빌려 쓴다"
//
// 시퀀스가 필요로 하는 것은 셋뿐이다: **명령 하나 보내기 / 끝났는지 묻기 / 접기.**
// 그 셋만 `ISlotHost` 로 두면 시퀀스는 슬롯의 내부(우선순위·차용·복귀)를 몰라도 된다.
//
// ⚠ DPC 단계는 여전히 TODO 다 — 레지스터가 확정되지 않았다 (04 §6.3 C5).
//    **분리와 구현은 별개다.** 여기서는 옮기기만 하고 TODO 는 그대로 둔다.

#include <atomic>
#include <cstdint>
#include <mutex>

#include "orin_firmware_bridge/rd_logger.hpp"

namespace orin_bridge {

// 시퀀스가 슬롯 관리자에게 요구하는 전부. RdCommand 가 구현한다.
class ISlotHost {
public:
    virtual ~ISlotHost() = default;

    // `target`(TARGET::ECU / TARGET::DPC) 의 addr 에 1바이트를 쓰는 once 명령을
    // **자동 슬롯**으로 예약한다. 실패하면 false (빈 슬롯이 없고 차용도 불가한 경우).
    virtual bool PostAutoWriteTo(uint8_t target, uint16_t addr, uint8_t value) = 0;

    // ECU 전용 단축형 — 기존 호출부 호환.
    bool PostAutoWrite(uint16_t addr, uint8_t value);

    // 직전 PostAutoWrite* 가 끝났는가. 끝났으면 *ok 에 성공 여부.
    virtual bool AutoCommandDone(bool* ok) const = 0;

    // 50Hz cmd_vel 정기 write 정지/재개 (soft ESTOP 구간 동안).
    virtual void SetCmdVelPaused(bool paused) = 0;
    virtual bool IsCmdVelPaused() const = 0;

    // ── DPC 상태 조회 (2026-07-30 레지스터 확정 후 추가) ──
    //
    // ⚠ **신선도를 반드시 함께 준다.** DPC 섀도는 0으로 초기화되는데
    //   `sys_state == 0` 은 `CTRL`(정상 기본값)이라, 한 번도 안 읽은 상태와
    //   "정상 대기 중" 이 **값으로 구분되지 않는다.** 값만 돌려주면 시퀀스가
    //   읽지도 않은 0을 보고 "이미 도착했다" 고 판정한다.
    //   (같은 함정을 IMU 에서 이미 한 번 밟았다 — 08 §8.2)
    virtual bool DpcSysState(uint8_t* out) const = 0;
};

class RdSequence {
public:
    explicit RdSequence(ISlotHost* host) : host_(host) {}

    void SetLogger(ILogger* lg) { log_ = lg; }

    // jeongae 토픽 수신 (open=true). lock 중이면 무시된다.
    void Trigger();
    void SetLock(bool lock);
    bool GetLock() const { return lock_.load(); }

    // 5Hz 로 스케줄러가 호출. 상태 전이는 여기서만 일어난다.
    void Tick();

    // 진행 중인가 — GET_STATUS·진단용.
    bool Busy() const;

    // §3.2 전개 단계. DPC 쪽 상태값은 `dpc::STATE_*` (rd_register_dpc.hpp) 다.
    //
    //   CTRL ──target=INIT──▶ INIT ▶ DESCEND_1 ▶ DESCEND_2 ▶ WAIT(★카메라 위치)
    //                                                          │ target=ASCEND_1
    //   CTRL ◀─자동복귀─ FINISH ◀ ASCEND_2 ◀ ASCEND_1 ◀─────────┘
    //
    // 브리지가 쓰는 것은 **두 번뿐**이다: 전개 시작(INIT)과 회수 시작(ASCEND_1).
    // 나머지는 DPC 가 스스로 밟고, 브리지는 `sys_state` 로 따라간다.
    enum class Seq : uint8_t {
        IDLE = 0,
        ESTOP_SET,         // ECU WRITE 189=0 → 성공 시 50Hz 정지
        DPC_STATE_CHECK,   // DPC 가 읽히는가 + ERROR 아닌가 + 이미 전개 중이 아닌가
        DPC_DEPLOY,        // DPC WRITE 127 = STATE_INIT(2)
        DPC_WAIT_CAMERA,   // DPC sys_state == STATE_WAIT(5) — FSM 이 여기서 멈춰 기다린다
        CAMERA_ACTION,     // TODO: deploy camera ROS2 Action — **카메라 미연결**
        DPC_RETRACT,       // DPC WRITE 127 = STATE_ASCEND_1(6)
        DPC_WAIT_RETRACT,  // DPC sys_state == FINISH(8) 또는 CTRL(0) 자동복귀
        ESTOP_RELEASE,     // ECU WRITE 189=1 → 성공 시 50Hz 재개
    };
    Seq State() const;
    static const char* SeqName(Seq s);

    // 상태 대기 상한. 넘으면 Abort — 전개 도중 매달려 있는 것이 가장 나쁘다.
    // 5Hz tick 기준 (§3.2). 기본 30초.
    static constexpr uint32_t kWaitTicksMax = 150;

private:
    void AbortLocked(const char* reason);
    // DPC 목표 상태를 쓰고 다음 단계로. 실패 시 Abort.
    bool PostDpcTargetLocked(uint8_t target_state, const char* what);
    // sys_state 가 want 가 될 때까지 대기. 도달=true / 계속=false / 실패는 Abort.
    // `also_ok` 는 함께 받아 줄 두 번째 값 (자동 복귀처럼 놓치기 쉬운 상태용).
    bool WaitDpcStateLocked(uint8_t want, const char* what, int also_ok = -1);

    ISlotHost* host_;
    ILogger*   log_ = nullptr;

    mutable std::mutex mutex_;
    Seq  seq_ = Seq::IDLE;
    uint32_t wait_ticks_ = 0;      // 현재 대기 단계에서 보낸 tick 수

    std::atomic<bool> trigger_{false};
    std::atomic<bool> lock_{false};   // §2.4: orin 기본 변수, default 0
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_SEQUENCE_HPP_
