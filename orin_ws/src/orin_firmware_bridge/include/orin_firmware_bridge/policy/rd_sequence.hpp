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

    // ── 카메라 캡처 (CAMERA_ACTION, 2026-08-12 추가) ──
    //
    // dpy_camera 노드의 `std_srvs/Trigger` 서비스(`/dpy_camera/capture`)를 비동기로
    // 부른다. 200Hz RT 스레드(rd_schedule.cpp)에서 Tick() 이 돌므로 여기서 절대
    // 블로킹하면 안 된다 — PostAutoWriteTo/AutoCommandDone 과 같은 "발사 후 폴링" 형태다.
    //
    // 반환 false = 요청 자체를 못 보냄(서비스 미준비 등). 슬롯 확보 실패와 같은 의미다.
    virtual bool TriggerCameraCapture() = 0;
    // 직전 TriggerCameraCapture() 가 끝났는가. 끝났으면 *ok 에 서비스 응답의 success.
    virtual bool CameraCaptureDone(bool* ok) const = 0;
};

class RdSequence {
public:
    explicit RdSequence(ISlotHost* host) : host_(host) {}

    void SetLogger(ILogger* lg) { log_ = lg; }

    // jeongae 토픽 수신 (open=true). lock 중이면 무시된다.
    void Trigger();
    void SetLock(bool lock);
    bool GetLock() const { return lock_.load(); }

    // **200Hz(5ms)** 로 스케줄러가 호출한다 — `rd_schedule.cpp` 의 project/manual 분기에
    // 레이트 분주가 없다. 상태 전이는 여기서만 일어난다.
    // (구 주석은 "5Hz" 였고 그 오해가 kWaitTicksMax 를 30초가 아니라 0.75초로 만들었다.)
    void Tick();

    // 진행 중인가 — GET_STATUS·진단용.
    bool Busy() const;

    // §3.2 전개 단계. DPC 쪽 상태값은 `dpc::STATE_*` (rd_register_dpc.hpp) 다.
    //
    //   CTRL ──target=INIT──▶ INIT ▶ DESCEND_1 ▶ DESCEND_2 ▶ WAIT(★카메라 위치)
    //                                                          │ target=ASCEND_1
    //   CTRL ◀─자동복귀─ FINISH ◀ ASCEND_2 ◀ ASCEND_1 ◀─────────┘
    //
    // 브리지가 `sys_state_target`(127) 에 쓰는 것은 **두 번뿐**이다: 전개 시작(INIT)과
    // 회수 시작(ASCEND_1). 나머지는 DPC 가 스스로 밟고, 브리지는 `sys_state` 로 따라간다.
    //
    // ## ⚠ 그 전에 `mode`(126)=AUTO 를 써야 한다 (09 §3.2, 2026-08-05 펌웨어 확인)
    //
    // DPC 의 전개 FSM 은 **`CTL->MODE == 1`(AUTO) 일 때만 돈다** (`rd_control.c:111`).
    // MANUAL 이면 `RD_CONTROL_CASE_IDLE` 만 돌고 `STATE` 는 읽히지도 않는다.
    // 그런데 `sys_state`(57)는 `DPC_CTL.STATE` 를 **그대로 복사해 발행**하므로
    // (`rd_map_dpcb.c:184`) — **MANUAL 에서 127에 2를 쓰면 sys_state 가 2로 읽히는데
    // 아무것도 안 움직인다.** 시퀀스는 그걸 "전개 중" 으로 보고 30초 기다리다 Abort 한다.
    // 관측값이 정상 진행과 완전히 같아 **증상만으로는 원인을 알 수 없다.**
    //
    // `mode` 는 **읽을 수 없다** (소비 후 0xFF 로 되돌리는 트리거이고, `DPC_CTL.MODE` 를
    // 발행하는 R/O 필드가 레지스터 맵에 없다). "확인 후 진행" 이라는 선택지가 없으므로
    // **매번 쓴다.** 조작자가 패널로 고른 MANUAL 을 덮어쓰는 것이 이 결정의 대가다.
    enum class Seq : uint8_t {
        IDLE = 0,
        ESTOP_SET,         // ECU WRITE 189=0 → 성공 시 50Hz 정지
        DPC_STATE_CHECK,   // DPC 가 읽히는가 + CTRL/HOLD 인가 (ERROR·전개중이면 Abort)
        DPC_SET_AUTO,      // DPC WRITE 126 = MODE_AUTO(1) — **이게 없으면 FSM 이 안 돈다**
        DPC_DEPLOY,        // DPC WRITE 127 = STATE_INIT(2)
        DPC_WAIT_CAMERA,   // DPC sys_state == STATE_WAIT(5) — FSM 이 여기서 멈춰 기다린다
        CAMERA_ACTION,     // dpy_camera `/dpy_camera/capture`(Trigger) 호출 후 응답 대기
        DPC_RETRACT,       // DPC WRITE 127 = STATE_ASCEND_1(6)
        DPC_WAIT_RETRACT,  // DPC sys_state == FINISH(8) 또는 CTRL(0) 자동복귀
        ESTOP_RELEASE,     // ECU WRITE 189=1 → 성공 시 50Hz 재개
    };
    Seq State() const;
    static const char* SeqName(Seq s);

    // GET_STATUS 용 스냅샷 (09 §5.3 ④, U12). **셋을 한 번에 뜬다** — 따로 물으면
    // 그 사이에 Tick 이 끼어들어 "단계는 DPC_DEPLOY 인데 wait_ticks 는 다음 단계 것"
    // 같은 섞인 값이 나간다. 화면은 그걸 진행 상황으로 읽는다.
    struct Snapshot_t {
        Seq      seq        = Seq::IDLE;
        uint32_t wait_ticks = 0;
        bool     locked     = false;
    };
    Snapshot_t SnapshotState() const;

    // 대기 상한까지 남은 tick — 화면이 "얼마나 더 기다리나" 를 말할 수 있게.
    static constexpr uint32_t WaitTicksMax() { return kWaitTicksMax; }

    // 상태 대기 상한. 넘으면 Abort — 전개 도중 매달려 있는 것이 가장 나쁘다.
    //
    // ## ⚠ tick 은 **200Hz(5ms)** 다 — 5Hz 가 아니다 (2026-08-06, U12 실기에서 발견)
    //
    // 종전 값 150 은 *"5Hz 기준 30초"* 라는 주석과 함께 있었다. **둘 다 틀렸다.**
    // `TickAutoSequence()` 를 부르는 곳은 `rd_schedule.cpp` 의 project/manual 분기이고
    // **레이트 분주가 없다** — 200Hz 루프의 매 tick 마다 불린다. 그래서 실효 상한이
    // 150 × 5ms ≈ **0.75초**였고, 전개를 걸면 `DPC_WAIT_CAMERA` 에서 곧바로 Abort 했다.
    // DPC 가 정상 동작하더라도 **0.75초 안에 안 끝나는 동작은 무조건 실패**한다.
    //
    // 실기에서 `wait_ticks` 가 151 까지 오르는 데 10초가 안 걸리는 것으로 확인했다.
    // (U12 가 `wait_ticks` 를 GET_STATUS 로 꺼내면서 비로소 보였다 — 그전에는 "타임아웃이
    //  걸렸다" 는 사실만 있고 얼마 만에 걸렸는지는 아무 데도 안 나왔다.)
    //
    // → **분주를 넣지 않고 상수를 고친다** (사용자 결정 A). 이 FSM 은 통신을 하지 않고
    //   상태 전이만 하므로 빨리 도는 것이 해롭지 않고, 분주를 넣으면 오히려 반응이
    //   200ms 단위로 굼떠진다.
    //
    // ## ⚠ 이 상한은 **"국면"당**이지 "DPC 상태"당이 아니다
    //
    // `wait_ticks_` 는 **브리지 단계**에 진입할 때 리셋되고(219·243행), 대기 중에 DPC 가
    // 중간 상태를 밟는 것으로는 **리셋되지 않는다** — `WaitDpcStateLocked` 는 목표에
    // 도달했을 때만 0 으로 되돌린다. 그런데 브리지의 `DPC_WAIT_CAMERA` 하나가 DPC 의
    // `INIT(2)→DESCEND_1→DESCEND_2→WAIT(5)` **전 구간**을 덮는다.
    //
    //   DPC_WAIT_CAMERA   ← 2→3→4→5 **전체**가 이 상한 안에 들어와야 한다
    //   DPC_WAIT_RETRACT  ← 6→7→8(또는 0) **전체**
    //
    // 즉 **"한 상태에서 기다리는 시간" 이 아니라 "그 국면이 끝나는 데 걸리는 시간"** 이다.
    // 이 구분이 애매해서 처음에 10초로 잡았고, 그러면 실제 전개(≈40초)가 무조건 실패한다.
    //
    // **실측 기준 (사용자, 2026-08-07): 실제 전개 동작 ≈ 40초.** 여유를 20초 두어 60초.
    //
    // 200Hz × 12000 tick = **60초**.
    static constexpr uint32_t kTickHz        = 200;
    static constexpr uint32_t kWaitTimeoutS  = 60;
    static constexpr uint32_t kWaitTicksMax  = kTickHz * kWaitTimeoutS;   // 12000

    // CAMERA_ACTION 재시도 간격 (2026-08-12, 사용자 결정) — 촬영 실패 시 **곧바로 회수하지
    // 않고** DPC 를 WAIT(5) 에 세워 둔 채 이 간격마다 촬영을 다시 시도한다. 5초마다
    // 재시도해 로그 스팸 없이 카메라가 살아나길 기다린다.
    static constexpr uint32_t kCameraRetrySec   = 5;
    static constexpr uint32_t kCameraRetryTicks = kTickHz * kCameraRetrySec;   // 1000
    // CAMERA_ACTION 포기 상한 (2026-08-12) — 재시도를 무한히 하지 않는다. kCameraMaxAttempts
    // 회(서비스 미준비/촬영 실패 합산)를 넘기면 촬영 없이 회수로 진행한다 — 카메라가
    // 영영 안 살아나는 경우까지 DPC 를 세워 두면 그게 더 나쁘다.
    static constexpr uint32_t kCameraMaxAttempts = 5;

private:
    void AbortLocked(const char* reason);
    // DPC 목표 상태를 쓰고 다음 단계로. 실패 시 Abort.
    bool PostDpcTargetLocked(uint8_t target_state, const char* what);
    // sys_state 가 want 가 될 때까지 대기. 도달=true / 계속=false / 실패는 Abort.
    // `also_ok` 는 함께 받아 줄 두 번째 값 (자동 복귀처럼 놓치기 쉬운 상태용).
    bool WaitDpcStateLocked(uint8_t want, const char* what, int also_ok = -1);
    // CAMERA_ACTION 포기(kCameraMaxAttempts 초과) — 촬영 없이 회수로 진행. Abort 와
    // 다르다: jeongae 시퀀스 자체는 실패가 아니라 정상적으로 회수까지 이어간다.
    void GiveUpCameraLocked(const char* why);

    ISlotHost* host_;
    ILogger*   log_ = nullptr;

    mutable std::mutex mutex_;
    Seq  seq_ = Seq::IDLE;
    uint32_t wait_ticks_ = 0;      // 현재 대기 단계에서 보낸 tick 수
    // CAMERA_ACTION 전용 — true 면 "이번엔 아직 트리거를 못 보냈다" (서비스 미준비로
    // 재시도 대기 중이거나, 직전 촬영 실패로 재시도를 기다리는 중). false 면 요청을
    // 보내고 응답(CameraCaptureDone)을 기다리는 중.
    bool camera_attempt_pending_ = false;
    // 이번 WAIT 진입 이후 시도한 횟수 (서비스 미준비 재시도 + 촬영 실패 재시도 합산).
    // kCameraMaxAttempts 도달 시 포기. DPC_WAIT_CAMERA 진입 시 0으로 리셋.
    uint32_t camera_attempts_ = 0;

    std::atomic<bool> trigger_{false};
    std::atomic<bool> lock_{false};   // §2.4: orin 기본 변수, default 0
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_SEQUENCE_HPP_
