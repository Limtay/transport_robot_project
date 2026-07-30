#ifndef ORIN_FIRMWARE_BRIDGE__RD_CONTROL_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_CONTROL_HPP_

#include <mutex>
#include <string>
#include <cstdint>

#include <atomic>

#include "orin_firmware_bridge/rd_clock.hpp"
#include "orin_firmware_bridge/rd_logger.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

namespace orin_bridge {

// 제어 FSM (testbed_spec.md §2)
//
// 핵심 불변식: RW 자체는 모든 상태에서 계속 돈다 (read 스트림 = 피드백 발행 유지).
// 상태가 결정하는 것은 오직 **write 소스** 뿐이다.
//
//   INIT    (루프 시작 전)          motor_mask 검증 성공 → IDLE / 실패 → 노드 종료 (§3.1, C-3)
//   IDLE    0A 고정                 기본 상태. config service 전부 허용
//   RUNNING profile(t)              프로파일 goal 수락 (C-4). config 거부
//   STREAM  최신 /carrier/cmd_motor (Task 3 예약 — 이번 범위 밖). config 거부
//   LOCKED  0A 래치                 스테일·write 연속 거부 등. config REARM 만 허용
//
// LOCKED 는 "모터락" 아이디어의 구현 — 0A 로 조용히 자동 복귀하지 않고 원인 확인 후
// 명시적 재무장(REARM)을 요구한다. ECU 측 mtr_lock/AUTO_TIMEOUT 은 그 뒤의 최후방 방어선.
enum class ControlState : uint8_t {
    INIT    = 0,
    IDLE    = 1,
    RUNNING = 2,
    STREAM  = 3,
    LOCKED  = 4,
};

const char* ControlStateStr(ControlState s);

// write 소스 선택 결과 — 스케줄 루프가 매 tick Encode 직전에 받아 shadow reg 에 반영.
// 스트림 명령 스냅샷 (`/carrier/control/cmd_motor`). L3 구독자가 넣고 FSM 이 소비한다.
// **arm=on 일 때만** 소비된다 (01 §6.1.3) — off 면 구독은 하되 버린다.
struct StreamCmd_t {
    uint8_t ctr_mode[4] = {0, 0, 0, 0};
    float   position[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // [deg]
    float   velocity[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // [RPM]
    float   current[4]  = {0.0f, 0.0f, 0.0f, 0.0f};   // [A]
    double  stamp       = 0.0;                        // 수신 epoch [s] — 스테일 판정 기준
};

struct ControlWrite_t {
    // 단위는 **auto_mode 가 결정**한다 (A / RPM / deg). 이름이 current 인 것은 이력이며,
    // PrepareWrite 가 auto_mode 별 레지스터에 꽂는다 (01 §6.1.2).
    float    current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t goal_id      = 0;      // 활성 프로파일 (0 = 없음) — 피드백 태그
    float    profile_time = 0.0f;   // 프로파일 경과 [s]
    uint16_t segment_index = 0;
};

// ⚠ **락 순서 규약**: `shadow_->state_mutex` -> `mutex_` -> `ctr_mutex_`.
//    PrepareWrite 가 state_mutex 를 쥔 채 mutex_ 를 잡으므로, 반대 순서로 잡는 경로가
//    생기면 즉시 데드락이다. SafeStop 이 mutex_ 를 **스코프로 먼저 놓고** state_mutex 를
//    잡는 것은 이 규약을 지키기 위한 것이다 — 붙여 쓰면 안 된다.
//
// FSM 상태·전이 소유자. 스케줄 루프(200Hz)와 service/action 콜백(spin 스레드)이
// 동시 접근하므로 전 메서드가 mutex_ 로 보호된다 — 상태 변수는 이 클래스 밖으로 새지 않는다.
class RdControl {
public:
    // 로거 주입 (A5). 미주입이면 RdLogf 가 DefaultLogger 로 폴백한다.
    void SetLogger(ILogger* lg) { log_ = lg; }
    // 스테일 판정에 시계가 필요하다 (A5). 미주입이면 DefaultClock 으로 폴백.
    void SetClock(IClock* c) { clock_ = c; }

    // ── write 값 선택 (02 §5.2: rd_control 이 "write 값 선택" 을 소유한다) ──
    // shadow·클램프 상한은 L3 가 1회 주입. L2 는 BridgeConfig 를 직접 읽지 않는다.
    void Bind(RobotState_t* shadow, float clamp_max) { shadow_ = shadow; clamp_max_ = clamp_max; }

    // 재생 중 유효한 클램프 [lo, hi] — **단위는 프로파일의 mode 가 정한다** (05 §2.4).
    // goal 수락 시 1회 주입. RUNNING 이 아닐 때는 쓰이지 않는다 (IDLE 안전값은 §6.1.2).
    // 구 코드는 wire 직전에 무조건 cmd_current_max([A])를 썼다 — position 프로파일이
    // 각도째 잘렸다.
    void SetProfileLimits(float lo, float hi) { prof_lo_.store(lo); prof_hi_.store(hi); }

    // §2.6 auto_mode — write 범위가 여기서 파생된다. config service 로 런타임 변경되므로 atomic.
    uint8_t AutoMode() const { return auto_mode_.load(); }
    void    SetAutoMode(uint8_t m) { auto_mode_.store(m); }

    // in-span(ctr_mode 128~131) 섀도 — IDLE 에서 PrepareWrite 가 이 값을 쓴다.
    void SetCtrMode(int idx, uint8_t mode);
    uint8_t CtrMode(int idx) const;

    // [스케줄 스레드] 매 tick — FSM 이 고른 write 값을 shadow 에 반영한다.
    // 구 RdNode::PrepareControlCommand (02 A1-c 로 L3 에서 이동).
    void PrepareWrite();

    ControlState State() const;
    bool IsRunning() const { return State() == ControlState::RUNNING; }

    // INIT → IDLE. §3.1 INIT 플로우(motor_mask/mode 검증) 성공 시 호출 (C-3).
    void MarkInitDone();

    // IDLE → RUNNING. 프로파일 goal 수락 시 (C-4). IDLE 이 아니면 false 반환 —
    // 동시 goal 불가 규칙(§3.2)이 여기서 걸린다.
    bool BeginProfile(uint32_t goal_id);
    // RUNNING → IDLE (완료/취소/에러 공통). 되돌아갈 때 write 소스는 0A 로 복귀.
    void EndProfile();

    // 어느 상태에서든 LOCKED 로 래치. reason 은 REARM 응답·로그에 실린다.
    void Lock(const std::string& reason);
    // LOCKED → IDLE (config REARM 전용). LOCKED 가 아니면 false.
    bool Rearm();
    std::string LockReason() const;

    // config service 수락 여부 (§2 표 "허용 입력" 열).
    // RUNNING/STREAM = 전부 거부 (실험 오염 방지), LOCKED = REARM 만.
    bool AcceptsConfig(bool is_rearm) const;

    // RW write 거부 스트릭 감시 — 연속 kWriteErrLockStreak tick 거부 시 LOCKED 래치.
    // 스케줄 루프가 매 tick 호출 (streak 는 RdMap 이 세는 누적 카운터).
    void NoteWriteErrStreak(uint64_t streak);

    // ── STREAM / arm (01 §6.1.3 — 웹 run/stop 버튼) ──
    //
    // arm 을 요구하는 이유: STREAM 을 "메시지가 도착하면 RUNNING" 으로 두면, 노드 기동 직후
    // 남아 있던 발행자(웹 탭, 죽다 만 MPC)가 **조작자 모르게 모터를 움직인다.**
    //
    // IDLE 에서만 on 가능. 실패 시 false + why 에 사유.
    bool SetStreamArm(bool on, std::string* why);
    bool StreamArmed() const { return stream_armed_.load(); }
    // [L3 구독자 스레드] 최신 스트림 명령 반영. arm 과 무관하게 받아 두고, 소비 여부는 FSM 이 정한다.
    void PushStreamCommand(const StreamCmd_t& c);
    // 스테일 한계 [s]. L3 가 BridgeConfig 에서 1회 주입한다.
    void SetStreamTimeout(double sec) { stream_timeout_ = sec; }

    // ── in-span 1회성 WRITE = 펄스 (01 §6.3 C-1) ──
    //
    // SET_ORIGIN(AK mode 5)이 이 분류를 만들게 했다: **동작은 1회성인데 그것을 담는 그릇
    // (ctr_mode, addr 128)은 매 tick 전송되는 레벨 레지스터**다. "지속" 으로 처리하면
    // shadow 의 5 가 매 tick 나가고, 원점이 매번 다시 잡히면 위치 제어가 적분기처럼 굴러버린다.
    //
    // 규약: 다음 tick 에 **무조건** 원복한다. write_err 여부와 무관 — 실패해도 두 번 보내지
    //       않는다 (엣지 명령을 재시도하면 의도가 두 번 실행된다).
    // 결과 확인은 다음 트랜잭션의 read 로 한다 (fb_position 이 0 근처인지).
    bool RequestOriginPulse(std::string* why);
    bool OriginPulsePending() const;

    // safe_stop (01 §6.2) — 설정 변경 WRITE 의 게이트.
    //   ControlState in {IDLE, LOCKED} and |vel|<5RPM and |cur|<1A and 직전 write 정상
    // 위반 시 why 에 **위반한 조건을 명시**한다 — 원인 불명 실패를 만들지 않는다.
    bool SafeStop(std::string* why) const;

    // 매 tick Encode 직전 — 현재 상태의 write 소스를 확정해 반환.
    // IDLE/LOCKED/INIT = 0A, RUNNING = 프로파일 샘플(C-4 에서 주입), STREAM = Task 3.
    ControlWrite_t SelectWrite();

    // RUNNING 중 프로파일 재생기가 매 tick 갱신 (C-4). RUNNING 이 아니면 무시.
    void SetProfileSample(const float current[4], float t, uint16_t seg_idx);

    // 연속 write 거부 tick 수 — 이 값을 넘으면 LOCKED (200Hz 기준 0.25s).
    static constexpr uint64_t kWriteErrLockStreak = 50;

private:
    ILogger* log_   = nullptr;
    IClock*  clock_ = nullptr;

    // STREAM 상태 (01 §6.1.3)
    std::atomic<bool> stream_armed_{false};
    StreamCmd_t       stream_cmd_;                 // mutex_ 보호
    double            stream_timeout_ = 0.1;       // [s] — cmd_vel_topic_timeout 과 같은 기본값
    // arm on 상태에서 스트림이 신선한지 확인하고 상태를 전이시킨다. mutex_ 를 이미 쥔 채 호출.
    void UpdateStreamStateLocked();

    // 펄스 (01 §6.3 C-1). 동시 1건.
    enum class PulseState : uint8_t { NONE = 0, ARMED, SENT };
    PulseState pulse_ = PulseState::NONE;
    uint8_t    pulse_restore_[4] = {0, 0, 0, 0};   // 직전 ctr_mode — 원복값
    uint64_t   last_write_streak_ = 0;             // NoteWriteErrStreak 가 갱신

    static constexpr float kSafeStopVelRpm = 5.0f;
    static constexpr float kSafeStopCurA   = 1.0f;
    RobotState_t* shadow_ = nullptr;  // ECU 섀도 (L3 가 Bind 로 주입). FSM 의 state_ 와 다른 것.
    float clamp_max_ = 30.0f;         // [A] 전역 클램프 (IDLE·전류 모드)
    std::atomic<float> prof_lo_{0.0f};   // 재생 중 실효 클램프 (05 §2.4)
    std::atomic<float> prof_hi_{0.0f};
    std::atomic<uint8_t> auto_mode_{ecu::AUTO_MODE_CURRENT};
    mutable std::mutex ctr_mutex_;
    uint8_t ctr_mode_[4] = {ecu::CTR_MODE_CURRENT, ecu::CTR_MODE_CURRENT,
                            ecu::CTR_MODE_CURRENT, ecu::CTR_MODE_CURRENT};
    mutable std::mutex mutex_;
    ControlState state_       = ControlState::INIT;
    std::string  lock_reason_;
    uint32_t     goal_id_     = 0;

    // RUNNING 중 재생기가 채우는 현재 tick 샘플 (mutex_ 보호)
    float    profile_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float    profile_time_       = 0.0f;
    uint16_t segment_index_      = 0;
};

} // namespace orin_bridge

#endif
