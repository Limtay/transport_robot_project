#ifndef ORIN_FIRMWARE_BRIDGE__RD_SCHEDULE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_SCHEDULE_HPP_

#include <chrono>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <pthread.h>
#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/sched/rd_telemetry_sink.hpp"
#include "orin_firmware_bridge/policy/rd_command.hpp"
#include "orin_firmware_bridge/core/rd_clock_sync.hpp"
#include "orin_firmware_bridge/rd_clock.hpp"
#include "orin_firmware_bridge/rd_config.hpp"
#include "orin_firmware_bridge/policy/rd_oos.hpp"
#include "orin_firmware_bridge/policy/rd_profile_player.hpp"
#include "orin_firmware_bridge/rd_logger.hpp"
#include "orin_firmware_bridge/sched/rd_slot_table.hpp"

namespace orin_bridge {

// ===== 스케줄 프레임 (Code_modify.md §1) =====
// 200Hz tick (5ms), 프레임 = 40 tick (200ms)
//   짝수 tick           : 100Hz | ECU READ 62~127 (66B, IMU+ENC+UART2+RC+MOTOR)
//   홀수 tick odd_idx 짝 :  50Hz | ECU WRITE 180~187 (8B, cmd_lin/ang_vel)
//   홀수 tick odd_idx 홀 : 서브 슬롯 10개 순환 (200ms 1회전)
//       [E10, PCU, DPC, C1, C2, E10, PCU, DPC, C3, C4]
//       E10 = ECU READ 46~61 (10Hz) / PCU·DPC READ (10Hz, 레지스터 미정 TODO)
//       C1~C4 = 커맨드 슬롯 0~3 (각 5Hz)

class RdSchedule {
public:
    // log/clock/gate 는 A5 주입점. nullptr 이면 기본 구현으로 폴백한다
    // (stderr 로거 / system_clock / 항상 실행) — 단위 테스트는 이 폴백을 쓴다.
    // cfg 는 기동 1회 확정 후 불변이며 수명이 이 객체보다 길다 (A1-a) — 참조로 잡는다.
    RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state,
               ITelemetrySink* sink, RdCommand* command,
               const BridgeConfig& cfg, RdOos* oos, RdProfilePlayer* player,
               RdControl* control,
               // (d) L3 고유 관심사 2건은 콜백으로 주입한다 — L2 인 척 옮기지 않는다.
               //   on_init_done  : INIT 을 끝냈다. 인자는 **명령을 쓰는 구성인가** —
               //                   false 면 L3 가 조작 입구만 열고 재생 서버는 열지 않는다.
               //                   (읽기 전용에서 goal 을 수락하면 "재생됐다는데 아무것도
               //                    안 움직인다" 가 된다 — rd_control_api.hpp 참조)
               //   skip_cmd_write: cmd_vel 토픽 신선도 판정 (구 ShouldSkipCmdWrite)
               std::function<void(bool /*with_profiles*/)> on_init_done = nullptr,
               std::function<bool()> skip_cmd_write = nullptr,
               ILogger* log = nullptr, IClock* clock = nullptr, IRunGate* gate = nullptr);
    ~RdSchedule();

    // 루프의 **유일한** 입구. 호출한 스레드에서 그대로 돈다 (별도 스레드 경로 없음).
    // 프로세스 종료 코드를 반환 — §3.1 INIT 검증 실패는 재시도 없이 exit≠0 이어야 한다.
    int  MainLoopStart();
    void Stop();

    // 04 §2.4.2 읽기 프리셋 교체. **IDLE 판정은 호출자(config 서비스)의 몫**이고
    // 여기서는 id 범위만 본다 — 없는 프리셋으로 바꿨다고 응답하는 일이 없어야 한다.
    // 성공하면 why 에 적용된 프리셋 이름·응답 크기가 담긴다.
    bool    SetReadPreset(uint8_t id, std::string* why);
    // 이름을 ...Id 로 두는 이유: `ReadPreset` 은 타입 이름이라, 멤버 함수로 같은 이름을
    // 쓰면 멤버 본문 안에서 타입이 가려진다 (실제로 컴파일이 깨졌다).
    uint8_t ReadPresetId() const { return read_preset_.load(std::memory_order_relaxed); }

private:
    RdComm*  comm_;
    RdMap*   map_;
    RobotState_t* robot_state_;
    ITelemetrySink* sink_;
    RdCommand* command_;
    const BridgeConfig& cfg_;
    RdOos*           oos_;      // L2 직접 참조 (A1-c)
    RdProfilePlayer* player_;   // L2 직접 참조 (A1-c)
    RdControl*       control_;  // L2 직접 참조 (A1-c)
    std::function<void(bool)> on_init_done_;
    std::function<bool()> skip_cmd_write_;
    ILogger*   log_;
    IClock*    clock_;
    IRunGate*  gate_;

    void SupervisorLoop();
    RD_RET RunLoop();
    void ApplyRtScheduling(pthread_t thread);  // SCHED_FIFO + CPU 코어 고정

    // 스케줄링 슬롯
    // 프레임 길이는 **슬롯 테이블이 소유한다** (rd_slot_table.hpp frames::kProject.ticks).
    // 여기 상수로 두면 표와 두 곳에 생기고, 둘이 갈라져도 아무도 모른다.
    // **읽기·쓰기 구간도 표가 소유한다** (04 §2.2.1, 07 §3.1). 종전에는 task_100hz_ 처럼
    // 손으로 만든 멤버 5개가 구간을 들고 있어서, 표를 봐도 그 슬롯이 무엇을 읽는지 알 수
    // 없었다 (그리고 DPC·PCU 슬롯은 표현할 방법 자체가 없었다).
    // 여기서는 표를 tick 순서대로 펼쳐 **기동 시 한 번** TaskConfig_t 로 굽는다 —
    // 200Hz 스레드는 인덱싱만 한다 (04 §2.4.2 "런타임에 조립하지 않는다").
    TaskConfig_t project_task_[kMaxFrameTicks];
    // Q3 RW 폴백. project 의 ECU 슬롯은 RW(읽기+cmd_vel 쓰기)인데, cmd_vel 이 일시정지되거나
    // 안전장치가 걸리면 **쓰기만** 빠져야 한다. RW 는 한 패킷이라 게이트로 write 만 뺄 수
    // 없으므로, 같은 구간을 읽기만 하는 태스크를 미리 구워 두고 그 tick 만 갈아 끼운다.
    // (종전 40칸 프레임에서는 READ 와 WRITE 가 별개 tick 이라 그냥 건너뛰면 됐다.)
    TaskConfig_t project_task_read_[kMaxFrameTicks];
    // manual 은 별도 프레임(ECU READ)이라 따로 굽는다 — 01 §4.1.
    TaskConfig_t manual_task_[kMaxFrameTicks];
    // auto_mode: none 용 READ 태스크 — 프리셋에서 파생 (구 task_traction_).
    TaskConfig_t task_control_read_[ecu::kPresetCount];
    // §2.6 auto_mode 파생 write 범위 — **두 개를 생성자에서 미리 만들고 매 tick 골라 쓴다**.
    // 런타임에 TaskConfig_t 를 수정하면 200Hz 스레드와 레이스가 되므로 구조체 변경 금지.
    // (읽기 프리셋 × auto_mode) 조합을 기동 시 전부 만들어 둔다 (04 §2.4.2).
    // 교체는 atomic 인덱스만 바꾼다 — 200Hz 루프가 읽는 데이터를 건드리지 않는다.
    enum : uint8_t { kIdxCurrent = 0, kIdxDirect, kIdxVelocity, kIdxPosition, kIdxModeCount };
    TaskConfig_t task_control_[ecu::kPresetCount][kIdxModeCount];
    std::atomic<uint8_t> read_preset_{0};   // 기본 = kPresetControl (현행 배치 그대로)
    const TaskConfig_t& SelectControlTask(uint8_t auto_mode) const;
    // 07 §3.2 — control 프레임의 양보 tick 처리. 실행할 커맨드가 있었으면 true.
    bool RunUserSlot(RD_RET* ret_val);

    PACKET_comm_t packet_obj_;
    uint64_t tick_count_;
    uint64_t rx_count = 0;
    uint64_t tx_count = 0;

    // §2.5 시간 동기·지연 계측: ExecuteTask 가 매 트랜잭션 갱신, control 분기가
    // `sink_->OnTxn()` 으로 소비한다 (동일 스레드 — 락 불필요)
    static constexpr size_t kWireOverheadBytes = 8;  // Header2+ID1+Len2+Inst1+CRC2
    TxnTiming_t last_txn_;

    // 주기 타이밍 통계 — 헤더비트 구간(400 tick ≈ 2s)마다 집계 후 리셋.
    // 기존엔 한 tick만 샘플링했지만, 전체 구간 평균/최대/초과율로 확장.
    uint64_t stat_sum_us_  = 0;   // 구간 내 time_elapsed 합 (평균용)
    uint64_t stat_cnt_     = 0;   // 구간 내 tick 수
    int64_t  stat_max_us_  = 0;   // 구간 내 최대 time_elapsed
    uint64_t exceeded_cnt_ = 0;   // 구간 내 주기(5ms) 초과 횟수
    // 스파이크 원인 분리: wake(스케줄 깨어남 지연) vs proc(I/O 처리시간)
    int64_t  stat_wake_max_ = 0;  // 구간 내 최대 wake latency (sleep_until 오버슛)
    int64_t  stat_proc_max_ = 0;  // 구간 내 최대 처리시간 (ExecuteTask 등)
    // proc 스파이크를 시리얼 단계별로 분해 (어느 호출이 블록되는지)
    int64_t  stat_clear_max_ = 0; // 구간 내 최대 comm_->Clear() (tcflush)
    int64_t  stat_write_max_ = 0; // 구간 내 최대 comm_->Write() (write+tcdrain)
    int64_t  stat_read_max_  = 0; // 구간 내 최대 comm_->Read()
    static constexpr int64_t kBudgetUs = 4000;  // 처리시간 예산 (헤더2 + 바디2 ms)

    // 실시간 스케줄링: SCHED_FIFO + CPU 코어 고정 (마우스/GUI 선점 방지)
    // Orin 12코어 기준 마지막 코어(11)를 제어 전용으로 분리.
    // sudo 없이 실패하면 WARN 로그 출력 후 일반 우선순위로 계속.
    static constexpr int kRtPriority = 80;   // SCHED_FIFO 1~99
    static constexpr int kCpuCore    = 11;   // Orin: 0~11

    std::atomic<bool> is_running_;

    RD_RET Initialize();

    // === §3.1 제어 INIT 플로우 (control_mode 전용) ===
    // ① motor_mask(192) WRITE+검증 → ② mode(190)=1(AUTO) WRITE+검증 → 성공 시 FSM IDLE.
    // 각 단계 kInitRetryIntervalMs 간격 kInitMaxRetry 회 재시도, 전부 실패 시 노드 종료(exit≠0).
    // RW write 범위(128~179) 밖이라 일반 WRITE 패킷 경로를 쓴다 (§2 out-of-span, 루프 시작 전이라 안전).
    RD_RET InitControl();
    // out-of-span 대상 레지스터(mask192/mode190)의 ECU 섀도 바이트 위치.
    // 대상이 아니면 nullptr — 호출부가 조용히 엉뚱한 곳을 쓰지 않도록 화이트리스트로 둔다.
    // ※ state_mutex 를 잡은 상태에서 호출할 것.
    uint8_t* RegBytePtr(uint16_t addr);
    // 1바이트 레지스터 WRITE 후 같은 주소 READ 로 read-back 검증. shadow_field 는 해당
    // 레지스터의 섀도 위치 — READ decode 가 이 자리를 ECU 실값으로 덮으므로 비교에 그대로 쓴다.
    bool WriteVerifyByte(uint16_t addr, uint8_t* shadow_field, uint8_t expect, const char* label);

    static constexpr int kInitMaxRetry        = 10;
    static constexpr int kInitRetryIntervalMs = 200;
    // INIT 검증 실패 = 설정 오류이므로 SupervisorLoop 의 재접속 재시도 대상이 아니다.
    bool init_fatal_ = false;
    RD_RET ExecuteTask(const TaskConfig_t& config, RD_RET* tx_result = nullptr);
};

} // namespace orin_bridge

#endif
