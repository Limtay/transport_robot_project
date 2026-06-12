#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMMAND_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMMAND_HPP_

// 커맨드 슬롯 매니저 (Code_modify.md §2, §3)
//  - 4개 슬롯, 각 5Hz 로 스케줄러가 발사 (slot 번호 낮을수록 우선순위 높음)
//  - duration: 0=forever / 1=once(RET_OK 까지 반복, 2s timeout) / 2~100 [sec]
//  - REBOOT: 응답 수신 후 해당 보드 3초 blackout + is_connected=false
//  - 우선순위 규칙(§2.3): auto command 요청 시 빈 슬롯 최상단 배치,
//    모두 차있으면 최하위 슬롯 일시정지 → 자리 차용 → 처리 후 복귀
//  - jeongae 자동 전개 시퀀스 FSM(§3) — DPC/카메라 단계는 레지스터 미확정으로 TODO

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include "orin_firmware_bridge/rd_map.hpp"

namespace orin_bridge {

// CommandSet.srv 의 inst 값과 1:1
enum class CmdInst : uint8_t {
    WRITE_REG  = 0,  // 기능1: 현재 reg 섀도 값 전송 (start_addr, data_len)
    WRITE_DATA = 1,  // 기능2: 사용자 데이터 전송 (start_addr, data[])
    READ       = 2,  // READ — 성공 시 reg 갱신 후 RCLCPP_INFO 로 출력
    REBOOT     = 3,  // 보드 reboot — 3초 접근 차단
};

constexpr uint16_t CMD_DURATION_FOREVER = 0;
constexpr uint16_t CMD_DURATION_ONCE    = 1;
constexpr uint16_t CMD_DURATION_MAX_SEC = 100;
constexpr uint8_t  CMD_SLOT_AUTO        = 255;  // SET 시 빈 슬롯 자동 선택
constexpr int      CMD_NUM_SLOTS        = 4;

// CommandSet.srv request 와 동일한 평면 구조 (서비스 의존성 없이 코어 분리)
struct CommandRequest_t {
    uint8_t  slot       = CMD_SLOT_AUTO;
    uint8_t  action     = 1;            // 0=RESET / 1=SET
    uint8_t  target_id  = 0;
    uint8_t  inst       = 0;            // CmdInst
    uint16_t start_addr = 0;
    uint16_t data_len   = 0;
    std::vector<uint8_t> data;
    uint16_t duration   = CMD_DURATION_ONCE;
};

class RdCommand {
public:
    explicit RdCommand(RobotState_t* state);

    // ===== ROS 스레드에서 호출 =====
    bool HandleRequest(const CommandRequest_t& req, std::string* out_msg);
    void TriggerJeongae();                 // jeongae 토픽 수신 시 (open=true)
    void SetJeongaeLock(bool lock);
    bool GetJeongaeLock() const { return jeongae_lock_.load(); }

    // ===== 스케줄러 스레드에서 호출 =====
    // 슬롯 차례에 발사할 task 를 꺼낸다. false = 이번 차례 skip (빈 슬롯/만료/blackout)
    bool GetSlotTask(int slot, TaskConfig_t* out_task);
    // 트랜잭션 결과 보고 (tx→rx→decode 전체 결과)
    void ReportResult(int slot, RD_RET ret);
    // REBOOT 후 3초간 해당 보드 전체 접근 차단
    bool IsTargetBlackedOut(uint8_t target_id);
    // jeongae 시퀀스 soft ESTOP 동안 50Hz cmd_vel WRITE 정지 (§3.2)
    bool IsCmdVelPaused() const { return cmd_vel_paused_.load(); }
    // 자동 시퀀스 FSM tick — 스케줄러 루프에서 주기 호출
    void TickAutoSequence();

private:
    struct Slot_t {
        bool     active     = false;
        bool     is_auto    = false;   // auto sequence 가 차용한 슬롯
        uint8_t  target_id  = 0;
        CmdInst  inst       = CmdInst::READ;
        uint16_t start_addr = 0;
        uint16_t data_len   = 0;
        std::vector<uint8_t> user_data;
        uint16_t duration   = CMD_DURATION_ONCE;
        std::chrono::steady_clock::time_point start_time;
        uint32_t err_streak = 0;       // 연속 실패 수 (forever/N초 로그 억제용)
    };

    // jeongae 전개 시퀀스 (§3.2) — DPC 단계는 레지스터 미확정으로 통과(TODO)
    enum class Seq : uint8_t {
        IDLE = 0,
        ESTOP_SET,         // ECU WRITE 189=0 → 성공 시 50Hz 정지
        DPC_STATE_CHECK,   // TODO: DPC state READ (완전한 상태 확인) — 레지스터 미확정
        DPC_DEPLOY,        // TODO: DPC 전개 요청 (공벽 1/2번칸, 전개판 구분 미정)
        DPC_WAIT_CAMERA,   // TODO: DPC state 대기 (카메라 위치, LED on)
        CAMERA_ACTION,     // TODO: deploy camera ROS2 Action
        DPC_RETRACT,       // TODO: DPC 회수 요청
        DPC_WAIT_RETRACT,  // TODO: 회수 완료 state 대기
        ESTOP_RELEASE,     // ECU WRITE 189=1 → 성공 시 50Hz 재개
    };

    RobotState_t* state_;
    std::mutex    mutex_;   // slots_ / saved_slot_ / seq 상태 보호 (ROS ↔ sched 스레드)

    std::array<Slot_t, CMD_NUM_SLOTS> slots_;

    // 우선순위 규칙: auto 가 차용하며 일시정지된 원래 슬롯 백업
    Slot_t saved_slot_;
    int    saved_slot_idx_ = -1;

    // REBOOT blackout
    std::array<std::chrono::steady_clock::time_point, 3> blackout_until_{}; // ECU/DPC/PCU
    static int TargetIndex(uint8_t target_id);

    // jeongae 시퀀스 상태
    Seq  seq_ = Seq::IDLE;
    int  seq_slot_ = -1;            // 시퀀스가 사용 중인 슬롯
    bool seq_cmd_done_ = false;     // 시퀀스 발행 커맨드 완료 플래그
    bool seq_cmd_ok_   = false;
    std::atomic<bool> jeongae_trigger_{false};
    std::atomic<bool> jeongae_lock_{false};      // §2.4: orin 기본 변수, default 0
    std::atomic<bool> cmd_vel_paused_{false};

    // 내부 헬퍼 (mutex_ 잡은 상태에서 호출)
    bool SetSlotLocked(int idx, const CommandRequest_t& req, std::string* out_msg);
    void ClearSlotLocked(int idx);
    int  AcquireAutoSlotLocked();   // §3.1: 빈 칸 최상단, 없으면 최하위 일시정지
    void ReleaseAutoSlotLocked(int idx);
    bool StartSeqWriteLocked(uint16_t addr, uint8_t value);  // once WRITE_DATA 1바이트
    void AbortSequenceLocked(const char* reason);

    static const char* TargetName(uint8_t target_id);
};

} // namespace orin_bridge

#endif
