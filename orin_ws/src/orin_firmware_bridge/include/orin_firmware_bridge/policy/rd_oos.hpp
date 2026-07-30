#ifndef ORIN_FIRMWARE_BRIDGE__RD_OOS_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_OOS_HPP_

// RdOos — out-of-span 단발 WRITE 큐 (redesign/02 A2 신설, 01 §6.2)
//
// RW write 범위(128~179) 밖 레지스터(motor_mask 192 / mode 190)는 패킷에 끼워넣지 못하므로
// **RW 1 tick 을 일반 WRITE/READ 패킷으로 대체**해 처리한다. IDLE 은 write 값이 0A 라
// 1~2 tick 대체가 무해하다는 것이 근거 (01 §6.3 표).
//
// 제약 (01 §6.2):
//   1) IDLE 에서만, 동시 1건 — in-flight 중 새 요청은 즉시 거부 (큐잉 없음)
//   2) 대체된 tick 은 read 스냅샷이 없다 → 그 tick 의 피드백 발행 skip (보간 금지)
//   3) 대체 WRITE 실패 시 재시도 없이 다음 tick 정상 RW 복귀 (재시도는 호출자 몫)
//   4) 검증은 WRITE tick + READ tick, 최대 2 tick 소모
//
// 왜 L3 에서 떼어냈나: 이 로직에 ROS 는 하나도 안 쓰인다. 노드 안에 있던 탓에 L1 스케줄러가
// 매 tick 노드를 거꾸로 참조해야 했다 (A1-(c)). 여기로 옮기면 L1 → L2 정방향이 된다.
//
// 스레드: `Request` 는 service 스레드, `TakeStep`/`ReportResult` 는 200Hz 스케줄 스레드.

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

namespace orin_bridge {

enum class OosPhase : uint8_t { NONE = 0, WRITE, READ, DONE };

struct OosStep_t {
    OosPhase phase = OosPhase::NONE;
    uint16_t addr  = 0;
    uint8_t  value = 0;
};

class RdOos {
public:
    // 01 §6.2 검증 규칙: 10 tick(50ms) 내 read-back 미확인 시 실패 응답
    static constexpr int kVerifyTicks     = 10;
    static constexpr int kVerifyTimeoutMs = 50;

    // [service 스레드] 요청을 걸고 완료(또는 50ms 타임아웃)까지 대기한다.
    // 성공 시 true, 실패 시 msg 에 사유.
    bool Request(uint16_t addr, uint8_t value, std::string* msg);

    // [스케줄 스레드] 매 tick 첫머리. 이번 tick 을 대체해야 하면 true + 수행할 단계.
    bool TakeStep(OosStep_t* step);

    // [스케줄 스레드] 대체 tick 의 실행 결과 보고. readback 은 READ 단계에서만 의미 있음.
    void ReportResult(bool tx_ok, uint8_t readback);

private:
    std::mutex              mutex_;
    std::condition_variable cv_;
    OosStep_t step_;
    bool      in_flight_ = false;
    bool      write_ok_  = false;
    bool      read_ok_   = false;
    uint8_t   readback_  = 0;
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_OOS_HPP_
