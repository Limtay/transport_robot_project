#ifndef ORIN_FIRMWARE_BRIDGE__RD_TELEMETRY_SINK_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_TELEMETRY_SINK_HPP_

// ITelemetrySink — L1 이 정의하고 L3 가 구현하는 통지 인터페이스 (redesign/02 A1-b)
//
// **의존성 역전**: L1(스케줄)이 L3(노드)에게 "발행해라" 라고 알려야 하는데, 그 통지를 위해
// L1 이 L3 헤더를 include 하면 계층이 역류한다. 인터페이스를 L1 이 소유하면
// `rd_schedule.hpp` 에서 `#include "rd_node.hpp"` 를 지울 수 있다 — 그것이 이 파일의 목적이다.
//
// 스레드: 구현체의 메서드는 **200Hz 스케줄 스레드**에서 불린다. 구현은 블록하면 안 된다
// (02 §6.3 Q2). 실제 발행은 큐 뒤편의 발행 스레드가 한다.

#include <cstdint>

// TxnTiming_t 는 L0 `rd_clock_sync.hpp` 가 이미 정의한다 (시계 동기 추정기의 입력).
// 여기서 다시 정의하면 ODR 위반 — 아래로 내려가는 의존이므로 그대로 쓴다.
#include "orin_firmware_bridge/core/rd_clock_sync.hpp"
#include "orin_firmware_bridge/core/rd_read_preset.hpp"

namespace orin_bridge {

class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;

    // 한 tick 의 트랜잭션 타이밍 (§2.5). valid=false 여도 통계용으로 넘어온다.
    virtual void OnTxn(const TxnTiming_t& txn) = 0;

    // 이번 tick 의 피드백을 발행하라 (§3.4). out-of-span 대체 tick 에서는 호출되지 않는다.
    virtual void OnFeedback() = 0;

    // RW 응답의 에러 니블 (read_err | write_err<<4) — §3.4 rw_err
    virtual void OnRwErr(uint8_t rw_err) = 0;

    // RdMap 의 누적 write 거부 횟수 — action result 의 구간 차분에 쓰인다
    virtual void OnWriteErrTotal(uint64_t total) = 0;

    // 이번 tick 은 정규 RW 를 **하지 못했다** — out-of-span 단발이나 사용자 슬롯 양보가
    // 그 자리를 가져갔다. 그 tick 에는 read 스냅샷이 없어 피드백도 발행되지 않으므로,
    // 시계열에 1샘플 구멍이 남는다. **그 구멍의 개수가 이 카운터다.**
    virtual void OnIrregularTick() = 0;

    // 이번 tick 이 주기(5ms)를 넘겨 **위상이 리셋됐다** (rd_schedule.cpp 의 next_cycle 재설정).
    //
    // ⚠ tick 번호는 건너뛰지 않는다. 루프는 밀린 만큼 따라잡지 않고 시간축을 뒤로 미루고,
    //   프로파일의 시각은 여전히 `tick * kDt`(공칭 5ms)로 계산된다. 즉 **없어진 tick 이
    //   아니라 늘어난 시간**이다 — 10초 프로파일이 10.3초에 걸쳐 재생돼도 기록에는
    //   2000 tick 이 t=0~10.0s 로 남는다. 이 카운터가 그 차이를 아는 유일한 수단이다.
    virtual void OnLateTick() = 0;

    // 읽기 프리셋이 바뀌었다 (04 §2.4.2). **어떤 섀도 필드가 더 이상 갱신되지 않는지**를
    // 알려 주는 통지다 — 이걸 안 받으면 구현체는 낡은 값을 신선한 값처럼 계속 발행한다.
    // 프리셋 교체는 IDLE 전용이라 200Hz 경로가 아니다 (설정 스레드에서 1회).
    virtual void SetReadPreset(const ReadPreset* preset) = 0;
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_TELEMETRY_SINK_HPP_
