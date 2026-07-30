#ifndef ORIN_FIRMWARE_BRIDGE__RD_CLOCK_SYNC_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_CLOCK_SYNC_HPP_

// ECU tick(TIM5 10kHz free-run) <-> Orin(epoch) 시계 매핑 추정기 (testbed_spec.md §2.5)
//
// 원리 (NTP 유사, 단 wire 시간이 결정적이라 대칭 가정 불필요):
//   트랜잭션마다 ECU 의 realtime_tick latch 시각(t_latch)은 Orin 시각으로
//     lb = t_req  + wire_up                       <= t_latch
//     ub = t_resp - wire_down - proc_delta        >= t_latch
//   offset = t_latch - tick_s 이므로 [lb - tick_s, ub - tick_s] 구간 추정.
//   남는 미지수는 Orin USB/드라이버 지연뿐 → quality(ub-lb) 가 작은 샘플만 선별해
//   창(window) 내 선형 피팅으로 offset + drift(ppm) 를 추정한다.
//
// ROS 비의존 (chrono/deque 만) — 오프라인 단독 컴파일 테스트 가능.

#include <cstdint>
#include <cstddef>
#include <deque>

namespace orin_bridge {

// ExecuteTask 가 채우는 트랜잭션 계측 원자료
struct TxnTiming_t {
    double   t_req    = 0.0;   // 요청 write 직전 [s, epoch]
    double   t_resp   = 0.0;   // 응답 수신 완료 [s, epoch]
    size_t   req_bytes  = 0;   // 요청 wire 바이트 (헤더~CRC 전체)
    size_t   resp_bytes = 0;   // 응답 wire 바이트
    bool     valid    = false; // 응답 수신 + Decode 성공 여부
};

class RdClockSync {
public:
    struct Result_t {
        double offset    = 0.0;   // ECU tick[s] + offset = Orin epoch [s]
        double drift_ppm = 0.0;   // d(offset)/dt [ppm] — 음수 = ECU 시계가 Orin 보다 빠름
        double quality   = -1.0;  // 이번 샘플 ub-lb [s] (-1 = 계산 불가)
        double wire_up   = 0.0;   // [s]
        double wire_down = 0.0;   // [s]
        bool   valid     = false; // 창 수렴 여부
    };

    // proc_delta_prev: 직전 트랜잭션 STM 처리시간 [s] (<0 = 미상 → ub 에 0 적용, 보수적)
    Result_t Update(const TxnTiming_t& txn, uint32_t ecu_tick, double proc_delta_prev);

    // tick(×0.1ms, wrap ~4.97일) → 단조 증가 [s]. Update 외부에서 취득시각 복원 등에 사용.
    double UnwrapTickSec(uint32_t ecu_tick);

private:
    static constexpr double kByteTime  = 10.0 / 921600.0;  // [s/byte] 8N1 = 10bit
    static constexpr double kWindowSec = 20.0;   // 피팅 창
    static constexpr double kQualMargin = 2e-4;  // 선별 마진: 창 내 최소 quality + 0.2ms
    static constexpr size_t kMinFitSamples = 50; // 수렴 최소 샘플 수
    static constexpr double kMinFitSpan    = 5.0;// 수렴 최소 창 폭 [s] (drift 추정 안정성)
    static constexpr double kDiscontinuity = 0.1;// offset 점프 한계 [s] — 초과 시 창 리셋 (ECU 리부트/NTP 스텝)

    // tick unwrap 상태
    bool     first_tick_ = true;
    uint32_t last_tick_  = 0;
    int64_t  tick_accum_ = 0;   // unwrap 누적 (×0.1ms)

    struct Rec_t { double t; double off; double q; };  // t = t_resp 기준
    std::deque<Rec_t> win_;

    // 직전 결과 캐시 — 샘플 폐기 tick 에도 수렴값 연속 보고
    double last_offset_    = 0.0;
    double last_drift_ppm_ = 0.0;
    bool   last_valid_     = false;
};

} // namespace orin_bridge

#endif
