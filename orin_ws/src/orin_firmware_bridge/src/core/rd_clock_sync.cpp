#include "orin_firmware_bridge/core/rd_clock_sync.hpp"

namespace orin_bridge {

double RdClockSync::UnwrapTickSec(uint32_t ecu_tick) {
    if (first_tick_) {
        first_tick_ = false;
        last_tick_  = ecu_tick;
        tick_accum_ = static_cast<int64_t>(ecu_tick);
        return tick_accum_ * 1e-4;
    }
    // 32bit wrap (≈4.97일) 대응 — 진행량을 int32 차이로 누적
    int64_t diff = static_cast<int64_t>(ecu_tick) - static_cast<int64_t>(last_tick_);
    if (diff < -static_cast<int64_t>(UINT32_MAX / 2)) diff += static_cast<int64_t>(UINT32_MAX) + 1;
    last_tick_   = ecu_tick;
    tick_accum_ += diff;
    return tick_accum_ * 1e-4;
}

RdClockSync::Result_t RdClockSync::Update(const TxnTiming_t& txn, uint32_t ecu_tick,
                                          double proc_delta_prev) {
    Result_t res;
    res.wire_up   = txn.req_bytes  * kByteTime;
    res.wire_down = txn.resp_bytes * kByteTime;
    if (!txn.valid) return res;

    const double tick_s = UnwrapTickSec(ecu_tick);
    const double proc   = (proc_delta_prev >= 0.0) ? proc_delta_prev : 0.0;

    // offset 구간 추정: latch 는 [t_req+wire_up, t_resp-wire_down-proc] 사이
    const double lb = txn.t_req  + res.wire_up  - tick_s;
    const double ub = txn.t_resp - res.wire_down - proc - tick_s;
    if (ub <= lb) {
        // 비정상 샘플(proc 과대 등) 폐기 — 직전 수렴값은 유지해 보고
        res.offset    = last_offset_;
        res.drift_ppm = last_drift_ppm_;
        res.valid     = last_valid_;
        return res;
    }

    res.quality = ub - lb;
    const double off_mid = 0.5 * (lb + ub);

    // 불연속 가드: ECU 리부트(tick 리셋) 또는 Orin NTP 스텝 시 offset 이 점프한다.
    // 신구 샘플이 섞이면 피팅이 오염되므로 창을 비우고 재수렴 (수 초 내 복귀).
    if (!win_.empty() && (off_mid - win_.back().off > kDiscontinuity ||
                          win_.back().off - off_mid > kDiscontinuity)) {
        win_.clear();
    }

    win_.push_back({txn.t_resp, off_mid, res.quality});
    while (!win_.empty() && (txn.t_resp - win_.front().t) > kWindowSec) win_.pop_front();

    // 창 내 최소 quality + 마진 이내의 샘플만 선별 (USB 지연 스파이크 제거)
    double qmin = win_.front().q;
    for (const auto& r : win_) if (r.q < qmin) qmin = r.q;
    const double qsel = qmin + kQualMargin;

    // 선별 샘플 선형 피팅: off(t) = a + b·(t - t0)  → offset(now)=a+b·span, drift=b
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    size_t n = 0;
    const double t0 = win_.front().t;
    for (const auto& r : win_) {
        if (r.q > qsel) continue;
        const double x = r.t - t0;
        sx += x; sy += r.off; sxx += x * x; sxy += x * r.off;
        n++;
    }
    const double span = win_.back().t - t0;
    if (n >= kMinFitSamples && span >= kMinFitSpan) {
        const double denom = n * sxx - sx * sx;
        if (denom > 1e-12) {
            const double b = (n * sxy - sx * sy) / denom;
            const double a = (sy - b * sx) / n;
            res.offset    = a + b * (txn.t_resp - t0);
            res.drift_ppm = b * 1e6;
            res.valid     = true;
        }
    }
    if (!res.valid) {
        // 미수렴: 이번 샘플 중앙값을 참고치로 반환 (valid=false)
        res.offset = win_.back().off;
    }
    last_offset_    = res.offset;
    last_drift_ppm_ = res.drift_ppm;
    last_valid_     = res.valid;
    return res;
}

} // namespace orin_bridge
