#!/usr/bin/env python3
"""시간 동기·지연 특성화 분석 (testbed_spec.md §2.5).

control_mode 200Hz 트랜잭션당 1건 발행되는 `/carrier/testbed/comm_latency`
(CommLatency.msg) bag 을 읽어 ① RTT 및 구간 지연 분포, ② ECU tick↔Orin 시계
offset·drift, ③ 200Hz 루프 건전성을 정량화한다. traction 분석과 달리 로드셀·
전류가 아니라 "명령/센서 지연과 시간축 자체"가 대상 — 이후 모든 실험의 시간축
보정(clock_offset)과 MPC 지연 보상 파라미터의 근거가 된다.

사용법:
    python3 latency_analysis.py [bag_dir] [--out OUT_DIR]
    (bag_dir 기본값 = data/rosbags/V1_commlatency_07-20_19-08, 스크립트 위치 기준)

산출물 (out/<bag이름>/):
    summary.json        아래 리포트의 기계판독본
    rtt_breakdown.png   RTT 분포 + 구간 분해(wire_up/down·proc_delta·USB 잔차)
    clock_drift.png     clock_offset(t) + 선형피팅 교차검증 + 잔차
    loop_health.png     200Hz 루프 주기(dt) 분포 + 순시 발행률

의존성: numpy, matplotlib (ROS 불필요 — sqlite3 + 수동 CDR 디코딩).

시간축 주의 (V1 07-20 실측): ECU TIM5 는 HSI(내부 RC, PLL 소스) 기반이라
drift ≈ -2% (수십 ppm 이 아니라 % 스케일). raw ecu_tick 을 10kHz 정확으로
가정하지 말고 반드시 보정된 clock_offset 으로 Orin 시각에 매핑할 것.
"""
import argparse
import glob
import json
import os
import sqlite3
import struct
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── 상수 ────────────────────────────────────────────────────────────────────
NOMINAL_HZ      = 200.0
NOMINAL_DT      = 1.0 / NOMINAL_HZ          # 5 ms
OVER_PERIOD_DT  = 1.5 * NOMINAL_DT          # 이보다 긴 tick 간격 = over-period (7.5 ms)
TICK_TO_S       = 1e-4                       # ECU realtime_tick 단위 [0.1 ms]
PROC_STALE      = -0.5                       # proc_delta_prev < 이 값이면 stale(-1) 로 간주
CMD_TICK_STALE  = 0xFF                       # cmd_delta_tick raw stale 마커

# dataviz 검증 팔레트 (traction_analysis.py 와 동일 하우스 스타일) ───────────
PALETTE = ["#2a78d6", "#1baf7a", "#eda100", "#4a3aa7", "#e34948", "#e87ba4"]
TEXT, TEXT2, GRID = "#1a1a19", "#5f5e56", "#e5e4dc"

plt.rcParams.update({
    'figure.facecolor': 'white', 'axes.facecolor': 'white',
    'axes.edgecolor': GRID, 'axes.labelcolor': TEXT2, 'axes.grid': True,
    'grid.color': GRID, 'grid.linewidth': 0.6,
    'xtick.color': TEXT2, 'ytick.color': TEXT2,
    'text.color': TEXT, 'font.size': 9,
    'axes.spines.top': False, 'axes.spines.right': False,
    'lines.linewidth': 1.4, 'legend.frameon': False,
})


# ── rosbag 파싱 (수동 CDR) ──────────────────────────────────────────────────
def _align(o, n):
    return (o + n - 1) & ~(n - 1)


def _decode_latency(blob):
    """CommLatency.msg CDR 디코딩 → 필드 dict (단위: 초 / raw tick).

    프레임 정렬 규칙만으로 오프셋을 계산하므로 frame_id 길이에 무관하다.
    """
    d = blob[4:]                                          # encapsulation 헤더 4B skip
    o = 0
    o += 8                                                # header.stamp (sec+nanosec)
    slen = struct.unpack_from('<I', d, o)[0]
    o += 4 + slen                                         # frame_id (null 포함)
    o = _align(o, 8)
    t_req, t_resp = struct.unpack_from('<2d', d, o); o += 16
    ecu_tick = struct.unpack_from('<I', d, o)[0]; o += 4
    rtt, wire_up, wire_down, proc_delta, quality = struct.unpack_from('<5f', d, o); o += 20
    o = _align(o, 8)
    clock_offset = struct.unpack_from('<d', d, o)[0]; o += 8
    drift_ppm = struct.unpack_from('<f', d, o)[0]; o += 4
    offset_valid = d[o]; o += 1
    cmd_delta = struct.unpack_from('<4B', d, o)
    return (t_req, t_resp, ecu_tick, rtt, wire_up, wire_down, proc_delta,
            quality, clock_offset, drift_ppm, offset_valid, cmd_delta)


def load_latency_bag(bag_dir):
    """comm_latency 토픽만 읽어 컬럼 배열 dict 로. (feedback 토픽은 무시.)"""
    db3 = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not db3:
        raise FileNotFoundError(f'no .db3 in {bag_dir}')

    rows = []
    for path in db3:
        con = sqlite3.connect(path)
        # topics 테이블에서 comm_latency 토픽 id 를 찾아 그 메시지만 선별
        tid = con.execute(
            "SELECT id FROM topics WHERE name LIKE '%comm_latency%'").fetchone()
        if tid is None:
            con.close()
            continue
        rows += con.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (tid[0],)).fetchall()
        con.close()

    n = len(rows)
    if n == 0:
        raise ValueError(f'comm_latency 메시지 0건 — bag 에 토픽이 있는지 확인: {bag_dir}')

    cols = dict(
        t_req=np.empty(n), t_resp=np.empty(n), ecu_tick=np.empty(n, np.int64),
        rtt=np.empty(n), wire_up=np.empty(n), wire_down=np.empty(n),
        proc_delta=np.empty(n), quality=np.empty(n),
        clock_offset=np.empty(n), drift_ppm=np.empty(n),
        offset_valid=np.empty(n, bool), cmd_delta=np.empty((n, 4), np.int64))
    for i, (blob,) in enumerate(rows):
        (tr, ts, tk, rtt, wu, wd, pd, q, off, dr, ov, cd) = _decode_latency(blob)
        cols['t_req'][i] = tr; cols['t_resp'][i] = ts; cols['ecu_tick'][i] = tk
        cols['rtt'][i] = rtt; cols['wire_up'][i] = wu; cols['wire_down'][i] = wd
        cols['proc_delta'][i] = pd; cols['quality'][i] = q
        cols['clock_offset'][i] = off; cols['drift_ppm'][i] = dr
        cols['offset_valid'][i] = bool(ov); cols['cmd_delta'][i] = cd
    return cols


# ── 통계 헬퍼 ────────────────────────────────────────────────────────────────
def _stats_ms(x):
    """초 배열 → ms 요약 dict (mean/p50/p99/std/min/max)."""
    x = np.asarray(x, float) * 1e3
    return dict(mean=float(x.mean()), p50=float(np.percentile(x, 50)),
                p99=float(np.percentile(x, 99)), std=float(x.std()),
                min=float(x.min()), max=float(x.max()))


def analyze(cols):
    """모든 지표 계산 → report dict."""
    rep = {}
    n = len(cols['rtt'])
    rep['n_samples'] = int(n)

    # ── 200Hz 루프 건전성: Orin 시리얼 write 간격(t_req) 이 실제 루프 주기 ──
    dt = np.diff(cols['t_req'])
    dt = dt[dt > 0]                                    # 재시작·역행 방어
    over = int(np.sum(dt > OVER_PERIOD_DT))
    rep['loop'] = dict(
        rate_hz=float(1.0 / np.median(dt)),
        dt_mean_ms=float(dt.mean() * 1e3),
        dt_p99_ms=float(np.percentile(dt, 99) * 1e3),
        dt_max_ms=float(dt.max() * 1e3),
        over_period_cnt=over,
        over_period_pct=float(100.0 * over / len(dt)),
        duration_s=float(cols['t_req'][-1] - cols['t_req'][0]))

    # ── RTT + 구간 분해 (잔차 = RTT − wire_up − wire_down − proc_delta = USB) ──
    proc = cols['proc_delta'].copy()
    proc_ok = proc > PROC_STALE
    proc_used = np.where(proc_ok, proc, 0.0)          # stale 은 0 처리(잔차에 흡수)
    residual = cols['rtt'] - cols['wire_up'] - cols['wire_down'] - proc_used
    rep['rtt'] = _stats_ms(cols['rtt'])
    rep['breakdown_ms'] = dict(
        wire_up=float(cols['wire_up'].mean() * 1e3),
        wire_down=float(cols['wire_down'].mean() * 1e3),
        proc_delta=float(proc[proc_ok].mean() * 1e3) if proc_ok.any() else None,
        proc_stale_pct=float(100.0 * (~proc_ok).mean()),
        usb_residual=float(residual.mean() * 1e3),
        rtt_total=float(cols['rtt'].mean() * 1e3))

    # ── 명령 경로: cmd_delta_tick (모터별 CAN 송출 지연, raw x0.1ms) ──
    cmd = cols['cmd_delta']
    cmd_report = {}
    for m in range(4):
        col = cmd[:, m]
        good = col[col != CMD_TICK_STALE]
        if len(good):
            cmd_report[f'm{m + 1}'] = dict(
                mean_ms=float(good.mean() * 0.1), p99_ms=float(np.percentile(good, 99) * 0.1),
                stale_pct=float(100.0 * (col == CMD_TICK_STALE).mean()))
    rep['cmd_delta_tick'] = cmd_report

    # ── clock offset / drift ──
    valid = cols['offset_valid']
    conv_s = None
    if valid.any():
        first_valid = int(np.argmax(valid))
        conv_s = float(cols['t_req'][first_valid] - cols['t_req'][0])
    # 교차검증: offset_valid 구간에서 t_resp = a*ecu_tick + b 선형피팅.
    # 완벽한 10kHz 면 a = 1e-4; a 편차가 곧 ECU 시계 drift.
    drift_fit_ppm = None
    if valid.sum() > 100:
        tk = cols['ecu_tick'][valid].astype(float)
        tr = cols['t_resp'][valid]
        a, _b = np.polyfit(tk, tr, 1)                 # s per tick
        drift_fit_ppm = float((a / TICK_TO_S - 1.0) * 1e6)
    rep['clock'] = dict(
        converged=bool(valid.any()),
        convergence_s=conv_s,
        valid_pct=float(100.0 * valid.mean()),
        drift_estimator_ppm=float(cols['drift_ppm'][valid].mean()) if valid.any() else None,
        drift_fit_ppm=drift_fit_ppm,
        offset_span_s=float(cols['clock_offset'][valid].ptp()) if valid.any() else None)
    return rep


# ── 리포트 출력 ──────────────────────────────────────────────────────────────
def print_report(rep, bag_name):
    L = rep['loop']; b = rep['breakdown_ms']; c = rep['clock']; r = rep['rtt']
    print(f'\n=== 지연·시간동기 특성화 리포트 — {bag_name} ===')
    print(f'샘플 {rep["n_samples"]:,}건 · {L["duration_s"]:.1f}s')
    print(f'[루프] {L["rate_hz"]:.2f} Hz · dt {L["dt_mean_ms"]:.2f}ms(p99 {L["dt_p99_ms"]:.2f}, '
          f'max {L["dt_max_ms"]:.2f}) · over-period(>{OVER_PERIOD_DT*1e3:.1f}ms) '
          f'{L["over_period_pct"]:.2f}% ({L["over_period_cnt"]}건)')
    print(f'[RTT ] mean {r["mean"]:.2f}ms · p50 {r["p50"]:.2f} · p99 {r["p99"]:.2f} · '
          f'jitter(std) {r["std"]:.3f}ms')
    proc = f'{b["proc_delta"]:.3f}' if b['proc_delta'] is not None else 'stale'
    print(f'[분해] wire_up {b["wire_up"]:.3f} + wire_down {b["wire_down"]:.3f} + '
          f'proc_delta {proc} + USB잔차 {b["usb_residual"]:.3f} ≈ RTT {b["rtt_total"]:.3f} [ms] '
          f'(proc stale {b["proc_stale_pct"]:.1f}%)')
    if rep['cmd_delta_tick']:
        parts = [f'{k} {v["mean_ms"]:.2f}ms(p99 {v["p99_ms"]:.2f})'
                 for k, v in rep['cmd_delta_tick'].items()]
        print(f'[명령] CAN 송출지연 ' + ' · '.join(parts))
    conv = f'{c["convergence_s"]:.1f}s' if c['convergence_s'] is not None else '미수렴'
    de = c['drift_estimator_ppm']; df = c['drift_fit_ppm']
    de_s = f'{de:,.0f}ppm({de/1e4:+.2f}%)' if de is not None else 'n/a'
    df_s = f'{df:,.0f}ppm({df/1e4:+.2f}%)' if df is not None else 'n/a'
    print(f'[시계] 수렴 {conv} · valid {c["valid_pct"]:.1f}% · '
          f'drift 추정기 {de_s} / 회귀교차검증 {df_s}')
    if de is not None and abs(de) > 1000:
        print('       ⚠ drift 가 % 스케일 — ECU TIM5 는 HSI(내부 RC) 기반. '
              'raw ecu_tick 을 10kHz 로 가정 금지, clock_offset 으로 보정할 것.')


# ── 플롯 ─────────────────────────────────────────────────────────────────────
def plot_rtt_breakdown(cols, rep, path):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    # (a) RTT 히스토그램
    rtt_ms = cols['rtt'] * 1e3
    ax1.hist(rtt_ms, bins=80, color=PALETTE[0], alpha=0.85)
    ax1.axvline(rep['rtt']['p99'], color=PALETTE[4], ls='--', lw=1.2,
                label=f'p99 {rep["rtt"]["p99"]:.2f}ms')
    ax1.axvline(rep['rtt']['mean'], color=TEXT, ls='-', lw=1.0,
                label=f'mean {rep["rtt"]["mean"]:.2f}ms')
    ax1.set_xlabel('RTT [ms]'); ax1.set_ylabel('count'); ax1.set_title('Round-trip latency')
    ax1.legend()
    # (b) 구간 분해 누적 막대
    b = rep['breakdown_ms']
    labels = ['wire_up', 'wire_down', 'proc_delta', 'USB residual']
    vals = [b['wire_up'], b['wire_down'], b['proc_delta'] or 0.0, b['usb_residual']]
    bottom = 0.0
    for lab, v, col in zip(labels, vals, PALETTE):
        ax2.bar(0, v, bottom=bottom, color=col, width=0.5, label=f'{lab} {v:.2f}ms')
        bottom += v
    ax2.set_xticks([]); ax2.set_ylabel('[ms]')
    ax2.set_title(f'RTT breakdown (total {b["rtt_total"]:.2f}ms)')
    ax2.legend(loc='upper right', fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)


def plot_clock_drift(cols, rep, path):
    valid = cols['offset_valid']
    t = cols['t_req'] - cols['t_req'][0]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                                   gridspec_kw={'height_ratios': [3, 1]})
    ax1.plot(t[valid], cols['clock_offset'][valid], color=PALETTE[0], lw=1.0,
             label='clock_offset (valid)')
    if (~valid).any():
        ax1.plot(t[~valid], cols['clock_offset'][~valid], '.', color=TEXT2, ms=2,
                 label='pre-converge/invalid')
    # 선형 추세 (drift 시각화)
    if valid.sum() > 100:
        a, bfit = np.polyfit(t[valid], cols['clock_offset'][valid], 1)
        ax1.plot(t[valid], a * t[valid] + bfit, color=PALETTE[4], ls='--', lw=1.2,
                 label=f'trend {a*1e3:.3f} ms/s')
        ax2.plot(t[valid], (cols['clock_offset'][valid] - (a * t[valid] + bfit)) * 1e3,
                 color=PALETTE[1], lw=0.8)
    df = rep['clock']['drift_fit_ppm']
    txt = f'drift {df:,.0f} ppm ({df/1e4:+.2f}%)' if df is not None else ''
    ax1.set_ylabel('offset [s]'); ax1.set_title(f'ECU tick -> Orin clock offset · {txt}')
    ax1.legend(loc='best', fontsize=8)
    ax2.set_ylabel('trend resid [ms]'); ax2.set_xlabel('elapsed [s]')
    ax2.axhline(0, color=GRID, lw=0.8)
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)


def plot_loop_health(cols, rep, path):
    dt = np.diff(cols['t_req'])
    dt = dt[dt > 0] * 1e3
    t = (cols['t_req'][1:] - cols['t_req'][0])
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    ax1.hist(dt, bins=np.linspace(0, max(12, np.percentile(dt, 99.9)), 120),
             color=PALETTE[0], alpha=0.85)
    ax1.axvline(NOMINAL_DT * 1e3, color=TEXT, lw=1.0, label='nominal 5.0ms')
    ax1.axvline(OVER_PERIOD_DT * 1e3, color=PALETTE[4], ls='--', lw=1.0,
                label=f'over thr {OVER_PERIOD_DT*1e3:.1f}ms')
    ax1.set_yscale('log'); ax1.set_xlabel('tick interval dt [ms]'); ax1.set_ylabel('count(log)')
    ax1.set_title(f'Loop period · {rep["loop"]["rate_hz"]:.2f} Hz')
    ax1.legend(fontsize=8)
    # 순시 발행률 (다운샘플)
    rate = 1e3 / dt[dt > 0]
    step = max(1, len(rate) // 3000)
    ax2.plot(t[::step], rate[::step], color=PALETTE[1], lw=0.5)
    ax2.axhline(NOMINAL_HZ, color=TEXT, lw=1.0)
    ax2.set_ylim(0, NOMINAL_HZ * 1.4)
    ax2.set_xlabel('elapsed [s]'); ax2.set_ylabel('inst. rate [Hz]')
    ax2.set_title(f'over-period {rep["loop"]["over_period_pct"]:.2f}%  (>{OVER_PERIOD_DT*1e3:.1f}ms)')
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)


# ── 엔트리포인트 ──────────────────────────────────────────────────────────────
def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    default_bag = os.path.normpath(
        os.path.join(here, '..', '..', 'data', 'rosbags', 'V1_commlatency_07-20_19-08'))
    ap = argparse.ArgumentParser(description='시간 동기·지연 특성화 (testbed_spec §2.5)')
    ap.add_argument('bag_dir', nargs='?', default=default_bag, help='comm_latency bag 폴더')
    ap.add_argument('--out', default=os.path.join(here, 'out'), help='산출 루트')
    args = ap.parse_args(argv)

    bag_name = os.path.basename(os.path.normpath(args.bag_dir))
    out_dir = os.path.join(args.out, bag_name)
    os.makedirs(out_dir, exist_ok=True)

    cols = load_latency_bag(args.bag_dir)
    rep = analyze(cols)
    rep['bag'] = bag_name
    print_report(rep, bag_name)

    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)
    plot_rtt_breakdown(cols, rep, os.path.join(out_dir, 'rtt_breakdown.png'))
    plot_clock_drift(cols, rep, os.path.join(out_dir, 'clock_drift.png'))
    plot_loop_health(cols, rep, os.path.join(out_dir, 'loop_health.png'))
    print(f'\n산출물 → {out_dir}/ (summary.json + 3 png)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
