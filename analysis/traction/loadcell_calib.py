#!/usr/bin/env python3
"""Stage 1 — 로드셀 영점 검증 + 절대력 캘리브레이션 (test_plan §Stage 1).

두 모드:
  --zero BAG                 무부하 hold bag → baseline/노이즈/레일마진/드리프트 판정
  --calib BAG --loads a b c  기지 하중 bag(0A hold 중 무게추를 순차로 얹음) → cnt↔N 선형피팅

`--loads` 는 **적용한 하중 순서**를 kgf 로 나열한다(무부하 0 포함). 예: 무부하→10.46→누적 21.49→
누적 31.74 로 얹었으면 `--loads 0 10.46 21.49 31.74`. 도구가 로드셀 신호의 안정 평탄(plateau)을
자동 검출해 순서대로 하중과 짝지어 `cnt = k·F[N] + b` 를 피팅한다(F = kgf×9.80665).

산출: cnt/N 계수 k, 역계수(N/cnt = 분석 파이프라인 상수), R², 채널별. 결과 JSON + 플롯.

의존성: numpy, matplotlib (ROS 불필요). 디코더는 traction_analysis 재사용.
"""
import argparse
import glob
import json
import os
import sqlite3
import sys
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import traction_analysis as T   # _detect_format / _decode / _decode_testbed 재사용

G = 9.80665                      # kgf → N
ZERO_MIN_CNT   = 300.0           # 무부하 평균 하한 (레일 이탈 — test_plan Stage 1)
RAIL_LOW       = 8.0             # ADC_RAIL_LOW — 이 밑이면 단선 오검출 위험
STABLE_WIN_S   = 1.5             # plateau 판정 이동창
STABLE_STD_CNT = 8.0            # 이 이동표준편차 미만이면 '안정'
MIN_PLATEAU_S  = 3.0             # 이보다 짧은 안정구간은 plateau 로 안 봄

PALETTE = ["#2a78d6", "#1baf7a", "#eda100", "#4a3aa7", "#e34948"]
TEXT2, GRID = "#5f5e56", "#e5e4dc"
plt.rcParams.update({'figure.facecolor': 'white', 'axes.facecolor': 'white',
                     'axes.edgecolor': GRID, 'axes.grid': True, 'grid.color': GRID,
                     'grid.linewidth': 0.6, 'font.size': 9, 'axes.spines.top': False,
                     'axes.spines.right': False, 'legend.frameon': False})


def load_loadcell(bag_dir):
    """bag → (t[s], lc[N,2], goal_id). 두 로드셀 채널 모두 유지(캘리브레이션은 채널별)."""
    db3 = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not db3:
        raise FileNotFoundError(f'no .db3 in {bag_dir}')
    decode_fn, tick_scale, rows = None, 1e-3, []
    for path in db3:
        con = sqlite3.connect(path)
        decode_fn, tick_scale, tid = T._detect_format(con)
        q = ('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp'
             if tid is not None else 'SELECT data FROM messages ORDER BY timestamp')
        rows += (con.execute(q, (tid,)) if tid is not None else con.execute(q)).fetchall()
        con.close()
    n = len(rows)
    tick = np.empty(n, np.int64); lc = np.empty((n, 2)); gid = np.empty(n, np.int64)
    for i, (blob,) in enumerate(rows):
        tk, _ss, _f, l, g = decode_fn(blob)
        tick[i] = tk; lc[i] = l; gid[i] = g
    keep = np.concatenate([[True], np.diff(tick) != 0])
    t = (tick[keep] - tick[keep][0]) * tick_scale
    return t, lc[keep], gid[keep]


def find_plateaus(t, y):
    """안정 평탄 구간 검출 → [(t0, t1, mean, std), ...] (시간순)."""
    dt = np.median(np.diff(t))
    win = max(3, int(STABLE_WIN_S / dt))
    # 이동표준편차
    csum = np.cumsum(np.insert(y, 0, 0.0))
    csum2 = np.cumsum(np.insert(y * y, 0, 0.0))
    mstd = np.full(len(y), np.inf)
    for i in range(len(y) - win):
        s = csum[i + win] - csum[i]
        s2 = csum2[i + win] - csum2[i]
        var = max(0.0, s2 / win - (s / win) ** 2)
        mstd[i + win // 2] = np.sqrt(var)
    stable = mstd < STABLE_STD_CNT
    plateaus = []
    i = 0
    while i < len(stable):
        if stable[i]:
            j = i
            while j < len(stable) and stable[j]:
                j += 1
            if (t[j - 1] - t[i]) >= MIN_PLATEAU_S:
                # 가장자리 잘라 안정 중심부만 평균
                a, b = i + win // 2, j - win // 2
                seg = y[a:b]
                plateaus.append((float(t[a]), float(t[b - 1]),
                                 float(seg.mean()), float(seg.std())))
            i = j
        else:
            i += 1
    return plateaus


def run_zero(bag_dir, out_dir):
    t, lc, _gid = load_loadcell(bag_dir)
    rep = {'bag': os.path.basename(os.path.normpath(bag_dir)), 'mode': 'zero',
           'duration_s': float(t[-1]), 'channels': {}}
    fig, ax = plt.subplots(figsize=(10, 4))
    for ch in range(2):
        y = lc[:, ch]
        # 선형 드리프트 (온도) = 처음/끝 이동평균 차
        k = max(1, len(y) // 20)
        drift = float(y[-k:].mean() - y[:k].mean())
        info = dict(mean=float(y.mean()), std=float(y.std()),
                    min=float(y.min()), max=float(y.max()),
                    rail_margin=float(y.min() - RAIL_LOW), drift_cnt=drift,
                    pass_baseline=bool(y.mean() >= ZERO_MIN_CNT))
        rep['channels'][f'ch{ch}'] = info
        ax.plot(t, y, lw=0.6, color=PALETTE[ch], label=f'ch{ch} mean {info["mean"]:.0f}')
    ax.axhline(ZERO_MIN_CNT, color=PALETTE[4], ls='--', lw=1.0, label=f'min {ZERO_MIN_CNT:.0f}cnt')
    ax.set_xlabel('elapsed [s]'); ax.set_ylabel('loadcell raw [cnt]')
    ax.set_title('Stage 1 loadcell zero (no load)'); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'loadcell_zero.png'), dpi=120)
    plt.close(fig)

    print(f'\n=== 로드셀 영점 검증 — {rep["bag"]} ({rep["duration_s"]:.0f}s) ===')
    for ch, info in rep['channels'].items():
        verdict = 'PASS' if info['pass_baseline'] else f'FAIL(<{ZERO_MIN_CNT:.0f})'
        print(f'[{ch}] mean {info["mean"]:.1f} · std {info["std"]:.2f} · '
              f'range {info["min"]:.0f}~{info["max"]:.0f} · rail마진 {info["rail_margin"]:.0f} · '
              f'드리프트 {info["drift_cnt"]:+.1f}cnt · baseline {verdict}')
    return rep


def run_calib(bag_dir, loads_kgf, out_dir, channel=None):
    t, lc, _gid = load_loadcell(bag_dir)
    F = np.array(loads_kgf, float) * G                # 하중 [N]
    rep = {'bag': os.path.basename(os.path.normpath(bag_dir)), 'mode': 'calib',
           'loads_kgf': list(loads_kgf), 'loads_N': F.round(3).tolist(), 'channels': {}}
    chans = [channel] if channel is not None else [0, 1]

    fig, axes = plt.subplots(1, len(chans), figsize=(5.5 * len(chans), 4), squeeze=False)
    for ci, ch in enumerate(chans):
        y = lc[:, ch]
        plateaus = find_plateaus(t, y)
        means = np.array([p[2] for p in plateaus])
        entry = {'plateaus_cnt': [round(m, 1) for m in means.tolist()],
                 'n_plateaus': len(plateaus)}
        if len(plateaus) == len(F):
            # cnt = k·F + b  (F 오름 순 아닐 수 있으니 시간순 그대로 짝)
            k, b = np.polyfit(F, means, 1)
            resid = means - (k * F + b)
            ss = 1.0 - resid.var() / means.var() if means.var() > 0 else 0.0
            entry.update(counts_per_N=float(k), offset_cnt=float(b),
                         N_per_count=float(1.0 / k) if k else None,
                         r2=float(ss), resid_std_cnt=float(resid.std()))
            print(f'[ch{ch}] k={k:.3f} cnt/N · {1.0/k:.4f} N/cnt · R²={ss:.4f} · '
                  f'잔차 {resid.std():.1f}cnt · plateaus {len(plateaus)}')
        else:
            print(f'[ch{ch}] ⚠ plateau {len(plateaus)}개 ≠ 하중 {len(F)}개 — '
                  f'검출 평균 {[round(m,1) for m in means.tolist()]}. '
                  f'STABLE_STD_CNT/MIN_PLATEAU_S 조정 또는 --loads 재확인.')
        rep['channels'][f'ch{ch}'] = entry
        ax = axes[0][ci]
        ax.plot(t, y, lw=0.5, color=GRID)
        for (t0, t1, m, s) in plateaus:
            ax.hlines(m, t0, t1, color=PALETTE[0], lw=2.0)
        ax.set_xlabel('elapsed [s]'); ax.set_ylabel('loadcell raw [cnt]')
        ax.set_title(f'ch{ch} plateaus ({len(plateaus)})')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'loadcell_calib.png'), dpi=120)
    plt.close(fig)
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description='로드셀 영점·절대력 캘리브레이션 (test_plan Stage 1)')
    ap.add_argument('--zero', metavar='BAG', help='무부하 hold bag → 영점/노이즈 판정')
    ap.add_argument('--calib', metavar='BAG', help='기지하중 bag → cnt↔N 피팅')
    ap.add_argument('--loads', nargs='+', type=float, help='적용 하중 순서 [kgf] (무부하 0 포함)')
    ap.add_argument('--channel', type=int, choices=[0, 1], help='캘리브레이션 대상 채널(기본 둘 다)')
    ap.add_argument('--no-write-cal', action='store_true',
                    help='정준 loadcell_cal.json 기록 안 함 (traction_analysis 연동 차단)')
    ap.add_argument('--out', default=None, help='산출 폴더 (기본: bag 폴더/calib)')
    args = ap.parse_args(argv)
    if not (args.zero or args.calib):
        ap.error('--zero 또는 --calib 중 하나 필요')

    bag = args.zero or args.calib
    out_dir = args.out or os.path.join(os.path.dirname(os.path.normpath(bag)) or '.', 'calib_out')
    os.makedirs(out_dir, exist_ok=True)

    if args.calib and not args.loads:
        ap.error('--calib 에는 --loads 필요')
    rep = (run_zero(bag, out_dir) if args.zero
           else run_calib(bag, args.loads or [], out_dir, args.channel))
    with open(os.path.join(out_dir, 'loadcell_calib.json'), 'w') as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)

    # 절대력 캘리 성공 시 정준 파일(loadcell_cal.json)을 이 디렉터리에 기록 →
    # traction_analysis.py 가 읽어 로드셀을 [N] 으로 환산한다.
    if args.calib and not args.no_write_cal:
        tch = args.channel if args.channel is not None else 0
        chinfo = rep['channels'].get(f'ch{tch}', {})
        if chinfo.get('N_per_count'):
            here = os.path.dirname(os.path.abspath(__file__))
            cal = {'created': datetime.now().isoformat(timespec='seconds'),
                   'bag': rep['bag'], 'method': 'gravity_scale (중력 인가, 수평 캘리는 후속)',
                   'traction_channel': tch, 'channels': rep['channels']}
            cal_path = os.path.join(here, 'loadcell_cal.json')
            with open(cal_path, 'w') as f:
                json.dump(cal, f, indent=2, ensure_ascii=False)
            print(f'✔ 정준 캘리 기록: {cal_path} (ch{tch}, '
                  f'{chinfo["N_per_count"]:.4f} N/cnt) → traction_analysis 가 이제 [N] 로 출력')
        else:
            print(f'⚠ ch{tch} 피팅 실패 — loadcell_cal.json 미기록 (plateau/loads 확인)', file=sys.stderr)
    print(f'\n산출물 → {out_dir}/')
    return 0


if __name__ == '__main__':
    sys.exit(main())
