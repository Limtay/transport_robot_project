#!/usr/bin/env python3
"""로드셀 **간이** 재캘리브레이션 — span/offset 을 손댄 뒤 상수만 다시 잡을 때.

Stage 1(2026-07-21)에서 선형성은 이미 확증됐다 (R²=0.999988, 46점). 앰프 span 을
조절했다면 **직선의 기울기·절편만** 바뀌므로, 46점을 다시 뜰 필요 없이 무게추 2개로
`- → 1 → 1+2 → 1 → -` 한 사이클만 돌리면 된다. 상행·하행이 겹치므로 히스테리시스와
영점 복귀까지 같은 사이클에서 나온다.

입력은 라이브 로거가 남긴 CSV (`t_wall,ecu_tick,lc0,lc1,valid`) 와 단계별 창(steps.json).

  python3 loadcell_recal_quick.py TRACE.csv STEPS.json -o OUT.png [--json OUT.json]

steps.json = [{"idx","label","kg","t0","t1"}, ...] — 각 단계의 plateau 창(초).
"""
import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

G = 9.80665
RAIL_LOW = 8.0          # 이 밑은 단선 오검출 위험 (loadcell_calib.py 와 같은 기준)
ZERO_MIN_CNT = 300.0    # test_plan Stage 1 무부하 하한

PALETTE = ["#2a78d6", "#1baf7a", "#eda100", "#4a3aa7", "#e34948"]
TEXT2, GRID = "#5f5e56", "#e5e4dc"
# 라벨이 한국어다. matplotlib 의 폰트 캐시는 시스템(fontconfig)이 아는 나눔폰트를 못 볼 때가
# 있어 **경로로 직접 등록**한다 — 이름만 지정하면 조용히 DejaVu 로 폴백해 두부(□)가 된다.
for _p in ('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
           '/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf'):
    if os.path.exists(_p):
        matplotlib.font_manager.fontManager.addfont(_p)

plt.rcParams.update({'font.family': ['NanumGothic', 'NanumBarunGothic', 'DejaVu Sans'],
                     'axes.unicode_minus': False,
                     'figure.facecolor': 'white', 'axes.facecolor': 'white',
                     'axes.edgecolor': GRID, 'axes.grid': True, 'grid.color': GRID,
                     'grid.linewidth': 0.6, 'font.size': 9, 'axes.spines.top': False,
                     'axes.spines.right': False, 'legend.frameon': False})


def fit(F, cnt):
    """cnt = k·F + b 최소자승 → (k, b, r2, resid)."""
    k, b = np.polyfit(F, cnt, 1)
    pred = k * F + b
    resid = cnt - pred
    ss_res = float((resid ** 2).sum())
    ss_tot = float(((cnt - cnt.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return k, b, r2, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trace'); ap.add_argument('steps')
    ap.add_argument('-o', '--out', default='recal.png')
    ap.add_argument('--json', dest='js')
    ap.add_argument('--old-k', type=float, default=12.626969553707289,
                    help='비교용 이전 감도 [cnt/N]')
    a = ap.parse_args()

    d = np.genfromtxt(a.trace, delimiter=',', names=True)
    t, lc0, lc1 = d['t_wall'], d['lc0'], d['lc1']
    steps = json.load(open(a.steps))

    for s in steps:                                  # 창 안의 통계를 다시 계산(재현성)
        m = (t >= s['t0']) & (t <= s['t1'])
        for ch, y in (('ch0', lc0), ('ch1', lc1)):
            s[ch] = float(y[m].mean()); s[ch + '_std'] = float(y[m].std())
        s['n'] = int(m.sum())

    kg = np.array([s['kg'] for s in steps]); F = kg * G
    c0 = np.array([s['ch0'] for s in steps]); c1 = np.array([s['ch1'] for s in steps])
    k, b, r2, resid = fit(F, c0)
    k1, b1, r21, _ = fit(F, c1)

    up   = [s for s in steps if s['idx'] == 2][0]['ch0']
    down = [s for s in steps if s['idx'] == 4][0]['ch0']
    hyst_N = (down - up) / k
    zret_N = (steps[-1]['ch0'] - steps[0]['ch0']) / k

    res = {
        'created': None, 'method': 'gravity_scale / 2-weight single cycle (간이 재캘리)',
        'weights_kg': sorted({s['kg'] for s in steps if s['kg'] > 0}),
        'ch0': {'counts_per_N': float(k), 'N_per_count': float(1 / k),
                'offset_cnt': float(b), 'r2': float(r2),
                'resid_std_cnt': float(resid.std()), 'resid_max_N': float(np.abs(resid).max() / k),
                'n_points': len(steps),
                'noise_std_cnt': float(np.mean([s['ch0_std'] for s in steps])),
                'hysteresis_N': float(hyst_N), 'zero_return_N': float(zret_N),
                'span_ratio_vs_old': float(k / a.old_k)},
        'ch1': {'counts_per_N': float(k1), 'offset_cnt': float(b1), 'r2': float(r21)},
        'steps': steps,
    }
    if a.js:
        json.dump(res, open(a.js, 'w'), ensure_ascii=False, indent=1)

    # ── 플롯 ────────────────────────────────────────────────
    fig = plt.figure(figsize=(12, 7.2))
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 1], hspace=0.34, wspace=0.28)

    ax = fig.add_subplot(gs[0, :])                   # 전체 시계열 + 단계 창
    ax.plot(t, lc0, lw=0.7, color=PALETTE[0], label='ch0')
    ax.plot(t, lc1, lw=0.7, color=PALETTE[1], alpha=0.75, label='ch1')
    for s in steps:
        ax.axvspan(s['t0'], s['t1'], color=PALETTE[2], alpha=0.30, lw=0)
        ax.annotate(f"{s['idx']}\n{s['kg']:.2f}kg", ((s['t0'] + s['t1']) / 2, 1.02),
                    xycoords=('data', 'axes fraction'), ha='center', va='bottom',
                    fontsize=8, color=TEXT2)
    ax.set_xlabel('t [s]'); ax.set_ylabel('raw [cnt]')
    # 단계 라벨이 축 위에 붙으므로 제목을 그 위로 밀어낸다 (안 밀면 겹친다)
    ax.set_title('단계별 원시 신호 (200 Hz) — 음영 = 평균 낸 plateau 5 s', loc='left', pad=24)
    ax.legend(loc='upper left')

    ax = fig.add_subplot(gs[1, 0])                   # 캘리 직선
    xs = np.linspace(0, F.max() * 1.06, 50)
    ax.plot(xs, k * xs + b, color=TEXT2, lw=1.2, zorder=1,
            label=f'k={k:.3f} cnt/N\nb={b:.2f} cnt\nR²={r2:.7f}')
    for s in steps:
        up_leg = s['idx'] <= 3
        ax.scatter(s['kg'] * G, s['ch0'], s=52, zorder=3,
                   marker='o' if up_leg else '^',
                   color=PALETTE[0] if up_leg else PALETTE[4])
    ax.set_xlabel('힘 [N]'); ax.set_ylabel('ch0 [cnt]')
    ax.set_title('cnt ↔ N (○ 상행 / △ 하행)', loc='left'); ax.legend(loc='upper left')

    ax = fig.add_subplot(gs[1, 1])                   # 잔차
    ax.axhline(0, color=TEXT2, lw=0.8)
    ax.bar([f"{s['idx']}" for s in steps], resid / k, color=PALETTE[3], width=0.55)
    for i, s in enumerate(steps):
        ax.annotate(f'{resid[i] / k:+.3f}', (i, resid[i] / k), ha='center',
                    va='bottom' if resid[i] > 0 else 'top', fontsize=8, color=TEXT2)
    ax.set_xlabel('단계'); ax.set_ylabel('잔차 [N]')
    ax.set_title(f'직선 잔차 — max {np.abs(resid).max() / k:.3f} N', loc='left')

    ax = fig.add_subplot(gs[1, 2]); ax.axis('off')   # 요약표
    noise_N = np.mean([s['ch0_std'] for s in steps]) / k
    rows = [('감도 k', f'{k:.3f} cnt/N'),
            ('', f'{1 / k:.5f} N/cnt'),
            ('offset', f'{b:.2f} cnt'),
            ('span 배율(이전 대비)', f'{k / a.old_k:.3f}×'),
            ('R²', f'{r2:.7f}'),
            ('잔차 std', f'{resid.std():.2f} cnt = {resid.std() / k:.3f} N'),
            ('노이즈(plateau std)', f'{noise_N:.3f} N'),
            ('히스테리시스', f'{hyst_N:+.3f} N'),
            ('영점 복귀', f'{zret_N:+.3f} N'),
            ('무부하 cnt', f"{steps[0]['ch0']:.1f}"
                           + ('  (!) 레일 근접' if steps[0]['ch0'] < ZERO_MIN_CNT else ''))]
    for i, (a_, b_) in enumerate(rows):
        y = 0.95 - i * 0.093
        ax.text(0.0, y, a_, fontsize=9, color=TEXT2, transform=ax.transAxes)
        ax.text(1.0, y, b_, fontsize=9, ha='right', weight='bold', transform=ax.transAxes)
    ax.set_title('요약', loc='left')

    fig.savefig(a.out, dpi=140, bbox_inches='tight')
    print(f'k={k:.4f} cnt/N  b={b:.3f} cnt  R2={r2:.8f}  resid_std={resid.std():.3f} cnt')
    print(f'hyst={hyst_N:+.4f} N  zero_return={zret_N:+.4f} N  -> {a.out}')


if __name__ == '__main__':
    main()
