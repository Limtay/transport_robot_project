#!/usr/bin/env python3
"""램프 사이클 bag → F(I) 곡선 · 지표 · 플롯.

`t4_ramp_cycle` / `t4_probe` 처럼 **hold → ramp up → hold → ramp down → hold** 구조의 런을
읽어 상행·하행을 나누고 지표를 뽑는다.

  python3 tools/ramp_analysis.py data/rosbags/t4_p1_w40_r01_08-21_21-00 [-o out.png]
  python3 tools/ramp_analysis.py RUN1 RUN2 RUN3 --label p1_w40   # 셀 = 반복 묶음

상행/하행 분리는 **`segment_index` 로** 한다 (전류 미분 부호가 아니라) — 홀드 구간의 노이즈로
부호가 흔들리는 문제를 원천적으로 피한다. 브리지가 세그먼트 번호를 200 Hz 로 실어 준다.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import bagio                                     # noqa: E402
import calib                                     # noqa: E402
from plotstyle import plt, PALETTE, TEXT2, MONO  # noqa: E402

MOTOR = 0                 # m1 = 인덱스 0
LC_CH = 0
BIN_A = 0.5               # F(I) 빈 폭
SLOPE_BAND = (8.0, 13.0)  # 기울기 산출 구간 [A] — 07-22 와 같은 밴드
GAP_AT_A = 7.0            # 이력 갭을 읽는 전류
DEADBAND_N = 5.0          # 이 힘을 넘는 첫 전류 = 데드밴드 I0
# 슬립 판정은 **위치 누적 이동**으로 한다. fb_velocity 는 10 RPM 단위로 양자화돼 있어
# (실측 고유값 …,-20,-10,0,10,20,…) max|vel| 은 최소 검출단위 한두 칸에 쉽게 걸린다 —
# 2026-08-21 프로브에서 max 20 RPM 이 떴지만 전 런 위치 이동은 +1.6°(벨트 0.8 mm)로
# 탄성 take-up 이었다. 실제로 슬립하면 위치가 계속 밀린다.
SLIP_HOLD_DEG = 5.0       # 정점 홀드 중 이만큼 밀리면 슬립 의심
VEL_QUANT_RPM = 10.0      # 관측된 속도 양자 (참고용)
ABORT_N = 190.0


def analyze(run_dir):
    d = bagio.load_bag(run_dir)
    i_cmd = d['cmd'][:, MOTOR]
    lc = d['lc'][:, LC_CH]
    seg = d['segment_index']
    vel = d['fb_velocity'][:, MOTOR]

    segs = np.unique(seg)
    if len(segs) < 5:
        raise ValueError(f'세그먼트 {len(segs)}개 — 램프 사이클(5개)이 아니다: {run_dir}')
    s_tare, s_rise, s_top, s_fall, s_end = segs[0], segs[1], segs[2], segs[3], segs[4]

    tare = float(np.nanmean(lc[seg == s_tare]))          # 첫 hold = 영점
    F = calib.scale_N(lc - tare)                         # 상대력 [N]

    def curve(mask):
        """전류 빈별 힘 평균 → (I, F)."""
        ii, ff = i_cmd[mask], F[mask]
        edges = np.arange(0, np.nanmax(i_cmd) + BIN_A, BIN_A)
        idx = np.digitize(ii, edges) - 1
        out_i, out_f = [], []
        for b in range(len(edges) - 1):
            m = idx == b
            if m.sum() >= 3:
                out_i.append(float(np.nanmean(ii[m]))); out_f.append(float(np.nanmean(ff[m])))
        return np.array(out_i), np.array(out_f)

    Ir, Fr = curve(seg == s_rise)
    If, Ff = curve(seg == s_fall)

    band = (Ir >= SLOPE_BAND[0]) & (Ir <= SLOPE_BAND[1])
    slope = float(np.polyfit(Ir[band], Fr[band], 1)[0]) if band.sum() >= 3 else float('nan')
    over = Ir[Fr > DEADBAND_N]
    i0 = float(over[0]) if len(over) else float('nan')
    peak = float(np.nanmax(F[(seg == s_rise) | (seg == s_top)]))
    # curve() 는 전류 빈 순서대로 내므로 If/Ir 은 **둘 다 오름차순**이다. np.interp 는 xp 가
    # 오름차순일 것을 요구한다 — 하행이라고 뒤집으면 보간이 망가진다(갭 부호가 뒤집혔던 자리).
    gap = (float(np.interp(GAP_AT_A, If, Ff) - np.interp(GAP_AT_A, Ir, Fr))
           if len(If) and len(Ir) else float('nan'))
    ret = float(np.nanmean(F[seg == s_end]))
    hold = (seg == s_top)
    pos = d['fb_position'][:, MOTOR]
    slip_deg = float(pos[hold][-1] - pos[hold][0]) if hold.any() else float('nan')
    travel_deg = float(pos[-1] - pos[0])
    vel_max = float(np.nanmax(np.abs(vel[hold]))) if hold.any() else float('nan')

    return {
        'run': os.path.basename(os.path.normpath(run_dir)),
        'n': d['n'], 'dur_s': round(float(d['t'][-1]), 2),
        'tare_cnt': round(tare, 1),
        'peak_N': round(peak, 2), 'peak_I_A': round(float(np.nanmax(i_cmd)), 2),
        'slope_N_per_A': round(slope, 3), 'deadband_I0_A': round(i0, 2),
        f'gap_at_{GAP_AT_A:.0f}A_N': round(gap, 2),
        'baseline_return_N': round(ret, 2),
        'slip_hold_deg': round(slip_deg, 2),     # ★ 슬립 판정은 이것
        'travel_deg': round(travel_deg, 2),
        'vel_max_rpm': round(vel_max, 1),        # 참고 (10 RPM 양자)
        'slope_band_A': [round(float(Ir[band].min()), 2), round(float(Ir[band].max()), 2)]
                        if band.sum() >= 3 else None,
        'abort_exceeded': bool(peak > ABORT_N),
        'rw_err_nonzero': int((d['rw_err'] != 0).sum()),
        '_curves': {'I_rise': Ir.tolist(), 'F_rise': Fr.tolist(),
                    'I_fall': If.tolist(), 'F_fall': Ff.tolist()},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--label', default=None)
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--json', dest='js', default=None)
    a = ap.parse_args()

    res = [analyze(r) for r in a.runs]
    label = a.label or res[0]['run']

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.6), gridspec_kw={'wspace': 0.26})
    for k, r in enumerate(res):
        c = r['_curves']; col = PALETTE[k % len(PALETTE)]
        ax.plot(c['I_rise'], c['F_rise'], color=col, lw=1.5, label=f"{r['run']} 상행")
        ax.plot(c['I_fall'], c['F_fall'], color=col, lw=1.5, ls='--', alpha=0.8)
    ax.axhline(ABORT_N, color=PALETTE[4], ls=':', lw=1.0)
    ax.set_xlabel('cmd current [A]'); ax.set_ylabel('힘 [N] (tare 후)')
    ax.set_title(f'{label} — 실선 상행 / 파선 하행', loc='left')
    if len(res) <= 3:
        ax.legend(fontsize=8, loc='upper left')

    keys = ['peak_N', 'slope_N_per_A', 'deadband_I0_A', f'gap_at_{GAP_AT_A:.0f}A_N',
            'baseline_return_N', 'slip_hold_deg', 'travel_deg']
    ax2.axis('off')
    ax2.text(0, 1.0, f'{"지표":<22}' + ''.join(f'{r["run"][-6:]:>11}' for r in res),
             family=MONO, fontsize=8.5, va='top', transform=ax2.transAxes)
    for j, k in enumerate(keys):
        vals = [r[k] for r in res]
        line = f'{k:<22}' + ''.join(f'{v:>11.2f}' for v in vals)
        if len(vals) > 1:
            line += f'   CV {100 * np.std(vals) / max(1e-9, abs(np.mean(vals))):5.2f}%'
        ax2.text(0, 0.90 - j * 0.085, line, family=MONO, fontsize=8.5,
                 va='top', transform=ax2.transAxes, color=TEXT2)
    warn = [f"{r['run']}: abort 초과" for r in res if r['abort_exceeded']]
    warn += [f"{r['run']}: 슬립 의심 (홀드 중 {r['slip_hold_deg']}°)" for r in res
             if abs(r['slip_hold_deg']) > SLIP_HOLD_DEG]
    warn += [f"{r['run']}: rw_err {r['rw_err_nonzero']}건" for r in res if r['rw_err_nonzero']]
    ax2.text(0, 0.90 - len(keys) * 0.085 - 0.06,
             '\n'.join(warn) if warn else '이상 없음',
             fontsize=9, va='top', transform=ax2.transAxes,
             color=(PALETTE[4] if warn else PALETTE[1]), weight='bold')

    out = a.out or os.path.join('result', f'{label}.png')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches='tight')

    for r in res:
        print(json.dumps({k: v for k, v in r.items() if k != '_curves'}, ensure_ascii=False))
    print(f'-> {out}')
    if a.js:
        json.dump(res, open(a.js, 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
