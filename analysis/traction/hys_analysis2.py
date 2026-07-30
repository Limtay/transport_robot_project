#!/usr/bin/env python3
"""히스테리시스 분석 2단계 — 마이너루프 닫힘 / 작동점별 루프폭 / 중첩 기억(wiping-out)."""
import json, os, re
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

CACHE = '/home/swarm/tp_ws/analysis/traction/hys_cache'
OUT = '/home/swarm/tp_ws/analysis/result'
IDX = json.load(open(os.path.join(CACHE, 'index.json')))

def runs(pat): return sorted([i for i in IDX if re.match(pat, i['name'])], key=lambda i: i['name'])
def load(i): return np.load(i['npz'])

fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))

# ── 1) 마이너루프: 0->14->floor->14->0. 반환점 기억: 재상승이 14A 에서 원래 하강 시작점으로 복귀? ──
print("1) 부분 마이너루프 — 반환점 기억 (return-point memory)")
ax = axes[0]
for fl, color in zip((10, 7, 4), ('tab:blue', 'tab:green', 'tab:red')):
    for k, i in enumerate(runs(rf'h_minor_f{fl:02d}_')):
        z = load(i); t, fb, F = z['t'], z['fb'], z['Ft']
        ax.plot(fb, F, lw=0.6, color=color, alpha=0.6, label=f'floor {fl}A' if k == 0 else None)
        # 첫 14A 도달 시 힘 vs 마이너루프 후 재도달 시 힘
        cmd = z['cmd']
        at14 = np.where(cmd > 13.95)[0]
        gaps = np.where(np.diff(at14) > 200)[0]           # 두 방문 분리
        if len(gaps):
            v1 = at14[:gaps[0]+1]; v2 = at14[gaps[0]+1:]
            F1 = F[v1[-400:]].mean() if len(v1) > 400 else F[v1].mean()   # 첫 방문 끝
            F2 = F[v2[:400]].mean() if len(v2) > 400 else F[v2].mean()    # 재방문 초
            print(f"  floor {fl:2d}A [{i['session'][-5:]}]: F(14A,1st)={F1:6.1f}N  F(14A,return)={F2:6.1f}N  Δ={F2-F1:+5.1f}N")
ax.set_title('minor loops: 0-14-floor-14-0'); ax.set_xlabel('I [A]'); ax.set_ylabel('F tared [N]'); ax.legend(fontsize=8)

# ── 2) 사인 밴드: 루프 폭 vs 작동점 (마지막 2주기 사용, 과도 제외) ──
print("2) 작동점별 사인밴드 — 정상상태 루프 폭")
ax = axes[1]
for lab, center, color in [('lo', 5.5, 'tab:blue'), ('mid', 8.5, 'tab:green'), ('hi', 11.5, 'tab:red')]:
    widths = []
    for k, i in enumerate(runs(rf'h_band_{lab}_')):
        z = load(i); t, fb, F, cmd = z['t'], z['fb'], z['Ft'], z['cmd']
        # sine 구간: cmd 가 center±2.5 를 오가는 구간 → 마지막 20s (2주기)
        m = (np.abs(cmd - center) < 2.6)
        idx = np.where(m)[0]
        if len(idx) < 1000: continue
        seg = idx[idx > idx[-1] - 4000]                     # 마지막 20s
        ax.plot(fb[seg], F[seg], lw=0.5, color=color, alpha=0.6, label=f'{lab} c={center}' if k == 0 else None)
        # 폭: center 에서 상행/하행 힘차
        dcmd = np.gradient(cmd)
        up = seg[(np.abs(cmd[seg] - center) < 0.3) & (dcmd[seg] > 0)]
        dn = seg[(np.abs(cmd[seg] - center) < 0.3) & (dcmd[seg] < 0)]
        if len(up) and len(dn): widths.append(F[dn].mean() - F[up].mean())
    print(f"  band {lab} (center {center}A): loop width @center = {np.mean(widths):5.1f} ± {np.std(widths):.1f} N (n={len(widths)})")
ax.set_title('sine bands (last 2 cycles): loop vs operating point'); ax.set_xlabel('I [A]'); ax.legend(fontsize=8)

# ── 3) 중첩 반전 nest1: wiping-out 정성 확인 ──
print("3) 중첩 반전 — nest1 (0-14-5-10-3-12-0)")
ax = axes[2]
for k, i in enumerate(runs(r'h_nest1_')):
    z = load(i)
    ax.plot(z['fb'], z['Ft'], lw=0.6, alpha=0.7, label=i['session'][-5:])
ax.set_title('nested reversals nest1 (3 sets overlay)'); ax.set_xlabel('I [A]'); ax.legend(fontsize=8)
for a in axes: a.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'hys2_minor_band_nest.png'), dpi=110)
print("saved", os.path.join(OUT, 'hys2_minor_band_nest.png'))
