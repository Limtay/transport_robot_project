#!/usr/bin/env python3
"""히스테리시스 분석 3단계 — 이산 Preisach 모델 회귀 + 고전 PI 비교 + 미학습 궤적 검증.

모델: F(t) = Σ w_ab · R_ab[u](t) + w0,  R_ab = 릴레이(스위치업 a, 스위치다운 b, a>b)
      가중치 w ≥ 0 은 NNLS — 선형회귀 문제 (러닝 불필요 가설의 검정).
훈련: hys 세트(07-22)의 forc + desc + minor (51런). 검증: glide + nest + band (21런, 미학습 파형).
비교: 고전 PI(play 연산자, 대각 Preisach) — 루프폭=작동점 의존을 못 잡는지 확인.
입력 u = fb 전류(200Hz→20Hz 서브샘플), 출력 = tare 된 F[N].
"""
import json, os, re
import numpy as np
from scipy.optimize import nnls
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

CACHE = '/home/swarm/tp_ws/analysis/traction/hys_cache'
OUT = '/home/swarm/tp_ws/analysis/result'
IDX = json.load(open(os.path.join(CACHE, 'index.json')))
def runs(pat): return sorted([i for i in IDX if re.match(pat, i['name'])], key=lambda i: i['name'])

SUB = 10                      # 200Hz -> 20Hz
LV = np.arange(0.0, 14.5, 0.7)          # 스위치 레벨 격자 (21)
PAIRS = [(a, b) for a in LV for b in np.concatenate([[-0.7], LV]) if a > b]   # b=-0.7: 런 내 비가역
NP_ = len(PAIRS)
A = np.array([p[0] for p in PAIRS]); B = np.array([p[1] for p in PAIRS])

def relay_matrix(u):
    """u(T) -> R(T,NP_): 릴레이 상태 0/1, 초기 0 (런 시작 = 0A tare 상태)."""
    T = len(u); R = np.zeros((T, NP_), np.float32); s = np.zeros(NP_, np.float32)
    for k in range(T):
        s = np.where(u[k] >= A, 1.0, np.where(u[k] <= B, 0.0, s))
        R[k] = s
    return R

def play_matrix(u, radii):
    T = len(u); P = np.zeros((T, len(radii)), np.float32); z = np.zeros(len(radii))
    for k in range(T):
        z = np.maximum(u[k] - radii, np.minimum(u[k] + radii, z))
        P[k] = z
    return P

RADII = np.arange(0.0, 13.0, 0.65)      # PI play 반경 격자 (20)

def get(i):
    z = np.load(i['npz'])
    return z['fb'][::SUB].astype(np.float64), z['Ft'][::SUB].astype(np.float64)

TRAIN = runs(r'h_forc_') + runs(r'h_desc_') + runs(r'h_minor_')
VAL = {'glide': runs(r'h_glide'), 'nest': runs(r'h_nest'), 'band': runs(r'h_band_')}
print(f"train {len(TRAIN)} runs / val {sum(len(v) for v in VAL.values())} runs / relays {NP_} / plays {len(RADII)}")

# ── 설계행렬 구축 ──
Xp_tr, Xi_tr, y_tr = [], [], []
for i in TRAIN:
    u, F = get(i)
    Xp_tr.append(relay_matrix(u)); Xi_tr.append(play_matrix(u, RADII)); y_tr.append(F)
Xp = np.vstack(Xp_tr); Xi = np.vstack(Xi_tr); y = np.concatenate(y_tr)
ones = np.ones((len(y), 1), np.float32)

# Preisach NNLS (bias 는 자유부호 → [1,-1] 두 열)
Wp, _ = nnls(np.hstack([Xp, ones, -ones]), y)
resP = np.hstack([Xp, ones, -ones]) @ Wp - y
# PI 최소자승 (ridge 소량)
Xpi = np.hstack([Xi, ones])
Wpi = np.linalg.lstsq(Xpi.T @ Xpi + 1e-6 * np.eye(Xpi.shape[1]), Xpi.T @ y, rcond=None)[0]
resI = Xpi @ Wpi - y
print(f"TRAIN RMSE:  Preisach(NNLS) = {np.sqrt((resP**2).mean()):.2f} N   |   classical PI = {np.sqrt((resI**2).mean()):.2f} N")

# ── 검증 ──
print("VALIDATION (미학습 파형):")
report = {}
fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
for ax, (fam, items) in zip(axes, VAL.items()):
    rmsP, rmsI = [], []
    for k, i in enumerate(items):
        u, F = get(i)
        fhP = np.hstack([relay_matrix(u), np.ones((len(u),1)), -np.ones((len(u),1))]) @ Wp
        fhI = np.hstack([play_matrix(u, RADII), np.ones((len(u),1))]) @ Wpi
        rmsP.append(np.sqrt(((fhP - F)**2).mean())); rmsI.append(np.sqrt(((fhI - F)**2).mean()))
        if k == 0:
            tt = np.arange(len(u)) * SUB / 200.0
            ax.plot(tt, F, 'k', lw=1.0, label='measured')
            ax.plot(tt, fhP, 'r', lw=0.9, alpha=0.8, label='Preisach')
            ax.plot(tt, fhI, 'b', lw=0.8, alpha=0.6, label='classical PI')
            ax.set_title(f'{fam}: {i["name"][:16]}'); ax.set_xlabel('t [s]'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    report[fam] = (np.mean(rmsP), np.mean(rmsI))
    print(f"  {fam:6s}: Preisach RMSE = {np.mean(rmsP):5.2f} N (n={len(items)})   PI = {np.mean(rmsI):5.2f} N")
axes[0].set_ylabel('F tared [N]')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'hys3_model_validation.png'), dpi=110)

# 상대오차 (풀스케일 175N 기준)
fs = 175.0
print(f"→ 풀스케일 대비: Preisach {100*np.mean([r[0] for r in report.values()])/fs:.1f}%  "
      f"/ PI {100*np.mean([r[1] for r in report.values()])/fs:.1f}%  (반복성 바닥 ~2.8N = {100*2.8/fs:.1f}%)")
np.savez(os.path.join(CACHE, 'preisach_model.npz'), A=A, B=B, W=Wp, LV=LV, sub=SUB)
print("saved model ->", os.path.join(CACHE, 'preisach_model.npz'))
print("saved plot  ->", os.path.join(OUT, 'hys3_model_validation.png'))
