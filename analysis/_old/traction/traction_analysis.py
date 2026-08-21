#!/usr/bin/env python3
"""Traction mapping 실험 배치 분석 파이프라인.

test_index.csv 에 등록된 rosbag(TractionTest 토픽)들을 일괄 처리해서
전류-견인력(로드셀) 매핑의 선형성·반복성·노이즈를 정량화한다.
비교 대상은 lc[LC_CH] ↔ fb_current[MOTOR_IDX] 한 쌍으로 고정.

사용법:
    python3 traction_analysis.py [test_index.csv] [--out OUT_DIR]

test_index.csv 컬럼 (bag 경로는 CSV 위치 기준 상대경로 허용):
    bag,payload_kg,velocity_mps,body_angle_deg,contact_point,ramp_slope_aps,note

bag 하나당 산출물 (out/<bag이름>/):
    timeseries.png   시간축 sanity (전류 cmd/fb + 로드셀 tared)
    scatter_fit.png  로드셀 vs fb전류, 램프별 + 데드밴드 피팅 + 빈 평균±σ 리본
    residuals.png    피팅 잔차 vs 전류
    hysteresis.png   램프별 rise vs fall 빈 곡선 + 갭
캠페인 전체 (out/):
    summary.csv      램프별 slope/I0/R²/노이즈/히스테리시스/크리프/베이스라인
    compare.png      조건(bag)별 slope·I0 비교
    repeat_<pos>.png 같은 시작위치 반복 bag 들의 rise 곡선 오버레이 + σ
    position.png     시작위치(p1~p4) 그룹 평균 곡선 비교
    sprocket.png     램프 시작 스프로켓 위치 vs slope/deadband
    verdict.csv      맵 전략별(전역/위치별/위상별) 예측 오차 분해

의존성: numpy, matplotlib (ROS 불필요 — sqlite3 + 수동 CDR 디코딩)

주의: 200Hz 발행 중 STM 100Hz 갱신으로 인한 중복 샘플(2026-07-07 이전 bag)은
tick 기준으로 자동 제거된다. STM 200Hz 수정 이후 bag 도 동일 코드로 처리 가능.
"""
import argparse
import csv
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

# ── 비교 대상 (실험 셋업 변경 시 여기만 수정) ───────────────────────────────
MOTOR_IDX = 1     # 램프 인가 모터 배열 idx (cmd/fb_current[MOTOR_IDX])
LC_CH     = 0     # 견인력 측정 로드셀 채널 (loadcell_raw[LC_CH])

# ── 로드셀 cnt→N 캘리브레이션 (loadcell_calib.py 가 생성한 loadcell_cal.json) ──
# 있으면 로드셀을 [N] 으로 환산해 전 분석/플롯 단위가 N 이 된다. 없으면 raw cnt 유지(하위호환).
# 절대력은 tare(baseline 차감) 후라 offset 은 상쇄 — 스케일(N_per_count)만 쓴다.
LC_CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'loadcell_cal.json')


def _load_lc_cal():
    if os.path.isfile(LC_CAL_FILE):
        try:
            with open(LC_CAL_FILE) as f:
                c = json.load(f)
            ch = c.get('traction_channel', LC_CH)
            npc = c['channels'][f'ch{ch}'].get('N_per_count')
            if npc:
                return float(npc)
        except (KeyError, ValueError, TypeError):
            pass
    return None


LC_N_PER_CNT = _load_lc_cal()          # None → cnt 단위
U = 'N' if LC_N_PER_CNT else 'cnt'     # 사용자-대면 힘 단위 라벨

# ── 상수 ────────────────────────────────────────────────────────────────────
CMD_ACTIVE_A     = 0.05   # 이 이상이면 램프 활성 구간
RAMP_MIN_S       = 2.0    # 이보다 짧은 활성 구간은 램프로 안 봄
TARE_MARGIN_S    = 0.5    # tare 창: 첫 램프 시작 이 시간 전까지
SETTLE_S         = 1.0    # 램프 종료 후 베이스라인 판정 전 안정화 대기
BASELINE_WARN    = 50.0 * (LC_N_PER_CNT or 1.0)  # 픽스처 밀림 플래그 임계 (50cnt 를 단위 환산)
I0_GRID_STEP_A   = 0.05   # 데드밴드 I0 그리드 탐색 간격
FIT_BIN_A        = 0.25   # 반복성 리본의 전류 빈 폭
MOVAVG_N         = 25     # 역학 노이즈 산출용 이동평균 창 (@100Hz = 0.25 s)
GRID_MAX_A       = 16.0   # 곡선 비교용 공통 전류 그리드 상한
I_GRID = np.arange(0.0, GRID_MAX_A + FIT_BIN_A, FIT_BIN_A)  # 공통 빈 경계
HOLD_MIN_S       = 2.0    # 크리프 판정 최소 hold 길이
BIN_MIN_N        = 5      # 빈 평균에 필요한 최소 샘플 수

# dataviz 검증 팔레트 (light) — 램프/시리즈 식별용
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


def _decode(blob):
    """구 TractionTest.msg CDR 디코딩 → (tick, sys, f16, lc2, goal_id=0).

    tick 단위는 구펌웨어 규약(×1ms). loadcell 은 uint16. goal_id 개념 없음(항상 0)."""
    d = blob[4:]                                   # 4B encapsulation 헤더 스킵
    o = 0
    o += 8                                         # header.stamp (sec+nanosec)
    slen = struct.unpack_from('<I', d, o)[0]
    o += 4 + slen                                  # frame_id (null 포함)
    o = _align(o, 4)
    tick = struct.unpack_from('<I', d, o)[0]; o += 4
    sys_state = d[o]; o += 1
    o = _align(o, 4)
    f = struct.unpack_from('<16f', d, o); o += 64  # cmd/fb_cur/fb_vel/fb_pos ×4
    lc = struct.unpack_from('<2H', d, o)
    return tick, sys_state, f, lc, 0


def _decode_testbed(blob):
    """신 TestbedFeedback.msg CDR 디코딩 → (tick, sys, f16, lc2, goal_id).

    구 _decode 와 반환 형태 호환(goal_id 만 추가). tick 은 ×0.1ms(신펌웨어),
    loadcell 은 int32. cmd/fb 배열 순서는 TractionTest 와 동일(cmd/fb_cur/fb_vel/fb_pos).
    필드 정의: mgs01_base_msgs/msg/TestbedFeedback (testbed_spec §3.4)."""
    d = blob[4:]
    o = 0
    o += 8                                         # header.stamp
    slen = struct.unpack_from('<I', d, o)[0]
    o += 4 + slen                                  # frame_id
    o = _align(o, 4)
    tick = struct.unpack_from('<I', d, o)[0]; o += 4   # ecu_tick_ms (×0.1ms)
    sys_state = d[o]; o += 3                        # sys_state, testbed_state, motor_mask
    o = _align(o, 4)
    goal_id = struct.unpack_from('<I', d, o)[0]; o += 4
    o += 4                                          # profile_time (float32)
    o += 2                                          # segment_index (uint16)
    o = _align(o, 4)
    f = struct.unpack_from('<16f', d, o); o += 64  # cmd/fb_cur/fb_vel/fb_pos ×4
    lc = struct.unpack_from('<2i', d, o)           # loadcell_raw int32×2
    return tick, sys_state, f, lc, goal_id


def _binned(x, y, grid=I_GRID):
    """공통 그리드 빈 평균 곡선. 샘플 < BIN_MIN_N 빈은 NaN."""
    bi = np.digitize(x, grid)
    out = np.full(len(grid) - 1, np.nan)
    for k in range(1, len(grid)):
        m = bi == k
        if m.sum() >= BIN_MIN_N:
            out[k - 1] = y[m].mean()
    return out


BC = I_GRID[:-1] + FIT_BIN_A / 2   # 빈 중심


def _detect_format(con):
    """topics 테이블로 bag 포맷·대상 토픽 판별.

    반환: (decode_fn, tick_scale_s, topic_id | None).
    신 bag 은 feedback + comm_latency 2토픽이라 feedback 만 골라야 하므로 topic_id 필수.
    구 bag 은 단일 TractionTest 토픽이라 topic_id=None(전체 스캔)."""
    for tid, name, typ in con.execute('SELECT id, name, type FROM topics').fetchall():
        if 'TestbedFeedback' in (typ or '') or name.endswith('/feedback'):
            return _decode_testbed, 1e-4, tid          # ×0.1ms tick
    return _decode, 1e-3, None                          # 구 TractionTest(×1ms), 전체 스캔


def load_bag(bag_dir):
    """bag 디렉토리 → dict(tick, t, sys_state, cmd, fb, lc, pos, goal_id). 중복 tick 제거.

    구 TractionTest / 신 TestbedFeedback 양쪽 자동 판별(§6 #9). cmd/fb 는 MOTOR_IDX 모터,
    lc 는 LC_CH 채널만 추출(비교 대상 고정). 신 bag 은 goal_id 로 실험 자동 분할 가능."""
    db3 = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not db3:
        raise FileNotFoundError(f'no .db3 in {bag_dir}')
    decode_fn, tick_scale, rows = None, 1e-3, []
    for path in db3:
        con = sqlite3.connect(path)
        decode_fn, tick_scale, tid = _detect_format(con)
        if tid is not None:
            rows += con.execute('SELECT data FROM messages WHERE topic_id=? '
                                'ORDER BY timestamp', (tid,)).fetchall()
        else:
            rows += con.execute('SELECT data FROM messages ORDER BY timestamp').fetchall()
        con.close()

    n = len(rows)
    tick = np.empty(n, np.int64)
    sys_state = np.empty(n, np.uint8)
    cmd = np.empty(n, np.float64)
    fb = np.empty(n, np.float64)
    pos = np.empty(n, np.float64)
    lc = np.empty(n, np.float64)
    gid = np.empty(n, np.int64)
    for i, (blob,) in enumerate(rows):
        tk, ss, f, l, g = decode_fn(blob)
        tick[i], sys_state[i], gid[i] = tk, ss, g
        cmd[i], fb[i], lc[i] = f[MOTOR_IDX], f[4 + MOTOR_IDX], l[LC_CH]
        pos[i] = f[12 + MOTOR_IDX]

    if LC_N_PER_CNT:
        lc = lc * LC_N_PER_CNT                             # cnt → N (offset 은 tare 로 상쇄)
    keep = np.concatenate([[True], np.diff(tick) != 0])   # STM 100Hz 중복(구 bag) 제거
    dup_ratio = 1.0 - keep.mean()
    d = dict(tick=tick[keep], sys_state=sys_state[keep],
             cmd=cmd[keep], fb=fb[keep], lc=lc[keep], pos=pos[keep],
             goal_id=gid[keep], dup_ratio=dup_ratio, n_raw=n)
    d['t'] = (d['tick'] - d['tick'][0]) * tick_scale
    return d


def split_by_goal(d):
    """신 bag 을 goal_id 로 실험 단위 분할(§6 #9, §3.4 자동 분할 키).

    goal_id>0 인 연속 구간마다 하나의 실험 dict 를 만든다. IDLE(goal_id=0) 샘플은
    앞뒤로 TARE_MARGIN_S 만큼만 포함(분석기 tare 창 보존). 구 bag(goal_id 전부 0)은
    분할 없이 원본 1개를 그대로 반환한다."""
    gid = d['goal_id']
    if not np.any(gid > 0):
        return [d]
    t = d['t']
    segs = []
    for g in sorted(set(gid[gid > 0].tolist())):
        idx = np.where(gid == g)[0]
        t0, t1 = t[idx[0]] - TARE_MARGIN_S, t[idx[-1]] + SETTLE_S
        m = (t >= t0) & (t <= t1)
        sub = {k: (v[m] if isinstance(v, np.ndarray) and v.shape[:1] == gid.shape else v)
               for k, v in d.items()}
        sub['goal_id_active'] = int(g)
        segs.append(sub)
    return segs


# ── 구간 분리 / 피팅 ────────────────────────────────────────────────────────
def find_ramps(cmd, t):
    """활성(cmd>임계) 연속 구간 목록. 각 구간을 rise/hold/fall 위상으로 라벨."""
    active = cmd > CMD_ACTIVE_A
    edges = np.where(np.diff(active.astype(int)) != 0)[0]
    ramps = []
    for seg in np.split(np.arange(len(cmd)), edges + 1):
        if not active[seg[0]] or (t[seg[-1]] - t[seg[0]]) < RAMP_MIN_S:
            continue
        dc = np.gradient(cmd[seg], t[seg])         # A/s — 램프 기울기 스케일
        if len(dc) > 51:                           # 미분 노이즈 스무딩 (~0.25 s)
            dc = np.convolve(dc, np.ones(51) / 51, mode='same')
        phase = np.where(dc > 0.1, 1, np.where(dc < -0.1, -1, 0))
        # 유의미한 run(전류 변화 ≥1A)의 방향 전환 횟수로 램프/임의 전류 구분.
        # 계단 램프의 hold 진입 시 역방향 blip(<1A)은 무시된다.
        signs = []
        for run in np.split(np.arange(len(phase)),
                            np.where(np.diff(phase) != 0)[0] + 1):
            c = cmd[seg[run]]
            if phase[run[0]] != 0 and abs(c[-1] - c[0]) >= 1.0:
                signs.append(phase[run[0]])
        flips = int(np.sum(np.diff(signs) != 0)) if len(signs) > 1 else 0
        ramps.append(dict(idx=seg, phase=phase, random=flips > 4))
    return ramps


def fit_deadband(x, y):
    """y = a·max(x−I0, 0) 피팅 (I0 그리드 + a 폐형해). → a, I0, R², resid_std"""
    best = None
    sst = np.sum((y - y.mean()) ** 2)
    for i0 in np.arange(0.0, max(x.max() * 0.8, I0_GRID_STEP_A), I0_GRID_STEP_A):
        xs = np.maximum(x - i0, 0.0)
        denom = np.sum(xs * xs)
        if denom <= 0:
            continue
        a = np.sum(xs * y) / denom
        sse = np.sum((y - a * xs) ** 2)
        if best is None or sse < best[0]:
            best = (sse, a, i0)
    sse, a, i0 = best
    resid = y - a * np.maximum(x - i0, 0.0)
    r2 = 1.0 - sse / sst if sst > 0 else float('nan')
    return a, i0, r2, resid.std()


def mech_noise(y):
    """이동평균 대비 잔차 std — stick-slip 등 역학적 노이즈 크기 [count]."""
    if len(y) < MOVAVG_N * 2:
        return float('nan')
    k = np.ones(MOVAVG_N) / MOVAVG_N
    smooth = np.convolve(y, k, mode='same')
    m = slice(MOVAVG_N, -MOVAVG_N)                 # 컨볼브 경계 왜곡 제외
    return float((y[m] - smooth[m]).std())


# ── bag 하나 분석 ───────────────────────────────────────────────────────────
def analyze_bag(bag_dir, meta, out_root):
    name = os.path.basename(os.path.normpath(bag_dir))
    out_dir = os.path.join(out_root, name)
    os.makedirs(out_dir, exist_ok=True)

    d = load_bag(bag_dir)
    t, cmd, fb, pos = d['t'], d['cmd'], d['fb'], d['pos']
    ramps = find_ramps(cmd, t)
    if not ramps:
        print(f'  [WARN] {name}: 램프 구간 없음 — 스킵')
        return [], []

    # tare: 램프 직전마다 재측정 (잔류 눌림 = 0 정의; F−R=0 힘평형 기준)
    # 픽스처 침하로 무부하 베이스라인이 이동해도 각 램프는 자기 영점에서 출발.
    t_first = t[ramps[0]['idx'][0]]
    pre = t < (t_first - TARE_MARGIN_S)
    if pre.sum() < 50:
        print(f'  [WARN] {name}: tare 창 {pre.sum()}샘플 — 시작부 무부하 확보 권장')
        pre = np.arange(len(t)) < 200
    tare = d['lc'][pre].mean()                     # bag 시작 영점 (침하 진단용)
    sensor_noise = d['lc'][pre].std()
    tare_vec = np.full(len(t), tare)
    prev_end, last_tare = 0, tare
    for r in ramps:
        seg = r['idx']
        win = np.zeros(len(t), bool)
        win[prev_end + 1:seg[0]] = True
        win &= (cmd < CMD_ACTIVE_A) & (t < t[seg[0]] - TARE_MARGIN_S)
        if prev_end > 0:
            win &= t > t[prev_end] + SETTLE_S
        idx = np.where(win)[0][-200:]              # 램프 직전 ~2 s 창
        if len(idx) > 20:
            last_tare = d['lc'][idx].mean()
        start = prev_end + 1 if prev_end > 0 else 0
        tare_vec[start:] = last_tare
        prev_end = seg[-1]
    lc = d['lc'] - tare_vec
    lc_bag = d['lc'] - tare                        # bag 영점 기준 (베이스라인 진단)

    # 램프 사이/이후 무부하 창에서 베이스라인 복귀 확인
    baseline_after = []
    for ri, r in enumerate(ramps):
        end = r['idx'][-1]
        nxt = ramps[ri + 1]['idx'][0] if ri + 1 < len(ramps) else len(t)
        win = (t > t[end] + SETTLE_S) & (np.arange(len(t)) < nxt) & (cmd < CMD_ACTIVE_A)
        baseline_after.append(float(lc_bag[win].mean()) if win.sum() > 20 else float('nan'))

    # 위상별 피팅 (rise / fall) + 공통 그리드 빈 곡선 + 크리프/스프로켓
    results, curves, vals = [], [], []
    for ri, r in enumerate(ramps):
        seg = r['idx']
        if r['random']:                             # 임의 전류 구간 → 검증셋
            vals.append(dict(bag=name, ramp=ri,
                             contact_point=meta.get('contact_point', '?'),
                             t=t[seg], fb=fb[seg], lc=lc[seg]))
            continue
        # hold 구간(피크 유지 등) 크리프: lc 시간 기울기 [count/s]
        hold = seg[r['phase'] == 0]
        creep = float('nan')
        if len(hold) > 10:
            th, yh = t[hold], lc[hold]
            spans = np.split(np.arange(len(hold)),
                             np.where(np.diff(hold) > 1)[0] + 1)
            spans = [s for s in spans if th[s[-1]] - th[s[0]] >= HOLD_MIN_S]
            if spans:
                s = max(spans, key=len)              # 가장 긴 hold 대표
                creep = float(np.polyfit(th[s], yh[s], 1)[0])
        pos_start = float(pos[seg[0]])
        pos_delta = float(pos[seg[-1]] - pos[seg[0]])
        rise_sel = seg[r['phase'] == 1]
        slope_meas = (float(np.median(np.gradient(cmd[rise_sel], t[rise_sel])))
                      if len(rise_sel) > 10 else float('nan'))
        for pname, pval in (('rise', 1), ('fall', -1)):
            sel = seg[r['phase'] == pval]
            if len(sel) < 100:
                continue
            a, i0, r2, rstd = fit_deadband(fb[sel], lc[sel])
            results.append(dict(
                bag=name, ramp=ri, phase=pname, motor=MOTOR_IDX, lc_ch=LC_CH,
                slope_cnt_per_A=round(a, 2), deadband_A=round(i0, 2),
                r2=round(r2, 4), resid_std=round(rstd, 1),
                mech_noise=round(mech_noise(lc[sel]), 2),
                sensor_noise=round(float(sensor_noise), 2),
                ramp_slope_meas_aps=round(slope_meas, 3),
                peak_A=round(float(fb[seg].max()), 2),
                hold_creep_cps=round(creep, 2),
                pos_start=round(pos_start, 3), pos_delta=round(pos_delta, 4),
                tare=round(float(tare), 1),
                retare_off=round(float(tare_vec[seg[0]] - tare), 1),
                baseline_return=round(baseline_after[ri], 1),
                dup_ratio=round(d['dup_ratio'], 3), **meta))
            curves.append(dict(
                bag=name, ramp=ri, phase=pname,
                contact_point=meta.get('contact_point', '?'),
                pos_start=pos_start, peak_A=float(fb[seg].max()),
                curve=_binned(fb[sel], lc[sel])))

    _plot_bag(name, out_dir, t, cmd, fb, lc, ramps, results)
    if curves:
        _plot_hysteresis(name, out_dir, curves)

    flagged = [r for r in results if abs(r['baseline_return']) > BASELINE_WARN]
    flag = f'  ⚠ 베이스라인 미복귀 {len(flagged)}건' if flagged else ''
    rnd = f', 랜덤 {len(vals)}구간' if vals else ''
    n_ramp = sum(not r['random'] for r in ramps)
    print(f'  {name}: 램프 {n_ramp}회{rnd}, 중복 {d["dup_ratio"]*100:.0f}%{flag}')
    return results, curves, vals


# ── 플롯 ────────────────────────────────────────────────────────────────────
def _plot_bag(name, out_dir, t, cmd, fb, lc, ramps, results):
    # 1) 시간축
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
    axes[0].plot(t, cmd, color=TEXT2, ls='--', lw=1.0, label='cmd')
    axes[0].plot(t, fb, color=PALETTE[0], label='fb')
    axes[0].set_title(f'{name} — M{MOTOR_IDX} current (A)', loc='left', color=TEXT)
    axes[0].legend(loc='upper left')
    axes[1].plot(t, lc, color=PALETTE[0])
    axes[1].set_title(f'lc[{LC_CH}] raw (tared)', loc='left', color=TEXT)
    axes[1].set_xlabel('ecu time (s)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'timeseries.png'), dpi=140)
    plt.close(fig)

    # 2) 산점도 + 피팅 + 반복성 리본
    fig, ax = plt.subplots(figsize=(7.5, 5))
    all_x, all_y = [], []
    for ri, r in enumerate(ramps):
        if r.get('random'):
            continue
        sel = r['idx'][r['phase'] == 1]
        ax.plot(fb[sel], lc[sel], '.', ms=2, alpha=0.3,
                color=PALETTE[ri % len(PALETTE)], label=f'ramp{ri}')
        all_x.append(fb[sel]); all_y.append(lc[sel])
        fit = next((q for q in results
                    if q['ramp'] == ri and q['phase'] == 'rise'), None)
        if fit:
            xf = np.linspace(0, fb[sel].max(), 100)
            ax.plot(xf, fit['slope_cnt_per_A'] * np.maximum(xf - fit['deadband_A'], 0),
                    color=PALETTE[ri % len(PALETTE)], lw=1.0)
    # 전류 빈별 mean±σ (전체 rise 샘플) — 램프 간 반복성 리본
    if not all_x:                                   # 전 구간 random 인 bag
        plt.close(fig)
        return
    x = np.concatenate(all_x); y = np.concatenate(all_y)
    bins = np.arange(0, x.max() + FIT_BIN_A, FIT_BIN_A)
    bi = np.digitize(x, bins)
    bm = np.array([y[bi == k].mean() if (bi == k).sum() > 5 else np.nan
                   for k in range(1, len(bins))])
    bs = np.array([y[bi == k].std() if (bi == k).sum() > 5 else np.nan
                   for k in range(1, len(bins))])
    bc = bins[:-1] + FIT_BIN_A / 2
    ax.fill_between(bc, bm - bs, bm + bs, color=TEXT2, alpha=0.15, lw=0)
    ax.plot(bc, bm, color=TEXT2, lw=1.0)
    ax.set_title(f'{name} — lc[{LC_CH}] vs M{MOTOR_IDX} fb current (rise)',
                 loc='left', color=TEXT)
    ax.set_xlabel('fb current (A)')
    ax.set_ylabel('loadcell (tared)')
    ax.legend(loc='upper left', markerscale=4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'scatter_fit.png'), dpi=140)
    plt.close(fig)

    # 3) 잔차 vs 전류
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for ri, r in enumerate(ramps):
        if r.get('random'):
            continue
        sel = r['idx'][r['phase'] == 1]
        fit = next((q for q in results
                    if q['ramp'] == ri and q['phase'] == 'rise'), None)
        if fit is None:
            continue
        resid = lc[sel] - fit['slope_cnt_per_A'] * np.maximum(
            fb[sel] - fit['deadband_A'], 0)
        ax.plot(fb[sel], resid, '.', ms=2, alpha=0.3, color=PALETTE[ri % len(PALETTE)])
    ax.axhline(0, color=TEXT2, lw=0.8)
    ax.set_title(f'{name} — lc[{LC_CH}] fit residual', loc='left', color=TEXT)
    ax.set_xlabel('fb current (A)')
    ax.set_ylabel(f'residual ({U})')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'residuals.png'), dpi=140)
    plt.close(fig)


def _plot_hysteresis(name, out_dir, curves):
    """램프별 rise(실선) vs fall(점선) 빈 곡선 + 갭 크기."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2),
                             gridspec_kw={'width_ratios': [3, 2]})
    ramps = sorted({c['ramp'] for c in curves})
    for ri in ramps:
        col = PALETTE[ri % len(PALETTE)]
        rise = next((c for c in curves if c['ramp'] == ri and c['phase'] == 'rise'), None)
        fall = next((c for c in curves if c['ramp'] == ri and c['phase'] == 'fall'), None)
        if rise is not None:
            axes[0].plot(BC, rise['curve'], color=col, label=f'ramp{ri} rise')
        if fall is not None:
            axes[0].plot(BC, fall['curve'], color=col, ls='--', alpha=0.7)
        if rise is not None and fall is not None:
            gap = fall['curve'] - rise['curve']
            axes[1].plot(BC, gap, color=col)
    axes[0].set_title(f'{name} — rise(—) vs fall(--)', loc='left', color=TEXT)
    axes[0].set_xlabel('fb current (A)'); axes[0].set_ylabel('loadcell (tared)')
    axes[0].legend(loc='upper left', fontsize=7)
    axes[1].axhline(0, color=TEXT2, lw=0.8)
    axes[1].set_title('hysteresis gap (fall − rise)', loc='left', color=TEXT)
    axes[1].set_xlabel('fb current (A)'); axes[1].set_ylabel(f'\u0394 {U}')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'hysteresis.png'), dpi=140)
    plt.close(fig)


# ── 캠페인 레벨: 반복성 / 위치 효과 / 스프로켓 / 판정 ──────────────────────
def _curve_stack(curves, phase):
    """같은 phase 곡선들을 (n_curve, n_bin) 행렬로. 전부 NaN 빈은 그대로."""
    sel = [c for c in curves if c['phase'] == phase]
    if not sel:
        return None, []
    return np.vstack([c['curve'] for c in sel]), sel


def _map_error(stack, ref):
    """ref 곡선을 맵으로 썼을 때 각 곡선의 오차: RMS/최대 [count], 유효 빈만."""
    err = stack - ref
    valid = ~np.isnan(err)
    if valid.sum() == 0:
        return float('nan'), float('nan')
    e = err[valid]
    return float(np.sqrt(np.mean(e ** 2))), float(np.abs(e).max())


def _interp_map(curve, x):
    """빈 곡선(NaN 포함)을 x 에서 선형보간. 범위 밖은 끝값 유지."""
    v = ~np.isnan(curve)
    return np.interp(x, BC[v], curve[v])


def _validate(vals, group_maps, global_maps, out_root, add):
    """임의 전류 구간을 rise/fall 2-맵 예측기로 검증.

    방향 = 스무딩한 dfb/dt 부호 (0 은 직전 방향 유지) → rise/fall 맵 선택."""
    k = np.ones(51) / 51                            # ~0.5 s 스무딩 창
    for vs in vals:
        t, fb, lc = vs['t'], vs['fb'], vs['lc']
        if len(fb) < 200:
            continue
        maps = group_maps.get(vs['contact_point'], global_maps)
        if maps.get('fall') is None:
            maps = global_maps
        dfb = np.convolve(np.gradient(fb, t), k, mode='same')
        dirn = np.where(dfb > 0.2, 1, np.where(dfb < -0.2, -1, 0))
        last = 1
        for i in range(len(dirn)):                  # hold 는 직전 방향 유지
            if dirn[i] == 0:
                dirn[i] = last
            last = dirn[i]
        pred = np.where(dirn > 0, _interp_map(maps['rise'], fb),
                        _interp_map(maps['fall'], fb))
        err = lc - pred
        tag = f"{vs['bag']}_r{vs['ramp']}"
        scope = f"{vs['contact_point']}:{tag.split('_07-10_')[-1]}"
        add('validation_2map', scope, err[None, :], np.zeros_like(err)[None, :], 1)

        # 고무 점탄성 근사: 예측에 1차 지연 적용, τ 그리드 탐색
        dt = float(np.median(np.diff(t)))
        best_tau, best_rms, pred_f = 0.0, float(np.sqrt(np.mean(err ** 2))), pred
        for tau in np.arange(0.2, 4.01, 0.2):
            al = dt / (tau + dt)
            pf = np.empty_like(pred)
            acc = pred[0]
            for i, p in enumerate(pred):            # 지수 필터 (순차)
                acc += al * (p - acc)
                pf[i] = acc
            rms = float(np.sqrt(np.mean((lc - pf) ** 2)))
            if rms < best_rms:
                best_tau, best_rms, pred_f = tau, rms, pf
        err_f = lc - pred_f
        add('validation_2map_lag', f'{scope} τ={best_tau:.1f}s',
            err_f[None, :], np.zeros_like(err_f)[None, :], 1)

        fig, axes = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True,
                                 gridspec_kw={'height_ratios': [2, 1]})
        axes[0].plot(t, lc, color=PALETTE[0], label='measured')
        axes[0].plot(t, pred, color=PALETTE[2], lw=0.7, alpha=0.5,
                     label='2-map (static)')
        axes[0].plot(t, pred_f, color=PALETTE[3], lw=1.0,
                     label=f'2-map + lag τ={best_tau:.1f}s')
        axes[0].set_title(f'{tag} — 2-map prediction vs measured', loc='left', color=TEXT)
        axes[0].set_ylabel('loadcell (tared)'); axes[0].legend(loc='upper left')
        axes[1].plot(t, err_f, color=PALETTE[4], lw=0.8)
        axes[1].axhline(0, color=TEXT2, lw=0.8)
        axes[1].set_ylabel(f'error ({U})'); axes[1].set_xlabel('ecu time (s)')
        fig.tight_layout()
        fig.savefig(os.path.join(out_root, vs['bag'], f'validation_r{vs["ramp"]}.png'),
                    dpi=140)
        plt.close(fig)


def plot_groups(curves, results, out_root, vals=()):
    """그룹(시작위치) 반복성 + 위치 비교 + 스프로켓 영향 + 맵 전략별 판정."""
    groups = sorted({c['contact_point'] for c in curves})
    rise_all, _ = _curve_stack(curves, 'rise')
    global_map = np.nanmean(rise_all, axis=0)
    fs = np.nanmax(global_map)                      # 풀스케일 [count]
    # 전역 평균 기울기 [count/A] — 오차의 전류 등가 환산용
    v = ~np.isnan(global_map)
    g_slope = float(np.polyfit(BC[v], global_map[v], 1)[0])

    verdict = []

    def add(strategy, scope, stack, ref, n):
        rms, mx = _map_error(stack, ref)
        verdict.append(dict(
            strategy=strategy, scope=scope, n_curves=n,
            rms_cnt=round(rms, 1), max_cnt=round(mx, 1),
            rms_pct_fs=round(100 * rms / fs, 2),
            rms_A_equiv=round(rms / g_slope, 3)))

    # 1) 전역 단일 맵 (rise 전체)
    add('global_rise', 'all', rise_all, global_map, rise_all.shape[0])
    # rise 맵으로 fall 까지 커버할 때 (히스테리시스 무시 비용)
    fall_all, _ = _curve_stack(curves, 'fall')
    global_maps = {'rise': global_map, 'fall': None}
    if fall_all is not None:
        add('global_rise_on_fall', 'all', fall_all, global_map, fall_all.shape[0])
        fall_map = np.nanmean(fall_all, axis=0)
        global_maps['fall'] = fall_map
        add('global_perphase', 'fall', fall_all, fall_map, fall_all.shape[0])

    # 2) 위치별 맵 + 반복성 오버레이 플롯
    group_maps = {}
    fig_p, ax_p = plt.subplots(figsize=(8, 5))
    for gi, g in enumerate(groups):
        gc = [c for c in curves if c['contact_point'] == g]
        stack, sel = _curve_stack(gc, 'rise')
        if stack is None:
            continue
        gmap = np.nanmean(stack, axis=0)
        gfall, _ = _curve_stack(gc, 'fall')
        group_maps[g] = {'rise': gmap,
                         'fall': np.nanmean(gfall, axis=0) if gfall is not None else None}
        if stack.shape[0] >= 2:
            add('per_position', g, stack, gmap, stack.shape[0])
        col = PALETTE[gi % len(PALETTE)]
        ax_p.plot(BC, gmap, color=col, label=f'{g} (n={stack.shape[0]})')
        sd = np.nanstd(stack, axis=0)
        ax_p.fill_between(BC, gmap - sd, gmap + sd, color=col, alpha=0.15, lw=0)
        # 그룹 내 반복성 오버레이 (bag 별 색)
        fig, ax = plt.subplots(figsize=(8, 5))
        bags = sorted({c['bag'] for c in sel})
        for c in sel:
            bcol = PALETTE[bags.index(c['bag']) % len(PALETTE)]
            ax.plot(BC, c['curve'], color=bcol, lw=0.9, alpha=0.8)
        for bi, b in enumerate(bags):
            ax.plot([], [], color=PALETTE[bi % len(PALETTE)], label=b.split('_07-10_')[-1])
        ax.fill_between(BC, gmap - sd, gmap + sd, color=TEXT2, alpha=0.15, lw=0)
        ax.set_title(f'{g} — rise curves, {stack.shape[0]} ramps / {len(bags)} bags',
                     loc='left', color=TEXT)
        ax.set_xlabel('fb current (A)'); ax.set_ylabel('loadcell (tared)')
        ax.legend(loc='upper left', fontsize=7, title='bag (h-m)')
        fig.tight_layout()
        fig.savefig(os.path.join(out_root, f'repeat_{g}.png'), dpi=140)
        plt.close(fig)
    ax_p.set_title('start-position group mean ±σ (rise)', loc='left', color=TEXT)
    ax_p.set_xlabel('fb current (A)'); ax_p.set_ylabel('loadcell (tared)')
    ax_p.legend(loc='upper left')
    fig_p.tight_layout()
    fig_p.savefig(os.path.join(out_root, 'position.png'), dpi=140)
    plt.close(fig_p)

    # 3) 스프로켓 시작 위치 vs slope / deadband (rise)
    rise_res = [r for r in results if r['phase'] == 'rise']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for pi, key in enumerate(('slope_cnt_per_A', 'deadband_A')):
        ax = axes[pi]
        for gi, g in enumerate(groups):
            rr = [r for r in rise_res if r['contact_point'] == g]
            ax.plot([r['pos_start'] for r in rr], [r[key] for r in rr], 'o',
                    ms=5, alpha=0.8, color=PALETTE[gi % len(PALETTE)], label=g)
        ax.set_xlabel('sprocket pos @ ramp start')
        ax.set_title(key, loc='left', color=TEXT)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_root, 'sprocket.png'), dpi=140)
    plt.close(fig)

    # 4) 임의 전류 구간 검증 (rise/fall 2-맵 예측기)
    if vals:
        _validate(vals, group_maps, global_maps, out_root, add)

    # verdict.csv + 콘솔
    with open(os.path.join(out_root, 'verdict.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(verdict[0].keys()))
        w.writeheader(); w.writerows(verdict)
    print('\n── 맵 전략별 예측 오차 (rms, 풀스케일 %, 전류 등가 A) ──')
    for r in verdict:
        print(f"  {r['strategy']:<22}{r['scope']:<6}n={r['n_curves']:<3}"
              f"rms {r['rms_cnt']:7.1f} {U}  {r['rms_pct_fs']:5.2f}% FS"
              f"  ≈{r['rms_A_equiv']:5.3f} A")
    return verdict


def plot_compare(results, out_root):
    """조건(bag)별 slope / I0 비교 — 램프별 점 + bag 평균 바."""
    rise = [r for r in results if r['phase'] == 'rise']
    bags = sorted({r['bag'] for r in rise})
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for pi, (key, label) in enumerate((('slope_cnt_per_A', f'slope ({U}/A)'),
                                       ('deadband_A', 'deadband I0 (A)'))):
        ax = axes[pi]
        xs, ys = [], []
        for bi, b in enumerate(bags):
            vals = [r[key] for r in rise if r['bag'] == b]
            ax.plot([bi] * len(vals), vals, 'o', ms=5, alpha=0.7, color=PALETTE[0])
            xs.append(bi); ys.append(np.mean(vals))
        ax.plot(xs, ys, '_', ms=16, mew=2, color=TEXT)
        ax.set_xticks(range(len(bags)))
        ax.set_xticklabels(bags, rotation=20, ha='right', fontsize=7)
        ax.set_title(label, loc='left', color=TEXT)
    fig.tight_layout()
    fig.savefig(os.path.join(out_root, 'compare.png'), dpi=140)
    plt.close(fig)


# ── main ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('index', nargs='?', default=None, help='test_index.csv 경로')
    ap.add_argument('--out', default=None, help='출력 디렉토리 (기본: index 옆 out/)')
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    index_csv = args.index or os.path.join(here, 'test_index.csv')
    base = os.path.dirname(os.path.abspath(index_csv))
    out_root = args.out or os.path.join(base, 'out')
    os.makedirs(out_root, exist_ok=True)

    results, curves, vals = [], [], []
    with open(index_csv, newline='') as f:
        for row in csv.DictReader(f):
            bag = row.pop('bag').strip()
            if not bag or bag.startswith('#'):
                continue
            bag_dir = bag if os.path.isabs(bag) else os.path.join(base, bag)
            meta = {k: v.strip() for k, v in row.items()}
            try:
                res, cur, val = analyze_bag(bag_dir, meta, out_root)
                results += res
                curves += cur
                vals += val
            except Exception as e:
                print(f'  [ERROR] {bag}: {e}', file=sys.stderr)

    if not results:
        sys.exit('처리된 bag 없음')

    with open(os.path.join(out_root, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
    plot_compare(results, out_root)
    plot_groups(curves, results, out_root, vals)

    # 콘솔 리포트: bag별 rise 기울기 반복성 (CV%)
    print('\n── 반복성 (rise slope, 램프 간 CV%) ──')
    for b in sorted({r['bag'] for r in results}):
        v = [r['slope_cnt_per_A'] for r in results
             if r['bag'] == b and r['phase'] == 'rise']
        if len(v) >= 2:
            cv = 100 * np.std(v) / np.mean(v)
            print(f'  {b}: slope {np.mean(v):7.1f} ±{np.std(v):5.1f}  CV {cv:4.1f}%')
    print(f'\n출력: {out_root}/summary.csv, compare.png, <bag>/*.png')


if __name__ == '__main__':
    main()
