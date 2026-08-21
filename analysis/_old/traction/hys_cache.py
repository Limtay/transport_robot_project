#!/usr/bin/env python3
"""히스테리시스 분석 0단계 — 전 런 bag 을 npz 캐시로 변환.

각 런: t(200Hz), cmd, fb(전류 A), F(로드셀, 절대 N = (cnt-214.6)/12.627), F_tare(초기 0A hold 기준).
세션 JSON(battery_*.json)에서 run_dir 목록을 모아 처리. 출력: analysis/traction/hys_cache/<name>.npz
"""
import glob, json, os, sqlite3, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traction_analysis as T

ROOT = '/home/swarm/tp_ws'
CACHE = os.path.join(ROOT, 'analysis/traction/hys_cache')
os.makedirs(CACHE, exist_ok=True)

cal = json.load(open(os.path.join(ROOT, 'analysis/traction/loadcell_cal.json')))
OFF = cal['channels']['ch0']['offset_cnt']          # 214.607
CPN = cal['channels']['ch0']['counts_per_N']        # 12.627

SESSIONS = sorted(glob.glob(os.path.join(ROOT, 'data/rosbags/battery_*.json')))

def load_raw(bag_dir):
    """bag → (tick, cmd, fb, cnt). load_bag 과 동일 디코드지만 로드셀은 raw cnt 유지."""
    db3 = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    con = sqlite3.connect(db3[0])
    decode_fn, tick_scale, tid = T._detect_format(con)
    rows = con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)).fetchall()
    con.close()
    n = len(rows)
    tick = np.empty(n, np.int64); cmd = np.empty(n); fb = np.empty(n); cnt = np.empty(n)
    for i, (blob,) in enumerate(rows):
        tk, ss, f, l, g = decode_fn(blob)
        tick[i] = tk; cmd[i] = f[T.MOTOR_IDX]; fb[i] = f[4 + T.MOTOR_IDX]; cnt[i] = l[T.LC_CH]
    keep = np.concatenate([[True], np.diff(tick) != 0])
    t = (tick[keep] - tick[keep][0]) * tick_scale
    return t, cmd[keep], fb[keep], cnt[keep]

def main():
    index = []
    for sp in SESSIONS:
        d = json.load(open(sp))
        sess = os.path.basename(sp).replace('.json', '')
        for r in d['runs']:
            rd = r.get('run_dir')
            if not rd or not os.path.isdir(rd): continue
            name = os.path.basename(rd)           # 라벨_일시 → 전역 유일
            out = os.path.join(CACHE, name + '.npz')
            if not os.path.exists(out):
                t, cmd, fb, cnt = load_raw(os.path.join(rd, 'bag'))
                F = (cnt - OFF) / CPN             # 절대 N
                base = np.median(F[(t >= 0.5) & (t <= 4.5)])   # 초기 0A hold tare
                np.savez_compressed(out, t=t, cmd=cmd, fb=fb, F=F, Ft=F - base, base=base)
            z = np.load(out)
            index.append(dict(name=name, session=sess, tag=r.get('tag', ''),
                              profile=r.get('profile', r.get('stem', '')),
                              base=float(z['base']), n=int(len(z['t'])),
                              Fmax=float(z['Ft'].max()), npz=out))
    json.dump(index, open(os.path.join(CACHE, 'index.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"cached {len(index)} runs -> {CACHE}")
    # 세션별 요약
    from collections import Counter
    print(Counter([i['session'] for i in index]))

if __name__ == '__main__':
    main()
