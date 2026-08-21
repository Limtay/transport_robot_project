#!/usr/bin/env python3
"""한 셀(= 같은 조건 N 반복)을 돌리는 러너. RULES.md §3.

  python3 tools/run_block.py profiles/t4_ramp_cycle.yaml --label t4_p1_w40 --repeats 3

구 `run_campaign.py` 를 대체한다 — 그쪽은 재설계에서 없어진 `testbed_cli` 를 부르고 있었다.
지금 실행층은 **`control_cli run <profile> --record --name <label>`** 이다.

각 런마다 `result.json` 을 확인하고, `success=false` / `write_err_cnt>0` / `clamp_cnt>0` 이면
**즉시 중단**한다 (test_plan §5 안전경계). 런 사이에는 settle 을 둔다 — 종료 잔류가
15~20 s 에 걸쳐 이완하므로 그 전에 다음 런을 시작하면 영점이 오염된다.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import time

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
SETTLE_S = 20.0


def cli(args):
    exe = shutil.which('control_cli')
    cmd = [exe] + args if exe else ['ros2', 'run', 'control_cli', 'control_cli'] + args
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def newest_run(label):
    hits = sorted(glob.glob(os.path.join(REPO, 'data', 'rosbags', f'{label}_*')),
                  key=os.path.getmtime)
    return hits[-1] if hits else None


def check(run_dir):
    """result.json 검증 → (ok, 메시지)."""
    rj = os.path.join(run_dir, 'result.json')
    if not os.path.isfile(rj):
        return False, 'result.json 없음'
    r = json.load(open(rj))
    bad = []
    if not r.get('success'):
        bad.append(f"success=false ({r.get('message')})")
    for k in ('write_err_cnt', 'clamp_cnt', 'drop_cnt', 'irregular_tick_cnt'):
        if r.get(k, 0):
            bad.append(f'{k}={r[k]}')
    if not r.get('clock_converged', True):
        bad.append('clock 미수렴')
    return (not bad), ('; '.join(bad) if bad else
                       f"ticks={r.get('ticks_executed')} 이상 없음")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('profile')
    ap.add_argument('--label', required=True, help='라벨 접두 — 런은 <label>_r<NN> 로 기록된다')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--start', type=int, default=1,
                    help='반복 번호 시작값 — 셀을 나눠 돌릴 때 r 번호가 겹치지 않게 한다')
    ap.add_argument('--settle', type=float, default=SETTLE_S)
    a = ap.parse_args()

    if not os.path.isfile(a.profile):
        print(f'ERROR: 프로파일 없음: {a.profile}', file=sys.stderr); return 2

    rc, out = cli(['status'])
    if rc != 0:
        print(f'ERROR: 브리지 status 실패 — 기동됐는지 확인\n{out}', file=sys.stderr); return 2
    print(f'[사전확인] {out.strip().splitlines()[0] if out.strip() else "OK"}')

    done = []
    for k in range(a.start, a.start + a.repeats):
        name = f'{a.label}_r{k:02d}'
        print(f'\n=== r{k:02d} ({k - a.start + 1}/{a.repeats})  {name} ===')
        # control_cli 는 레포 루트에서 돈다(data/rosbags 가 상대경로) — 프로파일은
        # **절대경로**로 넘겨야 한다. 상대경로면 루트 기준으로 풀려 "파일 없음" 이 난다.
        rc, out = cli(['run', os.path.abspath(a.profile), '--record', '--name', name])
        print(out.strip()[-400:])
        if rc != 0:
            print(f'중단: run 종료코드 {rc}', file=sys.stderr); return 1
        run_dir = newest_run(name)
        ok, msg = check(run_dir) if run_dir else (False, '기록 폴더 못 찾음')
        print(f'[검증] {msg}')
        if not ok:
            print('중단: 이 런이 정상이 아니다 — 원인 확인 후 재개할 것', file=sys.stderr)
            return 1
        done.append(run_dir)
        if k < a.repeats:
            print(f'[settle] {a.settle:.0f}s (종료 잔류 이완)')
            time.sleep(a.settle)

    print('\n=== 블록 완료 ===')
    for r in done:
        print(' ', r)
    print(f'\n분석: python3 tools/ramp_analysis.py {" ".join(done)} --label {a.label}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
