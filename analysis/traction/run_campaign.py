#!/usr/bin/env python3
"""반복성/스윕 캠페인 러너 — testbed_spec §5.1 CLI 를 배치 구동 (test_plan §3 AI 자동화 실행층).

한 프로파일을 N회 반복 실행(`testbed_cli run --record`)하고, 각 run 의 종료코드·result.json 을
모아 session_summary.json 으로 남긴다. 이상(비정상 종료코드) 발생 시 **즉시 중단**(§3 안전경계).
반복성 게이트(동일 조건 N run)와 하중 스윕(블록 사이 무게추 교체 프롬프트)에 쓴다.

전제: bridge 가 control_mode + auto_mode=1(CURRENT)로 IDLE, active_motors 가 실연결 모터에 일치
(testbed_spec §3.1). ROS 환경 source 완료. 이 스크립트는 testbed_cli 를 subprocess 로만 호출한다.

사용:
  # 반복성 게이트: std_ramp_cycle 5회 + 재장착 후 2회
  run_campaign.py --profile ../../data/profiles/std_ramp_cycle.yaml \
                  --label rep_w31 --repeats 5 --reseat-after 5 --extra-after-reseat 2

  # 단일 블록 무인 실행(프롬프트 없음)
  run_campaign.py --profile P.yaml --label sweep_w10 --repeats 3 --no-prompt

종료코드: 0=전 run 성공 / 1=일부 run 실패(중단) / 2=사용법·환경 오류.
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time
from datetime import datetime

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))


def sh(cmd):
    """testbed_cli 호출 → (returncode, stdout+stderr)."""
    p = subprocess.run(cmd, capture_output=True, text=True)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def find_run_dir(bag_dir, label):
    """방금 만들어진 <label>_<MM-DD_HH-MM>/ 중 최신 (record.py 규격)."""
    cands = sorted(glob.glob(os.path.join(bag_dir, label + '_*')), key=os.path.getmtime)
    return cands[-1] if cands else None


def read_result(run_dir):
    p = os.path.join(run_dir, 'result.json')
    if run_dir and os.path.isfile(p):
        with open(p) as f:
            return json.load(f)
    return None


def preflight(bag_dir):
    if not shutil_which('testbed_cli') and not shutil_which('ros2'):
        print('ERROR: testbed_cli/ros2 를 PATH 에서 못 찾음 — ROS 환경 source 확인', file=sys.stderr)
        return False
    rc, out = sh(['testbed_cli', 'status'])
    print('status:', out.strip())
    if rc != 0:
        print('ERROR: bridge 가 IDLE(control_mode)로 떠 있지 않음 — 먼저 기동', file=sys.stderr)
        return False
    os.makedirs(bag_dir, exist_ok=True)
    return True


def shutil_which(x):
    from shutil import which
    return which(x)


def do_runs(profile, label, n, bag_dir, settle, start_idx, summary):
    """n 회 반복 실행. 실패 시 False 반환(호출부가 중단)."""
    for i in range(start_idx, start_idx + n):
        name = f'{label}_r{i:02d}'
        print(f'\n───── run {i} : {name} ─────')
        rc, out = sh(['testbed_cli', 'run', profile, '--record', '--name', name,
                      '--bag-dir', bag_dir])
        print(out.strip())
        run_dir = find_run_dir(bag_dir, name)
        res = read_result(run_dir)
        summary['runs'].append(dict(idx=i, name=name, exit=rc,
                                    run_dir=run_dir, result=res))
        if rc != 0:
            print(f'\n⚠ run {i} 종료코드 {rc} (≠0) — 안전상 캠페인 중단. '
                  f'원인 확인 후 재개. (2=거부 3=중단/LOCKED 4=bridge없음 5=타임아웃)',
                  file=sys.stderr)
            return False
        if res and not res.get('success', False):
            print(f'\n⚠ run {i} result.success=false ({res.get("message")}) — 중단.', file=sys.stderr)
            return False
        if settle > 0 and i < start_idx + n - 1:
            time.sleep(settle)
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description='반복성/스윕 캠페인 러너 (test_plan §3)')
    ap.add_argument('--profile', required=True, help='프로파일 YAML 경로')
    ap.add_argument('--label', required=True, help='실험 라벨 접두 (폴더명 = <label>_rNN_<일시>)')
    ap.add_argument('--repeats', type=int, default=5, help='반복 횟수 (기본 5)')
    ap.add_argument('--bag-dir', default=os.path.join(REPO, 'data', 'rosbags'), help='기록 루트')
    ap.add_argument('--settle', type=float, default=5.0, help='run 간 대기 [s]')
    ap.add_argument('--reseat-after', type=int, default=0,
                    help='K회 후 트랙 재장착 프롬프트 (재현성 검증, 0=off)')
    ap.add_argument('--extra-after-reseat', type=int, default=0, help='재장착 후 추가 run 수')
    ap.add_argument('--no-prompt', action='store_true', help='프롬프트 없이 무인 실행')
    args = ap.parse_args(argv)

    if not os.path.isfile(args.profile):
        print(f'ERROR: 프로파일 없음: {args.profile}', file=sys.stderr); return 2
    if not preflight(args.bag_dir):
        return 2

    summary = dict(profile=os.path.abspath(args.profile), label=args.label,
                   started_at=datetime.now().isoformat(timespec='seconds'), runs=[])
    ok = do_runs(args.profile, args.label, args.repeats, args.bag_dir, args.settle, 1, summary)

    if ok and args.reseat_after and args.extra_after_reseat:
        if not args.no_prompt:
            input(f'\n▶ {args.reseat_after}회 완료. 트랙을 재장착(재정렬)한 뒤 Enter — '
                  f'재현성 검증 {args.extra_after_reseat}회 진행: ')
        ok = do_runs(args.profile, args.label, args.extra_after_reseat, args.bag_dir,
                     args.settle, args.repeats + 1, summary)

    summary['ended_at'] = datetime.now().isoformat(timespec='seconds')
    summary['all_success'] = ok and all(r['exit'] == 0 for r in summary['runs'])
    out = os.path.join(args.bag_dir, f'session_{args.label}_'
                       f'{datetime.now().strftime("%m-%d_%H-%M")}.json')
    with open(out, 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f'\n캠페인 종료 — {len(summary["runs"])} run, '
          f'{"전부 성공" if summary["all_success"] else "실패/중단 포함"}. 요약: {out}')
    return 0 if summary['all_success'] else 1


if __name__ == '__main__':
    sys.exit(main())
