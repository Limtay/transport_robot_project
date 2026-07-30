#!/usr/bin/env python3
"""자율 프로파일 배터리 러너 (매니페스트 구동) — 한 세팅(payload 고정)에서 무인 데이터 수집.

매니페스트 JSON 을 받아 runs 순서대로 testbed_cli run --record 를 돌린다. 런 사이 settle 후 FSM
상태를 재확인(IDLE 아니면 = HW_ESTOP/FAULT → 즉시 중단)하고, 각 런의 result.json 을 검증한다.
block_size 런마다 block_cooldown_s 만큼 IDLE 쿨다운(모터 열 완화 — control 모드는 온도 토픽 미발행이라
SW 온도 가드 불가, 시간 기반 쿨다운 + 사용자 물리 감시로 대체). 이상 시 abort 후 배터리 정지.

사용: run_battery.sh <manifest.json>   (ROS 소싱은 .sh 담당)
매니페스트: {label,payload_kg,max_current,motor,settle_s,block_cooldown_s,block_size,
             runs:[{profile,repeats,tag}]}
"""
import glob, json, os, subprocess, sys, time
from datetime import datetime

ROOT = '/home/swarm/tp_ws'
PROF = os.path.join(ROOT, 'data/profiles')
BAGROOT = os.path.join(ROOT, 'data/rosbags')
CLI = ['ros2', 'run', 'testbed_cli', 'testbed_cli']

def log(msg): print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)

def cli(args, timeout=180):
    r = subprocess.run(CLI + args, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout or '') + (r.stderr or '')

def get_state():
    rc, out = cli(['status'], timeout=20)
    if rc != 0: return None, out.strip()
    for tok in out.split():
        if tok.startswith('state='): return tok.split('=', 1)[1], out.strip()
    return None, out.strip()

def newest_run_dir(name):
    cands = sorted(glob.glob(os.path.join(BAGROOT, f'{name}_*')), key=os.path.getmtime)
    return cands[-1] if cands else None

def main():
    if len(sys.argv) < 2:
        log("ERROR: 매니페스트 경로 필요 — run_battery.py <manifest.json>"); return 2
    man = json.load(open(sys.argv[1]))
    label = man.get('label', 'battery')
    settle_s = float(man.get('settle_s', 20.0))
    cooldown_s = float(man.get('block_cooldown_s', 0.0))
    block_size = int(man.get('block_size', 0))
    # runs 전개 (repeats)
    plan = []
    for r in man['runs']:
        for i in range(1, int(r.get('repeats', 1)) + 1):
            plan.append((r['profile'], r.get('tag', ''), i))
    total = len(plan)

    ts = datetime.now().strftime('%m-%d_%H-%M')
    session = dict(label=label, manifest=os.path.abspath(sys.argv[1]),
                   started=datetime.now().isoformat(), payload_kg=man.get('payload_kg'),
                   settle_s=settle_s, block_cooldown_s=cooldown_s, block_size=block_size,
                   max_current=man.get('max_current'), motor=man.get('motor'), runs=[])
    session_path = os.path.join(BAGROOT, f'battery_{label}_{ts}.json')

    st, raw = get_state()
    log(f"preflight status: {raw}")
    if st != 'IDLE':
        log(f"ABORT: 시작 상태가 IDLE 아님 ({st}) — 배터리 미실행")
        session['result'] = f'preflight_not_idle:{st}'
        json.dump(session, open(session_path, 'w'), ensure_ascii=False, indent=2); return 2

    log(f"BATTERY START '{label}' — {total} runs, settle {settle_s}s, "
        f"cooldown {cooldown_s}s/{block_size}runs, payload {man.get('payload_kg')}kg, "
        f"max {man.get('max_current')}A")
    done = 0
    for profile_fn, tag, rep_i in plan:
        profile = os.path.join(PROF, profile_fn)
        stem = os.path.splitext(profile_fn)[0].replace('hys_', '').replace('camp_', '')
        if not os.path.isfile(profile):
            log(f"ABORT: 프로파일 없음 {profile}"); session['result'] = f'missing:{profile_fn}'; break
        done += 1
        name = f"h_{stem}_w40_r{rep_i:02d}"
        log(f"=== [{done}/{total}] RUN {name} (tag={tag}) ===")
        t0 = time.time()
        rc, out = cli(['run', profile, '--record', '--name', name], timeout=400)
        dur = time.time() - t0
        run_dir = newest_run_dir(name)
        res = None
        if run_dir and os.path.isfile(os.path.join(run_dir, 'result.json')):
            try: res = json.load(open(os.path.join(run_dir, 'result.json')))
            except Exception: res = None
        session['runs'].append(dict(name=name, profile=profile_fn, tag=tag, rc=rc,
                                    run_dir=run_dir, dur_s=round(dur, 1), result=res))
        json.dump(session, open(session_path, 'w'), ensure_ascii=False, indent=2)

        ok = (rc == 0) and (res is not None) and res.get('success', False)
        werr = (res or {}).get('write_err_cnt', -1)
        log(f"    rc={rc} success={(res or {}).get('success')} ticks={(res or {}).get('ticks_executed')} "
            f"write_err={werr} clamp={(res or {}).get('clamp_cnt')} ({dur:.0f}s)")
        if not ok:
            log("    !!! 런 실패/비정상 — abort 후 배터리 중단")
            cli(['abort'], timeout=20); session['result'] = f'run_failed:{name}'
            json.dump(session, open(session_path, 'w'), ensure_ascii=False, indent=2)
            log("BATTERY ABORTED"); return 1
        if werr and werr > 0:
            log(f"    WARN: write_err={werr} (>0) — 기록 유지, 계속")

        # 마지막 런 뒤엔 짧게
        if done >= total:
            time.sleep(3.0); break
        # 블록 쿨다운 (모터 열 완화) or 일반 settle
        if block_size and cooldown_s and done % block_size == 0:
            log(f"    ⟳ BLOCK COOLDOWN {cooldown_s:.0f}s (모터 열 완화) — {done}/{total} 완료")
            time.sleep(cooldown_s)
        else:
            log(f"    settle {settle_s:.0f}s ...")
            time.sleep(settle_s)
        st, raw = get_state()
        if st != 'IDLE':
            log(f"    !!! 런 후 상태 {st} (IDLE 아님 — ESTOP/FAULT 의심) — 배터리 중단")
            session['result'] = f'not_idle_after:{name}:{st}'
            json.dump(session, open(session_path, 'w'), ensure_ascii=False, indent=2)
            log("BATTERY ABORTED"); return 1

    session['result'] = session.get('result', 'complete')
    session['finished'] = datetime.now().isoformat()
    json.dump(session, open(session_path, 'w'), ensure_ascii=False, indent=2)
    log(f"session summary: {session_path}")
    log(f"BATTERY {'COMPLETE' if session['result']=='complete' else session['result'].upper()} "
        f"— {len(session['runs'])}/{total} runs recorded")
    return 0 if session['result'] == 'complete' else 1

if __name__ == '__main__':
    sys.exit(main())
