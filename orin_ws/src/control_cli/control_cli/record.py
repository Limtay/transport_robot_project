"""실험 1회 = 폴더 1개 = 자기완결 기록 (testbed_spec.md §5.1).

이 모듈은 **ROS 에 의존하지 않는다** — 폴더 규격이 CLI 의 계약이고, 그 계약은
ECU 없이도 검증 가능해야 하기 때문이다. ROS 호출은 cli.py 가 담당한다.

    data/rosbags/<name>_<MM-DD_HH-MM>/
      bag/            # ros2 bag record 산출물
      profile.yaml    # 제출한 프로파일 원문 사본 (실험 기록의 일부)
      result.json     # goal_id·성공여부·tick·에러 집계 등 요약

전역 유일 키는 (폴더명, goal_id) 쌍 — goal_id 는 세션 내 단조증가일 뿐이라
폴더명이 전역 유일성을 담당한다 (§3.4).
"""

import ast
import hashlib
import glob
import json
import os
import re
import signal
import subprocess
import sys
from datetime import datetime

# bag 에 담을 토픽. **여기 하나만 둔다** — CLI 와 웹이 각자 목록을 들고 있으면 웹으로 돌린
# 런의 bag 에만 어떤 토픽이 빠지는 일이 생기고, 그건 분석 시점에야 드러난다.
#
# 기본은 **1개다** (05 §7). `comm_diag` 를 무조건 넣으면 안 되는 이유:
# 브리지는 `comm_diag_enable:=true` 일 때만 그 발행자를 만들고 **기본값은 false** 다
# (`rd_config.hpp` / `rd_telemetry.cpp`). 없는 토픽을 요청하면 `ros2 bag record` 가
# 죽지는 않지만 bag 메타데이터에 0건 토픽이 남아, 나중에 "계측이 왜 비었나" 를
# 브리지가 아니라 bag 에서 찾게 된다.
RECORD_TOPICS = ['/carrier/control/feedback']
DIAG_TOPIC = '/carrier/control/comm_diag'
DEFAULT_BAG_BASE = 'data/rosbags'
# rosbag2 가 구독을 붙일 시간. 없으면 런 초반 샘플이 빠진다.
BAG_SETTLE_SEC = 1.0


def record_topics(diag=False):
    """이 런에서 기록할 토픽 목록. `diag=True` 는 **브리지가 comm_diag_enable 로 떠 있을
    때만** 의미가 있다 — 호출자가 그것을 알고 넘긴다."""
    return list(RECORD_TOPICS) + ([DIAG_TOPIC] if diag else [])


def start_bag(bag_dir, log=None, diag=False):
    """`ros2 bag record` 를 자식으로 시작. **실패해도 실험은 계속한다** (기록만 유실)."""
    cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + record_topics(diag)
    try:
        return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, FileNotFoundError):
        msg = 'WARN: ros2 bag 실행 불가 — bag 없이 진행'
        (log or (lambda m: print(m, file=sys.stderr)))(msg)
        return None


def stop_bag(proc, log=None):
    """SIGINT 로 정상 종료 — SIGKILL 하면 메타데이터가 안 써져 bag 이 열리지 않는다."""
    if proc is None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        msg = 'WARN: bag 프로세스가 SIGINT 에 응답하지 않아 강제 종료'
        (log or (lambda m: print(m, file=sys.stderr)))(msg)

BAG_SUBDIR = 'bag'
PROFILE_COPY = 'profile.yaml'
RESULT_FILE = 'result.json'

# result.json 스키마 버전 (05 §5.2).
# v1 기록이 이미 84런 있다 — 분석이 분기할 수 있어야 하므로 버전을 명시한다.
SCHEMA_VERSION = 2

# v2 필수 키. **값이 null 이어도 키는 있어야 한다** — "없는 것" 과 "안 채운 것" 을
# 구분하려면 키의 존재 자체가 계약이어야 한다.
REQUIRED_KEYS = (
    'schema_version',
    'name', 'mode', 'goal_id', 'success', 'message',
    'started_at', 'finished_at',
    'profile_source', 'profile_sha256', 'seed',
    'ticks_executed', 'write_err_cnt', 'clamp_cnt',
    'irregular_tick_cnt', 'drop_cnt', 'late_tick_cnt', 'slew_exempt_ticks',
    'clock_converged', 'drift_ppm',
    'node_params', 'bag_dir',
)

MODE_NAMES = {0: 'current', 1: 'velocity', 2: 'position'}


def sanitize_name(name):
    """실험 라벨을 폴더명에 쓸 수 있게 정리한다.

    경로 구분자·공백·특수문자가 그대로 들어가면 폴더가 깨지거나 상위로 탈출한다.
    빈 이름은 'run' 으로 대체 — 이름 없는 실험도 기록은 남아야 한다.
    """
    name = (name or '').strip()
    name = re.sub(r'[^\w.\-]+', '_', name)   # 영숫자/밑줄/점/하이픈만 남긴다
    name = name.strip('._-')
    return name or 'run'


def run_dir_name(name, when=None):
    """<name>_<MM-DD_HH-MM> — 기존 bag 명명 관례(TEST2_* 등)와 같은 형태."""
    when = when or datetime.now()
    return '{}_{}'.format(sanitize_name(name), when.strftime('%m-%d_%H-%M'))


def create_run_dir(base_dir, name, when=None):
    """실험 폴더를 만들고 경로를 돌려준다.

    같은 분(minute)에 두 번 돌리면 이름이 겹치므로 _2, _3 을 붙인다 —
    덮어쓰면 앞 실험 기록이 사라진다.
    """
    base = os.path.abspath(base_dir)
    stem = run_dir_name(name, when)
    path = os.path.join(base, stem)
    suffix = 2
    while os.path.exists(path):
        path = os.path.join(base, '{}_{}'.format(stem, suffix))
        suffix += 1
    os.makedirs(path)
    return path


def bag_path(run_dir):
    """ros2 bag record -o 에 넘길 경로. 미리 만들지 않는다 (rosbag2 가 직접 만든다)."""
    return os.path.join(run_dir, BAG_SUBDIR)


def copy_profile(run_dir, profile_text):
    """제출한 YAML 원문을 그대로 보관 — 재생된 것과 기록된 것이 반드시 같아야 한다."""
    path = os.path.join(run_dir, PROFILE_COPY)
    with open(path, 'w') as f:
        f.write(profile_text)
    return path


def sha256_text(text):
    """프로파일 원문의 sha256. **재생된 것과 기록된 것이 같음을 증명하는 값**이다."""
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def sha256_file(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def write_result(run_dir, result):
    """result 요약 기록. 분석 스크립트가 읽는 계약이므로 키 이름을 함부로 바꾸지 말 것."""
    path = os.path.join(run_dir, RESULT_FILE)
    with open(path, 'w') as f:
        json.dump(result, f, indent=2, ensure_ascii=False, sort_keys=True)
        f.write('\n')
    return path


def build_result(name, profile_path, goal_id, success, message,
                 ticks_executed=0, write_err_cnt=0, clamp_cnt=0,
                 started_at=None, finished_at=None, extra=None,
                 profile_text=None, mode=None, seed=None,
                 irregular_tick_cnt=0, drop_cnt=0, late_tick_cnt=0, slew_exempt_ticks=0,
                 clock_converged=None, drift_ppm=None,
                 node_params=None, bag_dir=None):
    """result.json schema v2 (05 §5.2).

    **실패한 런도 반드시 기록한다** — 왜 실패했는지가 데이터다. reject 로 끝난 경우
    ticks_executed=0, message 에 사유, bag_dir 은 만들어진 만큼 남긴다.

    키 이름은 **코드 쪽이 정본**이다 (`name`/`finished_at`/`profile_source`).
    스펙 §5.2 는 label/ended_at 을 썼지만 사용처가 없는 반면, 이 이름들은 이미 84런의
    기록 자산에 박혀 있고 run_campaign.py 가 읽는다 (05 §5.1).
    """
    out = {
        'schema_version': SCHEMA_VERSION,
        'name': name,
        # 해석된 **실효** mode. 프로파일에 mode: 가 없어도 브리지가 정한 값이 여기 남는다.
        'mode': MODE_NAMES.get(mode) if isinstance(mode, int) else mode,
        'profile_source': profile_path,
        # 사본이 유실돼도 동일성 판정이 가능하도록 해시를 남긴다 (05 §5.2).
        'profile_sha256': sha256_text(profile_text) if profile_text is not None else None,
        # seed 미지정 시 브리지가 시각 기반으로 정한다 — **기록하지 않으면 영구 소실**이라
        # noise/prbs 파형을 두 번 다시 만들 수 없다.
        'seed': int(seed) if seed is not None else None,
        'goal_id': int(goal_id),
        'success': bool(success),
        'message': message or '',
        'ticks_executed': int(ticks_executed),
        'write_err_cnt': int(write_err_cnt),
        'clamp_cnt': int(clamp_cnt),
        # out-of-span 등으로 정규 RW 가 대체된 tick — 그 tick 은 피드백이 결손이다.
        'irregular_tick_cnt': int(irregular_tick_cnt),
        # 발행 큐 드롭 누적. **시계열에 구멍이 있었는지를 사후에 아는 유일한 수단**이다.
        'drop_cnt': int(drop_cnt),
        # 주기를 넘겨 위상이 리셋된 tick. **tick 번호는 안 건너뛴다** — 루프가 따라잡지 않고
        # 시간축을 미루므로, 이 값이 크면 기록된 t(=tick*5ms)와 실제 경과가 벌어진 것이다.
        # 위 셋은 서로 안 겹친다: irregular=자리를 뺏김 / late=늦음 / drop=기록을 놓침.
        'late_tick_cnt': int(late_tick_cnt),
        # E3 — slew 검사를 건너뛴 tick 수 (= noise 구간 길이). 그 구간은 레이트 제한이
        # 없었다는 사실을 분석이 알아야 한다 (05 §3.4).
        'slew_exempt_ticks': int(slew_exempt_ticks),
        # 미수렴이면 header.stamp 가 Orin 수신 시각 fallback 이라 시간축 등급이 다르다.
        # 분석이 그런 런을 수렴 런과 섞으면 안 된다 (05 §5.2 B1).
        'clock_converged': bool(clock_converged) if clock_converged is not None else None,
        'drift_ppm': float(drift_ppm) if drift_ppm is not None else None,
        'started_at': (started_at or datetime.now()).isoformat(timespec='seconds'),
        'finished_at': (finished_at or datetime.now()).isoformat(timespec='seconds'),
        'node_params': node_params if node_params is not None else {},
        # 폴더를 옮겨도 원 위치를 안다.
        'bag_dir': bag_dir,
    }
    if extra:
        out.update(extra)
    return out


def _parse_param_value(text):
    """`ros2 param get` 의 사람용 출력에서 값만 뽑아 JSON 친화 타입으로.

    array('q', [1]) 같은 파이썬 repr 이 기록에 남으면 나중에 읽는 쪽이 그걸 파싱해야 한다.
    실패하면 **원문 문자열 그대로 남긴다** — 못 읽은 것을 지어내지 않는다.
    """
    text = text.strip()
    m = re.search(r"array\([^,]+,\s*(\[.*\])\s*\)", text)
    if m:
        text = m.group(1)
    if text in ('True', 'False'):
        return text == 'True'
    try:
        return json.loads(text)
    except ValueError:
        pass
    try:
        return ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return text


def bridge_node_params(bridge_node='/firmware_bridge_node'):
    """브리지 기동 파라미터 스냅샷 (05 §5.2 node_params).

    `clamp_cnt` 를 해석하려면 **어떤 한계가 걸려 있었는지**를 알아야 한다.
    읽기 실패는 치명적이지 않다 — 없으면 그 키만 빠지고 런은 계속된다.
    """
    out = {}
    for n in ('bridge_mode', 'auto_mode', 'active_motors', 'cmd_current_max', 'stream_timeout'):
        try:
            r = subprocess.run(['ros2', 'param', 'get', bridge_node, n],
                               capture_output=True, text=True, timeout=5)
            if r.returncode != 0:
                continue
            # `ros2 param get` 은 "Integer value is: 2" 처럼 **사람용 문장**으로 답한다.
            txt = r.stdout.strip()
            val = txt.split(' is: ', 1)[1] if ' is: ' in txt else \
                  txt.split(' are: ', 1)[1] if ' are: ' in txt else txt
            out[n] = _parse_param_value(val)
        except Exception:
            continue
    return out


def result_from_action(name, run_dir, result, started_at, profile_text, node_params=None):
    """action Result -> schema v2 dict. **이 매핑을 여기 하나만 둔다.**

    05 §5.3 의 `mode`·`seed`·`irregular_tick_cnt`·`drop_cnt`·`late_tick_cnt`·
    `slew_exempt_ticks`·`clock_converged`·`drift_ppm` 은
    **브리지만 아는 값**이라 Result 로 받아 그대로 옮긴다.
    CLI 가 지어낼 수 있는 값이 아니다.

    ⚠ 매핑이 두 벌이 되면 한쪽만 필드를 빠뜨리고, 그건 **분석할 때가 되어서야** 드러난다.
       실제로 웹 쪽을 따로 짰다가 위 7개 + node_params 를 통째로 빠뜨렸다 (07 §5 W5).
    """
    return build_result(
        name, run_dir, result.goal_id, result.success, result.message,
        result.ticks_executed, result.write_err_cnt, result.clamp_cnt,
        started_at=started_at, profile_text=profile_text,
        mode=result.mode, seed=result.seed,
        irregular_tick_cnt=result.irregular_tick_cnt,
        drop_cnt=result.drop_cnt,
        late_tick_cnt=result.late_tick_cnt,
        slew_exempt_ticks=result.slew_exempt_ticks,
        clock_converged=result.clock_converged,
        drift_ppm=result.drift_ppm,
        node_params=node_params if node_params is not None else bridge_node_params(),
        bag_dir=bag_path(run_dir))


def verify_run_dir(run_dir):
    """폴더 규격 검사 — 문제 리스트를 돌려준다 (빈 리스트 = 규격 충족).

    종전에는 **파일 존재만** 봤다. result.json 이 비어 있거나 키가 빠져도, bag 폴더만
    생기고 rosbag2 가 실패했어도 통과했다. 05 §6.4 가 요구하는 4가지를 추가한다:

      1. result.json 이 v2 필수 키를 전부 갖는가 (값이 null 이어도 키는 있어야 한다)
      2. schema_version 이 아는 값인가
      3. bag/ 안에 metadata.yaml 과 *.db3 이 실제로 있는가
      4. **profile.yaml 의 sha256 이 result.json.profile_sha256 과 일치하는가**

    4번이 핵심이다 — "재생된 것과 기록된 것이 같다" 를 런 종료 시점에 실제로 검증하는
    유일한 지점이다. 불일치하면 그 런은 못 쓴다.
    """
    problems = []
    if not os.path.isdir(run_dir):
        return ['run_dir']

    bag = os.path.join(run_dir, BAG_SUBDIR)
    prof = os.path.join(run_dir, PROFILE_COPY)
    res = os.path.join(run_dir, RESULT_FILE)

    if not os.path.isdir(bag):
        problems.append(BAG_SUBDIR)
    else:
        # 폴더만 생기고 rosbag2 가 실패한 경우를 잡는다 (③).
        if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            problems.append('bag/metadata.yaml')
        if not glob.glob(os.path.join(bag, '*.db3')) and \
           not glob.glob(os.path.join(bag, '*.mcap')):
            problems.append('bag/*.db3')

    if not os.path.isfile(prof):
        problems.append(PROFILE_COPY)
    if not os.path.isfile(res):
        problems.append(RESULT_FILE)
        return problems      # 아래 검사는 result.json 이 있어야 가능하다

    try:
        with open(res) as f:
            data = json.load(f)
    except (ValueError, OSError) as e:
        problems.append('{}: 읽을 수 없음 ({})'.format(RESULT_FILE, e))
        return problems

    if not isinstance(data, dict):
        problems.append('{}: 객체가 아님'.format(RESULT_FILE))
        return problems

    # ② 아는 스키마인가
    ver = data.get('schema_version')
    if ver != SCHEMA_VERSION:
        problems.append('schema_version={} (알 수 없음, 기대 {})'.format(ver, SCHEMA_VERSION))

    # ① 필수 키 — 값이 null 이어도 키는 있어야 한다
    for k in REQUIRED_KEYS:
        if k not in data:
            problems.append('result.json 키 누락: {}'.format(k))

    # ④ 재생된 것과 기록된 것이 같은가 — 이것이 이 함수의 존재 이유다
    recorded = data.get('profile_sha256')
    if os.path.isfile(prof):
        if not recorded:
            problems.append('profile_sha256 없음 — 사본 동일성을 검증할 수 없다')
        else:
            actual = sha256_file(prof)
            if actual != recorded:
                problems.append(
                    'profile.yaml sha256 불일치 — 기록된 것과 재생된 것이 다르다 '
                    '(result={}..., 사본={}...)'.format(recorded[:12], actual[:12]))

    return problems


def profile_label(profile_text, fallback):
    """실험 라벨 결정: YAML 의 `name:` 이 있으면 그걸 쓰고, 없으면 파일명.

    프로파일이 스스로 밝힌 이름이 실험의 정체성이다 (§4.1 name = 실험 라벨).
    파싱 실패는 여기서 문제 삼지 않는다 — YAML 검증은 bridge 의 몫이고,
    CLI 는 폴더 이름 하나 때문에 실행을 막지 않는다.
    """
    try:
        import yaml
        doc = yaml.safe_load(profile_text)
        if isinstance(doc, dict):
            name = doc.get('name')
            if isinstance(name, str) and name.strip():
                return name.strip()
    except Exception:
        pass
    return fallback
