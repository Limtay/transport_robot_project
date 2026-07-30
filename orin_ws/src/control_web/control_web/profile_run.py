"""Tab2 계획형 — 프로파일 조립 · 실행 · 기록 (redesign/07 §2, §5 W5).

## 왜 YAML 을 브라우저에서 만들지 않는가

`RunProfile` 액션은 **YAML 문자열**을 받는다. 브라우저에서 문자열을 이어 붙여 만들 수도
있지만, 따옴표·부동소수 표기·들여쓰기를 손으로 맞추는 순간 "웹으로 만든 프로파일만
파싱이 다르다" 가 생긴다. 브라우저는 **구조(JSON)** 만 보내고 여기서 PyYAML 로 덤프한다.

## 기록은 CLI 와 같은 모듈을 쓴다

`control_cli/record.py` 를 그대로 import 한다. 웹으로 돌린 런도 **동일한 schema v2
`result.json` + 같은 토픽의 bag** 이 남아 분석 파이프라인에 그대로 들어간다.
여기에 기록 로직을 다시 짜면 두 벌이 되고, v3 로 갈 때 갈라진다.

## 진행 상태를 폴링으로 주는 이유

액션 피드백은 이 노드가 받고, 브라우저는 `/api/profile/status` 를 폴링한다. SSE 를 하나 더
열 만큼의 데이터가 아니고(1~10Hz), 재생 중 새로고침해도 상태가 이어진다.
"""

import os
import threading
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.action import ActionClient

from mgs_tp_msgs.action import RunProfile

from control_cli import record

RUN_ACTION = '/carrier/control/run_profile'

# 세그먼트 타입별 필수 인자. **브리지 검증의 사본이 아니라 UI 안내용이다** —
# 진짜 검증은 브리지가 하고(rd_profile.cpp), 거부 사유가 그대로 올라온다.
# 여기서 막는 것은 "인자를 안 채우고 보낸 것" 뿐이다.
SEG_ARGS = {
    'hold':   ['duration', 'value'],
    'ramp':   ['duration', 'from', 'to'],
    'step':   ['duration', 'from', 'to', 't_step'],
    'sine':   ['duration', 'amp', 'freq'],
    'chirp':  ['duration', 'amp', 'f0', 'f1'],
    'prbs':   ['duration', 'low', 'high', 'bit_duration'],
    'noise':  ['duration', 'mean', 'std'],
    'stair':  ['step_duration', 'values'],
    'custom': ['samples'],
}


def build_yaml(spec):
    """브라우저의 JSON → 프로파일 YAML.

    `spec` = {name, mode, limits:{...}, motors:{m1:[seg,...], ...}}
    반환: (yaml_text, error). error 가 있으면 yaml_text 는 None.
    """
    name = (spec.get('name') or 'web').strip()
    mode = spec.get('mode') or 'current'
    motors = spec.get('motors') or {}
    if not motors:
        return None, '모터가 하나도 없다'

    doc = {'name': name, 'mode': mode}
    lim = {k: v for k, v in (spec.get('limits') or {}).items() if v not in (None, '', [])}
    if lim:
        doc['limits'] = lim
    if spec.get('seed') not in (None, ''):
        doc['seed'] = int(spec['seed'])

    out_motors = {}
    for mkey, segs in motors.items():
        if not segs:
            continue
        clean = []
        for i, s in enumerate(segs):
            t = s.get('type')
            if t not in SEG_ARGS:
                return None, '{} 세그[{}]: 알 수 없는 type={}'.format(mkey, i, t)
            d = {'type': t}
            for a in SEG_ARGS[t]:
                if a not in s or s[a] in (None, ''):
                    return None, '{} 세그[{}] ({}): {} 가 비어 있다'.format(mkey, i, t, a)
                d[a] = s[a]
            # 선택 인자는 있을 때만 (기본값을 우리가 적어 넣으면 브리지 기본값과 갈라진다)
            for a in ('offset', 'rate'):
                if s.get(a) not in (None, ''):
                    d[a] = s[a]
            clean.append(d)
        if clean:
            out_motors[mkey] = clean
    if not out_motors:
        return None, '세그먼트가 하나도 없다'
    doc['motors'] = out_motors
    return yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, default_flow_style=None), None


class ProfileRunner:
    """한 번에 하나만 돈다. 진행 상태는 폴링으로 노출한다."""

    def __init__(self, node, bag_base=None, diag_enabled=None):
        self.node = node
        self.act = ActionClient(node, RunProfile, RUN_ACTION)
        self.bag_base = bag_base or record.DEFAULT_BAG_BASE
        # comm_diag 토픽을 bag 에 넣을지 — **브리지를 어떻게 띄웠는지에 달렸다.**
        # 기동 파라미터를 아는 것은 supervisor 뿐이라 콜러블로 받는다 (여기서 다시
        # 판정하면 두 벌이 되고, 웹 기동 패널이 그 파라미터를 받는 한 갈라진다).
        self._diag_enabled = diag_enabled or (lambda: False)
        self._lock = threading.Lock()
        self._st = {'state': 'idle', 'progress': 0.0, 't': 0.0, 'segment': 0,
                    'message': '', 'run_dir': None, 'result': None, 'yaml': None}
        self._handle = None
        self._bag = None

    def status(self):
        with self._lock:
            return dict(self._st)

    def start(self, spec, do_record=True):
        with self._lock:
            if self._st['state'] == 'running':
                return False, '이미 재생 중이다 — 끝나거나 중단한 뒤에 다시'
        text, err = build_yaml(spec)
        if err:
            return False, err
        if not self.act.wait_for_server(timeout_sec=5.0):
            return False, 'action 서버({}) 없음 — 브리지가 control 로 떠 있는지 확인'.format(RUN_ACTION)

        name = record.profile_label(text, spec.get('name') or 'web')
        started = datetime.now()
        run_dir, bag = None, None
        if do_record:
            # CLI 와 **같은 모듈**로 같은 폴더 구조·같은 토픽을 남긴다.
            run_dir = record.create_run_dir(self.bag_base, name)
            record.copy_profile(run_dir, text)
            bag = record.start_bag(record.bag_path(run_dir),
                                   log=lambda m: self.node.get_logger().warn(m),
                                   diag=self._diag_enabled())
            time.sleep(record.BAG_SETTLE_SEC)

        with self._lock:
            self._st = {'state': 'running', 'progress': 0.0, 't': 0.0, 'segment': 0,
                        'message': '재생 요청 중…', 'run_dir': run_dir, 'result': None,
                        'yaml': text}
            self._bag = bag
            self._started = started
            self._name = name
            self._text = text

        goal = RunProfile.Goal()
        goal.name = name
        goal.profile_yaml = text
        fut = self.act.send_goal_async(goal, feedback_callback=self._on_fb)
        fut.add_done_callback(self._on_goal)
        return True, '재생 요청 (기록 {})'.format(run_dir or '없음')

    def abort(self):
        with self._lock:
            h = self._handle
        if h is None:
            return False, '진행 중인 재생이 없다'
        h.cancel_goal_async()
        return True, '취소 요청'

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def _on_fb(self, fb):
        with self._lock:
            self._st['progress'] = fb.feedback.progress
            self._st['t'] = fb.feedback.t
            self._st['segment'] = fb.feedback.segment_index
            self._st['message'] = '재생 중'

    def _on_goal(self, fut):
        h = fut.result() if fut.done() else None
        if h is None or not h.accepted:
            self._finish(None, 'goal 거부됨 — 브리지 로그에서 사유 확인 '
                               '(FSM 상태·프로파일 검증·mode/auto_mode 불일치)')
            return
        with self._lock:
            self._handle = h
            self._st['message'] = '재생 중'
        h.get_result_async().add_done_callback(
            lambda f: self._finish(f.result().result if f.done() else None, None))

    def _finish(self, result, reject_msg):
        with self._lock:
            bag, run_dir = self._bag, self._st['run_dir']
            started, name, text = self._started, self._name, self._text
            self._bag = None
            self._handle = None
        record.stop_bag(bag, log=lambda m: self.node.get_logger().warn(m))

        if run_dir:
            # **거부된 런도 기록한다 — 왜 거부됐는지가 데이터다** (05 §5.2).
            # 매핑은 CLI 와 **같은 함수**를 쓴다: 여기서 다시 적었다가 브리지만 아는 값
            # 7개 + node_params 를 통째로 빠뜨린 적이 있다.
            if result is None:
                summary = record.build_result(
                    name, run_dir, 0, False, reject_msg or '결과 수신 실패',
                    started_at=started, profile_text=text,
                    node_params=record.bridge_node_params(),
                    bag_dir=record.bag_path(run_dir))
            else:
                summary = record.result_from_action(name, run_dir, result, started, text)
            record.write_result(run_dir, summary)
            problems = record.verify_run_dir(run_dir)
            if problems:
                # 규격 위반을 조용히 넘기면 분석 단계에서야 발견된다.
                for p in problems:
                    self.node.get_logger().error('기록 규격 위반: %s' % p)

        with self._lock:
            self._st['state'] = 'idle'
            if result is None:
                self._st['message'] = reject_msg or '결과 수신 실패'
                self._st['result'] = None
            else:
                self._st['progress'] = 1.0 if result.success else self._st['progress']
                self._st['message'] = ('완료' if result.success else '실패') + ': ' + result.message
                self._st['result'] = {
                    'success': bool(result.success), 'goal_id': int(result.goal_id),
                    'ticks': int(result.ticks_executed),
                    'write_err': int(result.write_err_cnt), 'clamp': int(result.clamp_cnt),
                    'drop': int(getattr(result, 'drop_cnt', 0)),
                    'irregular': int(getattr(result, 'irregular_tick_cnt', 0)),
                    'late': int(getattr(result, 'late_tick_cnt', 0)),
                    # 조용히 지나가면 안 되는 둘 — 성공했는데 데이터가 온전하지 않다는 뜻이다.
                    'clock_converged': bool(getattr(result, 'clock_converged', True)),
                    'run_dir': run_dir,
                }
