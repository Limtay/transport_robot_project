"""position_inst CLI (redesign/12 §7) — AI 가 로그 없이 판정할 수 있는가.

preflight 판정은 순수 함수라 ROS 없이, lin-zero·run --json 은 가짜 bridge 로 실제 프로세스를 돌린다.
"""

import json
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_cli.cli import build_parser, preflight_checks
from mgs_tp_msgs.action import RunProfile
from mgs_tp_msgs.srv import ControlConfig

CLI = [sys.executable, '-c',
       'import sys; from control_cli.cli import main; sys.exit(main())']


def good_status():
    return {
        'position_inst': True, 'control_state': 'IDLE', 'lock_reason': None,
        'last_reject': None, 'ecu_sys_state': 2, 'motor_mask': 0x09,
        'stamp_valid': True, 'safe_stop': True, 'safe_stop_detail': None,
        'nodes': {'ecu': {'connected': True}},
        'pos': {
            'pend': 0, 'frame_lost': 0,
            'rls': {'ew': 0, 'deg': [1.0, 2.0, 3.0], 'state': 0x02},
            'lin': {'x': 0, 'y': 0, 'zero_result': 0, 'zero_seq': 4},
            'loadcell': {'avg': [100, 200], 'state': 0x02},
        },
    }


def failed(checks):
    return {c['name'] for c in checks if not c['ok']}


def test_preflight_passes_on_a_good_status():
    assert failed(preflight_checks(good_status())) == set()


def test_preflight_names_each_violation_with_a_reason():
    cases = [
        (lambda d: d.update(position_inst=False), 'position_inst'),
        (lambda d: d['nodes']['ecu'].update(connected=False), 'ecu_connected'),
        (lambda d: d.update(ecu_sys_state=3), 'ecu_auto'),
        (lambda d: d.update(control_state='LOCKED'), 'fsm_idle'),
        (lambda d: d.update(motor_mask=0), 'active_motors'),
        (lambda d: d.update(stamp_valid=False), 'clock_converged'),
        (lambda d: d.update(safe_stop=False, safe_stop_detail='회전 중'), 'safe_stop'),
        (lambda d: d['pos'].update(pend=0x01), 'no_pend'),
        (lambda d: d['pos'].update(frame_lost=0x08), 'no_frame_lost'),
        (lambda d: d['pos']['loadcell'].update(state=0xF2), 'loadcell'),
    ]
    for mutate, name in cases:
        d = good_status()
        mutate(d)
        checks = preflight_checks(d)
        assert name in failed(checks), name
        assert all(c['detail'] for c in checks if not c['ok']), '사유 없는 실패가 있다'


def test_preflight_ignores_pend_on_inactive_motors():
    d = good_status()
    d['pos']['pend'] = 0x02    # M2 는 mask 0x09 밖이다
    assert failed(preflight_checks(d)) == set()


def test_preflight_checks_only_requested_rls_channels():
    d = good_status()
    d['pos']['rls']['ew'] = 0b10 << 6    # ch3 Error
    assert failed(preflight_checks(d, [0, 1])) == set()
    assert failed(preflight_checks(d, [3])) == {'rls_ch3'}


def test_preflight_fails_without_frame():
    d = good_status()
    d['pos'] = None
    assert 'frame' in failed(preflight_checks(d))


def test_new_commands_parse():
    p = build_parser()
    a = p.parse_args(['preflight', '--json', '--rls', '0', '3'])
    assert (a.cmd, a.json, a.rls) == ('preflight', True, [0, 3])
    assert p.parse_args(['lin-zero', '--json']).json is True
    assert p.parse_args(['run', 'x.yaml', '--json']).json is True


class FakePosBridge(Node):
    """status JSON + OP_LIN_ZERO + run_profile 만 흉내낸다."""

    def __init__(self, zero_result=1, respond=True, accept=True):
        super().__init__('fake_pos_bridge')
        self.status = good_status()
        self.status['last_reject'] = None if accept else 'position_ref: start 필수'
        self.zero_result, self.respond, self.accept = zero_result, respond, accept
        self.zero_requests = 0
        self.srv = self.create_service(ControlConfig, '/carrier/control/config', self._on_config)
        self.act = ActionServer(
            self, RunProfile, '/carrier/control/run_profile',
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT)

    def _on_config(self, req, res):
        res.ok = True
        if req.op == ControlConfig.Request.OP_LIN_ZERO:
            self.zero_requests += 1
            res.message = 'sent'
            if self.respond:
                self.status['pos']['lin'].update(zero_seq=5, zero_result=self.zero_result)
        else:
            res.message = json.dumps(self.status)
        return res

    def _execute(self, gh):
        gh.succeed()
        res = RunProfile.Result()
        res.success, res.message, res.goal_id, res.ticks_executed = True, '완료', 3, 200
        res.origin_cnt = [1, 0, 0, 0]
        res.start_abs_deg = [10.0, 0.0, 0.0, 0.0]
        res.end_abs_deg = [3110.0, 0.0, 0.0, 0.0]
        res.max_track_err_deg = [2.5, 0.0, 0.0, 0.0]
        return res


class Bridge:
    def __init__(self, **kw):
        self.kw = kw

    def __enter__(self):
        rclpy.init()
        self.node = FakePosBridge(**self.kw)
        self.exec = MultiThreadedExecutor()
        self.exec.add_node(self.node)
        self.th = threading.Thread(target=self.exec.spin, daemon=True)
        self.th.start()
        time.sleep(0.3)
        return self.node

    def __exit__(self, *a):
        self.exec.shutdown()
        self.th.join(timeout=5)
        self.node.destroy_node()
        rclpy.shutdown()


def run_cli(*args):
    return subprocess.run(CLI + list(args), capture_output=True, text=True, timeout=60)


def test_lin_zero_success_and_board_reject():
    with Bridge(zero_result=1) as b:
        r = run_cli('lin-zero', '--json')
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout)['result'] == 'ok'
        assert b.zero_requests == 1
    with Bridge(zero_result=2):
        r = run_cli('lin-zero', '--json')
        assert r.returncode == 1
        assert json.loads(r.stdout)['result'] == 'board_reject'


def test_lin_zero_without_response_is_indeterminate_and_not_retried():
    with Bridge(respond=False) as b:
        r = run_cli('lin-zero', '--json')
        assert r.returncode == 1
        assert json.loads(r.stdout)['result'] == 'indeterminate'
        assert b.zero_requests == 1, '엣지 명령을 재전송하면 초기화가 두 번 실행될 수 있다'


def test_run_json_prints_exactly_one_json_object(tmp_path):
    prof = tmp_path / 'p.yaml'
    prof.write_text('name: t\n')
    with Bridge():
        r = run_cli('run', str(prof), '--json')
    assert r.returncode == 0, r.stderr
    d = json.loads(r.stdout)     # 진행 표시가 섞이면 여기서 깨진다
    assert d['success'] is True and d['rejected'] is False
    assert d['position']['origin_cnt'] == [1, 0, 0, 0]
    assert d['position']['end_abs_deg'][0] == 3110.0


def test_run_json_reports_reject_reason(tmp_path):
    prof = tmp_path / 'p.yaml'
    prof.write_text('name: t\n')
    with Bridge(accept=False):
        r = run_cli('run', str(prof), '--json')
    assert r.returncode == 1
    d = json.loads(r.stdout)
    assert d['rejected'] is True
    assert 'position_ref' in d['last_reject']
