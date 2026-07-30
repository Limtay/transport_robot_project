"""C-7 CLI end-to-end (ECU 없이).

가짜 bridge(config 서비스 + run_profile action)를 띄우고 CLI 를 **실제 프로세스로** 실행해
배선과 종료 코드, --record 폴더 규격을 확인한다.
막는 것: 서비스/액션 이름·필드 오배선, op 코드 오지정, run --record 가 폴더를 규격대로
        못 만드는 것, 실패한 실험이 기록에서 누락되는 것.
"""

import json
import os
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mgs_tp_msgs.action import RunProfile
from mgs_tp_msgs.srv import ControlConfig
from control_cli import record

CLI = [sys.executable, '-c',
       'import sys; from control_cli.cli import main; sys.exit(main())']


class FakeBridge(Node):
    """실제 bridge 의 인터페이스 계약만 흉내낸다 (로직은 C-2~C-8 하네스가 검증)."""

    def __init__(self, accept_goal=True, succeed=True):
        super().__init__('fake_bridge')
        self.accept_goal = accept_goal
        self.succeed = succeed
        self.last_request = None
        self.last_goal = None
        self.srv = self.create_service(ControlConfig, '/carrier/control/config', self._on_config)
        self.act = ActionServer(
            self, RunProfile, '/carrier/control/run_profile',
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT if self.accept_goal else GoalResponse.REJECT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT)

    def _on_config(self, req, res):
        self.last_request = (req.op, list(req.motors), req.value)
        res.ok = True
        res.message = 'state=IDLE motor_mask=0x0F op={} motors={} value={}'.format(
            req.op, list(req.motors), req.value)
        return res

    def _execute(self, goal_handle):
        self.last_goal = goal_handle.request
        fb = RunProfile.Feedback()
        for i in range(3):
            fb.t = float(i)
            fb.progress = i / 3.0
            fb.segment_index = i
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)
        if self.succeed:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        res = RunProfile.Result()
        res.success = self.succeed
        res.message = '완료' if self.succeed else '중단'
        res.goal_id = 7
        res.ticks_executed = 1234
        res.write_err_cnt = 0
        res.clamp_cnt = 5
        return res


class BridgeRunner:
    def __init__(self, **kw):
        self.kw = kw

    def __enter__(self):
        rclpy.init()
        self.node = FakeBridge(**self.kw)
        self.exec = MultiThreadedExecutor()
        self.exec.add_node(self.node)
        self.thread = threading.Thread(target=self.exec.spin, daemon=True)
        self.thread.start()
        time.sleep(0.5)
        return self.node

    def __exit__(self, *a):
        self.exec.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=5)


def run_cli(*args, timeout=90):
    return subprocess.run(CLI + list(args), capture_output=True, text=True, timeout=timeout)


def test_status_and_config_reach_the_bridge():
    with BridgeRunner() as bridge:
        r = run_cli('status')
        assert r.returncode == 0, r.stderr
        assert 'state=IDLE' in r.stdout
        assert bridge.last_request[0] == ControlConfig.Request.OP_GET_STATUS

        r = run_cli('config', 'motors', '2', '3')
        assert r.returncode == 0, r.stderr
        op, motors, _ = bridge.last_request
        assert op == ControlConfig.Request.OP_SET_ACTIVE_MOTORS
        assert motors == [2, 3]

        # ctr_mode: 마지막 인자가 mode, 앞이 대상 모터
        r = run_cli('config', 'ctr_mode', '2', '3', '1')
        assert r.returncode == 0, r.stderr
        op, motors, value = bridge.last_request
        assert op == ControlConfig.Request.OP_SET_CTR_MODE
        assert motors == [2, 3] and value == 1

        r = run_cli('config', 'auto_mode', '2')
        assert r.returncode == 0, r.stderr
        op, _, value = bridge.last_request
        assert op == ControlConfig.Request.OP_SET_AUTO_MODE and value == 2

        r = run_cli('rearm')
        assert r.returncode == 0
        assert bridge.last_request[0] == ControlConfig.Request.OP_REARM


def test_run_sends_yaml_and_reports_result(tmp_path):
    prof = tmp_path / 'p.yaml'
    prof.write_text('name: e2e\nmotors: {m1: [{type: hold, duration: 0.1, value: 0}]}\n')
    with BridgeRunner() as bridge:
        r = run_cli('run', str(prof))
        assert r.returncode == 0, r.stderr + r.stdout
        assert 'goal_id=7' in r.stdout
        assert '1234 tick' in r.stdout
        # 제출한 YAML 원문이 그대로 전달돼야 한다
        assert bridge.last_goal.profile_yaml == prof.read_text()
        assert bridge.last_goal.name == 'e2e', 'YAML 의 name 이 라벨이 된다'


def test_run_record_builds_folder_to_spec(tmp_path):
    prof = tmp_path / 'hyst.yaml'
    text = 'name: hyst_label\nmotors: {m1: [{type: hold, duration: 0.1, value: 0}]}\n'
    prof.write_text(text)
    base = tmp_path / 'rosbags'
    with BridgeRunner():
        r = run_cli('run', str(prof), '--record', '--bag-dir', str(base))
        assert r.returncode == 0, r.stderr + r.stdout

    runs = list(base.iterdir())
    assert len(runs) == 1, '실험 1회 = 폴더 1개'
    run_dir = str(runs[0])
    # 폴더명은 YAML 의 name 을 따른다 (파일명 hyst 가 아니라 name: hyst_label)
    assert os.path.basename(run_dir).startswith('hyst_label_'), os.path.basename(run_dir)

    # 프로파일 사본은 원문 그대로
    assert open(os.path.join(run_dir, record.PROFILE_COPY)).read() == text

    res = json.load(open(os.path.join(run_dir, record.RESULT_FILE)))
    assert res['goal_id'] == 7
    assert res['success'] is True
    assert res['ticks_executed'] == 1234
    assert res['clamp_cnt'] == 5
    assert res['name'] == 'hyst_label'

    # bag 은 ros2 bag 가용 여부에 따라 갈린다 — 있으면 폴더 규격 충족, 없으면 CLI 가 경고해야 한다
    missing = record.verify_run_dir(run_dir)
    if missing:
        assert 'WARN' in r.stderr, '규격 미충족을 조용히 넘기면 분석 단계에서야 발견된다'
        assert missing == [record.BAG_SUBDIR]
    else:
        assert os.path.isdir(os.path.join(run_dir, record.BAG_SUBDIR))


def test_rejected_goal_still_records_and_exits_nonzero(tmp_path):
    prof = tmp_path / 'bad.yaml'
    prof.write_text('name: bad\nmotors: {}\n')
    base = tmp_path / 'rosbags'
    with BridgeRunner(accept_goal=False):
        r = run_cli('run', str(prof), '--record', '--bag-dir', str(base))
        assert r.returncode == 1, '거부는 실패 종료 코드여야 스크립트가 감지한다'
        assert '거부' in r.stderr

    runs = list(base.iterdir())
    assert len(runs) == 1
    res = json.load(open(os.path.join(str(runs[0]), record.RESULT_FILE)))
    assert res['success'] is False
    assert res['message'] == 'goal rejected'
    assert res['goal_id'] == 0


def test_missing_profile_file_fails_cleanly():
    r = run_cli('run', '/nonexistent/x.yaml')
    assert r.returncode == 2
    assert '없음' in r.stderr
