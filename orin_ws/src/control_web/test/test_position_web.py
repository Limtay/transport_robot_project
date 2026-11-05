"""웹의 INST_POS 처리 (redesign/13).

슬라이더가 절대값이 되면서 웹이 직접 위험을 만들 수 있는 곳만 잠근다 — 브리지 판정은 흉내내지 않는다.
"""

import json
import math
import os
import re
import time
from types import SimpleNamespace

import pytest
import rclpy

from mgs_tp_msgs.msg import CmdMotor, ControlFeedback, PosFeedback

from control_cli import record
from control_web.profile_run import ProfileRunner, _json_position
from control_web.server import ControlWeb, POS_INST_SLIDER_SPAN

WWW = os.path.join(os.path.dirname(__file__), '..', 'www')
STATUS = {'control_state': 'IDLE', 'motor_mask': 1, 'auto_mode': 'position',
          'ctr_mode': ['position'] * 4, 'position_inst': True, 'pos': None, 'last_reject': None}


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = ControlWeb()
    with n._lock:
        n._status = dict(STATUS)
    yield n
    n.destroy_node()


def pos_msg(abs0=965.0, flag=0):
    m = PosFeedback()
    m.fb_abs = [abs0, 0.0, 0.0, 0.0]
    m.cmd_abs = [float('nan'), 0.0, 0.0, 0.0]
    m.offset = [3000.0, 0.0, 0.0, 0.0]
    m.origin_seq = [1, 0, 0, 0]
    m.origin_flag = flag
    return m


def motor_fb(pos0):
    m = ControlFeedback()
    m.fb_position = [pos0, 0.0, 0.0, 0.0]
    return m


def test_untouched_slider_follows_abs_not_motor_frame(node):
    """원점 이동 뒤 모터 좌표는 약 3000° 작다. 그대로 나가면 그만큼 튄다."""
    node._on_pos_fb(pos_msg(3165.0))
    node._on_fb(motor_fb(165.0))
    assert node.snapshot()['setpoint'][0] == pytest.approx(3165.0)


def test_rw_position_still_follows_motor_frame(node):
    with node._lock:
        node._status = dict(STATUS, position_inst=False)
    node._on_fb(motor_fb(42.0))
    assert node.snapshot()['setpoint'][0] == pytest.approx(42.0)


def test_nan_abs_keeps_last_setpoint(node):
    node._on_pos_fb(pos_msg(965.0))
    node._on_pos_fb(pos_msg(float('nan')))
    assert node.snapshot()['setpoint'][0] == pytest.approx(965.0)


def test_snapshot_is_strict_json(node):
    """NaN 이 나가면 브라우저 JSON.parse 가 깨져 화면 전체가 "응답 없음" 이 된다."""
    node._on_pos_fb(pos_msg())
    text = json.dumps(node.snapshot(), allow_nan=False)
    assert json.loads(text)['pos_fb']['cmd_abs'][0] is None


def test_slider_range_follows_position(node):
    """±180° 고정이면 965° 에서 슬라이더를 건드리는 순간 170° 로 간다."""
    node._on_pos_fb(pos_msg(965.0))
    lo, hi, _ = node.snapshot()['ranges'][0]
    assert (lo, hi) == (965.0 - POS_INST_SLIDER_SPAN, 965.0 + POS_INST_SLIDER_SPAN)


def test_rw_slider_range_is_unchanged(node):
    with node._lock:
        node._status = dict(STATUS, position_inst=False)
    assert tuple(node.snapshot()['ranges'][0][:2]) == (-180.0, 180.0)


@pytest.mark.parametrize('flag,word', [(0x10, 'FRAME_LOST'), (0x01, '원점')])
def test_arm_refused_while_frame_unsafe(node, flag, word):
    node._on_pos_fb(pos_msg(flag=flag))
    ok, msg = node.do_arm(True)
    assert not ok and word in msg
    assert not node._armed


def test_arm_refused_before_first_pos_feedback(node):
    """setpoint 초기값 0 이 절대 0° 명령으로 나가는 경우다."""
    ok, msg = node.do_arm(True)
    assert not ok and 'pos_feedback' in msg


def test_arm_refused_when_pos_feedback_is_stale(node):
    node._on_pos_fb(pos_msg())
    node._pos_stamp = time.time() - 5.0
    assert not node.do_arm(True)[0]


def test_arm_refused_before_status_is_known(node):
    with node._lock:
        node._status = dict(STATUS, control_state='?', position_inst=None)
    ok, msg = node.do_arm(True)
    assert not ok and '상태' in msg


def test_arm_off_is_never_blocked(node):
    """감속 방향은 막지 않는다 (01 §6.2)."""
    node._call_config = lambda *a, **k: (False, 'no bridge')
    node._armed = True
    node.do_arm(False)
    assert not node._armed


def test_tick_publishes_abs_for_inactive_motor(node):
    got = []
    node.pub_cmd = SimpleNamespace(publish=got.append)
    m = pos_msg()
    m.fb_abs = [965.0, 3100.0, 0.0, 0.0]
    node._on_pos_fb(m)
    node._on_fb(motor_fb(100.0))
    node._armed = True
    node._last_client = time.time()
    node._tick()
    assert isinstance(got[0], CmdMotor)
    assert got[0].position[0] == pytest.approx(965.0)
    assert got[0].position[1] == pytest.approx(3100.0)


class FakeBagStart:
    def __init__(self):
        self.pos = None

    def __call__(self, path, log=None, diag=False, pos=False):
        self.pos = pos
        return None


class FakeAction:
    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        return SimpleNamespace(add_done_callback=lambda cb: None)


@pytest.mark.parametrize('inst', [True, False])
def test_web_run_records_pos_topic_only_in_position_inst(node, monkeypatch, tmp_path, inst):
    """웹 기록에 pos_feedback 이 빠지던 구멍 (HANDOFF §3). 판정은 GET_STATUS 값을 따른다."""
    bag = FakeBagStart()
    monkeypatch.setattr(record, 'start_bag', bag)
    monkeypatch.setattr(record, 'BAG_SETTLE_SEC', 0.0)
    r = ProfileRunner(node, bag_base=str(tmp_path), position_inst=lambda: inst)
    r.act = FakeAction()
    spec = {'name': 'p', 'mode': 'position', 'position_ref': 'start',
            'limits': {'range': [-5.0, 5.0], 'max_track_err': 15.0},
            'motors': {'m1': [{'type': 'hold', 'duration': 1.0, 'value': 0.0}]}}
    ok, _ = r.start(spec)
    assert ok and bag.pos is inst
    assert r._pos_run is inst


def test_position_result_is_strict_json():
    res = SimpleNamespace(origin_cnt=[1, 0, 0, 0],
                          start_abs_deg=[1.0, float('nan'), 0.0, 0.0],
                          end_abs_deg=[2.0, 0.0, 0.0, 0.0],
                          max_track_err_deg=[3.0, 0.0, 0.0, 0.0],
                          lock_reason='')
    out = _json_position(res)
    json.dumps(out, allow_nan=False)
    assert out['start_abs_deg'][1] is None and out['origin_cnt'][0] == 1


def test_register_decoder_knows_every_pos_fb_type():
    """Tab4 는 맵을 읽어 그린다 (13 §6). 디코더가 모르는 타입이면 값이 조용히 깨진다."""
    with open(os.path.join(WWW, 'regmap.json'), encoding='utf-8') as f:
        block = next(b for b in json.load(f)['blocks'] if b['name'] == 'POS_FB')
    with open(os.path.join(WWW, 'regmap.js'), encoding='utf-8') as f:
        js = f.read()
    size_table = re.search(r'SIZE\s*=\s*\{([^}]*)\}', js).group(1)
    for t in {fld['type'] for fld in block['fields']}:
        assert re.search(r'\b%s\s*:' % t, size_table), t
        assert "case '%s'" % t in js, t


def test_html_has_position_controls():
    with open(os.path.join(WWW, 'index.html'), encoding='utf-8') as f:
        html = f.read()
    for needle in ('id="c-posinst"', 'id="pos-panel"', 'id="btn-rearm"', 'id="pf-ref"',
                   'id="pf-trackerr"', 'position_inst: q('):
        assert needle in html, needle
    # 패널이 Tab1·Tab2 를 오가므로 control 기동 패널 안에 있어야 한다.
    assert html.index('id="bp-ctrl"') < html.index('id="pos-panel"') < html.index('id="bp-proj"')
    assert math.isfinite(POS_INST_SLIDER_SPAN)
