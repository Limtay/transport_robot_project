import math

import pytest

from carrier_teleop.pos_jog import PosJog

DT = 0.01


def jog(**kw):
    p = dict(signs=[1, 1], v_max=60.0, accel=20.0, decel=40.0, max_lead=20.0, s_range=[-720, 720])
    p.update(kw)
    j = PosJog(**p)
    j.anchor([100.0, 200.0])
    return j


def run(j, n, fb=None, **keys):
    k = dict(fwd=False, back=False, ctrl=False, shift=False, stop=False)
    k.update(keys)
    out = None
    for _ in range(n):
        f = fb if fb is not None else j.targets()      # 완벽 추종
        out = j.step(DT, fb_abs=f, **k)
    return out


def test_w_alone_does_not_move():
    """기본 속도 0 — W 만 눌러서는 급출발하지 않는다."""
    j = jog()
    assert run(j, 100, fwd=True) == [100.0, 200.0]


def test_ctrl_ramps_speed_then_holds_when_released():
    j = jog()
    run(j, 100, fwd=True, ctrl=True)           # 1 s
    assert j.v == pytest.approx(20.0)
    run(j, 100, fwd=True)                      # Ctrl 뗌 — 속도 유지
    assert j.v == pytest.approx(20.0)
    assert j.s == pytest.approx(0.5 * 20 * 1 + 20 * 1, abs=0.3)


def test_release_direction_resets_speed():
    j = jog()
    run(j, 100, fwd=True, ctrl=True)
    run(j, 1)
    assert j.v == 0.0
    s = j.s
    run(j, 50, fwd=True)
    assert j.s == s


def test_speed_capped_and_shift_slows():
    j = jog()
    run(j, 1000, fwd=True, ctrl=True)
    assert j.v == pytest.approx(60.0)
    run(j, 50, fwd=True, shift=True)
    assert j.v == pytest.approx(40.0)


def test_same_command_with_signs():
    j = jog(signs=[1, -1])
    t = run(j, 200, back=True, ctrl=True)
    assert t[0] - 100.0 == pytest.approx(-(t[1] - 200.0))
    assert t[0] < 100.0


def test_range_clamps_and_stops():
    j = jog(s_range=[-5, 5])
    run(j, 1000, fwd=True, ctrl=True)
    assert j.s == pytest.approx(5.0)
    assert j.v == 0.0 and '범위' in j.flag


def test_lead_limit_blocks_runaway_when_motor_does_not_follow():
    """모터가 안 따라오면 (전원 꺼짐·걸림) 목표가 max_lead 이상 달아나지 않는다."""
    j = jog()
    t = run(j, 2000, fb=[100.0, 200.0], fwd=True, ctrl=True)
    assert abs(t[0] - 100.0) <= 20.0 + 1.0
    assert '앞섬' in j.flag


def test_invalid_feedback_freezes():
    j = jog()
    run(j, 100, fwd=True, ctrl=True)
    s = j.s
    j.step(DT, True, False, True, False, False, [math.nan, 200.0])
    assert j.s == s


def test_anchor_keeps_travel_counter():
    j = jog()
    run(j, 200, fwd=True, ctrl=True)
    s = j.s
    j.anchor([150.0, 250.0])
    assert j.targets() == pytest.approx([150.0, 250.0])
    assert j.s == s
