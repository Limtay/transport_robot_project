"""텔레메트리 스트림 층 (redesign/07 §4).

**여기서 지켜야 하는 것은 두 가지다.**

  ① 미판독(255 / NaN)이 0 이나 그럴듯한 값으로 둔갑하지 않는다.
     플롯에서 0 으로 찍히면 "프리셋이 안 읽는 블록" 이 "정상 0" 으로 보이고,
     04 §2.3 이 `Covers()` 를 만든 이유가 통째로 무효가 된다.
  ② 200Hz → 50Hz 로 내리면서 **스파이크가 사라지지 않는다.**
     평균을 내면 전류 피크·지연 튐이 지워지는데, 그게 이 실험의 관심사다.
"""

import math

from mgs_tp_msgs.msg import ControlFeedback

from control_web import telemetry


def _hub():
    return telemetry.TelemetryHub(telemetry.enumerate_series(ControlFeedback),
                                  stream_hz=50.0, history_sec=1.0)


# ── ① 필드 열거는 메시지 정의에서 나온다 ───────────────────────────────────
def test_series_are_enumerated_from_message_not_hardcoded():
    specs = telemetry.enumerate_series(ControlFeedback)
    keys = {s.key for s in specs}

    # 배열은 펼친다 — 플롯 한 줄이 계열 하나여야 한다.
    for i in range(4):
        assert 'fb_current[%d]' % i in keys
        assert 'dt_motor[%d]' % i in keys
    for i in range(5):
        assert 'link_angle[%d]' % i in keys
    for i in range(8):
        assert 'lc[%d]' % i in keys

    # 스칼라는 그대로.
    assert 'hw_error' in keys and 'rw_err' in keys and 'ecu_tick' in keys
    # header 는 시간축이라 계열이 아니다.
    assert not any(k.startswith('header') for k in keys)

    # 타입 분류 — 미판독 규칙이 여기서 갈린다.
    kinds = {s.key: s.kind for s in specs}
    assert kinds['fb_current[0]'] == 'f'
    assert kinds['hw_error'] == 'u8'
    assert kinds['ecu_tick'] == 'i'       # uint32 — 센티넬 없음
    assert kinds['stamp_valid'] == 'b'


def test_every_series_has_a_group_for_the_ui_tree():
    for s in telemetry.enumerate_series(ControlFeedback):
        assert s.group, s.key            # 분류가 없으면 트리에서 사라진다


# ── ② 미판독 ───────────────────────────────────────────────────────────────
def test_uint8_sentinel_becomes_null_not_255():
    assert telemetry._clean(255, 'u8') is None
    assert telemetry._clean(0, 'u8') == 0        # 정상 0 을 미판독으로 만들지 않는다
    assert telemetry._clean(254, 'u8') == 254


def test_nan_becomes_null_not_zero():
    assert telemetry._clean(float('nan'), 'f') is None
    assert telemetry._clean(float('inf'), 'f') is None
    assert telemetry._clean(0.0, 'f') == 0.0


def test_uint32_has_no_sentinel():
    # ecu_tick 255 는 **정상 값**이다. u8 규칙을 여기에 적용하면 tick 이 사라진다.
    assert telemetry._clean(255, 'i') == 255


def test_unread_sample_poisons_the_whole_frame():
    """읽은 표본만 골라 남기면 "언제부터 안 읽혔는지" 가 지워진다."""
    hub = _hub()
    m = ControlFeedback()
    m.fb_current = [1.0, 0.0, 0.0, 0.0]
    hub.add(m)
    m2 = ControlFeedback()
    m2.fb_current = [float('nan'), 0.0, 0.0, 0.0]
    hub.add(m2)
    hub.flush(1.0)
    assert hub.history(['fb_current[0]'])['d']['fb_current[0]'] == [None]


# ── ③ 다운샘플이 스파이크를 지우지 않는다 ──────────────────────────────────
def test_downsample_preserves_min_and_max():
    hub = _hub()
    for v in (0.0, 5.0, -3.0, 0.0):          # 200Hz 4개 = 50Hz 한 프레임
        m = ControlFeedback()
        m.fb_current = [v, 0.0, 0.0, 0.0]
        hub.add(m)
    hub.flush(1.0)

    got = hub.history(['fb_current[0]'])['d']['fb_current[0]']
    assert got == [[-3.0, 5.0]], '평균을 내면 5.0 피크가 사라진다'


def test_constant_window_collapses_to_scalar():
    """창 안에서 안 변했으면 [v, v] 대신 v — 90계열 × 50Hz 라 바이트가 그냥 늘어난다."""
    hub = _hub()
    for _ in range(4):
        m = ControlFeedback()
        m.fb_current = [2.0, 0.0, 0.0, 0.0]
        hub.add(m)
    hub.flush(1.0)
    assert hub.history(['fb_current[0]'])['d']['fb_current[0]'] == [2.0]


def test_flush_without_samples_emits_nothing():
    """브리지가 없으면 프레임도 없다 — 빈 프레임을 넣으면 시간축에 가짜 점이 생긴다."""
    hub = _hub()
    hub.flush(1.0)
    hub.flush(2.0)
    assert hub.stats()['frames'] == 0
    assert hub.history(['fb_current[0]'])['t'] == []


# ── ④ 구독 ─────────────────────────────────────────────────────────────────
def test_new_subscriber_gets_history_first():
    """체크한 순간부터 그리면 방금 무슨 일이 있었는지 볼 수 없다 (07 §4 ④)."""
    hub = _hub()
    for i in range(3):
        m = ControlFeedback()
        m.fb_current = [float(i), 0.0, 0.0, 0.0]
        hub.add(m)
        hub.flush(float(i))
    h = hub.history(['fb_current[0]'])
    assert h['t'] == [0.0, 1.0, 2.0]
    assert h['d']['fb_current[0]'] == [0.0, 1.0, 2.0]


def test_subscriber_receives_only_selected_keys():
    hub = _hub()
    sub = hub.subscribe(['fb_current[0]'])
    q, _ev, _keys = sub
    m = ControlFeedback()
    m.fb_current = [7.0, 0.0, 0.0, 0.0]
    hub.add(m)
    hub.flush(1.0)
    assert len(q) == 1
    assert list(q[0]['d'].keys()) == ['fb_current[0]']
    hub.unsubscribe(sub)
    assert hub.stats()['subs'] == 0


def test_slow_subscriber_drops_old_frames_instead_of_blocking():
    """브라우저가 멈춰도 서버가 붙잡히면 안 된다 — 오래된 것부터 버린다."""
    hub = _hub()
    sub = hub.subscribe(['fb_current[0]'])
    q, _ev, _keys = sub
    for i in range(1000):
        m = ControlFeedback()
        m.fb_current = [float(i), 0.0, 0.0, 0.0]
        hub.add(m)
        hub.flush(float(i))
    assert len(q) == q.maxlen
    assert q[-1]['t'] == 999.0               # 최신은 살아 있다
    hub.unsubscribe(sub)


def test_history_ring_is_bounded():
    hub = _hub()                              # history_sec=1.0, 50Hz → 50칸
    for i in range(200):
        m = ControlFeedback()
        m.fb_current = [float(i), 0.0, 0.0, 0.0]
        hub.add(m)
        hub.flush(float(i))
    h = hub.history(['fb_current[0]'])
    assert len(h['t']) == 50
    assert h['t'][-1] == 199.0


# ── ⑤ 전송 형식 ────────────────────────────────────────────────────────────
def test_sse_pack_is_one_line_and_rejects_nan():
    """데이터에 개행이 들어가면 SSE 프레임이 깨진다. NaN 은 JSON 이 아니다 —
    미판독은 이미 None 으로 바뀌어 있어야 하고, 안 바뀐 것이 있으면 여기서 터진다."""
    blob = telemetry.sse_pack('f', {'t': 1.0, 'd': {'a': None, 'b': [1.0, 2.0]}})
    assert blob.endswith(b'\n\n')
    body = blob.decode().split('data: ')[1]
    assert '\n' not in body.rstrip('\n')

    try:
        telemetry.sse_pack('f', {'v': float('nan')})
    except ValueError:
        pass
    else:
        raise AssertionError('NaN 이 그대로 직렬화됐다 — 브라우저에서 파싱 실패한다')
