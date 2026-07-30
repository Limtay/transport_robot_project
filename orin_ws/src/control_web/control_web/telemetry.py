"""텔레메트리 스트림 — plotjuggler 방식 모니터링의 데이터 층 (redesign/07 §4).

종전 웹은 0.5s 마다 **스냅샷**만 폴링했다. 시계열이 없으니 "지금 값" 밖에 못 그린다.
필드를 끌어다 그래프로 보려면 세 가지가 필요하다: 흘려보낼 것(SSE), 무엇을 흘릴지
정하는 목록(필드 열거), 그리고 200Hz 를 그대로 보내지 않는 다운샘플.


## ① 필드 목록은 **자동으로** 뽑는다 (§4.2)

`ControlFeedback` 은 40 필드고 배열을 펼치면 90여 계열이다. 이걸 손으로 적으면 메시지가
바뀔 때 조용히 갈라진다 — 이 프로젝트가 반복해서 지워 온 결함 형태다. `rosidl` 이 이미
`get_fields_and_field_types()` 로 타입까지 준다. 그것을 진실 원천으로 쓴다.


## ② 미판독은 **선을 끊는다** (§4.3)

`255`(uint8) / `NaN`(float) 은 "안 읽음" 이다. 플롯에서 0 으로 찍으면 프리셋이 읽지 않는
블록이 "정상 0" 으로 보인다 — 04 §2.3 이 `Covers()` 를 만든 이유가 통째로 무효가 된다.
여기서 `None` 으로 바꾸고, 브라우저는 그 자리를 **gap** 으로 그린다.

⚠ uint8 의 255 를 센티넬로 보는 것은 이 메시지의 규약이다 (`hw_error`·`lc`·`hs`·
`degraded_cnt`·`control_state`·`motor_mask` 전부 실효 범위가 255 미만이다).
uint32(`ecu_tick`·`drop_cnt`)에는 센티넬이 없다 — 그 자리를 None 으로 만들면 안 된다.


## ③ 200Hz 를 그대로 보내지 않는다 (§4.1)

브라우저로 50Hz 로 내린다. **평균이 아니라 min/max 를 보존한다** — 평균을 내면 스파이크가
사라지는데, 스파이크가 이 실험의 관심사다 (전류 피크·지연 튐).

## ④ 새로 구독한 필드는 **과거부터** 보인다

체크박스를 켠 순간부터 그리기 시작하면 "방금 무슨 일이 있었나" 를 볼 수 없다. 링버퍼에
최근 구간을 들고 있다가 구독 시점에 먼저 보낸다 — plotjuggler 에서 필드를 끌어다 놓으면
곧바로 과거가 보이는 것과 같다.
"""

import collections
import json
import math
import threading
import time

# 브라우저로 내리는 주기 [Hz]. 200Hz tick 4개가 한 프레임이 된다.
STREAM_HZ = 50.0
# 링버퍼 길이 [s]. 50Hz × 30s × 90계열 ≈ 135k 표본 — 로컬에서 문제되지 않는 크기다.
HISTORY_SEC = 30.0

# uint8 미판독 센티넬 (rd_telemetry 의 규약과 같은 값이어야 한다).
U8_UNREAD = 255


class SeriesSpec:
    """계열 하나 = 플롯 한 줄. `fb_current[0]` 처럼 배열은 펼친다."""

    __slots__ = ('key', 'field', 'index', 'kind', 'group')

    def __init__(self, key, field, index, kind, group):
        self.key = key          # 'fb_current[0]'
        self.field = field      # 'fb_current'
        self.index = index      # 0 or None
        self.kind = kind        # 'f' | 'u8' | 'i' | 'b'
        self.group = group      # UI 트리의 묶음 이름

    def as_dict(self):
        return {'key': self.key, 'kind': self.kind, 'group': self.group}


# 필드 이름 → UI 트리 묶음. 접두사로 판정하며, 안 맞으면 'etc' 로 떨어진다.
# **목록에 없다고 계열이 사라지지는 않는다** — 분류가 틀리는 것과 값이 빠지는 것은 다르다.
_GROUPS = (
    ('fb_',        'motor/feedback'),
    ('cmd',        'motor/command'),
    ('dt_',        'timing/delay'),
    ('imu_',       'sensor/imu'),
    ('link_angle', 'sensor/encoder'),
    ('loadcell',   'sensor/loadcell'),
    ('lc',         'diag/lifecycle'),
    ('hs',         'diag/health'),
    ('degraded',   'diag/degraded'),
    ('hw_',        'diag/error'),
    ('rw_err',     'diag/error'),
    ('drop_cnt',   'diag/stream'),
    ('irregular',  'diag/stream'),
    ('late_tick',  'diag/stream'),
    ('ecu_tick',   'timing/clock'),
    ('stamp_',     'timing/clock'),
    ('drift_ppm',  'timing/clock'),
    ('control_state', 'state'),
    ('write_source',  'state'),
    ('auto_mode',     'state'),
    ('motor_mask',    'state'),
    ('ecu_state',     'state'),
    ('goal_id',       'state'),
    ('profile_time',  'state'),
    ('segment_index', 'state'),
)


def _group_of(field):
    for prefix, g in _GROUPS:
        if field.startswith(prefix):
            return g
    return 'etc'


def enumerate_series(msg_cls):
    """메시지 정의에서 계열 목록을 만든다 (§4.2) — 손으로 적지 않는다."""
    out = []
    for field, tname in msg_cls.get_fields_and_field_types().items():
        if field == 'header':
            continue                          # 시간축은 따로 다룬다
        base, count = tname, 1
        if tname.endswith(']'):
            base, _, n = tname[:-1].partition('[')
            count = int(n)
        if base.startswith('sequence<') or '/' in base:
            continue                          # 가변 배열·중첩 메시지는 이 스트림 대상이 아니다
        if base in ('float', 'double'):
            kind = 'f'
        elif base == 'uint8':
            kind = 'u8'
        elif base == 'boolean':
            kind = 'b'
        elif base.startswith(('int', 'uint')):
            kind = 'i'
        else:
            continue                          # string 등 — 플롯 대상이 아니다
        group = _group_of(field)
        if count == 1:
            out.append(SeriesSpec(field, field, None, kind, group))
        else:
            for i in range(count):
                out.append(SeriesSpec('%s[%d]' % (field, i), field, i, kind, group))
    return out


def _clean(v, kind):
    """미판독을 `None` 으로 (§4.3). 여기가 "안 읽음" 과 "정상 0" 의 경계다."""
    if kind == 'f':
        f = float(v)
        return None if (math.isnan(f) or math.isinf(f)) else f
    if kind == 'u8':
        i = int(v)
        return None if i == U8_UNREAD else i
    if kind == 'b':
        return 1 if v else 0
    return int(v)


class TelemetryHub:
    """200Hz 수신 → 50Hz 프레임 → 링버퍼 + SSE 구독자.

    스레드 셋이 만난다: ROS 콜백(add), 50Hz 타이머(flush), HTTP 핸들러(history/subscribe).
    락 하나로 묶는다 — 이 규모에서 잘게 쪼개 봐야 얻는 것보다 틀릴 여지가 크다.
    """

    def __init__(self, specs, stream_hz=STREAM_HZ, history_sec=HISTORY_SEC):
        self.specs = specs
        self._lock = threading.Lock()
        self._n = max(1, int(history_sec * stream_hz))
        self._t = collections.deque(maxlen=self._n)
        # key -> deque of [lo, hi] (또는 None)
        self._ring = {s.key: collections.deque(maxlen=self._n) for s in specs}
        self._win = {}            # 이번 프레임에 모인 것 (key -> [lo, hi])
        self._win_n = 0
        self._subs = []           # [(deque, threading.Event)]
        self.frames = 0
        self.samples = 0

    # ── 200Hz ────────────────────────────────────────────────────────────
    def add(self, msg):
        """RW 트랜잭션 1건 = 메시지 1개. 창(window)에 min/max 로 접는다."""
        vals = {}
        for s in self.specs:
            raw = getattr(msg, s.field)
            if s.index is not None:
                raw = raw[s.index]
            vals[s.key] = _clean(raw, s.kind)
        with self._lock:
            self.samples += 1
            self._win_n += 1
            for k, v in vals.items():
                if v is None:
                    # **미판독이 하나라도 섞이면 그 프레임은 미판독이다.** 읽은 표본만
                    # 골라 평균 내면 "언제부터 안 읽혔는지" 가 지워진다.
                    self._win[k] = None
                    continue
                cur = self._win.get(k, 'x')
                if cur is None:
                    continue                  # 이미 미판독으로 확정
                if cur == 'x':
                    self._win[k] = [v, v]
                else:
                    if v < cur[0]:
                        cur[0] = v
                    if v > cur[1]:
                        cur[1] = v

    # ── 50Hz ─────────────────────────────────────────────────────────────
    def flush(self, t):
        """창을 링버퍼에 넣고 구독자에게 민다. 창이 비어 있으면 아무것도 하지 않는다."""
        with self._lock:
            if self._win_n == 0:
                return
            frame = {}
            for s in self.specs:
                cur = self._win.get(s.key, 'x')
                if cur == 'x' or cur is None:
                    v = None
                elif cur[0] == cur[1]:
                    v = cur[0]                # 창 안에서 안 변했다 — 스칼라로 줄인다
                else:
                    v = cur                   # [min, max] — 스파이크를 남긴다
                frame[s.key] = v
                self._ring[s.key].append(v)
            self._t.append(t)
            self._win = {}
            self._win_n = 0
            self.frames += 1
            subs = list(self._subs)
            payload = None

        for q, ev, keys in subs:
            if payload is None:
                payload = frame
            sel = {k: payload.get(k) for k in keys} if keys else payload
            q.append({'t': t, 'd': sel})
            ev.set()

    # ── HTTP ─────────────────────────────────────────────────────────────
    def field_list(self):
        return [s.as_dict() for s in self.specs]

    def history(self, keys):
        """구독 시점의 과거 (§4). 요청한 계열만 — 90계열 전부 보내면 첫 프레임이 무겁다."""
        with self._lock:
            ts = list(self._t)
            d = {}
            for k in (keys or []):
                if k in self._ring:
                    d[k] = list(self._ring[k])
            return {'t': ts, 'd': d}

    def subscribe(self, keys):
        # maxlen 으로 **느린 구독자가 서버를 붙잡지 못하게** 한다. 브라우저가 멈추면
        # 오래된 프레임부터 버려지고, 그 사실은 drop 카운터로 드러난다.
        q = collections.deque(maxlen=int(STREAM_HZ * 4))
        ev = threading.Event()
        sub = (q, ev, set(keys) if keys else None)
        with self._lock:
            self._subs.append(sub)
        return sub

    def unsubscribe(self, sub):
        with self._lock:
            try:
                self._subs.remove(sub)
            except ValueError:
                pass

    def stats(self):
        with self._lock:
            return {'frames': self.frames, 'samples': self.samples,
                    'subs': len(self._subs), 'history': len(self._t),
                    'series': len(self.specs)}


def sse_pack(event, obj):
    """SSE 한 덩어리. 데이터에 개행이 들어가면 프레임이 깨지므로 JSON 은 한 줄로 낸다."""
    return ('event: %s\ndata: %s\n\n' % (event, json.dumps(obj, allow_nan=False))).encode()
