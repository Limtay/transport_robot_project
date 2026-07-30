"""control 모드 웹 조작 UI — 브리지의 run/stop + 슬라이더 (redesign/01 §6.1.3).

웹이 하는 일은 여섯 가지뿐이다. 그 이상은 넣지 않는다 (01 §6.1.3, 사용자 확정):

    ① 현재 자세 확인      ControlFeedback.fb_position 실시간 표시
    ② (선택) 원점 잡기     SET_ORIGIN 펄스 — run 전에만
    ③ 시작 위치 맞추기     슬라이더를 원하는 값에 놓는다 (아직 나가지 않는다)
    ④ run                arm=on → 슬라이더 값이 STREAM 으로 나가기 시작
    ⑤ 조작                슬라이더를 움직이면 모터가 따라온다
    ⑥ stop               arm=off → IDLE, 안전값 유지

**진입 시 변위는 브리지도 웹도 검사하지 않는다** (01 §6.1.3 기각 결정). ①이 실시간으로
보이고 ③④가 분리되어 있으므로 조작자가 이미 판단할 수 있다. 조작자 책임이다.


## 이 노드가 왜 슬라이더 값을 대신 발행하는가

브라우저가 `cmd_motor` 를 직접 200Hz 로 쏘게 만들 수는 없다. 그래서 **브라우저는 setpoint 만
바꾸고, 발행은 이 노드가 고정 주기로 한다.** 브리지의 스테일 timeout 은 0.1s 라, 조작자가
슬라이더를 가만히 두면 곧바로 IDLE 로 떨어져 버리기 때문이다.

그 대가로 **데드맨이 필요하다.** 탭을 닫거나 브라우저가 멈춰도 이 노드는 마지막 값을 계속
발행할 수 있고, 그러면 조작자가 손을 뗐는데 모터는 명령을 받는 상태가 된다. 그래서
`client_timeout` 동안 브라우저 접촉이 없으면 **발행을 멈추고 arm 을 내린다.**
"조작자가 손을 뗀 것을 시스템이 기억하면 안 된다"(01 §6.1.3)는 규칙을 웹 쪽에서 지키는 장치다.


## 단위는 브리지가 정한다

슬라이더의 단위는 `auto_mode` 가, DIRECT 면 **모터별 `ctr_mode`** 가 정한다 (01 §6.1.2).
웹이 임의로 고르지 않고 `ControlConfig(OP_GET_STATUS)` 로 물어서 그대로 따른다.
응답은 **정형 JSON** 이다 (04 §4 C3) — 종전에는 한국어 문장을 정규식으로 뜯어 썼고,
그래서 웹이 알 수 있는 것이 mask·ctr_mode·auto_mode 넷뿐이었다 (06 §9.1).
`CmdMotor.ctr_mode` 로 지정하는 길은 없다 — 그 필드는 예약이며 브리지가 읽지 않는다
(CmdMotor.msg 참조). 모드 변경은 `control_cli config` (IDLE 전용)의 몫이다.
"""

import json
import os
import threading
import urllib.parse
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mgs_tp_msgs.msg import CmdMotor, ControlFeedback
from mgs_tp_msgs.srv import ControlConfig

from control_web.supervisor import BridgeSupervisor
from control_web import telemetry
from control_web.profile_run import ProfileRunner
from control_cli import record

# ControlFeedback 의 정수 enum (토픽은 아직 숫자다 — 04 §4 는 서비스 응답 스키마다)
STATE_NAMES = {0: 'INIT', 1: 'IDLE', 2: 'RUNNING', 3: 'STREAM', 4: 'LOCKED'}
WRITE_SOURCE_NAMES = {0: 'NONE', 1: 'CMD_VEL', 2: 'PROFILE', 3: 'STREAM'}

# 브리지가 없을 때의 상태. **모르는 값을 그럴듯한 기본값으로 채우지 않는다** —
# `control_state: '?'` 가 "INIT" 이나 "IDLE" 이면 UI 가 조작 가능한 것처럼 보인다.
UNKNOWN_STATUS = {
    'control_state': '?', 'motor_mask': 0,
    'ctr_mode': ['current'] * 4, 'auto_mode': 'current',
    'bridge_mode': '?', 'ecu_mode': '?', 'ecu_sys_state': 0,
    'write_span': '-', 'read_preset': '-', 'active_motors': [],
    'safe_stop': False, 'safe_stop_detail': None,
    'stamp_valid': False, 'lock_reason': None,
}

# 슬라이더가 어떤 필드로 나가는가. (단위, CmdMotor 필드, 기본 범위)
# 범위는 **조작 편의를 위한 UI 값이지 안전 한계가 아니다** — 실제 클램프는 브리지가 건다
# (전류는 cmd_current_max, 재생 중에는 프로파일 실효 한계). 여기 숫자를 넓혀도 브리지를
# 넘어서지 못하고, 좁혀도 브리지가 보장하는 것을 대신하지 못한다.
UNIT_TABLE = {
    'current':  ('A',   'current',  -20.0,  20.0, 0.1),
    'velocity': ('RPM', 'velocity', -300.0, 300.0, 1.0),
    'position': ('deg', 'position', -180.0, 180.0, 0.5),
}



def unit_for(auto_mode, ctr_mode):
    """이 모터의 슬라이더가 무슨 단위인지. 브리지 규칙(01 §6.1.2)을 그대로 옮긴 것.

    인자는 **JSON 의 문자열 enum** 이다 (04 §4). 숫자를 다시 정의하지 않는 이유는
    그 표가 브리지와 웹 두 곳에 생기면 반드시 갈라지기 때문이다.
    """
    if auto_mode in ('position', 'velocity', 'current'):
        return auto_mode                       # 비-DIRECT 는 auto_mode 가 곧 단위다
    if auto_mode == 'direct':
        # DIRECT 에서만 모터마다 다를 수 있다.
        if ctr_mode in ('position', 'velocity', 'current'):
            return ctr_mode
        if ctr_mode == 'current_brake':
            return 'current'
        return None      # estop / set_origin — 조작 대상이 아니다
    return None          # kinematic / control — control 모드에서 기동이 막힌다


class ControlWeb(Node):
    def __init__(self):
        super().__init__('control_web')

        self.port = self.declare_parameter('port', 8080).value
        self.bind = self.declare_parameter('bind', '0.0.0.0').value
        self.publish_rate = float(self.declare_parameter('publish_rate', 50.0).value)
        # 데드맨. 브리지의 스테일(0.1s)보다 넉넉해야 한다 — 네트워크 지연으로 arm 이
        # 툭툭 끊기면 조작이 불가능해진다. 그렇다고 길면 손 뗀 뒤 명령이 오래 남는다.
        self.client_timeout = float(self.declare_parameter('client_timeout', 1.0).value)
        # bag 저장 위치. **상대경로면 웹의 CWD 기준**인데, 웹은 `ros2 run` 이 어디서 뜨느냐에
        # 따라 CWD 가 달라진다 — CLI 는 보통 레포 루트에서 돌므로 그대로 두면 웹으로 돌린
        # 런만 다른 곳에 쌓이고, 그 사실은 분석할 때가 되어서야 드러난다.
        # 기동 로그에 **해석된 절대경로**를 남겨 어디에 쌓이는지 숨지 않게 한다.
        self.bag_dir = os.path.abspath(
            self.declare_parameter('bag_dir', record.DEFAULT_BAG_BASE).value)

        self._lock = threading.Lock()
        self._fb = None                       # 최신 ControlFeedback
        self._fb_stamp = 0.0
        self._setpoint = [0.0, 0.0, 0.0, 0.0]
        self._setpoint_touched = [False] * 4  # 조작자가 아직 안 만진 슬라이더는 자세를 따라간다
        self._last_client = 0.0
        self._armed = False                   # 우리가 arm 을 켰다고 믿는 상태
        # 04 §4 스키마의 부분집합을 기본값으로 — 첫 응답 전에도 키가 있어야 렌더가 안 깨진다.
        self._status = dict(UNKNOWN_STATUS)
        self._notice = ''                     # 조작자에게 보여 줄 마지막 사유

        # 07 §1 — 웹이 브리지를 띄우는 주체다. **웹은 브리지 없이도 살아야 한다** —
        # 종전에는 브리지가 있다고 가정해서, UI 가 "브리지가 없다" 와 "대답이 늦다" 를
        # 구분하지 못했다. bridge_proc 을 1급 상태로 둔다.
        self.sup = BridgeSupervisor(logger=self.get_logger())

        # 07 §4 — 모니터링 데이터 층. 계열 목록은 메시지 정의에서 뽑는다 (손으로 안 적는다).
        self.hub = telemetry.TelemetryHub(telemetry.enumerate_series(ControlFeedback))
        # 07 §2 Tab2 — 프로파일 재생. 기록은 control_cli/record.py 를 공유한다.
        # diag 토픽 기록 여부는 **브리지 기동 파라미터**에서 온다 (05 §7) — 기동 패널이
        # comm_diag_enable 을 그대로 통과시키므로, 그 값을 아는 supervisor 에게 묻는다.
        self.runner = ProfileRunner(self, bag_base=self.bag_dir,
                                    diag_enabled=self._diag_enabled)
        self.create_timer(1.0 / telemetry.STREAM_HZ, self._flush_stream)

        # ⚠ QoS 는 **브리지에 맞춘다** (둘 다 기본 RELIABLE). 여기를 BEST_EFFORT 로 두면
        # 브리지의 RELIABLE 구독과 호환되지 않아 rclpy 가
        # "requesting incompatible QoS. No messages will be sent to it" 만 남기고
        # **슬라이더가 조용히 안 먹는다.** 실제로 처음 그렇게 짰다가 기동 로그에서 잡혔다.
        self.pub_cmd = self.create_publisher(CmdMotor, '/carrier/control/cmd_motor', 10)
        self.create_subscription(ControlFeedback, '/carrier/control/feedback', self._on_fb, 10)
        self.cli_config = self.create_client(ControlConfig, '/carrier/control/config')
        self.cli_command = None       # Tab3 에서 처음 쓸 때 만든다 (07 §2)

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.create_timer(0.5, self._poll_status)

        self.get_logger().info(
            'control_web — http://%s:%d  (발행 %.0fHz, 데드맨 %.1fs)'
            % (self.bind, self.port, self.publish_rate, self.client_timeout))
        self.get_logger().info('bag 저장 위치: %s  (-p bag_dir:=… 로 변경)' % self.bag_dir)

    # ── ROS ────────────────────────────────────────────────────────────────
    def _on_fb(self, msg):
        self.hub.add(msg)                     # 07 §4 — 200Hz 전부가 창으로 들어간다
        with self._lock:
            self._fb = msg
            self._fb_stamp = time.time()
            # 아직 안 만진 슬라이더는 현재 자세를 따라간다 — 그래야 run 을 누른 순간
            # 변위가 0 이다. **만진 뒤에는 따라가지 않는다**: 조작자가 시작 위치를 미리
            # 맞춰 두는 것이 ③ 의 목적이고, 브리지가 슬라이더를 끌고 가면 안 된다
            # (01 §6.1.2 "재시드 대상은 wire 로 나가는 값이지 슬라이더가 아니다").
            for i in range(4):
                if not self._setpoint_touched[i]:
                    u = unit_for(self._status.get('auto_mode'), self._status['ctr_mode'][i])
                    self._setpoint[i] = float(msg.fb_position[i]) if u == 'position' else 0.0

    def _diag_enabled(self):
        """이 웹이 띄운 브리지가 `comm_diag_enable:=true` 인가 (05 §7 기록 토픽 판정).

        ROS 파라미터는 문자열로도 오고 불리언으로도 온다 — 기동 패널이 값을 그대로
        통과시키기 때문이다. 양쪽을 다 받는다. 브리지가 없으면 False (기록할 토픽이 없다).
        """
        v = self.sup.snapshot()['params'].get('comm_diag_enable', False)
        if isinstance(v, str):
            return v.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(v)

    def _flush_stream(self):
        """50Hz — 창을 프레임으로 굳혀 링버퍼·구독자에게 민다 (07 §4.1).

        시간축은 **브리지가 준 header.stamp** 다 (ECU 취득 시각을 Orin 축으로 변환한 값).
        웹의 수신 시각을 쓰면 200Hz 스트림의 지터가 그대로 x축에 실린다. `stamp_valid` 가
        false 면 브리지 쪽에서 이미 Orin 수신 시각 fallback 이며, 그 사실은 계열로도 나간다.
        """
        with self._lock:
            fb = self._fb
        if fb is None:
            return
        t = fb.header.stamp.sec + fb.header.stamp.nanosec * 1e-9
        self.hub.flush(t)

    def _poll_status(self):
        if not self.cli_config.service_is_ready():
            # 브리지가 없다 → **이전 브리지의 값을 남기지 않는다.** 남기면 모드·마스크가
            # 죽은 값인데 살아 있는 것처럼 보이고, 그 위에서 조작 UI 가 활성화된다.
            # 판정을 sup 상태가 아니라 **서비스 존재**로 하는 이유: 터미널에서 띄운
            # 브리지에 웹이 붙는 경로도 계속 유효해야 한다.
            with self._lock:
                if self._status.get('control_state') != '?':
                    self._status = dict(UNKNOWN_STATUS)
            return
        req = ControlConfig.Request()
        req.op = ControlConfig.Request.OP_GET_STATUS
        fut = self.cli_config.call_async(req)
        fut.add_done_callback(self._on_status)

    def _on_status(self, fut):
        """정형 JSON 을 **그대로** 보관한다 (04 §4).

        키를 골라 담지 않는 이유: 스키마가 계약이므로 웹이 부분집합을 다시 정의하면
        그 부분집합이 곧 두 번째 계약이 된다. 필요한 것을 그때그때 꺼내 쓴다.
        """
        try:
            d = json.loads(fut.result().message)
        except Exception:
            return                       # 구 브리지(문장 응답)와 섞여 돌 때는 조용히 무시
        with self._lock:
            self._status = d

    def _tick(self):
        """발행 주기. **arm 이 켜져 있고 브라우저가 살아 있을 때만** 내보낸다."""
        with self._lock:
            if not self._armed:
                return
            stale = (time.time() - self._last_client) > self.client_timeout
            if stale:
                self._notice = ('브라우저 접촉 끊김 (%.1fs) — stop 처리했다. '
                                '재개하려면 run 을 다시 누를 것' % self.client_timeout)
                self._armed = False
                self.get_logger().warn(self._notice)
                # 락 밖에서 서비스 호출 — 아래에서 처리
                need_disarm = True
            else:
                need_disarm = False
            snapshot = list(self._setpoint)
            status = dict(self._status)
            fb = self._fb

        if need_disarm:
            self._call_config(ControlConfig.Request.OP_SET_STREAM_ARM, value=0)
            return

        msg = CmdMotor()
        msg.header.stamp = self.get_clock().now().to_msg()
        mask = status.get('motor_mask', 0)
        for i in range(4):
            u = unit_for(status.get('auto_mode'), status['ctr_mode'][i])
            if u is None:
                continue
            active = bool((mask >> i) & 1)
            if active:
                v = snapshot[i]
            else:
                # 마스크 밖 모터는 ECU 가 무시하지만, position 이면 실측을 넣어 둔다 —
                # 마스크가 바뀌는 순간 튀지 않게 (브리지 IDLE 재시드와 같은 취지).
                v = float(fb.fb_position[i]) if (u == 'position' and fb) else 0.0
            getattr(msg, UNIT_TABLE[u][1])[i] = v
        self.pub_cmd.publish(msg)

    def _call_config(self, op, value=0, motors=None, timeout=3.0):
        """설정 서비스 1회 호출. HTTP 스레드에서 부르므로 future 를 폴링한다."""
        if not self.cli_config.wait_for_service(timeout_sec=timeout):
            return False, '/carrier/control/config 서비스 없음 — 브리지 실행 확인'
        req = ControlConfig.Request()
        req.op = op
        req.value = int(value)
        if motors:
            req.motors = [int(m) for m in motors]
        fut = self.cli_config.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            return False, '응답 시간 초과'
        res = fut.result()
        return bool(res.ok), res.message

    # ── HTTP 핸들러가 부르는 것들 ───────────────────────────────────────────
    def touch_client(self):
        with self._lock:
            self._last_client = time.time()

    def snapshot(self):
        with self._lock:
            fb, status = self._fb, dict(self._status)
            sp, touched = list(self._setpoint), list(self._setpoint_touched)
            armed, notice = self._armed, self._notice
            fb_age = time.time() - self._fb_stamp if self._fb_stamp else None

        units, ranges = [], []
        for i in range(4):
            u = unit_for(status.get('auto_mode'), status['ctr_mode'][i])
            units.append(u)
            ranges.append(UNIT_TABLE[u][2:] if u else None)

        out = {
            'connected': fb is not None and fb_age is not None and fb_age < 1.0,
            'bridge': self.sup.snapshot(),        # 07 §1.1 — 1급 상태
            'armed': armed,
            'notice': notice,
            'auto_mode': status.get('auto_mode'),
            'ctr_mode': status['ctr_mode'],
            'motor_mask': status.get('motor_mask', 0),
            'bridge_state': status.get('control_state', '?'),
            # 04 §4 가 열어 준 것들 — 종전 문장 응답으로는 알 수 없던 값이다.
            'bridge_mode': status.get('bridge_mode'),
            'ecu_mode': status.get('ecu_mode'),
            'write_span': status.get('write_span'),
            'read_preset': status.get('read_preset'),
            'safe_stop': status.get('safe_stop'),
            'safe_stop_detail': status.get('safe_stop_detail'),
            'stamp_valid': status.get('stamp_valid'),
            'lock_reason': status.get('lock_reason'),
            'setpoint': sp,
            'touched': touched,
            'units': units,
            'ranges': ranges,
            'unit_labels': [UNIT_TABLE[u][0] if u else None for u in units],
        }
        if fb is not None:
            out.update({
                'fb_position': [round(float(v), 2) for v in fb.fb_position],
                'fb_velocity': [round(float(v), 1) for v in fb.fb_velocity],
                'fb_current': [round(float(v), 2) for v in fb.fb_current],
                'cmd': [round(float(v), 2) for v in fb.cmd],
                'control_state': STATE_NAMES.get(fb.control_state, '?'),
                'write_source': WRITE_SOURCE_NAMES.get(fb.write_source, '?'),
                'ecu_state': int(fb.ecu_state),
                'hw_error': int(fb.hw_error),
            })
        return out

    def set_setpoint(self, idx, value):
        with self._lock:
            if not 0 <= idx < 4:
                return False, '모터 번호 범위 밖'
            self._setpoint[idx] = float(value)
            self._setpoint_touched[idx] = True
            self._last_client = time.time()
        return True, 'ok'

    def do_arm(self, on):
        if on:
            # arm on 직전에 접촉 시각을 갱신한다 — 안 하면 첫 tick 이 곧바로 데드맨에 걸린다.
            with self._lock:
                self._last_client = time.time()
        ok, msg = self._call_config(ControlConfig.Request.OP_SET_STREAM_ARM, value=1 if on else 0)
        with self._lock:
            # **실패해도 arm off 는 내부적으로 반영한다** — 감속 방향이다 (01 §6.2).
            if ok or not on:
                self._armed = bool(on)
            self._notice = msg
        return ok, msg

    def do_origin(self):
        ok, msg = self._call_config(ControlConfig.Request.OP_SET_ORIGIN)
        with self._lock:
            self._notice = msg
            if ok:
                # 원점이 바뀌면 현재 자세가 0 이 된다 — 안 만진 슬라이더는 따라가게 둔다.
                for i in range(4):
                    if not self._setpoint_touched[i]:
                        self._setpoint[i] = 0.0
        return ok, msg

    # ── 07 §1 감독자 ──────────────────────────────────────────────────────
    def bridge_start(self, params):
        ok, msg = self.sup.start(params)
        with self._lock:
            self._notice = msg
            # 새 브리지의 상태가 올 때까지 이전 값을 남기지 않는다.
            self._status = dict(UNKNOWN_STATUS)
            self._armed = False
            self._setpoint_touched = [False] * 4
        return ok, msg

    def bridge_stop(self):
        """07 §1.3 — **arm 을 먼저 내리고** 프로세스를 종료한다.

        순서가 뒤바뀌면 STREAM 중에 프로세스가 사라져 마지막 명령이 ECU 에 남는다.
        AK 모터의 CAN timeout(200ms)이 받아 주지만, 안전망은 마지막 방어선이지
        정상 경로가 아니다.
        """
        notes = []
        if self._armed:
            ok_arm, msg_arm = self.do_arm(False)
            notes.append('arm off: ' + ('ok' if ok_arm else msg_arm))
            time.sleep(0.2)          # STREAM 에서 빠져나올 시간 (브리지 스테일 0.1s)
        ok, msg = self.sup.stop()
        notes.append(msg)
        with self._lock:
            self._notice = ' / '.join(notes)
            self._armed = False
            self._status = dict(UNKNOWN_STATUS)
        return ok, self._notice

    # ── 07 §2 Tab3 — 레지스터 맵 / 커맨드 슬롯 ────────────────────────────
    def read_registers(self, target_id=0):
        """섀도 덤프 + **신선도**. `fresh` 없이 그리면 한 번도 읽은 적 없는 자리의 0 이
        방금 읽은 값처럼 보인다 (OP_GET_REGISTERS 주석 참조)."""
        ok, msg = self._call_config(ControlConfig.Request.OP_GET_REGISTERS, value=target_id)
        if not ok:
            return False, {'error': msg}
        try:
            return True, json.loads(msg)
        except ValueError:
            return False, {'error': '레지스터 응답이 JSON 이 아니다: ' + msg[:120]}

    def command_set(self, req):
        """CommandSet 그대로 전달한다. **여기서 게이트를 흉내내지 않는다** — 판정은
        브리지의 카탈로그가 하고(B6), 웹이 같은 조건을 다시 적으면 갈라진다."""
        from mgs_tp_msgs.srv import CommandSet
        if self.cli_command is None:
            self.cli_command = self.create_client(CommandSet, '/carrier/command_set')
        if not self.cli_command.wait_for_service(timeout_sec=3.0):
            return False, '/carrier/command_set 서비스 없음 — 브리지 실행 확인'
        m = CommandSet.Request()
        m.slot      = int(req.get('slot', 255))
        m.action    = int(req.get('action', 1))
        m.target_id = int(req.get('target_id', 225))
        m.cmd       = int(req.get('cmd', 0))
        m.args      = [int(v) & 0xFF for v in (req.get('args') or [])]
        m.start_addr = int(req.get('start_addr', 0))
        m.data_len   = int(req.get('data_len', 0))
        m.data       = [int(v) & 0xFF for v in (req.get('data') or [])]
        m.duration   = int(req.get('duration', 1))
        fut = self.cli_command.call_async(m)
        deadline = time.time() + 4.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            return False, '응답 시간 초과'
        r = fut.result()
        with self._lock:
            self._notice = r.message
        return bool(r.accepted), r.message

    def release_slider(self, idx):
        """슬라이더를 '조작자 소유' 에서 놓아 준다 — 다시 자세를 따라간다."""
        with self._lock:
            if 0 <= idx < 4:
                self._setpoint_touched[idx] = False
        return True, 'ok'


def make_handler(node, www_dir):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass                                  # 200Hz 폴링 로그로 터미널을 덮지 않는다

        def _json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self, keys):
            """SSE (07 §4.1). WebSocket 이 아닌 이유: 단방향이고 표준 라이브러리로 된다.

            `ThreadingHTTPServer` 라서 이 연결이 다른 요청을 막지 않는다 — 단일 스레드
            서버였다면 스트림을 여는 순간 UI 전체가 굳는다.
            """
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()

            sub = node.hub.subscribe(keys)
            q, ev, _ = sub
            try:
                # 구독 시점의 **과거부터** 보낸다 — 체크한 순간부터 그리면 방금 무슨 일이
                # 있었는지 볼 수 없다 (07 §4 ④).
                self.wfile.write(telemetry.sse_pack('history', node.hub.history(keys)))
                self.wfile.flush()
                while True:
                    if not q:
                        # 타임아웃을 두어 **끊긴 연결을 알아챈다** — 브라우저가 탭을 닫아도
                        # 소켓 쓰기를 시도해야 예외가 나고 구독이 정리된다.
                        if not ev.wait(timeout=5.0):
                            self.wfile.write(b': keepalive\n\n')
                            self.wfile.flush()
                            continue
                    ev.clear()
                    while q:
                        self.wfile.write(telemetry.sse_pack('f', q.popleft()))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ValueError, OSError):
                pass                                  # 탭을 닫았다 — 정상 경로다
            finally:
                node.hub.unsubscribe(sub)

        def do_GET(self):
            if self.path.startswith('/api/state'):
                node.touch_client()
                return self._json(200, node.snapshot())
            if self.path.startswith('/api/profile/status'):
                return self._json(200, node.runner.status())
            if self.path.startswith('/api/registers'):
                ok, d = node.read_registers()
                return self._json(200 if ok else 503, d)
            if self.path.startswith('/api/fields'):
                return self._json(200, {'fields': node.hub.field_list(),
                                        'stream_hz': telemetry.STREAM_HZ,
                                        'stats': node.hub.stats()})
            if self.path.startswith('/api/stream'):
                _, _, qs = self.path.partition('?')
                params = urllib.parse.parse_qs(qs)
                raw = params.get('fields', [''])[0]
                keys = [k for k in raw.split(',') if k]
                return self._stream(keys)
            return self._static('index.html' if self.path == '/'
                                else self.path.lstrip('/').partition('?')[0])

        def _static(self, rel):
            """www/ 아래 정적 파일. **경로 탈출을 막는다** — 웹 서버가 로봇 위에서 돌고,
            여기로 임의 파일을 읽히면 그 자체가 구멍이다.

            ⚠ 검사는 `normpath`(텍스트 정규화)로 한다. 종전에는 `realpath` 로 했는데,
              그건 **심링크를 따라가서** `--symlink-install` 로 깐 트리를 전부 404 로 막았다:

                  install/.../www/index.html  --(symlink)-->  src/control_web/www/index.html

              realpath 가 후자를 돌려주고 그건 root 밖이라 가드가 막는다. 즉 `colcon build
              --symlink-install` 로 빌드하면 **API 는 되는데 UI 페이지만 안 뜬다.**

              막아야 하는 것은 클라이언트가 보낸 `../` 이고, 그건 normpath 가 정확히 없앤다
              (`www/../../../etc/passwd` → root 밖 → 거부). www/ 안의 심링크는 우리가
              설치한 것이므로 따라가도 된다 — 그게 symlink-install 의 요점이다.
            """
            root = os.path.realpath(www_dir)
            path = os.path.normpath(os.path.join(root, rel))
            if not path.startswith(root + os.sep) or not os.path.isfile(path):
                return self._json(404, {'error': 'not found'})
            ctype = {'.html': 'text/html; charset=utf-8',
                     '.js':   'application/javascript; charset=utf-8',
                     '.css':  'text/css; charset=utf-8'}.get(os.path.splitext(path)[1],
                                                             'application/octet-stream')
            with open(path, 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            n = int(self.headers.get('Content-Length') or 0)
            try:
                req = json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                return self._json(400, {'ok': False, 'message': '잘못된 JSON'})

            if self.path == '/api/setpoint':
                ok, msg = node.set_setpoint(int(req.get('motor', 1)) - 1, float(req.get('value', 0.0)))
            elif self.path == '/api/release':
                ok, msg = node.release_slider(int(req.get('motor', 1)) - 1)
            elif self.path == '/api/arm':
                ok, msg = node.do_arm(bool(req.get('on')))
            elif self.path == '/api/origin':
                ok, msg = node.do_origin()
            elif self.path == '/api/bridge/start':
                # 파라미터를 웹이 검증하지 않는다 — 브리지 기동 게이트가 정본이다
                # (06 §9.10). 두 벌이 되면 갈라진다. 실패 사유는 로그 패널에 뜬다.
                params = req.get('params') or {}
                if not isinstance(params, dict):
                    return self._json(400, {'ok': False, 'message': 'params 는 객체여야 한다'})
                ok, msg = node.bridge_start(params)
            elif self.path == '/api/bridge/stop':
                ok, msg = node.bridge_stop()
            elif self.path == '/api/command':
                ok, msg = node.command_set(req)
            elif self.path == '/api/profile/run':
                ok, msg = node.runner.start(req.get('spec') or {},
                                            do_record=bool(req.get('record', True)))
            elif self.path == '/api/profile/abort':
                ok, msg = node.runner.abort()
            elif self.path == '/api/profile/preview':
                # YAML 만 만들어 돌려준다 — 보내기 전에 무엇이 나가는지 볼 수 있어야 한다.
                from control_web.profile_run import build_yaml
                text, err = build_yaml(req.get('spec') or {})
                return self._json(200, {'ok': err is None, 'message': err or '',
                                        'yaml': text or ''})
            else:
                return self._json(404, {'ok': False, 'message': 'not found'})
            self._json(200, {'ok': ok, 'message': msg})

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = ControlWeb()

    try:
        www = os.path.join(get_package_share_directory('control_web'), 'www')
    except Exception:
        www = os.path.join(os.path.dirname(__file__), '..', 'www')

    httpd = ThreadingHTTPServer((node.bind, node.port), make_handler(node, www))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 arm 을 내린다 — 노드가 죽었는데 브리지가 STREAM 에 남아 있으면 안 된다.
        # (브리지도 0.1s 스테일로 스스로 내려오지만, 의도를 명시적으로 남긴다.)
        try:
            node.do_arm(False)
        except Exception:
            pass
        # 웹이 띄운 브리지는 웹이 내린다 — 고아를 남기면 다음 기동이 막힌다 (07 §1.4).
        try:
            if node.sup.snapshot()['pid']:
                node.sup.stop()
        except Exception:
            pass
        httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
