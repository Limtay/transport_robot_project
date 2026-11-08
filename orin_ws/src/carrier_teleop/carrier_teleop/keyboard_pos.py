#!/usr/bin/env python3
"""
carrier_teleop / keyboard_pos — position_inst 위치 조그
========================================================
`./rec.sh -m ... -a 1,2` 로 띄운 브리지(position_inst)에 /carrier/control/cmd_motor 스트림으로
절대 위치 명령을 넣는다. 지정한 모터(기본 1,2)에 같은 전진량을 준다.

조작
----
  W / S        전진 / 후진 (둘 다 떼면 정지, 속도 0 으로 초기화)
  Ctrl (홀드)  방향 키와 함께 누르는 동안 속도를 올린다. 떼면 그 속도 유지
  Shift (홀드) 속도를 내린다
  Space        정지
  E            arm (LOCKED 면 REARM 후 arm)
  Q / ESC      종료 — 정지 후 arm 해제. 브리지는 그 자리에서 잡는다

안전
----
  * 기본 속도 0 — W 만 눌러서는 움직이지 않는다.
  * 목표가 실제 위치보다 max_lead 넘게 앞서면 더 밀지 않는다 (스트림 중 브리지는 추종
    오차를 검사하지 않는다).
  * 전진량 s 는 처음 arm 한 자리 기준 s_range 안으로 제한된다.
  * pos_feedback 이 feedback_timeout 넘게 안 오면 발행을 멈춘다 → 브리지가 0.1 s 뒤 IDLE
    로 내려 그 자리에서 잡는다.
"""

import math
import os
import threading
import time

import pygame
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mgs_tp_msgs.msg import CmdMotor, PosInstFeedback
from mgs_tp_msgs.srv import ControlConfig

from carrier_teleop.pos_jog import PosJog

STATE = {0: 'INIT', 1: 'IDLE', 2: 'RUNNING', 3: 'STREAM', 4: 'LOCKED'}
FONT_PATHS = ('/usr/share/fonts/truetype/nanum/NanumGothicCoding.ttf',
              '/usr/share/fonts/truetype/nanum/NanumGothic.ttf')


class KeyboardPos(Node):
    def __init__(self):
        super().__init__('carrier_keyboard_pos')
        self.motors = [int(m) for m in self._p('motors', [1, 2])]
        signs = [float(s) for s in self._p('signs', [1.0, 1.0])]
        if len(signs) != len(self.motors) or not all(1 <= m <= 4 for m in self.motors):
            raise ValueError('motors 는 1~4, signs 는 motors 와 같은 길이')
        self.idx = [m - 1 for m in self.motors]
        self.rate = float(self._p('rate', 100.0))
        self.fb_timeout = float(self._p('feedback_timeout', 0.3))
        self.jog = PosJog(signs=signs,
                          v_max=self._p('v_max', 60.0),         # [deg/s] 모터 출력축
                          accel=self._p('accel', 20.0),         # [deg/s^2] Ctrl
                          decel=self._p('decel', 40.0),         # [deg/s^2] Shift
                          max_lead=self._p('max_lead', 20.0),   # [deg]
                          s_range=self._p('s_range', [-1080.0, 1080.0]))   # [deg]

        self.pub = self.create_publisher(CmdMotor, '/carrier/control/cmd_motor', 10)
        self.create_subscription(PosInstFeedback, '/carrier/control/pos_feedback', self._on_fb, 10)
        self.cli = self.create_client(ControlConfig, '/carrier/control/config')

        self._lock = threading.Lock()
        self.fb = None
        self.fb_time = 0.0
        self.notice = ''
        self.want_arm = True          # 기동 직후 한 번 arm 을 시도한다 (속도 0 이라 안 움직인다)
        self.busy = False             # 서비스 호출 중
        self.e_pressed = False

        self.get_logger().info(
            'keyboard_pos: motors=%s signs=%s v_max=%.0f°/s accel=%.0f°/s² max_lead=%.0f° s_range=%s'
            % (self.motors, signs, self.jog.v_max, self.jog.accel, self.jog.max_lead,
               [self.jog.s_min, self.jog.s_max]))

    def _p(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_fb(self, msg):
        with self._lock:
            self.fb = msg
            self.fb_time = time.monotonic()

    # ── 서비스 (pygame 루프를 막지 않게 스레드에서) ──
    def _call(self, op, value=0):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            return False, 'config 서비스 없음 — 브리지 확인'
        req = ControlConfig.Request()
        req.op = op
        req.value = int(value)
        fut = self.cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 3.0:
            time.sleep(0.01)
        if not fut.done():
            return False, '응답 시간 초과'
        r = fut.result()
        return bool(r.ok), r.message

    def _arm_async(self):
        if self.busy:
            return
        self.busy = True

        def work():
            try:
                with self._lock:
                    fb = self.fb
                if fb is not None and fb.control_state == 4:
                    ok, msg = self._call(ControlConfig.Request.OP_REARM)
                    self.notice = 'REARM: ' + msg
                    if not ok:
                        return
                    time.sleep(0.3)
                    with self._lock:
                        fb = self.fb
                cmd = [fb.cmd_abs[i] if math.isfinite(fb.cmd_abs[i]) else fb.fb_abs[i] for i in self.idx]
                if not all(math.isfinite(c) for c in cmd):
                    self.notice = 'arm 불가: 절대 위치 무효'
                    return
                self.jog.anchor(cmd)
                ok, msg = self._call(ControlConfig.Request.OP_SET_STREAM_ARM, 1)
                self.notice = 'arm: ' + msg
                if not ok:
                    self.jog.start = None
            finally:
                self.busy = False
        threading.Thread(target=work, daemon=True).start()

    # ── 루프 ──
    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((560, 400))
        pygame.display.set_caption('carrier keyboard pos (cmd_pos)')
        path = next((p for p in FONT_PATHS if os.path.exists(p)), None)
        font = pygame.font.Font(path, 16) if path else pygame.font.SysFont('monospace', 16)
        clock = pygame.time.Clock()
        running = True
        while running and rclpy.ok():
            dt = clock.tick(self.rate) / 1000.0
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif ev.key == pygame.K_e:
                        self.want_arm = True
                        self.e_pressed = True
            keys = pygame.key.get_pressed()
            mods = pygame.key.get_mods()

            with self._lock:
                fb, age = self.fb, time.monotonic() - self.fb_time
            fresh = fb is not None and age < self.fb_timeout
            # 기동 직후는 IDLE 일 때만 자동 arm. LOCKED 의 REARM 은 E 를 눌렀을 때만 한다.
            if fresh and self.want_arm and not self.busy and (
                    fb.control_state == 1 or (fb.control_state == 4 and self.e_pressed)):
                self.want_arm = False
                self.e_pressed = False
                self._arm_async()

            targets = None
            if fresh and self.jog.start is not None and not self.busy:
                # 쓰는 모터의 PEND(bit i)·FRAME_LOST(bit 4+i) 중에는 목표를 밀지 않는다.
                pend = any((fb.origin_flag >> i) & 1 or (fb.origin_flag >> (4 + i)) & 1 for i in self.idx)
                streaming = fb.control_state == 3
                fb_act = [fb.fb_abs[i] for i in self.idx]
                targets = self.jog.step(
                    dt, keys[pygame.K_w], keys[pygame.K_s], bool(mods & pygame.KMOD_CTRL),
                    bool(mods & pygame.KMOD_SHIFT),
                    keys[pygame.K_SPACE] or pend or not streaming, fb_act)
                if pend:
                    self.jog.flag = '원점 이동/FRAME_LOST — 정지'
                elif not streaming and (keys[pygame.K_w] or keys[pygame.K_s]):
                    self.jog.flag = '브리지가 STREAM 이 아님 — E 로 arm'
                self._publish(fb, targets)
            self._draw(screen, font, fb, fresh, targets)

        self._shutdown()
        pygame.quit()

    def _publish(self, fb, targets):
        m = CmdMotor()
        m.header.stamp = self.get_clock().now().to_msg()
        pos = [fb.cmd_abs[i] if math.isfinite(fb.cmd_abs[i]) else
               (fb.fb_abs[i] if math.isfinite(fb.fb_abs[i]) else 0.0) for i in range(4)]
        for k, i in enumerate(self.idx):
            pos[i] = targets[k]
        m.position = [float(v) for v in pos]
        self.pub.publish(m)

    def _shutdown(self):
        """정지 → 목표를 실제 위치 근처에서 잠깐 유지 → arm 해제. 브리지는 그 자리를 잡는다."""
        if self.jog.start is None:
            return
        self.jog.v = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            with self._lock:
                fb = self.fb
            if fb is not None:
                self._publish(fb, self.jog.targets())
            time.sleep(0.01)
        ok, msg = self._call(ControlConfig.Request.OP_SET_STREAM_ARM, 0)
        self.get_logger().info('종료 — arm 해제: %s' % msg)

    def _draw(self, screen, font, fb, fresh, targets):
        screen.fill((20, 20, 28))
        W, G, Y, R = (210, 210, 220), (90, 210, 120), (230, 200, 100), (240, 110, 90)
        st = STATE.get(fb.control_state, '?') if fb is not None else '-'
        lines = [('CMD_POS  motors %s  signs %s' % (self.motors, self.jog.signs), (200, 200, 210)),
                 ('bridge : %s%s' % (st, '' if fresh else '  (pos_feedback 없음)'),
                  G if st == 'STREAM' and fresh else R),
                 ('speed  : %5.1f °/s  (max %.0f)' % (self.jog.v, self.jog.v_max), Y),
                 ('s      : %+8.1f °  범위 [%.0f, %.0f]' % (self.jog.s, self.jog.s_min, self.jog.s_max), W)]
        if fb is not None:
            for k, i in enumerate(self.idx):
                t = targets[k] if targets else float('nan')
                lines.append(('M%d     : cmd %9.1f  fb %9.1f  차 %+6.1f °'
                              % (i + 1, t, fb.fb_abs[i], t - fb.fb_abs[i]), W))
            lines.append(('linear : x %+8.2f mm  y %+8.2f mm'
                          % (fb.lin_xy[0] * 0.01, fb.lin_xy[1] * 0.02), W))
        lines.append(('알림   : %s' % (self.jog.flag or self.notice or '-'), R if self.jog.flag else W))
        lines.append(('W/S 이동 · Ctrl 가속 · Shift 감속 · Space 정지', (120, 120, 130)))
        lines.append(('E arm/REARM · Q/ESC 종료', (120, 120, 130)))
        y = 12
        for text, c in lines:
            screen.blit(font.render(text, True, c), (14, y))
            y += 26
        pygame.display.flip()


def main(args=None):
    # Ctrl-C 를 rclpy 가 가로채면 컨텍스트가 먼저 죽어 '정지 → arm 해제' 를 못 보낸다.
    # 그래서 기본 핸들러를 끄고 KeyboardInterrupt 로 받아 정상 종료 경로를 탄다.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardPos()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        node.run()
    except KeyboardInterrupt:
        node._shutdown()
        pygame.quit()
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
