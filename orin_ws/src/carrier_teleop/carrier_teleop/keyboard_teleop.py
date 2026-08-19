#!/usr/bin/env python3
"""
carrier_teleop / keyboard_teleop
================================
키보드 멀티키 가상 조이스틱. /carrier_cmd_vel (geometry_msgs/Twist) 발행.

조작
----
  W / S        : 전진 / 후진      (동시입력 시 상쇄)
  A / D        : 좌회전 / 우회전  (W/A 동시 → lin·ang 동시 적용)
  Ctrl (홀드)  : 부스트 모드 — 배율 1.0 → boost_factor(기본 2.0)로 점진 상승,
                 떼면 점진 하강 (boost_ramp_time 동안)
  Space        : 즉시 정지 (출력 0)
  J            : /jeongae 를 open=true 로 jeongae_pulse(기본 1초)간 발행 후 중단
  R            : **무입력 정지(idle-stop) ON/OFF 토글** — 아래 "촬영용 연속 발행" 참조
  Q / ESC      : 종료 (정지 명령 몇 번 발행 후 종료)

안전
----
  * 키를 떼면 해당 축 0 (스프링 복귀). 모든 키 떼면 lin·ang = 0.
  * idle_timeout(기본 5초) 동안 입력이 전혀 없으면 cmd_vel 발행을 멈추고
    다음 키 입력까지 대기 → bridge 의 0.5s 워치독이 모터를 0 으로 유지.
  * 속도는 max_linear / max_angular 로 퍼블리셔에서 클램프 (bridge 는 스케일 안 함).
  * 실제 바퀴는 ECU FSM 이 SYS_STATE_AUTO 일 때만 돈다.

촬영용 연속 발행 (idle-stop OFF)
--------------------------------
`R` 로 끄면 **입력이 없어도 cmd_vel(0,0)을 계속 발행한다.** 촬영 중 토픽이 끊겨
rosbag/뷰어 그래프에 구멍이 생기는 것을 막으려는 것이다 (`idle_stop_enabled:=false`
로 처음부터 꺼진 채 띄울 수도 있다).

⚠ **이때 bridge 의 0.5s 워치독은 영원히 트리거되지 않는다.** 워치독은 "명령이 끊기면
멈춘다" 는 마지막 안전망인데, 조작자가 자리를 떠도 0 명령이 계속 흘러 그 그물이 걷힌
상태가 된다. 발행되는 값 자체는 0 이라 바퀴가 도는 것은 아니지만, **끄는 것은 촬영
구간에서만** 하고 끝나면 다시 켠다. 기본값은 ON 이고, 재시작하면 ON 으로 돌아온다.

기술 노트
---------
  터미널 raw 입력은 key-release / 동시입력 / Ctrl 모디파이어를 못 잡으므로
  pygame 의 키 상태 폴링(get_pressed)을 사용한다. 따라서 작은 포커스 창이 뜨며
  디스플레이가 필요하다. 헤드리스(SSH) 환경에서는 동작하지 않는다.
"""

import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from mgs01_base_msgs.msg import JeonGae
    _HAVE_JEONGAE = True
except Exception:  # 메시지 패키지 미빌드 시 cmd_vel 만으로도 동작
    _HAVE_JEONGAE = False

import pygame


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('carrier_keyboard_teleop')

        # ---- 파라미터 ----
        self.cmd_vel_topic = self._p('cmd_vel_topic', '/carrier_cmd_vel')
        self.jeongae_topic = self._p('jeongae_topic', '/jeongae')
        self.max_linear    = float(self._p('max_linear', 0.3))    # m/s
        self.max_angular   = float(self._p('max_angular', 0.5))   # rad/s
        self.rate          = float(self._p('rate', 50.0))         # Hz (루프 = 발행)
        self.boost_factor  = float(self._p('boost_factor', 2.0))  # Ctrl 시 최대 배율
        self.boost_ramp    = float(self._p('boost_ramp_time', 1.5))  # 1.0→factor 소요(s)
        # 무입력 → 발행중단까지의 시간(s). 촬영 중 토픽이 자주 끊기지 않도록 5초로 둔다
        # (종전 2초. 잠깐 손을 뗀 사이에 끊겨 bag 이 조각났다.)
        self.idle_timeout  = float(self._p('idle_timeout', 5.0))
        # idle-stop 자체를 끌 수 있다(R 키로도 토글). False = 무입력이어도 계속 발행.
        self.idle_stop     = bool(self._p('idle_stop_enabled', True))
        self.jeongae_pulse = float(self._p('jeongae_pulse', 1.0)) # j 펄스 길이(s)
        self.invert_linear  = bool(self._p('invert_linear', False))
        self.invert_angular = bool(self._p('invert_angular', False))

        # ---- 퍼블리셔 ----
        self.pub_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        if _HAVE_JEONGAE:
            self.pub_jeongae = self.create_publisher(JeonGae, self.jeongae_topic, 10)
        else:
            self.pub_jeongae = None
            self.get_logger().warn(
                'mgs01_base_msgs 미발견 → /jeongae(J 키) 비활성. cmd_vel 만 동작.')

        # ---- 상태 ----
        self.boost = 1.0
        self.last_activity = time.monotonic()
        self.jeongae_until = 0.0
        self.was_idle = False

        self.get_logger().info(
            f'KeyboardTeleop 시작: topic={self.cmd_vel_topic} '
            f'max_lin={self.max_linear} max_ang={self.max_angular} '
            f'boost x{self.boost_factor} rate={self.rate}Hz '
            f'idle-stop={"ON(%.1fs)" % self.idle_timeout if self.idle_stop else "OFF(연속 발행)"}')
        if not self.idle_stop:
            self.get_logger().warn(
                'idle-stop 이 꺼진 채 시작한다 — 무입력에도 발행이 계속되어 '
                'bridge 0.5s 워치독이 동작하지 않는다. 촬영이 끝나면 R 로 다시 켤 것.')

    def _p(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ #
    def run(self):
        pygame.init()
        # 높이는 _draw 의 줄 수에 맞춘다 (12 + 24*N). idle-stop 표시 + 키 안내 2줄 = 10줄.
        screen = pygame.display.set_mode((460, 268))
        pygame.display.set_caption('carrier keyboard teleop')
        font = pygame.font.SysFont('monospace', 16)
        clock = pygame.time.Clock()

        running = True
        while running and rclpy.ok():
            dt = clock.tick(self.rate) / 1000.0  # 실제 경과(s)
            now = time.monotonic()

            # --- 이벤트(엣지): 종료, J 펄스 트리거 ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif event.key == pygame.K_j and self.pub_jeongae is not None:
                        self.jeongae_until = now + self.jeongae_pulse
                        self.last_activity = now
                    elif event.key == pygame.K_r:
                        # idle-stop 토글. **토글 자체를 입력으로 친다** — 오래 놀다가 켰을 때
                        # 그 자리에서 바로 idle 로 떨어지면 켠 효과를 확인할 수 없다.
                        self.idle_stop = not self.idle_stop
                        self.last_activity = now
                        if self.idle_stop:
                            self.get_logger().info(
                                f'idle-stop ON — 무입력 {self.idle_timeout:.1f}초 후 발행 중단')
                        else:
                            self.get_logger().warn(
                                'idle-stop OFF — 무입력에도 계속 발행한다 '
                                '(bridge 0.5s 워치독이 동작하지 않는다)')

            # --- 키 상태(홀드): WASD + Ctrl + Space ---
            keys = pygame.key.get_pressed()
            mods = pygame.key.get_mods()
            w = keys[pygame.K_w]; s = keys[pygame.K_s]
            a = keys[pygame.K_a]; d = keys[pygame.K_d]
            ctrl  = bool(mods & pygame.KMOD_CTRL)
            space = keys[pygame.K_SPACE]

            active = bool(w or a or s or d or ctrl or space)
            if active:
                self.last_activity = now
            # idle-stop 이 꺼져 있으면 아무리 오래 놀아도 idle 로 보지 않는다 → 계속 발행.
            idle = self.idle_stop and (now - self.last_activity) > self.idle_timeout

            # --- 부스트 램프 (Ctrl 홀드 시 상승, 떼면 하강) ---
            ramp_step = (self.boost_factor - 1.0) * (dt / max(self.boost_ramp, 1e-3))
            if ctrl and not idle:
                self.boost = min(self.boost_factor, self.boost + ramp_step)
            else:
                self.boost = max(1.0, self.boost - ramp_step)

            # --- 명령 계산 (W/A 동시입력 → lin·ang 동시) ---
            fwd  = (1 if w else 0) - (1 if s else 0)   # [-1,1]
            turn = (1 if a else 0) - (1 if d else 0)   # +좌회전 / -우회전
            lin = fwd  * self.max_linear  * self.boost
            ang = turn * self.max_angular * self.boost
            if self.invert_linear:  lin = -lin
            if self.invert_angular: ang = -ang
            if space or idle:        # Space=즉시정지, idle=정지
                lin = 0.0
                ang = 0.0
            if idle:
                self.boost = 1.0     # 대기 진입 시 부스트 리셋

            # --- cmd_vel 발행 (idle 이면 발행 중단) ---
            if not idle:
                msg = Twist()
                msg.linear.x  = float(lin)
                msg.angular.z = float(ang)
                self.pub_vel.publish(msg)
            elif not self.was_idle:
                self.get_logger().info(
                    f'입력 {self.idle_timeout:.1f}초 이상 없음 → 발행 중단 (키 입력 대기). '
                    'R 로 끄면 무입력에도 계속 발행한다.')
            self.was_idle = idle

            # --- jeongae 1초 펄스 ---
            sending_jeongae = False
            if self.pub_jeongae is not None and now < self.jeongae_until:
                jg = JeonGae()
                jg.open = True
                self.pub_jeongae.publish(jg)
                sending_jeongae = True

            self._draw(screen, font, lin, ang, ctrl, space, idle,
                       sending_jeongae, max(0.0, self.jeongae_until - now),
                       w, a, s, d)

        # --- 종료: 안전 정지 명령 몇 회 발행 ---
        for _ in range(5):
            self.pub_vel.publish(Twist())
            time.sleep(0.02)
        pygame.quit()

    # ------------------------------------------------------------------ #
    def _draw(self, screen, font, lin, ang, ctrl, space, idle,
              jeongae_on, jeongae_left, w, a, s, d):
        screen.fill((20, 20, 28))
        state = 'IDLE (paused)' if idle else ('STOP' if space else 'ACTIVE')
        color = (120, 120, 130) if idle else ((230, 90, 90) if space else (90, 210, 120))
        # idle-stop 은 **꺼진 것이 예외 상태**다 — 워치독이 걷힌 채로 두고 잊는 일이
        # 없도록 꺼졌을 때 주황으로 눈에 띄게 둔다.
        if self.idle_stop:
            idle_txt, idle_col = f'ON  ({self.idle_timeout:.1f}s)', (150, 170, 200)
        else:
            idle_txt, idle_col = 'OFF (연속 발행 · 워치독 없음)', (240, 160, 60)
        held = ''.join(k for k, on in
                       (('W', w), ('A', a), ('S', s), ('D', d)) if on) or '-'
        lines = [
            ('CARRIER KEYBOARD TELEOP', (200, 200, 210)),
            (f'state : {state}', color),
            (f'lin   : {lin:+.2f} m/s', (210, 210, 220)),
            (f'ang   : {ang:+.2f} rad/s', (210, 210, 220)),
            (f'boost : x{self.boost:.2f} {"(CTRL)" if ctrl else ""}', (210, 200, 120)),
            (f'jeongae: {"sending %.1fs" % jeongae_left if jeongae_on else "-"}',
             (120, 180, 230)),
            (f'keys  : {held}', (180, 180, 190)),
            (f'idle-stop: {idle_txt}', idle_col),
            ('WASD move | Ctrl boost | Space stop | J jeongae',
             (110, 110, 120)),
            ('R idle-stop toggle | Q/ESC quit', (110, 110, 120)),
        ]
        y = 12
        for text, c in lines:
            screen.blit(font.render(text, True, c), (14, y))
            y += 24
        pygame.display.flip()


def main(args=None):
    # SDL 디스플레이 확인 (헤드리스 조기 진단)
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        print('[carrier_teleop] 경고: DISPLAY 미설정 — pygame 창을 열 수 없습니다. '
              '디스플레이 있는 머신에서 실행하세요.')

    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
