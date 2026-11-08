"""위치 조그 판단 로직 — ROS·pygame 없이 테스트한다 (keyboard_pos 가 쓴다).

목표 위치는 모터마다 `start[i] + sign[i] * s` 다. `s` [deg] 는 모든 활성 모터가 공유하는
"전진량" 이고, 키 입력은 `s` 를 속도 `v` 로 움직일 뿐이다 — 앞뒤 트랙에 같은 명령이 간다.

  W / S         방향. 둘 다 떼면 v = 0 (다음에 눌러도 0 에서 시작 — 급출발 없음)
  Ctrl (홀드)   방향 키를 누르는 동안 v 를 accel 로 올린다. 떼면 그 속도를 유지한다
  Shift (홀드)  v 를 decel 로 내린다
  Space         v = 0

브리지는 스트림(STREAM) 중에 추종 오차를 검사하지 않는다 (redesign/12 §4 — RUNNING 전용).
그래서 여기서 막는다: 목표가 실제 위치보다 max_lead 넘게 앞서면 s 를 더 밀지 않는다.
"""
import math


class PosJog:
    def __init__(self, signs, v_max, accel, decel, max_lead, s_range):
        self.signs = list(signs)
        self.v_max = float(v_max)
        self.accel = float(accel)
        self.decel = float(decel)
        self.max_lead = float(max_lead)
        self.s_min, self.s_max = float(s_range[0]), float(s_range[1])
        self.s = 0.0
        self.v = 0.0
        self.start = None             # 활성 모터별 기준 [deg]. arm 할 때 잡는다
        self.flag = ''                # 이번 tick 에 전진을 막은 사유

    def anchor(self, cmd_abs):
        """지금 브리지가 내고 있는 명령에서 이어 간다 — 그래야 arm 순간에 튀지 않는다.
        s 는 유지한다 (이동 범위는 처음 arm 한 자리 기준이다)."""
        self.start = [c - sg * self.s for c, sg in zip(cmd_abs, self.signs)]
        self.v = 0.0

    def targets(self, s=None):
        s = self.s if s is None else s
        return [st + sg * s for st, sg in zip(self.start, self.signs)]

    def step(self, dt, fwd, back, ctrl, shift, stop, fb_abs):
        """한 tick. fb_abs = 활성 모터 실제 위치(NaN 가능). 갱신된 목표 목록을 돌려준다."""
        self.flag = ''
        direction = (1 if fwd else 0) - (1 if back else 0)
        if direction == 0 or stop:
            self.v = 0.0
        elif shift:
            self.v = max(0.0, self.v - self.decel * dt)
        elif ctrl:
            self.v = min(self.v_max, self.v + self.accel * dt)

        if self.start is None or self.v == 0.0:
            return self.targets() if self.start is not None else None
        if any(not math.isfinite(f) for f in fb_abs):
            self.flag = '실제 위치 무효 — 정지'
            return self.targets()

        s_new = self.s + direction * self.v * dt
        if s_new > self.s_max or s_new < self.s_min:
            s_new = min(self.s_max, max(self.s_min, s_new))
            self.v = 0.0
            self.flag = '이동 범위 끝'
        lead = max(abs(t - f) for t, f in zip(self.targets(s_new), fb_abs))
        if lead > self.max_lead and lead > max(abs(t - f) for t, f in zip(self.targets(), fb_abs)):
            self.flag = '명령이 실제 위치보다 %.0f° 앞섬 — 대기' % lead
            return self.targets()     # 더 벌어지는 쪽으로는 안 민다
        self.s = s_new
        return self.targets()
