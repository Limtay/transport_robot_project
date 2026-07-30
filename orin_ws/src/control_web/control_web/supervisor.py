"""브리지 프로세스 감독자 (redesign/07 §1).

웹이 브리지에 **붙는** 클라이언트가 아니라 브리지를 **띄우는 주체**가 된다.

    종전:  [터미널] ros2 run … comm_test_node  →  [브라우저] 붙어서 조작
    확정:  [브라우저] 모드 고르고 [시작]  →  이 모듈이 브리지를 spawn  →  조작

01 **Q2("bridge_mode 는 기동 시 고정, 변경 = 재시작")를 뒤집지 않는다.** 재시작은 여전히
재시작이고, 다만 그 재시작을 사람이 터미널에서 하던 것을 웹이 대신한다. 프레임·프리셋을
런타임에 갈아끼우는 일은 하지 않는다 — 실험 시계열이 중간에 갈라지기 때문이다.

`control_web` 이 브리지와 **별도 프로세스**라서 성립한다. 웹 서버가 브리지 노드 안에
있었다면 자기를 죽여야 해서 불가능했을 것이다.


## 세 가지를 반드시 지킨다

### ① 기동 실패를 사람이 볼 수 있어야 한다 (07 §1.2)

06 §9.10 에서 기동 게이트를 조였다 — `bridge_mode`·`read_preset`·`auto_mode` 오타는 전부
**exit 1** 이고, 그 사유는 브리지 stderr 에만 있다. 그것을 캡처해 보여주지 않으면
사용자에게는 **"[시작] 을 눌렀는데 아무 일도 안 일어난다"** 로 보인다. 조용한 실패는
이 프로젝트가 반복해서 지워 온 결함 형태다.

### ② 그냥 죽이지 않는다 (07 §1.3)

STREAM 에서 프로세스를 kill 하면 마지막 명령이 ECU 에 남는다. AK 모터의 CAN timeout(200ms)이
최종 안전망이지만 **거기에 의존해 설계하지 않는다** — 안전망은 마지막 방어선이지 정상 경로가
아니다. arm 을 먼저 내리는 것은 호출자(서버)가 하고, 여기서는 SIGINT → SIGTERM → SIGKILL
단계적 종료를 맡는다.

### ③ 고아를 남기지 않는다 (07 §1.4)

`ros2 run` 은 실행 파일을 **자식으로 띄우는 래퍼**다. 부모에게만 신호를 주면 실제 브리지가
살아남는다. 그래서 `start_new_session=True` 로 **프로세스 그룹을 따로 만들고 그룹째** 신호를
보낸다. 웹이 죽을 때를 대비해 `atexit` 도 건다.
"""

import atexit
import collections
import os
import shlex
import signal
import subprocess
import threading
import time

# 브리지 노드 이름 (rd_bridge.cpp: Node("firmware_bridge_node")). 중복 기동 탐지에 쓴다.
BRIDGE_NODE_NAME = 'firmware_bridge_node'
BRIDGE_PKG = 'orin_firmware_bridge'
BRIDGE_EXE = 'comm_test_node'

# 이 줄이 나오면 200Hz 루프가 실제로 돌기 시작한 것이다 (rd_schedule.cpp).
# "프로세스가 살아 있다" 와 "브리지가 일하고 있다" 는 다르다 — INIT 에서 exit 1 로 죽는
# 경로가 있으므로 프로세스 생존만으로 running 이라고 하면 안 된다.
READY_MARK = 'Main Control Loop Started'

STOPPED, STARTING, RUNNING, EXITED = 'stopped', 'starting', 'running', 'exited'

# 종료 단계별 유예 [s]. SIGINT 로 안 죽으면 점점 세게 — 각 단계에서 로그가 남는다.
SIGINT_GRACE = 3.0
SIGTERM_GRACE = 3.0


class BridgeSupervisor:
    def __init__(self, log_lines=200, logger=None):
        self._lock = threading.Lock()
        self._proc = None
        self._state = STOPPED
        self._rc = None
        self._params = {}
        self._log = collections.deque(maxlen=log_lines)
        self._reader = None
        self._logger = logger
        self._started_at = 0.0
        atexit.register(self._atexit)

    # ── 상태 ──────────────────────────────────────────────────────────────
    def snapshot(self):
        """UI 가 그대로 렌더할 수 있는 형태. 브리지가 없어도 항상 유효한 값이 나온다."""
        with self._lock:
            return {
                'state': self._state,
                'rc': self._rc,
                'pid': self._proc.pid if self._proc else None,
                'params': dict(self._params),
                'uptime': (time.time() - self._started_at) if self._state == RUNNING else 0.0,
                'log': list(self._log),
            }

    def is_running(self):
        with self._lock:
            return self._state == RUNNING

    # ── 기동 ──────────────────────────────────────────────────────────────
    def start(self, params):
        """params: {'bridge_mode': 'control', 'auto_mode': 'direct', ...}

        **파라미터를 여기서 검증하지 않는다.** 브리지의 기동 게이트가 이미 전수 검증하고
        (06 §9.10) 사유까지 로그로 남긴다. 여기서 한 번 더 적으면 두 벌이 되고, 둘이 갈라지면
        웹이 통과시킨 값을 브리지가 거부하거나 그 반대가 된다. 그대로 넘기고 **실패 사유를
        보여주는 것**이 이 층의 일이다.
        """
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, '이미 기동 중이다 (state=%s, pid=%s)' % (self._state, self._proc.pid)

        orphan = _find_orphan_bridge()
        if orphan:
            return False, ('브리지가 이미 떠 있다 (pid %s) — 이 웹이 띄운 것이 아니다. '
                           '터미널에서 내린 뒤 다시 시도할 것' % orphan)

        cmd = ['ros2', 'run', BRIDGE_PKG, BRIDGE_EXE, '--ros-args']
        for k, v in params.items():
            cmd += ['-p', '%s:=%s' % (k, _fmt_param(v))]

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                # 자기 프로세스 그룹 — `ros2 run` 의 자식까지 그룹째 신호를 받는다.
                start_new_session=True)
        except OSError as e:
            return False, '기동 실패: %s' % e

        with self._lock:
            self._proc = proc
            self._state = STARTING
            self._rc = None
            self._params = dict(params)
            self._started_at = time.time()
            self._log.clear()
            self._log.append('$ ' + ' '.join(shlex.quote(c) for c in cmd))
            self._reader = threading.Thread(target=self._pump, args=(proc,), daemon=True)
            self._reader.start()
        return True, '기동 요청 (pid %d) — 로그에서 진행을 확인할 것' % proc.pid

    def _pump(self, proc):
        """브리지 출력을 링버퍼로. **이 스레드가 상태 전이의 진실 원천이다.**"""
        for line in proc.stdout:
            line = line.rstrip('\n')
            with self._lock:
                if proc is not self._proc:
                    return                      # 이미 다른 프로세스로 교체됐다
                self._log.append(line)
                if self._state == STARTING and READY_MARK in line:
                    self._state = RUNNING
                    self._started_at = time.time()
        rc = proc.wait()
        with self._lock:
            if proc is not self._proc:
                return
            self._rc = rc
            self._state = EXITED if rc != 0 else STOPPED
            self._log.append('[프로세스 종료: rc=%d]' % rc)
        if rc != 0 and self._logger:
            self._logger.error('브리지가 rc=%d 로 종료했다 — 웹 로그 패널의 마지막 줄이 사유다' % rc)

    # ── 종료 ──────────────────────────────────────────────────────────────
    def stop(self):
        """07 §1.3 — 단계적으로. 호출자가 arm 을 먼저 내려 두어야 한다."""
        with self._lock:
            proc = self._proc
        if proc is None or proc.poll() is not None:
            with self._lock:
                if self._state in (STARTING, RUNNING):
                    self._state = STOPPED
            return True, '이미 내려가 있다'

        for sig, grace, name in ((signal.SIGINT, SIGINT_GRACE, 'SIGINT'),
                                 (signal.SIGTERM, SIGTERM_GRACE, 'SIGTERM')):
            _signal_group(proc, sig)
            with self._lock:
                self._log.append('[%s 전송 — %.0fs 대기]' % (name, grace))
            try:
                proc.wait(timeout=grace)
                return True, '%s 로 종료' % name
            except subprocess.TimeoutExpired:
                pass

        _signal_group(proc, signal.SIGKILL)
        with self._lock:
            self._log.append('[SIGKILL — 정상 종료에 실패했다]')
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass
        return True, 'SIGKILL 로 강제 종료 (정상 종료 실패 — ECU 상태를 확인할 것)'

    def _atexit(self):
        proc = self._proc
        if proc is not None and proc.poll() is None:
            _signal_group(proc, signal.SIGINT)
            try:
                proc.wait(timeout=SIGINT_GRACE)
            except subprocess.TimeoutExpired:
                _signal_group(proc, signal.SIGKILL)


def _signal_group(proc, sig):
    """그룹째 보낸다 — `ros2 run` 래퍼만 죽이면 실제 브리지가 고아로 남는다."""
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError):
        try:
            proc.send_signal(sig)
        except ProcessLookupError:
            pass


def _fmt_param(v):
    """ROS 파라미터 문자열. 리스트는 `[1,2]` 형태여야 한다 (`-p active_motors:="[1]"`)."""
    if isinstance(v, (list, tuple)):
        return '[' + ','.join(str(x) for x in v) + ']'
    if isinstance(v, bool):
        return 'true' if v else 'false'
    return str(v)


def _find_orphan_bridge():
    """이 웹이 띄우지 않은 브리지가 이미 도는가 (07 §1.4).

    ROS 그래프가 아니라 **프로세스 목록**을 본다: 그래프는 노드가 INIT 중이면 아직 안 보이고,
    죽은 직후에도 잠깐 남는다. "포트를 잡고 있는가" 는 프로세스의 문제다.
    """
    try:
        out = subprocess.run(['pgrep', '-f', BRIDGE_EXE],
                             capture_output=True, text=True, timeout=3.0)
    except (OSError, subprocess.TimeoutExpired):
        return None
    pids = [p for p in out.stdout.split() if p.isdigit() and int(p) != os.getpid()]
    return pids[0] if pids else None
