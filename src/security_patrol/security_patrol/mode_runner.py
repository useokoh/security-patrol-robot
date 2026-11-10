"""앱 서버의 모드(순찰 / 지도 만들기) 바꾸기: `ros2 launch` 를 하위 프로세스 묶음으로 켜고 끈다.

ROS 없이 시험할 수 있다 (test/test_mode_runner.py).

ros2 launch 아래에는 노드가 여러 개 있어서, 새 프로세스 묶음(session)으로 켜고 끌 때는 묶음 전체에 신호를 보낸다.
  끄기 = SIGINT (터미널 Ctrl+C 와 같음, launch 가 노드들을 차례로 정리) → 안 끝나면 SIGTERM → SIGKILL
묶음으로 보내야 launch 가 먼저 죽어도 노드가 남지 않는다.
"""

import os
import signal
import subprocess
import threading


class ModeRunner:

    def __init__(self, stop_timeout=15.0, log=print):
        self.stop_timeout = stop_timeout
        self.log = log
        self.proc = None
        self.mode = None
        self.args = {}
        self._lock = threading.Lock()

    def running(self):
        with self._lock:
            return self.proc is not None and self.proc.poll() is None

    def start(self, mode, cmd, args=None):
        """새 모드를 켠다. 이미 켜진 모드가 있으면 먼저 끈다 (기다림). 실패하면 False."""
        self.stop()
        try:
            proc = subprocess.Popen(cmd, start_new_session=True)
        except OSError as e:
            self.log(f'모드 실행 실패 ({mode}): {e}')
            return False
        with self._lock:
            self.proc, self.mode, self.args = proc, mode, dict(args or {})
        self.log(f'모드 시작: {mode}')
        return True

    def stop(self):
        """켜진 모드를 끈다. 끝날 때까지 기다린다 (부르는 쪽이 따로 스레드에서 부른다)."""
        with self._lock:
            proc, mode = self.proc, self.mode
            self.proc, self.mode, self.args = None, None, {}
        if proc is None or proc.poll() is not None:
            return
        self.log(f'모드 끄는 중: {mode}')
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except (ProcessLookupError, PermissionError):
                return
            try:
                proc.wait(self.stop_timeout)
                return
            except subprocess.TimeoutExpired:
                self.log(f'모드가 안 꺼져서 다음 신호 보냄 ({sig.name} 다음)')

    def snapshot(self):
        with self._lock:
            alive = self.proc is not None and self.proc.poll() is None
            return {'mode': self.mode if alive else None, 'args': dict(self.args) if alive else {},
                    'exited': self.proc is not None and not alive}
