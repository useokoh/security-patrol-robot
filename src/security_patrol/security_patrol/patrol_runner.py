"""앱에서 순찰 시작·정지: patrol_node 를 하위 프로세스로 켜고 끈다. ROS 없이 시험할 수 있다 (test/test_patrol_runner.py).

순찰 코드는 그대로 두고, 터미널에서 하던 일을 대신한다.
  시작 = patrol_node --ros-args -p route_file:=... -p laps:=N 실행
  정지 = Ctrl+C 와 같은 SIGINT. patrol_node 가 진행 중인 Nav2 이동을 취소하고 끝난다.
         정해진 시간 안에 안 끝나면 SIGTERM, 그래도 안 되면 SIGKILL.

`ros2 run` 을 거치지 않고 실행 파일을 직접 켠다. ros2 run 은 신호를 자식에게 넘기지 않아서
앱이 보낸 SIGINT 가 patrol_node 까지 가지 않기 때문이다.
"""

import signal
import subprocess
import threading
import time


class PatrolRunner:

    def __init__(self, executable, stop_timeout=8.0, log=print):
        self.executable = executable          # patrol_node 실행 파일 경로 (또는 시험용 명령 목록)
        self.stop_timeout = stop_timeout
        self.log = log
        self.proc = None
        self.course = None                    # 지금 도는 코스 id
        self.laps = 0
        self.started_at = None
        self.last = None                      # 마지막으로 끝난 순찰 {'course', 'code', 'stopped'}
        self._stopping = False
        self._lock = threading.Lock()

    def command(self, route_file, laps, use_sim_time):
        base = list(self.executable) if isinstance(self.executable, (list, tuple)) else [self.executable]
        return base + ['--ros-args', '-p', f'route_file:={route_file}', '-p', f'laps:={int(laps)}',
                       '-p', f'use_sim_time:={"true" if use_sim_time else "false"}']

    def running(self):
        with self._lock:
            return self._poll()

    def _poll(self):
        """잠금 안에서 부른다. 프로세스가 끝났으면 정리하고 False."""
        if self.proc is None:
            return False
        code = self.proc.poll()
        if code is None:
            return True
        self.last = {'course': self.course, 'code': code, 'stopped': self._stopping}
        self.log(f'순찰 끝남 (코스 {self.course}, 종료 코드 {code}{", 앱에서 정지" if self._stopping else ""})')
        self.proc, self.course, self.started_at, self._stopping = None, None, None, False
        return False

    def start(self, course_id, route_file, laps, use_sim_time, now=None):
        """순찰 시작. 이미 돌고 있으면 'running' 을 돌려주고 아무것도 안 한다."""
        with self._lock:
            if self._poll():
                return 'running'
            cmd = self.command(route_file, laps, use_sim_time)
            try:
                self.proc = subprocess.Popen(cmd)
            except OSError as e:
                self.log(f'순찰 실행 실패: {e}')
                return 'failed'
            self.course, self.laps = course_id, int(laps)
            self.started_at = time.time() if now is None else now
            self._stopping = False
            self.log(f'순찰 시작: 코스 {course_id}, {"계속 반복" if laps == 0 else f"{laps}바퀴"}')
            return None

    def stop(self, wait=False):
        """정지 신호를 보낸다. wait=False 면 기다리는 일은 뒤에서 한다 (앱·ROS 를 멈추지 않게)."""
        with self._lock:
            if not self._poll() or self._stopping:
                return
            self._stopping = True
            proc = self.proc
        self.log('순찰 정지 요청 (SIGINT)')
        try:
            proc.send_signal(signal.SIGINT)
        except OSError:
            return
        if wait:
            self._finish(proc)
        else:
            threading.Thread(target=self._finish, args=(proc,), daemon=True).start()

    def _finish(self, proc):
        for sig in (None, signal.SIGTERM, signal.SIGKILL):
            if sig is not None:
                self.log(f'순찰이 안 끝나서 {sig.name} 보냄')
                try:
                    proc.send_signal(sig)
                except OSError:
                    return
            try:
                proc.wait(self.stop_timeout)
                return
            except subprocess.TimeoutExpired:
                continue

    def snapshot(self):
        with self._lock:
            running = self._poll()
            return {'running': running, 'course': self.course if running else None,
                    'laps': self.laps if running else None, 'stopping': running and self._stopping,
                    'last': self.last}
