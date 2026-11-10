"""앱 모드 바꾸기 시험 (ROS 불필요): ros2 launch 대신 자식 프로세스를 띄우는 가짜 프로그램을 켜고 끈다."""

import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from mode_runner import ModeRunner  # noqa: E402

# 가짜 launch: 자식(노드 흉내)을 하나 띄우고 둘 다 오래 산다. 자식 pid 를 파일에 적는다
LAUNCH = '''
import subprocess, sys, time
child = subprocess.Popen([sys.executable, '-c', 'import time\\nwhile True: time.sleep(0.05)'])
open(sys.argv[1], 'w').write(str(child.pid))
try:
    while True:
        time.sleep(0.05)
except KeyboardInterrupt:
    child.wait()
'''
STUBBORN = '''
import signal, sys, time
signal.signal(signal.SIGINT, signal.SIG_IGN)
open(sys.argv[1], 'w').write('0')
while True:
    time.sleep(0.05)
'''


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:                                   # 좀비(끝났지만 거둬지지 않음)는 죽은 것으로 본다
        with os.popen(f'ps -o stat= -p {pid}') as f:
            return 'Z' not in f.read()
    except OSError:
        return True


class ModeRunnerTest(unittest.TestCase):

    def launch(self, body):
        f = tempfile.NamedTemporaryFile('w', suffix='.py', delete=False)
        f.write(body)
        f.close()
        pid_file = f.name + '.pid'
        return [sys.executable, f.name, pid_file], pid_file

    def wait_file(self, path):
        end = time.time() + 5
        while time.time() < end and not (os.path.exists(path) and open(path).read()):
            time.sleep(0.02)

    def test_stop_kills_whole_group(self):
        r = ModeRunner(stop_timeout=3.0, log=lambda m: None)
        cmd, pid_file = self.launch(LAUNCH)
        self.assertTrue(r.start('patrol', cmd, {'map': 'a.yaml'}))
        self.wait_file(pid_file)
        child = int(open(pid_file).read())
        self.assertEqual(r.snapshot()['mode'], 'patrol')
        self.assertEqual(r.snapshot()['args'], {'map': 'a.yaml'})
        r.stop()
        self.assertFalse(r.running())
        end = time.time() + 3
        while time.time() < end and alive(child):
            time.sleep(0.05)
        self.assertFalse(alive(child))                 # 노드(자식)도 남지 않는다

    def test_start_replaces_previous_mode(self):
        r = ModeRunner(stop_timeout=3.0, log=lambda m: None)
        cmd1, f1 = self.launch(LAUNCH)
        cmd2, f2 = self.launch(LAUNCH)
        r.start('patrol', cmd1)
        self.wait_file(f1)
        first = r.proc
        r.start('explore', cmd2)
        self.assertIsNotNone(first.poll())             # 앞 모드는 꺼졌다
        self.assertEqual(r.snapshot()['mode'], 'explore')
        r.stop()

    def test_escalates_when_sigint_ignored(self):
        r = ModeRunner(stop_timeout=0.5, log=lambda m: None)
        cmd, pid_file = self.launch(STUBBORN)
        r.start('explore', cmd)
        self.wait_file(pid_file)
        t0 = time.time()
        r.stop()
        self.assertFalse(r.running())
        self.assertLess(time.time() - t0, 4.0)

    def test_crashed_mode_is_reported(self):
        r = ModeRunner(log=lambda m: None)
        r.start('patrol', [sys.executable, '-c', 'pass'])
        time.sleep(0.5)
        snap = r.snapshot()
        self.assertIsNone(snap['mode'])
        self.assertTrue(snap['exited'])

    def test_missing_command(self):
        r = ModeRunner(log=lambda m: None)
        self.assertFalse(r.start('patrol', ['/없는/ros2', 'launch']))
        r.stop()                                        # 꺼진 것을 또 꺼도 문제없다


if __name__ == '__main__':
    unittest.main()
