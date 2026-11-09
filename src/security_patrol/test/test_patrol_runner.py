"""앱 순찰 시작·정지 시험 (ROS 불필요: patrol_node 대신 가짜 프로그램을 켠다)."""

import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from patrol_runner import PatrolRunner  # noqa: E402

# Ctrl+C(SIGINT)를 받으면 정리하고 끝나는 가짜 순찰 (patrol_node 처럼)
POLITE = '''
import sys, time
try:
    while True:
        time.sleep(0.05)
except KeyboardInterrupt:
    sys.exit(0)
'''
# SIGINT 를 무시하는 가짜 순찰 (멈춘 프로그램 흉내)
STUBBORN = '''
import signal, time
signal.signal(signal.SIGINT, signal.SIG_IGN)
while True:
    time.sleep(0.05)
'''
# 정한 바퀴를 돌고 스스로 끝나는 가짜 순찰
QUICK = 'import time; time.sleep(0.2)'


def script(body):
    f = tempfile.NamedTemporaryFile('w', suffix='.py', delete=False)
    f.write(body)
    f.close()
    return [sys.executable, f.name]


def wait_until(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end and not cond():
        time.sleep(0.02)
    return cond()


class PatrolRunnerTest(unittest.TestCase):

    def runner(self, body, stop_timeout=3.0):
        r = PatrolRunner(script(body), stop_timeout=stop_timeout, log=lambda m: None)
        self.addCleanup(r.stop, True)
        return r

    def test_command_passes_course_and_laps(self):
        r = PatrolRunner('/x/patrol_node', log=lambda m: None)
        self.assertEqual(r.command('/c/route.yaml', 3, True),
                         ['/x/patrol_node', '--ros-args', '-p', 'route_file:=/c/route.yaml', '-p', 'laps:=3',
                          '-p', 'use_sim_time:=true'])

    def test_start_then_stop_with_sigint(self):
        r = self.runner(POLITE)
        self.assertIsNone(r.start('loop', '/c/route.yaml', 0, True))
        self.assertTrue(r.running())
        self.assertEqual(r.snapshot()['course'], 'loop')
        self.assertEqual(r.start('other', '/c/x.yaml', 0, True), 'running')     # 두 번 켜지 않는다
        time.sleep(0.3)                                 # 가짜 순찰이 신호를 받을 준비가 되도록
        r.stop()
        self.assertTrue(wait_until(lambda: not r.running()))
        last = r.snapshot()['last']
        self.assertEqual((last['course'], last['code'], last['stopped']), ('loop', 0, True))

    def test_escalates_when_sigint_ignored(self):
        r = self.runner(STUBBORN, stop_timeout=0.5)
        r.start('loop', '/c/route.yaml', 0, True)
        time.sleep(0.3)
        r.stop()
        self.assertTrue(wait_until(lambda: not r.running(), timeout=5.0))   # SIGTERM 으로 끝남
        self.assertTrue(r.snapshot()['last']['stopped'])

    def test_finishes_by_itself(self):
        r = self.runner(QUICK)
        r.start('loop', '/c/route.yaml', 1, False)
        self.assertTrue(wait_until(lambda: not r.running()))
        self.assertEqual(r.snapshot()['last'], {'course': 'loop', 'code': 0, 'stopped': False})

    def test_missing_executable(self):
        r = PatrolRunner('/없는/patrol_node', log=lambda m: None)
        self.assertEqual(r.start('loop', '/c/route.yaml', 0, True), 'failed')
        self.assertFalse(r.running())

    def test_stop_when_idle_is_harmless(self):
        r = self.runner(POLITE)
        r.stop()
        self.assertFalse(r.snapshot()['running'])


if __name__ == '__main__':
    unittest.main()
