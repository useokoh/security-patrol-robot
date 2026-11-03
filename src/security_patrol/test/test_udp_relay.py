"""맥 UDP 중계기 시험 (ROS 불필요, 이 컴퓨터 안에서만 주고받음)."""

import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools'))
from microros_udp_relay import Relay  # noqa: E402


def udp(port=0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', port))
    s.settimeout(2.0)
    return s


class RelayTest(unittest.TestCase):

    def setUp(self):
        self.logs = []
        self.agent = udp()                                   # 가상머신 agent 역할
        self.relay = Relay(self.agent.getsockname(), 0, log=self.logs.append)
        self.relay_addr = ('127.0.0.1', self.relay.listen.getsockname()[1])

    def tearDown(self):
        self.relay.close()
        self.agent.close()

    def test_round_trip(self):
        robot = udp()
        robot.sendto(b'hello', self.relay_addr)
        self.relay.step()
        data, upstream = self.agent.recvfrom(100)
        self.assertEqual(data, b'hello')
        self.agent.sendto(b'reply', upstream)                # agent 는 보낸 주소로 답한다
        self.relay.step()
        self.assertEqual(robot.recvfrom(100)[0], b'reply')
        self.assertEqual(self.relay.stats, {'to_vm': 1, 'to_robot': 1})
        robot.close()

    def test_robot_reboot_gets_new_upstream(self):
        first = udp()
        first.sendto(b'a', self.relay_addr)
        self.relay.step()
        _, upstream1 = self.agent.recvfrom(100)
        first.close()
        second = udp()                                       # 재부팅하면 출발 포트가 바뀐다
        second.sendto(b'b', self.relay_addr)
        self.relay.step()
        data, upstream2 = self.agent.recvfrom(100)
        self.assertEqual(data, b'b')
        self.assertNotEqual(upstream1, upstream2)
        self.agent.sendto(b'r', upstream2)
        self.relay.step()
        self.assertEqual(second.recvfrom(100)[0], b'r')
        self.assertTrue(any('재연결' in line for line in self.logs))
        second.close()

    def test_agent_down_does_not_crash(self):
        dead = udp()
        dead_addr = dead.getsockname()
        dead.close()                                         # agent 가 꺼져 있는 상황
        relay = Relay(dead_addr, 0, log=self.logs.append)
        robot = udp()
        robot.sendto(b'x', ('127.0.0.1', relay.listen.getsockname()[1]))
        for _ in range(3):
            relay.step(timeout=0.2)
        relay.close()
        robot.close()

    def test_agent_down_with_steady_robot_packets_never_hangs(self):
        # 2026-11-03 맥 실행: agent 가 꺼진 동안 로봇 패킷이 계속 오자, send 가 '연결 거부' 오류를 먼저 가져가고
        # select 가 읽을 수 있다고 했던 VM 소켓의 recv 가 영원히 멈췄다. 다른 스레드에서 돌려 멈추지 않는지 본다.
        import threading
        dead = udp()
        dead_addr = dead.getsockname()
        dead.close()
        relay = Relay(dead_addr, 0, log=self.logs.append)
        robot = udp()
        target = ('127.0.0.1', relay.listen.getsockname()[1])
        done = threading.Event()

        def run():
            for _ in range(60):
                robot.sendto(b'p', target)
                relay.step(timeout=0.05)
            done.set()

        threading.Thread(target=run, daemon=True).start()
        self.assertTrue(done.wait(5.0), 'relay.step 이 멈춤')
        self.assertGreater(relay.stats['to_vm'], 0)
        relay.close()
        robot.close()


if __name__ == '__main__':
    unittest.main()
