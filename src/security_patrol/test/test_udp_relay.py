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


if __name__ == '__main__':
    unittest.main()
