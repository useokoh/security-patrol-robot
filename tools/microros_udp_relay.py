#!/usr/bin/env python3
"""맥에서 실행: 로봇(ESP32)이 보내는 micro-ROS UDP 패킷을 가상머신의 micro-ROS agent 로 넘겨 준다.

왜 필요한가:
  로봇 펌웨어는 휴대폰 핫스팟에 붙어 agent 주소 172.20.10.6:8888 로 UDP 를 보낸다.
  agent 는 UTM 가상머신(공유 NAT, 192.168.64.x) 안에서 도는데, 핫스팟의 로봇은 그 주소에 닿지 못한다.
  그래서 맥을 핫스팟에서 172.20.10.6 으로 두고, 맥이 받은 패킷을 가상머신으로 전달한다.
  펌웨어 재업로드나 가상머신 네트워크 변경이 필요 없다.

    로봇 ──UDP──▶ 맥 172.20.10.6:8888 (이 프로그램) ──UDP──▶ VM 192.168.64.2:8888 (agent)
         ◀────────────────────────────────────────────────── (응답도 같은 길로 되돌려 줌)

실행 (맥 터미널, ROS 불필요, 표준 라이브러리만 사용):
  python3 tools/microros_udp_relay.py                 # 기본: VM 192.168.64.2
  python3 tools/microros_udp_relay.py --vm 192.168.64.3
Ctrl+C 로 끝낸다.
"""

import argparse
import select
import socket
import subprocess
import sys
import time

DEFAULT_VM = '192.168.64.2'
DEFAULT_PORT = 8888
ROBOT_AGENT_IP = '172.20.10.6'     # 펌웨어에 적힌 agent 주소 (맥이 이 주소여야 로봇 패킷이 온다)


class Relay:
    """UDP 중계기. 로봇 하나만 상대한다 (펌웨어가 재부팅해 출발 포트가 바뀌면 새 로봇으로 본다)."""

    def __init__(self, vm_addr, listen_port, log=print):
        self.vm_addr = vm_addr
        self.log = log
        self.listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen.bind(('0.0.0.0', listen_port))
        # 받을 것이 없으면 기다리지 않고 넘어간다. select 가 '읽을 수 있음' 이라고 한 뒤에도
        # VM 의 '연결 거부' 오류를 send 가 먼저 가져가 버리면 recv 가 영원히 멈출 수 있기 때문이다
        self.listen.setblocking(False)
        self.upstream = None
        self.robot = None
        self.stats = {'to_vm': 0, 'to_robot': 0}

    def _new_upstream(self):
        # 로봇이 바뀌면 VM 쪽 소켓도 새로 만든다. agent 가 이전 연결과 섞지 않고 새 세션으로 받게 하기 위해서다.
        if self.upstream is not None:
            self.upstream.close()
        self.upstream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.upstream.setblocking(False)
        self.upstream.connect(self.vm_addr)

    def step(self, timeout=1.0):
        """패킷이 오면 한 번 전달한다. 테스트에서도 이 함수를 직접 부른다."""
        sockets = [self.listen] + ([self.upstream] if self.upstream else [])
        readable, _, _ = select.select(sockets, [], [], timeout)
        for sock in readable:
            if sock is self.listen:
                try:
                    data, addr = self.listen.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    continue
                except OSError as e:          # 로봇 쪽 ICMP 오류 등. 다음 패킷을 계속 받는다
                    self.log(f'로봇 패킷 받기 실패: {type(e).__name__}')
                    continue
                if addr != self.robot:
                    self.log(f'로봇 연결: {addr[0]}:{addr[1]}' + (' (재연결)' if self.robot else ''))
                    self.robot = addr
                    self._new_upstream()
                try:
                    self.upstream.send(data)
                    self.stats['to_vm'] += 1
                except OSError as e:          # VM 이 꺼져 있으면 '연결 거부' 가 올 수 있다
                    self.log(f'VM 으로 보내기 실패: {type(e).__name__}')
            elif sock is self.upstream:       # 로봇이 바뀌어 닫은 이전 소켓은 건너뛴다
                try:
                    data = self.upstream.recv(65535)
                except (BlockingIOError, InterruptedError):
                    continue
                except OSError as e:          # agent 가 아직 안 켜졌으면 ICMP 거부가 여기서 나온다
                    self.log(f'VM agent 응답 없음: {type(e).__name__} (agent 를 켰는지 확인)')
                    continue
                if self.robot:
                    try:
                        self.listen.sendto(data, self.robot)
                        self.stats['to_robot'] += 1
                    except OSError as e:      # 로봇이 잠깐 핫스팟에서 빠진 경우 등
                        self.log(f'로봇으로 보내기 실패: {type(e).__name__}')

    def close(self):
        self.listen.close()
        if self.upstream:
            self.upstream.close()


def mac_has_ip(ip):
    """맥 네트워크 인터페이스 중에 ip 가 있는지 확인한다 (ifconfig 결과에서 찾음)."""
    try:
        out = subprocess.run(['ifconfig'], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return f'inet {ip} ' in out


def main():
    parser = argparse.ArgumentParser(description='micro-ROS UDP 중계: 로봇 → 맥 → 가상머신 agent')
    parser.add_argument('--vm', default=DEFAULT_VM, help=f'가상머신 IP (기본 {DEFAULT_VM})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='agent 포트 (기본 8888)')
    args = parser.parse_args()

    has_ip = mac_has_ip(ROBOT_AGENT_IP)
    if has_ip is False:
        print(f'[주의] 이 맥에 {ROBOT_AGENT_IP} 주소가 없습니다. 핫스팟 Wi-Fi 의 IPv4 를 수동으로 '
              f'{ROBOT_AGENT_IP} 로 지정해야 로봇 패킷이 들어옵니다.')

    relay = Relay((args.vm, args.port), args.port)
    print(f'중계 시작: 0.0.0.0:{args.port} → {args.vm}:{args.port}  (Ctrl+C 로 종료)')
    last = time.time()
    try:
        while True:
            relay.step()
            if time.time() - last >= 5.0:
                s = relay.stats
                state = '로봇 기다리는 중' if relay.robot is None else f'로봇 {relay.robot[0]}'
                print(f'[{time.strftime("%H:%M:%S")}] {state} | 5초 동안 로봇→VM {s["to_vm"]}개, '
                      f'VM→로봇 {s["to_robot"]}개')
                relay.stats = {'to_vm': 0, 'to_robot': 0}
                last = time.time()
    except KeyboardInterrupt:
        print('\n중계 종료')
    finally:
        relay.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
