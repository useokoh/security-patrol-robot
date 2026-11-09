"""앱 웹 서버 시험 (ROS 불필요, 이 컴퓨터 안에서만 접속)."""

import base64
import json
import os
import socket
import struct
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from web_server import WebServer, accept_key, encode_frame  # noqa: E402


def client_frame(text, opcode=0x1, fin=True, mask=b'\x01\x02\x03\x04'):
    """브라우저처럼 마스크한 앱 → 서버 프레임."""
    payload = text.encode() if isinstance(text, str) else text
    head = bytes([(0x80 if fin else 0) | opcode])
    n = len(payload)
    head += bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack('!H', n)
    return head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload))


def read_server_frame(sock):
    b1, b2 = sock.recv(2, socket.MSG_WAITALL)
    n = b2 & 0x7F
    if n == 126:
        n = struct.unpack('!H', sock.recv(2, socket.MSG_WAITALL))[0]
    elif n == 127:
        n = struct.unpack('!Q', sock.recv(8, socket.MSG_WAITALL))[0]
    return b1 & 0x0F, sock.recv(n, socket.MSG_WAITALL) if n else b''


class FrameTest(unittest.TestCase):

    def test_accept_key_rfc_example(self):
        self.assertEqual(accept_key('dGhlIHNhbXBsZSBub25jZQ=='), 's3pPLMBiTxaQ9kYGzzhZRbK+xOo=')

    def test_encode_lengths(self):
        self.assertEqual(encode_frame('hi')[:2], b'\x81\x02')
        self.assertEqual(encode_frame('a' * 200)[1], 126)
        self.assertEqual(encode_frame('a' * 70000)[1], 127)


class ServerTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        with open(os.path.join(self.dir, 'index.html'), 'w', encoding='utf-8') as f:
            f.write('<h1>안녕</h1>')
        self.received, self.connected = [], []
        self.web = WebServer(self.dir, host='127.0.0.1', port=0, log=lambda m: None,
                             on_message=lambda cid, m: self.received.append((cid, m)),
                             on_connect=self.connected.append).start_in_thread()

    def tearDown(self):
        self.web.stop()

    def http(self, path):
        s = socket.create_connection(('127.0.0.1', self.web.port), timeout=3)
        s.sendall(f'GET {path} HTTP/1.1\r\nHost: x\r\n\r\n'.encode())
        data = b''
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
        s.close()
        return data

    def open_ws(self):
        s = socket.create_connection(('127.0.0.1', self.web.port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        s.sendall(('GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                   f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
        head = b''
        while not head.endswith(b'\r\n\r\n'):
            head += s.recv(1)
        self.assertIn(b'101 Switching Protocols', head)
        self.assertIn(accept_key(key).encode(), head)
        return s

    def wait_for(self, cond, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end and not cond():
            time.sleep(0.01)
        return cond()

    def test_static_index_utf8(self):
        data = self.http('/')
        self.assertIn(b'200 OK', data)
        self.assertIn('안녕'.encode(), data)
        self.assertIn(b'charset=utf-8', data)

    def test_symlinked_file_served(self):
        # colcon --symlink-install 은 화면 파일을 다른 폴더의 원본에 대한 링크로 설치한다
        other = tempfile.mkdtemp()
        with open(os.path.join(other, 'app.js'), 'w', encoding='utf-8') as f:
            f.write('var ok = 1;')
        os.symlink(os.path.join(other, 'app.js'), os.path.join(self.dir, 'app.js'))
        data = self.http('/app.js')
        self.assertIn(b'200 OK', data)
        self.assertIn(b'var ok = 1;', data)

    def test_path_traversal_blocked(self):
        self.assertIn(b'404', self.http('/../../etc/passwd'))
        self.assertIn(b'404', self.http('/%2e%2e/secret'))

    def test_message_and_broadcast(self):
        s = self.open_ws()
        self.assertTrue(self.wait_for(lambda: self.connected))
        s.sendall(client_frame(json.dumps({'type': 'hello', 'n': 1})))
        self.assertTrue(self.wait_for(lambda: self.received))
        self.assertEqual(self.received[0][1], {'type': 'hello', 'n': 1})
        self.web.broadcast({'type': 'state', '값': 'ㄱ'})
        opcode, payload = read_server_frame(s)
        self.assertEqual(opcode, 0x1)
        self.assertEqual(json.loads(payload), {'type': 'state', '값': 'ㄱ'})
        s.close()

    def test_fragmented_message(self):
        s = self.open_ws()
        text = json.dumps({'type': 'drive', 'v': 0.1})
        s.sendall(client_frame(text[:5], 0x1, fin=False) + client_frame(text[5:], 0x0, fin=True))
        self.assertTrue(self.wait_for(lambda: self.received))
        self.assertEqual(self.received[0][1]['type'], 'drive')
        s.close()

    def test_ping_pong_and_bad_json(self):
        s = self.open_ws()
        s.sendall(client_frame('not json'))
        s.sendall(client_frame(b'abc', 0x9))
        opcode, payload = read_server_frame(s)
        self.assertEqual((opcode, payload), (0xA, b'abc'))
        self.assertEqual(self.received, [])
        s.close()

    def test_disconnect_reported(self):
        s = self.open_ws()
        self.assertTrue(self.wait_for(lambda: self.web.client_count() == 1))
        s.sendall(client_frame(b'', 0x8))
        self.assertTrue(self.wait_for(lambda: self.web.client_count() == 0))
        self.assertTrue(self.wait_for(lambda: any(m['type'] == 'disconnected' for _, m in self.received)))
        s.close()

    def test_unmasked_frame_closes_connection(self):
        s = self.open_ws()
        s.sendall(b'\x81\x02hi')          # 마스크 없는 앱 프레임은 규칙 위반
        self.assertTrue(self.wait_for(lambda: self.web.client_count() == 0))
        s.close()

    def test_slow_client_keeps_latest(self):
        s = self.open_ws()
        self.assertTrue(self.wait_for(lambda: self.web.client_count() == 1))
        for i in range(100):
            self.web.broadcast({'type': 'state', 'i': i})
        last = None
        s.settimeout(1.0)
        try:
            while True:
                _, payload = read_server_frame(s)
                last = json.loads(payload)['i']
        except (socket.timeout, ValueError):
            pass
        self.assertEqual(last, 99)
        s.close()


if __name__ == '__main__':
    unittest.main()
