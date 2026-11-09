"""앱용 작은 웹 서버: 정적 파일(HTML/JS/CSS) + WebSocket. 파이썬 표준 라이브러리만 쓴다.

가상머신에 웹 프레임워크를 따로 설치하지 않기 위해 직접 구현했다. 기능은 앱에 필요한 만큼만:
  - GET /, /app.js 같은 정적 파일
  - GET /ws : WebSocket (RFC 6455). 서버 → 앱은 JSON 텍스트, 앱 → 서버도 JSON 텍스트

ROS 노드는 다른 스레드에서 broadcast() 를 부르고, 앱이 보낸 명령은 on_message 콜백으로 받는다.
ROS 없이 시험할 수 있다 (test/test_web_server.py).
"""

import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
import threading

WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
MAX_MESSAGE = 64 * 1024          # 앱이 보내는 명령은 작다. 이보다 크면 연결을 끊는다
QUEUE_SIZE = 8                   # 느린 앱(휴대폰)에는 오래된 상태를 버리고 최신 것만 보낸다


def accept_key(client_key):
    """Sec-WebSocket-Key 에 대한 Sec-WebSocket-Accept 값."""
    digest = hashlib.sha1((client_key + WS_GUID).encode()).digest()
    return base64.b64encode(digest).decode()


def encode_frame(payload, opcode=0x1):
    """서버 → 앱 프레임 (마스크 없음, 한 조각)."""
    if isinstance(payload, str):
        payload = payload.encode('utf-8')
    head = bytes([0x80 | opcode])
    n = len(payload)
    if n < 126:
        head += bytes([n])
    elif n < 65536:
        head += bytes([126]) + struct.pack('!H', n)
    else:
        head += bytes([127]) + struct.pack('!Q', n)
    return head + payload


async def read_frame(reader):
    """앱 → 서버 프레임 하나를 읽는다. (fin, opcode, payload). 앱 프레임은 반드시 마스크되어 있다."""
    b1, b2 = await reader.readexactly(2)
    fin, opcode = bool(b1 & 0x80), b1 & 0x0F
    masked, n = bool(b2 & 0x80), b2 & 0x7F
    if n == 126:
        n = struct.unpack('!H', await reader.readexactly(2))[0]
    elif n == 127:
        n = struct.unpack('!Q', await reader.readexactly(8))[0]
    if n > MAX_MESSAGE:
        raise ValueError('message too large')
    if not masked:
        raise ValueError('client frame not masked')
    mask = await reader.readexactly(4)
    data = bytearray(await reader.readexactly(n))
    for i in range(n):
        data[i] ^= mask[i % 4]
    return fin, opcode, bytes(data)


class Client:
    def __init__(self, cid, writer):
        self.id = cid
        self.writer = writer
        self.queue = asyncio.Queue(maxsize=QUEUE_SIZE)


class WebServer:

    def __init__(self, static_dir, host='0.0.0.0', port=8080, on_message=None, on_connect=None, log=print):
        self.static_dir = os.path.abspath(static_dir)
        self.host, self.port = host, port
        self.on_message = on_message or (lambda cid, msg: None)
        self.on_connect = on_connect or (lambda cid: None)
        self.log = log
        self.clients = {}
        self.loop = None
        self.server = None
        self._next_id = 1
        self._ready = threading.Event()

    # ---------- 다른 스레드에서 부르는 함수 ----------

    def start_in_thread(self):
        threading.Thread(target=self._run, daemon=True).start()
        self._ready.wait(5.0)
        return self

    def broadcast(self, message):
        """모든 앱에 JSON 메시지를 보낸다 (스레드 안전)."""
        if self.loop is not None and not self.loop.is_closed():
            text = json.dumps(message, ensure_ascii=False, separators=(',', ':'))
            try:
                self.loop.call_soon_threadsafe(self._enqueue_all, text)
            except RuntimeError:          # 서버를 끄는 중
                pass

    def send_to(self, cid, message):
        if self.loop is not None and not self.loop.is_closed():
            text = json.dumps(message, ensure_ascii=False, separators=(',', ':'))
            try:
                self.loop.call_soon_threadsafe(self._enqueue, cid, text)
            except RuntimeError:          # 서버를 끄는 중
                pass

    def client_count(self):
        return len(self.clients)

    def stop(self):
        """연결을 모두 닫고 서버를 멈춘다 (진행 중인 작업을 정리한 뒤 루프를 멈춘다)."""
        loop, self.loop = self.loop, None
        if loop is None or loop.is_closed():
            return
        try:
            asyncio.run_coroutine_threadsafe(self._shutdown(loop), loop).result(3)
        except Exception:  # noqa: BLE001 - 끄는 중의 오류는 무시
            pass
        loop.call_soon_threadsafe(loop.stop)

    async def _shutdown(self, loop):
        self.server.close()
        for client in list(self.clients.values()):
            client.writer.close()
        tasks = [t for t in asyncio.all_tasks(loop) if t is not asyncio.current_task(loop)]
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    # ---------- 이벤트 루프 안 ----------

    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.server = loop.run_until_complete(asyncio.start_server(self._handle, self.host, self.port))
        self.port = self.server.sockets[0].getsockname()[1]   # port=0 이면 실제로 열린 포트
        self.loop = loop
        self._ready.set()
        loop.run_forever()
        loop.close()

    def _enqueue_all(self, text):
        for cid in list(self.clients):
            self._enqueue(cid, text)

    def _enqueue(self, cid, text):
        client = self.clients.get(cid)
        if client is None:
            return
        if client.queue.full():          # 밀리면 가장 오래된 것을 버린다
            client.queue.get_nowait()
        client.queue.put_nowait(text)

    async def _handle(self, reader, writer):
        try:
            request = await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), 10)
            lines = request.decode('latin-1').split('\r\n')
            method, path, _ = (lines[0].split(' ') + ['', '', ''])[:3]
            headers = {}
            for line in lines[1:]:
                if ':' in line:
                    k, v = line.split(':', 1)
                    headers[k.strip().lower()] = v.strip()
            if path == '/ws' and headers.get('upgrade', '').lower() == 'websocket':
                await self._websocket(reader, writer, headers)
            elif method == 'GET':
                await self._static(writer, path)
            else:
                writer.write(b'HTTP/1.1 405 Method Not Allowed\r\nContent-Length: 0\r\n\r\n')
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, asyncio.TimeoutError,
                ConnectionError, ValueError):
            pass
        finally:
            try:
                writer.close()
            except Exception:  # noqa: BLE001 - 이미 닫힌 연결
                pass

    async def _static(self, writer, path):
        path = path.split('?', 1)[0]
        if path in ('', '/'):
            path = '/index.html'
        # '..' 로 폴더 밖을 못 나가게 막는다. 링크는 따라가지 않고 판단한다
        # (colcon --symlink-install 은 화면 파일을 src 쪽 원본에 대한 링크로 설치한다)
        full = os.path.normpath(os.path.join(self.static_dir, path.lstrip('/')))
        if not full.startswith(self.static_dir + os.sep) or not os.path.isfile(full):
            body = b'not found'
            writer.write(b'HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n'
                         + f'Content-Length: {len(body)}\r\n\r\n'.encode() + body)
            await writer.drain()
            return
        with open(full, 'rb') as f:
            body = f.read()
        ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
        if ctype.startswith('text/') or ctype.endswith('javascript'):
            ctype += '; charset=utf-8'
        writer.write(f'HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n'
                     'Cache-Control: no-cache\r\n\r\n'.encode() + body)
        await writer.drain()

    async def _websocket(self, reader, writer, headers):
        key = headers.get('sec-websocket-key')
        if not key:
            return
        writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                      f'Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n').encode())
        await writer.drain()
        cid = self._next_id
        self._next_id += 1
        client = Client(cid, writer)
        self.clients[cid] = client
        sender = asyncio.ensure_future(self._sender(client))
        self.log(f'앱 연결 #{cid} (현재 {len(self.clients)}개)')
        try:
            self.on_connect(cid)
            await self._receiver(reader, writer, cid)
        finally:
            sender.cancel()
            self.clients.pop(cid, None)
            self.log(f'앱 연결 끊김 #{cid} (현재 {len(self.clients)}개)')
            try:
                self.on_message(cid, {'type': 'disconnected'})
            except Exception as e:  # noqa: BLE001 - 콜백 오류로 서버가 죽지 않게
                self.log(f'연결 종료 처리 오류: {type(e).__name__}')

    async def _sender(self, client):
        try:
            while True:
                text = await client.queue.get()
                client.writer.write(encode_frame(text))
                await client.writer.drain()
        except (ConnectionError, asyncio.CancelledError):
            pass

    async def _receiver(self, reader, writer, cid):
        parts, first_opcode = [], None
        while True:
            fin, opcode, payload = await read_frame(reader)
            if opcode == 0x8:                                   # 닫기
                writer.write(encode_frame(b'', 0x8))
                await writer.drain()
                return
            if opcode == 0x9:                                   # ping → pong
                writer.write(encode_frame(payload, 0xA))
                await writer.drain()
                continue
            if opcode == 0xA:
                continue
            if opcode in (0x1, 0x2):
                parts, first_opcode = [payload], opcode
            elif opcode == 0x0 and first_opcode is not None:    # 이어지는 조각
                parts.append(payload)
            else:
                continue
            if sum(len(p) for p in parts) > MAX_MESSAGE:
                raise ValueError('message too large')
            if not fin:
                continue
            data, parts, first_opcode = b''.join(parts), [], None
            try:
                msg = json.loads(data.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(msg, dict):
                try:
                    self.on_message(cid, msg)
                except Exception as e:  # noqa: BLE001 - 콜백 오류로 서버가 죽지 않게
                    self.log(f'명령 처리 오류: {type(e).__name__}')
