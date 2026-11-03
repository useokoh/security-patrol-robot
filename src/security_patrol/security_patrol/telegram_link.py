"""6단계 텔레그램 연결: 설정 파일, API 호출, 알림 전송 대기열, 확인 버튼 수신.

ROS 없이 동작해서 맥에서도 가짜 서버로 시험할 수 있다 (test/test_telegram_alert.py).
토큰은 URL 에 들어가므로 오류 메시지에 URL 을 넣지 않는다. 설정 파일은 저장소 밖에 둔다.
"""

import json
import os
import urllib.error
import urllib.request
from collections import deque

try:
    from security_patrol.alert_format import format_alert, parse_ack, remaining_keyboard
except ImportError:  # 맥 시험: 패키지 설치 없이 실행
    from alert_format import format_alert, parse_ack, remaining_keyboard

DEFAULT_CONFIG = os.path.expanduser('~/.config/security_patrol/telegram.json')
API_BASE = 'https://api.telegram.org'


class TelegramError(Exception):
    """retryable: 네트워크 오류처럼 다시 보내면 될 수 있는 경우 True."""

    def __init__(self, message, retryable):
        super().__init__(message)
        self.retryable = retryable


def load_config(path=DEFAULT_CONFIG):
    """{'token', 'chat_id'} 를 읽는다. 파일이 없으면 None."""
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def save_config(token, chat_id, path=DEFAULT_CONFIG):
    """본인만 읽을 수 있는 권한(600)으로 저장한다."""
    os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        json.dump({'token': token, 'chat_id': chat_id}, f)


class TelegramClient:

    def __init__(self, token, base_url=API_BASE):
        self.token = token
        self.base_url = base_url

    def call(self, method, params=None, timeout=15.0):
        """API 를 호출해 result 를 돌려준다. 실패하면 TelegramError."""
        request = urllib.request.Request(
            f'{self.base_url}/bot{self.token}/{method}',
            data=json.dumps(params or {}).encode('utf-8'),
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return json.loads(response.read().decode('utf-8'))['result']
        except urllib.error.HTTPError as e:
            e.close()
            # 429(너무 많이 보냄)와 서버 오류만 다시 시도한다. 나머지는 다시 보내도 같다.
            raise TelegramError(f'{method}: HTTP {e.code}', e.code == 429 or e.code >= 500) from None
        except (urllib.error.URLError, OSError, ValueError, KeyError) as e:
            raise TelegramError(f'{method}: {type(e).__name__}', True) from None


class AlertLink:
    """알림을 순서대로 보내고, 확인 버튼 입력을 사건 번호로 바꾼다."""

    def __init__(self, client, chat_id, session, points=None, max_queue=200):
        self.client = client
        self.chat_id = chat_id
        self.session = session
        self.points = points or []
        self.queue = deque(maxlen=max_queue)   # 넘치면 가장 오래된 알림부터 버림
        self.offset = None

    def enqueue(self, alert):
        self.queue.append(alert)

    def send_pending(self):
        """대기 중인 알림을 보낸다. 다시 시도할 오류가 나면 그 알림을 남기고 멈춘 뒤 오류를 올린다.

        다시 보내도 소용없는 오류(잘못된 토큰 등)면 그 알림을 버리고 오류를 올린다.
        """
        while self.queue:
            text, keyboard = format_alert(self.queue[0], self.points, self.session)
            try:
                self.client.call('sendMessage', {
                    'chat_id': self.chat_id, 'text': text,
                    'reply_markup': {'inline_keyboard': keyboard}})
            except TelegramError as e:
                if not e.retryable:
                    self.queue.popleft()
                raise
            self.queue.popleft()

    def poll_once(self, timeout=25):
        """버튼 입력을 기다렸다가 확인된 사건 번호 목록을 돌려준다 (실패하면 TelegramError)."""
        params = {'timeout': timeout, 'allowed_updates': ['callback_query']}
        if self.offset is not None:
            params['offset'] = self.offset
        updates = self.client.call('getUpdates', params, timeout=timeout + 10.0)
        acks = []
        for update in updates:
            self.offset = update['update_id'] + 1
            query = update.get('callback_query')
            if query:
                incident = self._handle_button(query)
                if incident is not None:
                    acks.append(incident)
        return acks

    def _handle_button(self, query):
        message = query.get('message') or {}
        chat = message.get('chat') or {}
        if chat.get('id') != self.chat_id:   # 등록한 대화방이 아니면 무시
            self._answer(query, '권한이 없습니다')
            return None
        data = query.get('data')
        incident = parse_ack(data, self.session)
        if incident is None:
            self._answer(query, '이전 실행의 알림입니다')
            return None
        self._answer(query, f'사건 #{incident} 확인했습니다')
        keyboard = (message.get('reply_markup') or {}).get('inline_keyboard', [])
        try:
            self.client.call('editMessageReplyMarkup', {
                'chat_id': self.chat_id, 'message_id': message.get('message_id'),
                'reply_markup': {'inline_keyboard': remaining_keyboard(keyboard, data)}})
        except TelegramError:
            pass   # 버튼 정리는 실패해도 확인 자체에는 영향이 없다
        return incident

    def _answer(self, query, text):
        try:
            self.client.call('answerCallbackQuery', {'callback_query_id': query['id'], 'text': text})
        except TelegramError:
            pass   # 안내 문구는 실패해도 확인 자체에는 영향이 없다
