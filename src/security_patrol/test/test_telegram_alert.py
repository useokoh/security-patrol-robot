"""알림 문구와 텔레그램 연결 시험 (ROS 불필요, 가짜 서버 사용):
python3 -m unittest discover -s src/security_patrol/test"""

import json
import os
import stat
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from alert_format import format_alert, nearest_point, parse_ack, remaining_keyboard  # noqa: E402
from telegram_link import (AlertLink, TelegramClient, TelegramError, extract_token,  # noqa: E402
                           load_config, save_config)

FAKE_TOKEN = '1234567890:' + 'A' * 35

CHAT = 777
SESSION = 123
POINTS = [{'name': '출발점', 'x': -3.0, 'y': 0.0}, {'name': '북쪽 가운데', 'x': 0.0, 'y': 2.4}]


def incident(number=1, level='주의', moving=False, x=-1.0, y=0.2):
    return {'incident': number, 'level': level, 'moving': moving, 'size': 'medium',
            'x': x, 'y': y, 'width': 0.25, 'acked': False}


class FakeTelegram(BaseHTTPRequestHandler):
    """요청을 기록하고, 서버에 정해 둔 응답을 돌려주는 가짜 텔레그램."""

    def do_POST(self):
        length = int(self.headers['Content-Length'])
        method = self.path.rsplit('/', 1)[1]
        self.server.calls.append((self.path, method, json.loads(self.rfile.read(length))))
        status = self.server.fail_status.pop(0) if self.server.fail_status else 200
        result = self.server.updates if method == 'getUpdates' else True
        if method == 'getUpdates':
            self.server.updates = []
        self.send_response(status)
        self.end_headers()
        self.wfile.write(json.dumps({'ok': status == 200, 'result': result}).encode())

    def log_message(self, *args):
        pass


def button(update_id, chat=CHAT, data=f'ack:{SESSION}:1'):
    return {'update_id': update_id, 'callback_query': {
        'id': 'q', 'data': data,
        'message': {'message_id': 5, 'chat': {'id': chat}, 'reply_markup': {'inline_keyboard': [
            [{'text': '#1 확인', 'callback_data': f'ack:{SESSION}:1'}],
            [{'text': '#2 확인', 'callback_data': f'ack:{SESSION}:2'}]]}}}}


class FormatTest(unittest.TestCase):

    def test_new_caution_message(self):
        text, keyboard = format_alert({'alert': 'new', 'incidents': [incident()]}, POINTS, SESSION)
        self.assertIn('[주의] 새 사건', text)
        self.assertIn('멈춰 있는 새 물체', text)
        self.assertIn('(-1.00, +0.20), 출발점 근처', text)
        self.assertEqual(keyboard[0][0]['callback_data'], f'ack:{SESSION}:1')

    def test_urgent_header_when_any_urgent(self):
        alert = {'alert': 'reminder', 'incidents': [incident(1), incident(2, '긴급', True)]}
        text, keyboard = format_alert(alert, POINTS, SESSION)
        self.assertTrue(text.splitlines()[0].endswith('[긴급] 미확인 재알림'))
        self.assertIn('움직이는 대상', text)
        self.assertEqual(len(keyboard), 2)

    def test_message_does_not_claim_intruder(self):
        text, _ = format_alert({'alert': 'new', 'incidents': [incident(moving=True)]}, POINTS, SESSION)
        self.assertNotIn('침입', text)
        self.assertNotIn('사람', text)

    def test_merged_incident_is_named(self):
        merged = dict(incident(1, '긴급', True), merged=[2])
        text, _ = format_alert({'alert': 'escalated', 'incidents': [merged]}, POINTS, SESSION)
        self.assertIn('(같은 대상: 사건 #2)', text)

    def test_nearest_without_points(self):
        self.assertIsNone(nearest_point(0, 0, []))

    def test_parse_ack(self):
        self.assertEqual(parse_ack(f'ack:{SESSION}:12', SESSION), 12)
        self.assertIsNone(parse_ack('ack:999:12', SESSION))
        self.assertIsNone(parse_ack(f'ack:{SESSION}:x', SESSION))
        self.assertIsNone(parse_ack('hello', SESSION))

    def test_remaining_keyboard(self):
        keyboard = [[{'callback_data': 'a'}], [{'callback_data': 'b'}]]
        self.assertEqual(remaining_keyboard(keyboard, 'a'), [[{'callback_data': 'b'}]])


class LinkTest(unittest.TestCase):

    def setUp(self):
        self.server = HTTPServer(('127.0.0.1', 0), FakeTelegram)
        self.server.calls = []
        self.server.fail_status = []
        self.server.updates = []
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        base = f'http://127.0.0.1:{self.server.server_port}'
        self.client = TelegramClient(FAKE_TOKEN, base_url=base)
        self.link = AlertLink(self.client, CHAT, SESSION, POINTS)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def methods(self):
        return [c[1] for c in self.server.calls]

    def test_send_uses_token_chat_and_button(self):
        self.link.enqueue({'alert': 'new', 'incidents': [incident()]})
        self.link.send_pending()
        path, method, body = self.server.calls[0]
        self.assertEqual(path, '/bot' + FAKE_TOKEN + '/sendMessage')
        self.assertEqual(body['chat_id'], CHAT)
        self.assertEqual(body['reply_markup']['inline_keyboard'][0][0]['text'], '#1 확인')
        self.assertEqual(len(self.link.queue), 0)

    def test_server_error_keeps_alert_in_order(self):
        self.link.enqueue({'alert': 'new', 'incidents': [incident(1)]})
        self.link.enqueue({'alert': 'new', 'incidents': [incident(2)]})
        self.server.fail_status = [500]
        with self.assertRaises(TelegramError) as caught:
            self.link.send_pending()
        self.assertTrue(caught.exception.retryable)
        self.assertEqual(len(self.link.queue), 2)
        self.link.send_pending()
        sent = [c[2]['text'] for c in self.server.calls if c[1] == 'sendMessage']
        self.assertIn('사건 #1', sent[1])
        self.assertIn('사건 #2', sent[2])

    def test_rate_limit_is_retryable(self):
        self.link.enqueue({'alert': 'new', 'incidents': [incident()]})
        self.server.fail_status = [429]
        with self.assertRaises(TelegramError) as caught:
            self.link.send_pending()
        self.assertTrue(caught.exception.retryable)
        self.assertEqual(len(self.link.queue), 1)

    def test_bad_request_drops_alert_so_queue_does_not_stick(self):
        self.link.enqueue({'alert': 'new', 'incidents': [incident()]})
        self.server.fail_status = [400]
        with self.assertRaises(TelegramError) as caught:
            self.link.send_pending()
        self.assertFalse(caught.exception.retryable)
        self.assertEqual(len(self.link.queue), 0)

    def test_error_message_hides_token(self):
        self.server.fail_status = [401]
        with self.assertRaises(TelegramError) as caught:
            self.client.call('getMe')
        self.assertNotIn(FAKE_TOKEN, str(caught.exception))
        offline = TelegramClient(FAKE_TOKEN, base_url='http://127.0.0.1:1')
        with self.assertRaises(TelegramError) as caught:
            offline.call('getMe', timeout=1.0)
        self.assertTrue(caught.exception.retryable)
        self.assertNotIn(FAKE_TOKEN, str(caught.exception))

    def test_control_characters_in_token_do_not_leak(self):
        # 실제로 겪은 일: 붙여넣기 때 제어 문자가 섞여 http.client.InvalidURL 이 URL(토큰)을 출력했다
        broken = TelegramClient(FAKE_TOKEN + '\x1bC\x1bV' + FAKE_TOKEN,
                                base_url=f'http://127.0.0.1:{self.server.server_port}')
        with self.assertRaises(TelegramError) as caught:
            broken.call('getMe')
        self.assertNotIn(FAKE_TOKEN, str(caught.exception))
        self.assertFalse(caught.exception.retryable)
        self.assertEqual(self.server.calls, [])   # 잘못된 토큰으로는 요청 자체를 보내지 않는다
        # 원래 오류(URL 포함)가 traceback 에 같이 찍히지 않아야 한다
        self.assertIsNone(caught.exception.__context__)
        self.assertIsNone(caught.exception.__cause__)

    def test_extract_token(self):
        self.assertEqual(extract_token(FAKE_TOKEN + '\x1bC\x1bV' + FAKE_TOKEN), FAKE_TOKEN)
        self.assertEqual(extract_token(f'  {FAKE_TOKEN}\n'), FAKE_TOKEN)
        self.assertIsNone(extract_token('hello'))

    def test_button_gives_ack_and_removes_only_pressed_button(self):
        self.server.updates = [button(10)]
        self.assertEqual(self.link.poll_once(timeout=0), [1])
        self.assertEqual(self.link.offset, 11)
        self.assertEqual(self.methods(), ['getUpdates', 'answerCallbackQuery', 'editMessageReplyMarkup'])
        keyboard = self.server.calls[2][2]['reply_markup']['inline_keyboard']
        self.assertEqual([row[0]['text'] for row in keyboard], ['#2 확인'])

    def test_other_chat_cannot_ack(self):
        self.server.updates = [button(10, chat=999)]
        self.assertEqual(self.link.poll_once(timeout=0), [])
        self.assertNotIn('editMessageReplyMarkup', self.methods())

    def test_button_from_previous_run_is_ignored(self):
        self.server.updates = [button(10, data='ack:1:1')]
        self.assertEqual(self.link.poll_once(timeout=0), [])
        self.assertEqual(self.link.offset, 11)

    def test_ack_survives_failed_button_cleanup(self):
        self.server.updates = [button(10)]
        self.server.fail_status = [200, 500, 500]   # getUpdates 성공, 안내와 버튼 정리 실패
        self.assertEqual(self.link.poll_once(timeout=0), [1])


class ConfigTest(unittest.TestCase):

    def test_save_is_private_and_load_returns_values(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, 'sub', 'telegram.json')
            self.assertIsNone(load_config(path))
            save_config('T', CHAT, path)
            self.assertEqual(load_config(path), {'token': 'T', 'chat_id': CHAT})
            self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o600)


if __name__ == '__main__':
    unittest.main()
