"""6단계 처음 한 번만 실행: 봇 토큰을 입력받아 대화방을 찾고 설정 파일을 저장한다.

토큰은 화면에 보이지 않게 입력받고, 저장소 밖 파일(~/.config/security_patrol/telegram.json)에
본인만 읽을 수 있는 권한으로 저장한다. 절대 저장소에 넣지 않는다.

  ros2 run security_patrol telegram_setup
"""

import getpass
import time

from security_patrol.telegram_link import DEFAULT_CONFIG, TelegramClient, TelegramError, save_config

WAIT_SECONDS = 180


def find_chat(client):
    """봇에게 온 메시지를 기다려 대화방 번호를 돌려준다. 시간이 지나면 None."""
    offset = None
    deadline = time.monotonic() + WAIT_SECONDS
    while time.monotonic() < deadline:
        params = {'timeout': 10, 'allowed_updates': ['message']}
        if offset is not None:
            params['offset'] = offset
        updates = client.call('getUpdates', params, timeout=20.0)
        for update in updates:
            offset = update['update_id'] + 1
            message = update.get('message')
            if message:
                # 다음 getUpdates 에서 이 메시지가 다시 오지 않게 읽음 처리
                client.call('getUpdates', {'offset': offset, 'timeout': 0})
                return message['chat']['id']
    return None


def main():
    token = getpass.getpass('봇 토큰을 붙여넣고 Enter (화면에 보이지 않습니다): ').strip()
    client = TelegramClient(token)
    try:
        bot = client.call('getMe')
        print(f'봇 확인: @{bot["username"]}')
        print(f'휴대폰 텔레그램에서 이 봇을 열고 시작(/start)을 눌러 주세요. {WAIT_SECONDS}초 기다립니다...')
        chat_id = find_chat(client)
        if chat_id is None:
            print('메시지를 받지 못했습니다. 다시 실행해 주세요.')
            return
        client.call('sendMessage', {'chat_id': chat_id, 'text': 'SENTINEL 알림 연결 완료'})
    except TelegramError as e:
        print(f'실패: {e} (HTTP 401 이면 토큰이 틀린 것입니다)')
        return
    save_config(token, chat_id)
    print(f'완료: 휴대폰에 시험 메시지가 왔는지 확인하세요. 설정 파일: {DEFAULT_CONFIG}')


if __name__ == '__main__':
    main()
