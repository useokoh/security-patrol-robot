"""6단계 알림 노드: /anomaly/alerts 를 텔레그램으로 보내고, 확인 버튼을 /anomaly/ack 로 전달한다.

구독: /anomaly/alerts (std_msgs/String, JSON)
발행: /anomaly/ack    (std_msgs/Int32)

전송과 수신은 별도 스레드에서 한다. 인터넷이 느리거나 끊겨도 감지와 순찰을 막지 않기 위해서다.
인터넷이 끊기면 알림을 대기열에 두었다가 다시 연결되면 순서대로 보낸다.
설정 파일이 없으면 경고만 내고 전송 없이 실행된다. 설정은 telegram_setup 으로 만든다.

  ros2 run security_patrol telegram_setup          # 처음 한 번
  ros2 launch security_patrol detect.launch.py     # 감지·사건 관리와 함께 실행
"""

import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Int32, String

from security_patrol.telegram_link import (AlertLink, DEFAULT_CONFIG, TelegramClient, TelegramError,
                                           load_config)


def load_points():
    """순찰 지점 이름과 위치를 읽는다 ('어느 지점 근처' 표시용)."""
    path = os.path.join(get_package_share_directory('security_patrol'), 'config', 'patrol_route.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)['patrol']['checkpoints']


class AlertNotifier(Node):

    def __init__(self):
        super().__init__('alert_notifier')
        config_file = self.declare_parameter('config_file', DEFAULT_CONFIG).value
        self.retry_seconds = self.declare_parameter('retry_seconds', 5.0).value
        config = load_config(config_file)
        if config is None:
            self.get_logger().warn(
                f'텔레그램 설정 파일이 없어 알림을 보내지 않습니다 ({config_file}). '
                'ros2 run security_patrol telegram_setup 을 먼저 실행하세요')
            return

        client = TelegramClient(config['token'])
        self.link = AlertLink(client, config['chat_id'], int(time.time()), load_points())
        self.wake = threading.Event()
        self.ack_pub = self.create_publisher(Int32, 'anomaly/ack', 10)
        self.create_subscription(String, 'anomaly/alerts', self.on_alert, 10)
        threading.Thread(target=self.send_loop, daemon=True).start()
        threading.Thread(target=self.poll_loop, daemon=True).start()
        self.get_logger().info('텔레그램 알림 시작')

    def on_alert(self, msg):
        self.link.enqueue(json.loads(msg.data))
        self.wake.set()

    def send_loop(self):
        while True:
            self.wake.wait()
            self.wake.clear()
            try:
                self.link.send_pending()
                self.get_logger().info('텔레그램 전송 완료')
            except TelegramError as e:
                self.get_logger().error(f'텔레그램 전송 실패 ({e}), 남은 알림 {len(self.link.queue)}개')
                if e.retryable:
                    time.sleep(self.retry_seconds)
                if self.link.queue:
                    self.wake.set()

    def poll_loop(self):
        while True:
            try:
                for incident in self.link.poll_once():
                    self.get_logger().info(f'휴대폰에서 사건 #{incident} 확인')
                    self.ack_pub.publish(Int32(data=incident))
            except TelegramError as e:
                self.get_logger().warn(f'버튼 수신 실패 ({e}), 다시 시도', throttle_duration_sec=60.0)
                time.sleep(self.retry_seconds)


def main(args=None):
    rclpy.init(args=args)
    node = AlertNotifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
