"""5-2 사건 관리 노드: 감지 이벤트를 사건으로 묶어 관리자에게 보낼 알림을 만든다.

규칙은 incident_logic.py 에 있고, 이 노드는 토픽을 연결하는 역할만 한다.

구독:
  /anomaly/detections (std_msgs/String, JSON) : anomaly_detector 의 감지 이벤트
  /patrol/status      (std_msgs/String, JSON) : 순찰 바퀴 번호 (소형 묶음 알림 시점)
  /anomaly/ack        (std_msgs/Int32)        : 관리자 확인. 6단계 휴대폰 버튼이 여기로 보낸다
발행:
  /anomaly/alerts     (std_msgs/String, JSON) : 6단계에서 휴대폰으로 보낼 알림

  ros2 launch security_patrol detect.launch.py    # 감지 노드와 함께 실행
  ros2 topic pub --once /anomaly/ack std_msgs/msg/Int32 "{data: 1}"    # 1번 사건 확인 (시험용)
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from security_patrol.incident_logic import IncidentBook

ALERT_NAMES = {'new': '새 사건', 'escalated': '등급 상승', 'reminder': '미확인 재알림', 'batch': '소형 묶음'}


class IncidentManager(Node):

    def __init__(self):
        super().__init__('incident_manager')
        p = self.declare_parameter
        self.book = IncidentBook(
            same_place_dist=p('same_place_distance', 0.5).value,
            relink_sec=p('relink_seconds', 10.0).value,
            relink_dist=p('relink_distance', 3.0).value,
            remind_after=p('remind_after_seconds', 300.0).value,
            batch_max_wait=p('batch_max_wait_seconds', 300.0).value)
        self.alert_pub = self.create_publisher(String, 'anomaly/alerts', 10)
        self.create_subscription(String, 'anomaly/detections', self.on_detection, 10)
        self.create_subscription(String, 'patrol/status', self.on_patrol, 10)
        self.create_subscription(Int32, 'anomaly/ack', self.on_ack, 10)
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info('사건 관리 시작')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_detection(self, msg):
        self.publish(self.book.on_event(json.loads(msg.data), self.now()))

    def on_patrol(self, msg):
        self.publish(self.book.on_lap(json.loads(msg.data)['lap'], self.now()))

    def on_timer(self):
        self.publish(self.book.tick(self.now()))

    def on_ack(self, msg):
        if self.book.acknowledge(msg.data, self.now()):
            self.get_logger().info(f'사건 #{msg.data} 확인됨')
        else:
            self.get_logger().warn(f'사건 #{msg.data} 없음')

    def publish(self, alerts):
        for alert in alerts:
            for i in alert['incidents']:
                self.get_logger().warn(
                    f'[{i["level"]}] 사건 #{i["incident"]} {ALERT_NAMES[alert["alert"]]}: '
                    f'({i["x"]:+.2f}, {i["y"]:+.2f}), {"이동" if i["moving"] else "정지"}, 폭 {i["width"]:.2f} m')
            self.alert_pub.publish(String(data=json.dumps(alert, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = IncidentManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
