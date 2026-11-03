"""5단계 이상 감지 노드: 라이다 점을 기준 지도와 비교해 '지도에 없던 물체'를 찾는다.

처리 순서 (라이다 한 번 들어올 때마다):
  1. 감지 가능 여부 확인: AMCL 불확실성(σ)이 크거나 로봇이 빠르게 회전 중이면 이번 스캔은 건너뜀 (오경보 방지)
  2. 라이다 점을 map 좌표로 변환 (TF: map → laser_frame)
  3. 주변 clearance_radius 안에 지도상 장애물이 없는 점 = '새로운 점'
  4. 새로운 점을 물체(클러스터)로 묶음. 가까운 클러스터(사람 발 두 개 등)는 하나로 합침
  5. 단면 폭으로 크기 등급 분류 (소형 / 중형 / 대형)
  6. 같은 자리에 등급별 횟수만큼 연속으로 보이면 '감지 확정' → 경고 출력 + /anomaly/detections 발행
  7. 확정된 물체의 위치 기록으로 이동 여부를 판정 (motion_judge.py). 크기와 이동은 올라가기만 한다
  8. 합치기 전 조각이 다리 두 개 모양으로 여러 번 보이면 '사람 후보'로 표시 (person_hint.py, 표시용)

발행 토픽:
  /anomaly/detections (std_msgs/String, JSON) : detected / upgraded / moving / person / cleared 이벤트 → incident_manager 에서 사용
  /anomaly/markers    (visualization_msgs/MarkerArray) : RViz 표시 (소형 노랑 / 중형 주황 / 대형 빨강, 후보는 회색, 이동은 글자 MOVE, 사람 후보는 P?)

  ros2 launch security_patrol detect.launch.py     # config/anomaly_detector.yaml 설정으로 실행
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from security_patrol.motion_judge import MotionJudge
from security_patrol.person_hint import is_leg_pair
from security_patrol.tracking import associate

SIZE_NAMES = {'small': '소형', 'medium': '중형', 'large': '대형'}
SIZE_LABELS = {'small': 'S', 'medium': 'M', 'large': 'L'}  # RViz 글자는 한글이 깨질 수 있어 영문
SIZE_COLORS = {'small': (1.0, 0.9, 0.0), 'medium': (1.0, 0.5, 0.0), 'large': (1.0, 0.0, 0.0)}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    """여러 스캔에 걸쳐 같은 자리에 보이는 물체 하나."""

    def __init__(self, track_id, cluster, stamp, judge):
        self.id = track_id
        self.x, self.y, self.width, self.points = cluster[:4]
        self.leg_frames = 0          # 다리 두 개 패턴으로 보인 스캔 수
        self.person = False          # 사람 후보 (표시용, 한 번 정해지면 유지)
        self.hits = 1
        self.misses = 0
        self.confirmed = False
        self.first_seen = stamp
        self.size = None
        self.judge = judge
        self.moving = False


class AnomalyDetector(Node):

    def __init__(self):
        super().__init__('anomaly_detector')
        p = self.declare_parameter
        # 지도 비교
        self.clearance = p('clearance_radius', 0.25).value        # 이 거리 안에 지도 장애물이 있으면 '원래 있던 것'
        self.unknown_is_obstacle = p('unknown_is_obstacle', False).value  # False: 지도의 '모르는 칸'에 보이는 것도 이상으로 봄
        self.max_range = p('max_range', 4.0).value                # 이보다 먼 점은 위치 오차가 커서 무시 [m]
        # 물체 묶기
        self.cluster_gap = p('cluster_gap', 0.10).value           # 이웃한 점 사이가 이보다 멀면 다른 물체 [m]
        self.merge_dist = p('merge_distance', 0.30).value         # 이 거리 안의 물체는 하나로 합침 (사람 발 두 개) [m]
        self.min_points = p('min_points', 3).value                # 이보다 점이 적으면 잡음으로 버림
        # 크기 등급 (13 cm 높이 단면 폭)
        self.small_max = p('small_max_width', 0.15).value         # 소형 < 0.15 m
        self.large_min = p('large_min_width', 0.60).value         # 대형 >= 0.60 m
        self.confirm = {
            'small': p('confirm_frames_small', 5).value,          # 잡음과 헷갈리기 쉬워 더 많이 확인
            'medium': p('confirm_frames_medium', 3).value,
            'large': p('confirm_frames_large', 3).value,
        }
        # 추적
        self.match_dist = p('match_distance', 0.40).value         # 다음 스캔에서 이 거리 안이면 같은 물체 [m]
        self.rematch_dist = p('moving_rematch_distance', 1.0).value  # 움직이는 대상은 이 거리 안이면 이어 붙임 [m]
        self.person_frames = p('person_frames', 3).value          # 다리 두 개 패턴이 이만큼 보이면 사람 후보
        self.max_misses = p('max_misses', 3).value                # 확정된 물체가 이만큼 연속으로 안 보이면 '사라짐'
        # 이동 판정 (motion_judge.py)
        self.motion_args = {
            'min_displacement': p('moving_min_displacement', 0.30).value,  # [m]
            'min_duration': p('moving_min_duration', 1.0).value,           # [초]
            'min_samples': p('moving_min_samples', 6).value,
            'window_sec': p('moving_window', 2.0).value,                   # [초]
            'robot_still_speed': p('robot_still_speed', 0.05).value,       # 이보다 빠르면 기준에 물체 폭을 더함 [m/s]
            'max_robot_speed': p('moving_max_robot_speed', 0.25).value,    # 이보다 빠르면 판정을 미룸 [m/s]
        }
        # 감지 중지 조건 (오경보 방지)
        self.max_sigma_xy = p('max_sigma_xy', 0.35).value         # AMCL 위치 불확실성 [m]
        self.max_sigma_yaw = math.radians(p('max_sigma_yaw_deg', 15.0).value)
        self.max_turn_rate = p('max_turn_rate', 0.3).value        # 이보다 빨리 회전 중이면 쉼 [rad/s]

        self.map_info = None
        self.known = None            # True = 지도상 장애물 근처 (원래 있던 것)
        self.amcl = None
        self.turn_rate = 0.0
        self.robot_speed = 0.0
        self.tracks = []
        self.next_id = 1
        self.paused_reason = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(String, 'anomaly/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'anomaly/markers', 10)
        self.get_logger().info('이상 감지 시작: 기준 지도(/map) 기다리는 중...')

    # ---------- 입력 ----------

    def on_map(self, msg):
        """기준 지도에서 '원래 있던 것' 영역을 만든다: 장애물 칸을 clearance_radius 만큼 넓힘."""
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        obstacle = grid >= 65
        if self.unknown_is_obstacle:
            obstacle |= grid < 0
        r = int(math.ceil(self.clearance / info.resolution))
        known = np.zeros_like(obstacle)
        padded = np.pad(obstacle, r)
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    known |= padded[r + dj:r + dj + info.height, r + di:r + di + info.width]
        self.map_info = info
        self.known = known
        self.get_logger().info(
            f'기준 지도 수신: {info.width}x{info.height}, 해상도 {info.resolution} m, '
            f'장애물 주변 {self.clearance:.2f} m 는 원래 있던 것으로 처리')

    def on_amcl(self, msg):
        self.amcl = msg

    def on_odom(self, msg):
        self.turn_rate = abs(msg.twist.twist.angular.z)
        self.robot_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    # ---------- 감지 ----------

    def can_detect(self):
        """감지해도 되는 상태인지 확인. 안 되면 이유를 돌려준다."""
        if self.known is None:
            return '기준 지도 없음'
        if self.amcl is None:
            return 'AMCL 위치 없음'
        cov = self.amcl.pose.covariance
        sigma_xy = math.sqrt(max(cov[0] + cov[7], 0.0))
        sigma_yaw = math.sqrt(max(cov[35], 0.0))
        if sigma_xy > self.max_sigma_xy or sigma_yaw > self.max_sigma_yaw:
            return f'위치 불확실 (σ {sigma_xy:.2f} m / {math.degrees(sigma_yaw):.0f}°)'
        if self.turn_rate > self.max_turn_rate:
            return '회전 중'
        return None

    def on_scan(self, scan):
        reason = self.can_detect()
        if reason != self.paused_reason:
            # 상태가 바뀔 때만 출력 (매 스캔 출력하면 너무 많음)
            self.get_logger().info(f'감지 일시 중지: {reason}' if reason else '감지 재개')
            self.paused_reason = reason
        if reason:
            return  # 쉬는 동안은 추적 중인 물체도 그대로 둔다 (놓친 것으로 세지 않음)

        # 스캔 시각의 TF 를 쓰고, 아직 안 들어왔으면 가장 최근 TF 를 쓴다 (회전 중엔 감지를 쉬므로 차이가 작음).
        # 콜백 안에서 TF 를 기다리면(timeout) 같은 스레드라 TF 수신이 막히므로 기다리지 않는다.
        try:
            tf = self.tf_buffer.lookup_transform('map', scan.header.frame_id,
                                                 Time.from_msg(scan.header.stamp))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform('map', scan.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().warn(f'TF 변환 실패: {e}', throttle_duration_sec=5.0)
                return

        points = self.new_points(scan, tf)
        clusters = self.make_clusters(points)
        self.update_tracks(clusters, Time.from_msg(scan.header.stamp))
        self.publish_markers(scan.header.stamp)

    def new_points(self, scan, tf):
        """지도에 없던 라이다 점들을 map 좌표 (N x 2) 로 돌려준다 (각도 순서 유지)."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= min(scan.range_max, self.max_range))
        r, a = ranges[valid], angles[valid]

        t = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)
        lx, ly = r * np.cos(a), r * np.sin(a)
        mx = t.x + math.cos(yaw) * lx - math.sin(yaw) * ly
        my = t.y + math.sin(yaw) * lx + math.cos(yaw) * ly

        info = self.map_info
        i = np.floor((mx - info.origin.position.x) / info.resolution).astype(int)
        j = np.floor((my - info.origin.position.y) / info.resolution).astype(int)
        inside = (i >= 0) & (i < info.width) & (j >= 0) & (j < info.height)
        is_new = np.zeros(len(mx), dtype=bool)
        is_new[inside] = ~self.known[j[inside], i[inside]]
        return np.column_stack([mx[is_new], my[is_new]])

    def make_clusters(self, pts):
        """새로운 점들을 물체로 묶는다. 결과: [(중심 x, 중심 y, 폭, 점 개수), ...]"""
        if len(pts) == 0:
            return []
        # 1) 각도 순서로 이웃한 점끼리 묶기
        groups = [[pts[0]]]
        for prev, cur in zip(pts[:-1], pts[1:]):
            if np.hypot(*(cur - prev)) <= self.cluster_gap:
                groups[-1].append(cur)
            else:
                groups.append([cur])
        if len(groups) > 1 and np.hypot(*(groups[0][0] - groups[-1][-1])) <= self.cluster_gap:
            groups[0] = groups.pop() + groups[0]   # 360° 한 바퀴 끝과 시작이 이어지는 경우
        groups = [np.array(g) for g in groups]

        # 2) 가까운 묶음끼리 합치기 (사람 발 두 개처럼 사이에 배경이 보이는 경우)
        #    합치기 전 조각(segments)은 사람 후보 판정에 쓰려고 따로 기억한다
        pieces = [[g] for g in groups]
        merged = True
        while merged and len(groups) > 1:
            merged = False
            for a in range(len(groups)):
                for b in range(a + 1, len(groups)):
                    d = np.min(np.hypot(groups[a][:, None, 0] - groups[b][None, :, 0],
                                        groups[a][:, None, 1] - groups[b][None, :, 1]))
                    if d <= self.merge_dist:
                        groups[a] = np.vstack([groups[a], groups.pop(b)])
                        pieces[a] += pieces.pop(b)
                        merged = True
                        break
                if merged:
                    break

        # 3) 점이 너무 적은 것은 잡음으로 버리고, 중심과 폭 계산
        clusters = []
        for g, parts in zip(groups, pieces):
            if len(g) < self.min_points:
                continue
            cx, cy = g.mean(axis=0)
            segments = [(*map(float, p.mean(axis=0)), self.width_of(p), len(p)) for p in parts]
            clusters.append((float(cx), float(cy), self.width_of(g), len(g), segments))
        return clusters

    @staticmethod
    def width_of(points):
        """점들 중 가장 먼 두 점 사이 거리 [m]."""
        diff = points[:, None, :] - points[None, :, :]
        return float(np.max(np.hypot(diff[..., 0], diff[..., 1])))

    def size_class(self, width):
        if width < self.small_max:
            return 'small'
        if width < self.large_min:
            return 'medium'
        return 'large'

    def update_tracks(self, clusters, stamp):
        """이번 스캔의 물체를 이전 물체와 짝지어 연속으로 보인 횟수를 센다."""
        pairs, new_clusters, unmatched = associate(self.tracks, clusters, self.match_dist, self.rematch_dist)
        for best, c, rematched in pairs:
            if rematched:
                # 크게 옮겨 보인 움직이는 대상: 평균을 내면 엉뚱한 중간 위치가 되므로 새 위치로 바로 옮긴다
                self.get_logger().info(
                    f'[#{best.id}] 움직이는 대상 다시 잡음: {math.hypot(best.x - c[0], best.y - c[1]):.2f} m 옮겨 보임')
                best.x, best.y = c[0], c[1]
            else:
                best.x = 0.5 * best.x + 0.5 * c[0]
                best.y = 0.5 * best.y + 0.5 * c[1]
            best.width = max(best.width, c[2])   # 보이는 면만 재므로 가장 넓게 보인 값을 사용
            best.points = c[3]
            best.hits += 1
            best.misses = 0
            self.count_legs(best, c)
            self.check_confirm(best, stamp)
            self.check_motion(best, c, stamp)
            self.check_person(best, stamp)
        for c in new_clusters:
            track = Track(self.next_id, c, stamp, MotionJudge(**self.motion_args))
            self.next_id += 1
            self.tracks.append(track)
            self.count_legs(track, c)
            self.check_confirm(track, stamp)
            self.check_motion(track, c, stamp)
            self.check_person(track, stamp)

        for t in unmatched:
            t.misses += 1
            # 확정 전에는 '연속'으로 보여야 하므로 한 번만 놓쳐도 버림
            if not t.confirmed or t.misses > self.max_misses:
                self.tracks.remove(t)
                if t.confirmed:
                    self.get_logger().info(
                        f'[#{t.id}] {SIZE_NAMES[t.size]} 물체 사라짐 ({t.x:+.2f}, {t.y:+.2f})')
                    self.publish_event('cleared', t, stamp)

    def check_motion(self, t, cluster, stamp):
        """이번 스캔의 실제 위치(평활 전)를 판정기에 넣는다. 한 번 이동으로 판정되면 되돌리지 않는다."""
        now = stamp.nanoseconds / 1e9
        is_moving = t.judge.update(now, cluster[0], cluster[1], t.width, self.robot_speed)
        if is_moving and t.confirmed and not t.moving:
            t.moving = True
            self.get_logger().warn(
                f'!!! [#{t.id}] 움직이는 대상: ({t.x:+.2f}, {t.y:+.2f}), 폭 {t.width:.2f} m')
            self.publish_event('moving', t, stamp)

    def count_legs(self, t, cluster):
        """다리 두 개 패턴이면 센다. check_confirm 보다 먼저 불러야 'detected' 이벤트에 사람 후보가 실린다."""
        if is_leg_pair(cluster[4]):
            t.leg_frames += 1

    def check_person(self, t, stamp):
        """다리 두 개 패턴이 person_frames 번 보이면 사람 후보로 표시한다 (표시용, 등급은 안 바꿈).

        확정과 같은 스캔에서 정해지면 check_confirm 이 'detected' 에 실어 보내므로 여기서는 확정 뒤만 처리한다.
        """
        if t.confirmed and not t.person and t.leg_frames >= self.person_frames:
            t.person = True
            self.get_logger().warn(
                f'[#{t.id}] 사람 후보: 다리 두 개 패턴 {t.leg_frames}번 ({t.x:+.2f}, {t.y:+.2f})')
            self.publish_event('person', t, stamp)

    def check_confirm(self, t, stamp):
        size = self.size_class(t.width)
        if t.confirmed and size != t.size:
            # width 는 가장 넓게 보인 값이라 크기 등급은 올라가기만 한다
            self.get_logger().warn(
                f'[#{t.id}] 크기 재평가: {SIZE_NAMES[t.size]} → {SIZE_NAMES[size]} (폭 {t.width:.2f} m)')
            t.size = size
            self.publish_event('upgraded', t, stamp)
        t.size = size
        if not t.confirmed and t.hits >= self.confirm[size]:
            t.confirmed = True
            # 확정 전에 이미 다리 패턴이 충분히 보였으면 첫 알림에 사람 후보를 실어 보낸다
            # (따로 보내면 사건 관리에서 '이미 알린 사건'이 되어 알림 문구에 빠진다)
            t.person = t.leg_frames >= self.person_frames
            delay = (stamp - t.first_seen).nanoseconds / 1e9
            note = ' (위치추정 오류일 수도 있음)' if size == 'large' else ''
            self.get_logger().warn(
                f'!!! [#{t.id}] {SIZE_NAMES[size]} 물체 감지: ({t.x:+.2f}, {t.y:+.2f}), '
                f'폭 {t.width:.2f} m, 점 {t.points}개, 처음 보인 뒤 {delay:.1f}초 만에 확정{note}'
                f'{", 사람 후보" if t.person else ""}')
            self.publish_event('detected', t, stamp)

    # ---------- 출력 ----------

    def publish_event(self, event, t, stamp):
        data = {'event': event, 'id': t.id, 'size': t.size, 'size_name': SIZE_NAMES[t.size],
                'x': round(t.x, 3), 'y': round(t.y, 3), 'width': round(t.width, 3),
                'moving': t.moving, 'person': t.person,
                'first_seen': t.first_seen.nanoseconds / 1e9,   # 7단계 확정 지연 측정용
                'stamp': stamp.nanoseconds / 1e9}
        self.det_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def publish_markers(self, stamp):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for t in self.tracks:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'anomaly'
            m.id = t.id
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y = t.x, t.y
            m.pose.orientation.w = 1.0
            d = max(t.width, 0.1)
            if t.confirmed:
                m.scale.x = m.scale.y = d
                m.scale.z = 0.5
                m.color.r, m.color.g, m.color.b = SIZE_COLORS[t.size]
                m.color.a = 0.8
            else:  # 확정 전 후보: 작은 회색
                m.scale.x = m.scale.y = d
                m.scale.z = 0.1
                m.color.r = m.color.g = m.color.b = 0.6
                m.color.a = 0.5
            markers.markers.append(m)
            if t.confirmed:
                label = Marker()
                label.header = m.header
                label.ns = 'anomaly_label'
                label.id = t.id
                label.type = Marker.TEXT_VIEW_FACING
                label.pose.position.x, label.pose.position.y = t.x, t.y
                label.pose.position.z = 0.7
                label.pose.orientation.w = 1.0
                label.scale.z = 0.2
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = f'#{t.id} {SIZE_LABELS[t.size]} {t.width:.2f}m{" MOVE" if t.moving else ""}{" P?" if t.person else ""}'
                markers.markers.append(label)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
