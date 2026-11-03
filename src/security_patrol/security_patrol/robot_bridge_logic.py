"""8단계: ESP32 로봇이 보내는 원시 데이터를 라이다 스캔과 odom 으로 바꾸는 계산.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_robot_bridge.py).
펌웨어(피지컬AI 강의 publish_encoder_gyroz_bat_rssi_lidar_stamped_stable)가 보내는 형식:

  range_segment (UInt8MultiArray, 라이다 한 바퀴를 4조각으로 나눠 보냄)
    [0:4]  ESP32 시각 sec (int32, 리틀 엔디언)  ← 한 바퀴의 4조각이 같은 값
    [4:8]  ESP32 시각 nanosec (uint32)
    [8]    조각 번호 0~3
    [9:]   거리 180개 (uint16, mm). 0 은 측정 실패
  encoder_deltas_gyroz_bat_rssi (Int64MultiArray, 10 ms 마다)
    [sec, nanosec, 앞왼쪽, 뒤왼쪽, 앞오른쪽, 뒤오른쪽 엔코더 변화량(tick), 자이로 z(원시값),
     배터리(mV), Wi-Fi 세기(dBm)]

ESP32 시계는 PC 와 맞춰져 있지 않으므로, 메시지 시각은 PC 가 받은 시각을 쓰고
ESP32 시각은 두 메시지 사이 간격(dt)을 구하는 데만 쓴다 (강의 코드와 같은 방식).
"""

import math
import struct

SCAN_SIZE = 720                   # 한 바퀴 점 수 (0.5° 간격)
SEGMENTS = 4
SEGMENT_SIZE = SCAN_SIZE // SEGMENTS
SEGMENT_BYTES = 4 + 4 + 1 + SEGMENT_SIZE * 2
TELEMETRY_SIZE = 9


def parse_segment(data):
    """라이다 조각 하나를 (ESP32 시각, 조각 번호, 거리[m] 목록) 으로 바꾼다. 형식이 틀리면 None."""
    data = bytes(data)
    if len(data) != SEGMENT_BYTES or data[8] >= SEGMENTS:
        return None
    sec, nanosec = struct.unpack_from('<iI', data, 0)
    ranges_mm = struct.unpack_from('<' + 'H' * SEGMENT_SIZE, data, 9)
    return (sec, nanosec), data[8], [r / 1000.0 for r in ranges_mm]


class ScanAssembler:
    """조각 4개를 모아 한 바퀴 스캔을 만든다.

    UDP 라서 조각이 빠질 수 있다. 같은 ESP32 시각을 가진 조각 4개가 모두 모였을 때만 내보내서
    서로 다른 바퀴의 조각이 섞인 스캔을 만들지 않는다.
    """

    def __init__(self):
        self.stamp = None
        self.ranges = [0.0] * SCAN_SIZE
        self.received = set()
        self.dropped = 0          # 다 모으지 못하고 버린 바퀴 수

    def add(self, data):
        """조각을 넣는다. 한 바퀴가 완성되면 거리 720개 목록을, 아니면 None 을 돌려준다."""
        parsed = parse_segment(data)
        if parsed is None:
            return None
        stamp, index, ranges = parsed
        if stamp != self.stamp:
            if self.received:
                self.dropped += 1
            self.stamp = stamp
            self.received = set()
        self.ranges[index * SEGMENT_SIZE:(index + 1) * SEGMENT_SIZE] = ranges
        self.received.add(index)
        if len(self.received) < SEGMENTS:
            return None
        self.received = set()
        self.stamp = None
        return list(self.ranges)


def yaw_to_quaternion(yaw):
    """z 축 회전만 있는 쿼터니언 (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdomIntegrator:
    """엔코더 이동 거리 + 자이로 회전으로 로봇 위치(x, y, yaw)를 누적한다.

    4륜 스키드 스티어는 제자리 회전할 때 바퀴가 미끄러져 엔코더로 구한 회전각이 부정확하다.
    그래서 이동 거리는 엔코더, 회전은 자이로(MPU6050)로 구한다 (강의 코드와 같은 방식).
    """

    def __init__(self, ticks_per_rev=7 * 298 * 4, wheel_radius=0.065 / 2,
                 gyro_lsb_per_dps=131.0, idle_ticks=3.0, max_dt=0.2):
        self.ticks_per_rev = ticks_per_rev        # 바퀴 한 바퀴당 엔코더 tick (7펄스 × 기어비 298 × 4체배)
        self.wheel_radius = wheel_radius          # [m]
        self.gyro_lsb_per_dps = gyro_lsb_per_dps  # MPU6050 ±250°/s 범위의 감도
        self.idle_ticks = idle_ticks              # 바퀴가 거의 안 돌면 자이로 흔들림(드리프트)을 무시
        self.max_dt = max_dt                      # 메시지가 오래 끊긴 뒤 회전각이 튀지 않게 제한 [초]
        self.x = self.y = self.yaw = 0.0
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]  # 앞왼쪽, 뒤왼쪽, 앞오른쪽, 뒤오른쪽 [rad]
        self.prev_stamp = None
        self.linear = self.angular = 0.0

    def update(self, values):
        """텔레메트리 9개 값을 넣는다. 위치를 갱신했으면 True."""
        if len(values) != TELEMETRY_SIZE:
            return False
        sec, nanosec, fl, rl, fr, rr, gz = values[:7]
        stamp = sec + nanosec / 1e9
        previous, self.prev_stamp = self.prev_stamp, stamp
        if previous is None:
            return False
        dt = stamp - previous
        if dt <= 0:            # ESP32 가 재부팅하면 시각이 처음부터 다시 시작한다
            return False
        dt = min(dt, self.max_dt)

        deltas = (fl, rl, fr, rr)
        for i, d in enumerate(deltas):
            self.wheel_angles[i] += d / self.ticks_per_rev * 2 * math.pi
        meters_per_tick = 2 * math.pi * self.wheel_radius / self.ticks_per_rev
        dist = sum(deltas) / 4.0 * meters_per_tick

        activity = math.sqrt(sum(d * d for d in deltas) / 4.0)
        rate = 0.0 if activity < self.idle_ticks else math.radians(gz / self.gyro_lsb_per_dps)
        delta_yaw = rate * dt
        self.x += dist * math.cos(self.yaw + delta_yaw / 2.0)
        self.y += dist * math.sin(self.yaw + delta_yaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))
        self.linear = dist / dt
        self.angular = rate
        return True


def telemetry_extras(values):
    """텔레메트리에서 (배터리 전압[V], Wi-Fi 세기[dBm]) 를 꺼낸다."""
    return values[7] / 1000.0, int(values[8])
