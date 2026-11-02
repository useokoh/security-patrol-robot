"""5-2 이동 판정: 추적 중인 물체 하나의 위치 기록을 보고 '실제로 움직이는지' 판단한다.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_motion_judge.py).

판정 조건 (모두 만족해야 이동):
  1. 최근 window_sec 안의 기록이 min_duration 초 이상, min_samples 개 이상
  2. 앞쪽 1/3 평균 위치 → 가운데 1/3 → 뒤쪽 1/3 이 같은 방향으로 나아감
  3. 앞쪽 1/3 과 뒤쪽 1/3 평균 위치 사이의 거리가 기준 이상
     기준 = min_displacement, 로봇이 움직이는 중이면 여기에 물체 폭을 더한다

기준에 폭을 더하는 이유: 로봇이 지나가면 물체의 보이는 면이 바뀌어,
가만히 있는 물체도 중심 위치가 최대 폭만큼 옮겨 보이기 때문이다.
"""

import math
from collections import deque


class MotionJudge:

    def __init__(self, min_displacement=0.30, min_duration=1.0, min_samples=6, window_sec=2.0,
                 max_gap=0.5, robot_still_speed=0.05, max_robot_speed=0.25):
        self.min_displacement = min_displacement   # [m]
        self.min_duration = min_duration           # [초]
        self.min_samples = min_samples
        self.window_sec = window_sec               # 이보다 오래된 기록은 버림 [초]
        self.max_gap = max_gap                     # 기록이 이만큼 끊기면 처음부터 다시 [초]
        self.robot_still_speed = robot_still_speed # 이보다 느리면 로봇이 멈춘 것으로 봄 [m/s]
        self.max_robot_speed = max_robot_speed     # 이보다 빠르면 판정을 미룸 [m/s]
        self.history = deque()                     # (시각, x, y)

    def update(self, t, x, y, width, robot_speed):
        """새 위치를 넣고 이번에 '이동'으로 판단되면 True 를 돌려준다."""
        if robot_speed > self.max_robot_speed:
            self.history.clear()   # 빠를 때 찍힌 위치는 믿지 않는다
            return False
        if self.history and t - self.history[-1][0] > self.max_gap:
            self.history.clear()   # 감지가 쉬었던 구간을 이어 붙이지 않는다
        self.history.append((t, x, y))
        while t - self.history[0][0] > self.window_sec:
            self.history.popleft()

        duration = t - self.history[0][0]
        if duration < self.min_duration or len(self.history) < self.min_samples:
            return False

        points = list(self.history)
        third = len(points) // 3
        first = self._mean(points[:third])
        middle = self._mean(points[third:-third])
        last = self._mean(points[-third:])
        dx, dy = last[0] - first[0], last[1] - first[1]
        distance = math.hypot(dx, dy)
        required = self.min_displacement
        if robot_speed > self.robot_still_speed:
            required += width
        if distance < required:
            return False
        # 가운데 구간이 시작과 끝 사이에 있어야 같은 방향으로 움직인 것이다
        progress = ((middle[0] - first[0]) * dx + (middle[1] - first[1]) * dy) / (distance * distance)
        return 0.0 < progress < 1.0

    @staticmethod
    def _mean(points):
        return (sum(p[1] for p in points) / len(points), sum(p[2] for p in points) / len(points))
