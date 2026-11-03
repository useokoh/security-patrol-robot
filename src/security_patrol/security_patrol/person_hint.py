"""5-3 사람 후보 판정: 라이다 높이(약 13 cm)에서 사람은 발목 두 개로 보인다는 점을 이용한다.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_person_hint.py).

한 스캔의 물체가 '다리 두 개 패턴'인지 본다:
  - 합치기 전 조각이 정확히 2개 (점 2개 이상인 조각만 셈)
  - 조각마다 폭이 min_leg ~ max_leg (발목·종아리 굵기)
  - 두 조각 중심 사이가 min_gap ~ max_gap (서 있거나 걸을 때 두 발 사이)
여러 스캔에서 이 패턴이 여러 번 보이면 '사람 후보'로 표시한다.

이 판정은 표시용이다. 탁자 다리 두 개도 같은 모양이라 사람이라고 단정하지 않고,
경보 등급은 바꾸지 않는다. 다만 사람 후보는 소형 묶음으로 미루지 않는다.
"""

import math


def is_leg_pair(segments, min_leg=0.02, max_leg=0.20, min_gap=0.15, max_gap=0.50, min_points=2):
    """segments: [(중심 x, 중심 y, 폭, 점 개수), ...] 가 다리 두 개 패턴이면 True."""
    legs = [s for s in segments if s[3] >= min_points]
    if len(legs) != 2:
        return False
    if not all(min_leg <= s[2] <= max_leg for s in legs):
        return False
    gap = math.hypot(legs[0][0] - legs[1][0], legs[0][1] - legs[1][1])
    return min_gap <= gap <= max_gap
