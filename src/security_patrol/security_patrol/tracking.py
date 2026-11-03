"""이번 스캔의 물체(클러스터)를 추적 중인 물체와 짝짓는다.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_tracking.py).

1차: 가장 가까운 추적 물체와 match_dist 안이면 같은 물체 (예전과 같은 방식).
2차: 남은 클러스터는, 이번 스캔에 짝을 못 찾은 '움직이는 대상'과 rematch_dist 안이면 같은 물체로 본다.
     걷는 사람이 기둥에 잠깐 가렸다 나오거나 스캔 사이에 크게 옮겨 보이면 match_dist 를 넘는데,
     이때 새 물체로 만들면 이전 물체가 '사라짐' 처리되기 전까지 둘이 동시에 보여 사건이 갈라진다.
"""

import math


def associate(tracks, clusters, match_dist, rematch_dist):
    """짝짓기 결과 (pairs, new_clusters, unmatched_tracks) 를 돌려준다.

    tracks: x, y, confirmed, moving 속성이 있는 객체 목록
    clusters: (x, y, 폭, 점 개수) 목록
    pairs: [(track, cluster, rematched)] — rematched 는 2차에서 이어 붙인 경우 True
    """
    unmatched = list(tracks)
    pairs, leftover = [], []
    for c in clusters:
        best = min(unmatched, key=lambda t: math.hypot(t.x - c[0], t.y - c[1]), default=None)
        if best is not None and math.hypot(best.x - c[0], best.y - c[1]) <= match_dist:
            unmatched.remove(best)
            pairs.append((best, c, False))
        else:
            leftover.append(c)

    new_clusters = []
    for c in leftover:
        movers = [t for t in unmatched if t.confirmed and t.moving]
        best = min(movers, key=lambda t: math.hypot(t.x - c[0], t.y - c[1]), default=None)
        if best is not None and math.hypot(best.x - c[0], best.y - c[1]) <= rematch_dist:
            unmatched.remove(best)
            pairs.append((best, c, True))
        else:
            new_clusters.append(c)
    return pairs, new_clusters, unmatched
