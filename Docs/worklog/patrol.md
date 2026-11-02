# 순찰 (4단계)

관련 문서: [navigation](navigation.md), [detection](detection.md)

## 2026-10-27 시작 시 위치가 (0, 0)으로 덮이는 문제
- 문제와 해결: `BasicNavigator.waitUntilNav2Active()`는 `/amcl_pose`를 아직 못 받았으면 초기 위치 (0, 0, 0)을 발행해 AMCL을 리셋한다. 실제 위치는 (-3, 0)이라 순찰이 0/3에서 멈췄고 오경보도 대량으로 났다. `patrol_node`와 `nav_test`가 `initial_pose_received`를 먼저 기다리게 고쳤다(`cc6826b`).
- 확인 결과: 다시 실행해 9/9 ALL PASS, 250초. 터미널을 빠르게 연달아 켜면 생기는 시작 순서 문제라, 이전 시험은 운 좋게 피한 것이다.

## 2026-10-27 4단계 완료
- 한 일: YAML 지점 목록을 반복 순회하는 `patrol_node`를 만들었다(`865db2a`). 상태는 `/patrol/status`로 발행하고, 3회 연속 실패하면 멈추며, Ctrl+C 때 이동 명령도 취소한다.
- 결정과 이유:
  - 9개 지점을 쓴다. 2단계 지도에서 빠진 캐비닛 주변을 들여다보는 지점 2곳을 포함한다.
  - 지점마다 2초 정지하고 회전하지 않는다. 라이다가 360°이고 회전이 위치추정에 가장 불리하기 때문이다.
- 바뀐 파일: `config/patrol_route.yaml`, `security_patrol/patrol_node.py`
- 확인 결과: `laps:=2 check_truth:=true`로 18/18 ALL PASS, 2바퀴째 264초. 이때 `loc_monitor` 요약은 확인하지 못했다(미확인).
