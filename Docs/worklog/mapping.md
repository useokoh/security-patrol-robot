# 지도 작성 (2단계)

관련 문서: [simulation](simulation.md), [navigation](navigation.md), [detection](detection.md)

## 2026-10-27 3.5단계 자율 탐사 보류
- 결정과 이유: m-explore 자율 탐사는 보류한다. 순찰 → 감지 → 경보 흐름을 먼저 완성하기 위해서다.

## 2026-10-27 RViz가 안 뜨던 문제
- 문제와 해결: `slam.launch.py`가 include에 넘긴 `rviz:=false`가 바깥 launch 값까지 덮어썼다. `GroupAction(scoped=True)`로 감쌌다(`4f11b42`). VM에서 다시 실행해 확인하지는 않았다(미확인). `nav.launch.py`도 같은 구조로 만들어 RViz가 정상으로 뜨는 것은 확인했다.

## 2026-10-27 2단계 완료
- 한 일: `auto_mapper`로 정해진 경로를 자동 주행해 기준 지도를 만들었다(`9b48d61`, 지도 `ea86756`).
- 결정과 이유: 지도 작성은 정해진 경로 자동 주행(B1)으로 했다.
- 바뀐 파일: `launch/slam.launch.py`, `config/slam_toolbox_mapping.yaml`, `config/mapping_route.yaml`, `security_patrol/auto_mapper.py`, `maps/restricted_area.*`
- 문제와 해결: 경로가 북동쪽 상자에 0.33 m까지 붙어 지점을 옮겼다. 최소 거리 0.36 m.
- 확인 결과: 지점 20/20 도달, 252초. 지도 160×120(8×6 m), 물체 위치 오차 5~8 cm. 캐비닛 서쪽·남쪽 면은 지도에서 빠졌다.
