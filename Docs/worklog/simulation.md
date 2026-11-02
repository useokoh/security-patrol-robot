# 시뮬레이션 (1단계)

관련 문서: [environment](environment.md), [mapping](mapping.md)

## 2026-10-27 1단계 완료
- 한 일: 8 × 6 m 통제 구역 월드, 강의에서 만든 ESP32 로봇 기반 모델, 점검 노드 `scan_check`, 주행 시험 `motion_test`를 만들었다.
- 결정과 이유: TurtleBot3 대신 실제로 쓸 ESP32 4륜 로봇으로 모델을 만들었다. 8단계에서 같은 토픽(`/cmd_vel`, `/odom`, `/scan`)을 쓰기 위해서다.
- 바뀐 파일: `worlds/restricted_area.world`, `urdf/patrol_bot.*.xacro`, `launch/sim_world.launch.py`, `security_patrol/scan_check.py`, `security_patrol/motion_test.py`
- 문제와 해결: 제자리 회전이 매우 느렸다. 바퀴 옆 방향 마찰을 0.2로 낮췄다(`fdir1=0 0 1`, `mu1=0.2`, `mu2=1.0`, `9185d68`).
- 확인 결과: `motion_test` 3/3 PASS. 전진 99%, 좌회전 79%, 우회전 85%(명령 대비 odom 비율, 기준 70%).
