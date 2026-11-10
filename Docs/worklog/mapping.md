# 지도 작성 (2단계)

관련 문서: [simulation](simulation.md), [navigation](navigation.md), [detection](detection.md)

## 2026-11-10 3.5단계 ③ 앱에서 지도 만들기 모드 (VM 미확인, 커밋 전)
- 결정과 이유: 순찰(기준 지도+AMCL+감지)과 탐사(SLAM)는 켜야 할 프로그램이 달라, app.launch 를 '로봇 바탕 + 앱 서버'로 바꾸고 앱 서버가 모드 launch 를 켜고 끄게 했다 (큰 구조 변경, 사용자 "다음단계 실행해"). 탐사 중 감지는 끈다(자라는 SLAM 지도를 기준 지도로 쓰면 안 됨).
- 한 일:
  - launch: `robot_base`(sim_world / real_bringup), `mode_patrol`(nav2 bringup + detect + rviz), `mode_explore`(slam_toolbox + navigation_launch + explorer + rviz), `explore.launch.py` 는 robot_base + mode_explore 로 정리, `app.launch.py` 는 robot_base + app_server(start_mode:=patrol). nav.launch·real_nav 는 그대로.
  - `mode_runner.py`: ros2 launch 를 새 프로세스 묶음으로 켜고 묶음 전체에 SIGINT → SIGTERM → SIGKILL (launch 가 먼저 죽어도 노드가 안 남게).
  - app_server: 모드 상태(mode), explore/status 구독, explore_start / map_use(explored|default) 메시지, 탐사 끝나면 코스 목록 새로 고침, 탐사·모드 바꾸는 중 수동 주행·순찰 거절, 종료 때 모드도 끔. ros2 run 으로 따로 켜면(start_mode 비움) 예전처럼 모드 관리 안 함.
  - 앱: `지도` 탭(모드·지도 이름, 탐사 통계 3칸, 상황별 버튼, 확인 창), 탐사 중 코스 대신 탐사 목표(주황 과녁).
  - 미리보기: 가짜 모드 전환(2초)과 가짜 탐사(빈 지도에서 explore_logic 으로 밝히며 1초마다 지도 전송, 끝나면 코스 추가).
- 문제와 해결: 빈 map 인자를 launch 파라미터로 넘기면 '값 없음'이 될 수 있어 app.launch 에서 값을 확정해 넘김. 지도 만들기 시작을 두 번 누르면 탐사가 다시 켜짐 → 이미 그 모드면 무시.
- 확인 결과: 시험 `test_mode_runner.py` 5개(자식까지 정리, 모드 교체, 신호 단계, 꺼짐 감지) 포함 전체 통과. 가짜 ros2 로 app_server: 시작 시 순찰 모드 명령, 탐사 시작 때 순찰 정지→모드 교체, 탐사 중 조이스틱·순찰 거절, 탐사 지도 없을 때 map_use 거절, 끝난 뒤 explored.yaml 로 순찰 모드. 미리보기(휴대폰 크기): 지도 만들기 → 5곳·38초에 방 전체 → 새 지도로 순찰 모드 → '탐사로 만든 코스' 목록에.
- 확인 못 한 것: VM 에서 실제 mode_patrol/mode_explore 켜고 끄기(Nav2 종료 시간, 포트·노드 이름 겹침), 탐사 지도로 바꾼 뒤 AMCL 처음 위치(출발점 복귀 + nav2_params initial_pose 에 의존).

## 2026-11-10 3.5단계 ② 자율 탐사 (VM 미확인, 커밋 전)
- 결정과 이유: 사용자 "된다고 가정하고 다음 단계까지 개발" → ① VM 확인 전에 ② 진행. 탐사는 직접 구현(프런티어), m-explore 미사용.
- 한 일: `explore_logic.py`(프런티어 덩어리, 관찰 지점 고르기, 포기·가 본 곳 기록), `explorer.py`(Nav2 로 목표 이동, 가는 도중 경계 확인, 출발점 복귀, map_saver_cli 로 지도 저장, course_planner 로 코스 저장, /explore/status), `launch/explore.launch.py`(slam.launch/real_slam.launch + nav2 navigation_launch + explorer), `setup.py`(explorer 등록), 시험 `test_explore_logic.py`.
- 문제와 해결 (맥 가짜 탐사에서 찾음):
  - 서 있는 자리에서 안 보이는 경계(지도 밖 틈 등)를 계속 목표로 골라 80번 제자리 → '서 있는 자리면 포기' 규칙.
  - 그 규칙 때문에 문틀에 가린 옆방을 바로 포기(두 방 시험 59%) → 포기 전에 덩어리 가운데 쪽 다른 관찰 지점을 한 번 더.
  - 두 지점 사이 60번 왕복 → 가 본 관찰 지점(visited)은 서 있는 자리처럼 취급.
  - 가는 도중 확인을 덩어리 가운데 기준으로 하면 휜 경계에서 바로 '보임' 판단 → 목표 1 m 안 경계로 확인.
  - SLAM 모드라 AMCL 이 없어 waitUntilNav2Active 가 초기 위치를 기다리는 문제 → localizer='controller_server'.
- 확인 결과: 시뮬 지도를 정답으로 한 가짜 탐사 15번 이동·98%, explorer 노드 + 가짜 Nav2 18번·97%, 탐사 지도로 만든 코스가 정답 지도 코스와 같음(지점 5개 19.18 m). 못 밝힌 곳은 상자·캐비닛 안쪽. 전체 시험 통과.
- 확인 못 한 것: VM 에서 실제 Gazebo·slam_toolbox·Nav2 와 함께 실행(navigation_launch 인자, map_saver_cli, 실제 이동 중 지도 갱신 속도).

## 2026-11-10 3.5단계 재개: 코스 자동 만들기 (커밋 전)
- 결정과 이유: 사용자 요청 "로봇이 직접 벽을 확인하며 코스를 만들게". 추천안 채택: ①자동 코스 → ②자율 탐사(직접 구현, 프런티어) → ③앱 '지도 만들기' 모드 → ④실제 로봇. 휴대폰 접속은 그 뒤.
- 한 일 (①): `course_planner.py` - 안전 거리(chamfer), 갈 수 있는 영역, 그리디 커버리지(새로 보이는 공간이 많은 곳부터, 감시 거리 view_range), 실제 이동 거리 순서(최근접+2-opt), 3x3 위치 이름. `route_file.route_yaml` 에 name 추가. `app_server` 에 course_plan / course_save 메시지(저장: 시뮬 ~/sentinel_courses/sim, 실제 ~/maps_real).
- 문제와 해결: 처음엔 1.2 m 칸마다 지점 → 28개로 과다. 그리디 커버리지로 변경. 4° 광선이 멀리서 빈틈을 남겨 '안 보임'으로 세던 문제 → 3° + 15 cm 묶음 칸으로 셈.
- 확인 결과: 시뮬 지도에서 감시 거리 1.5/2.5/3.5 m → 지점 8/5/3개, 보이는 공간 89/93/92%, 약 1초. 시험 13개(course_planner, route_file) 통과.
- 앱 화면(2026-11-10 이어서): 자동 주행 탭 `＋ 자동 만들기` → 감시 거리 선택 → 지도에 주황색 미리보기 → 이름 넣고 저장 → 목록에 추가·선택. 미리보기 도구도 같은 계산 사용(저장은 메모리).
- 확인 결과(이어서): 미리보기(휴대폰 크기)에서 만들기·감시 거리 변경·저장 확인. 가짜 ROS + 시뮬 지도로 app_server: 지점 5개 1.2초, 미리보기 없이 저장·계산 중 중복 요청은 안내, 이름의 따옴표·줄바꿈 정리, yaml 정상, 목록 갱신. 전체 시험 통과.
- VM 확인(`4e09176`, 2026-11-10): app_server 에 course_plan 요청 → 감시 거리 1.5/2.5/3.5 m 에서 지점 8/5/3개, 맥과 같은 값, 1.6~2.2초. 저장은 안 함.
- 참고: 감지 알림의 '어느 지점 근처'는 launch 때 정한 코스 파일 기준이라 자동 코스 이름은 반영되지 않음.

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
