# Security Patrol Robot

출입 통제 구역(아무도 있어서는 안 되는 공간)을 순찰하는 ROS2 경비 로봇입니다.

1. SLAM으로 구역의 **기준 지도**를 만들어 저장합니다.
2. 저장된 지도 위에서 정해진 경로를 **순찰**합니다.
3. 순찰 중 2D 라이다 데이터를 기준 지도와 비교해 **원래 없던 사람이나 물체**를 감지합니다.
4. 감지하면 관리자에게 **위치와 함께 경보**를 보냅니다.

## 하드웨어
- ESP32 기반 4륜 스키드 스티어 로봇 (micro-ROS)
- 360° 2D 라이다 (720점, 0.15~8 m, 약 6 Hz)
- ESP32-CAM (선택)

## 개발 환경
- Ubuntu 22.04 (ARM64, Apple Silicon 맥의 UTM 가상머신)
- ROS2 Humble
- Gazebo Fortress (`ros_gz`)

## 진행 상황
전체 단계는 [PLAN.md](PLAN.md)를 참고하세요.

- [x] 1단계: Gazebo 시뮬레이션 환경 (통제 구역 월드 + 로봇 모델)
- [x] 2단계: SLAM 기준 지도 작성
- [x] 3단계: Nav2 자율주행
- [x] 4단계: 순찰 노드
- [x] 5단계: 이상 감지 노드
- [x] 6단계: 경보 노드
- [x] 7단계: 시뮬레이션 통합 시험
- [ ] 8단계: 실제 로봇 이식

## 실행 방법 (1단계)

```bash
# 의존성 설치
sudo apt install -y ros-humble-desktop ros-humble-ros-gz ros-humble-xacro ros-humble-teleop-twist-keyboard

# 빌드
cd ~/security_patrol_ws
colcon build --symlink-install
source install/setup.bash

# 시뮬레이션 실행 (Gazebo + RViz)
ros2 launch security_patrol sim_world.launch.py

# 다른 터미널: 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 다른 터미널: 센서 점검
ros2 run security_patrol scan_check

# 다른 터미널: 자동 주행 테스트 (전진 / 제자리 회전 → PASS/FAIL)
ros2 run security_patrol motion_test --ros-args -p use_sim_time:=true
```

> UTM 가상머신(Apple Silicon)에서는 GPU 가 OpenGL 3.3 을 지원하지 않아 Gazebo 를 CPU 로 렌더링합니다 (`software_render:=true`, 기본값).

## 실행 방법 (2단계: SLAM 기준 지도)

```bash
# 시뮬레이션 + slam_toolbox + RViz(지도 보기)
ros2 launch security_patrol slam.launch.py

# 다른 터미널: 정해진 경로 자동 주행 (시뮬레이션 월드 전용 경로)
ros2 run security_patrol auto_mapper --ros-args -p use_sim_time:=true \
  --params-file ~/security_patrol_ws/install/security_patrol/share/security_patrol/config/mapping_route.yaml

# 주행이 끝나면 지도 저장 (.pgm 이미지 + .yaml 정보)
ros2 run nav2_map_server map_saver_cli -f ~/security_patrol_ws/src/security_patrol/maps/restricted_area
```

## 실행 방법 (3단계: Nav2 자율주행)

```bash
# 시뮬레이션 + 기준 지도 + AMCL 위치추정 + Nav2 + RViz (Gazebo 화면은 기본으로 끔, 보려면 gui:=true)
ros2 launch security_patrol nav.launch.py

# 다른 터미널: 위치추정 점검 (AMCL 추정 위치 vs 시뮬레이션 실제 위치, 1초마다)
ros2 run security_patrol loc_monitor --ros-args -p use_sim_time:=true

# 또 다른 터미널: 자동 목표 주행 테스트 (목표 4곳 → PASS/FAIL, 시뮬레이션에서는 실제 위치 오차도 확인)
ros2 run security_patrol nav_test --ros-args -p use_sim_time:=true
```

UTM 가상머신은 CPU 로 화면을 그리기 때문에 CPU 가 부족하면 AMCL 이 위치를 잃을 수 있습니다.
`loc_monitor` 의 `RTF`(시뮬레이션 속도)가 0.5 아래로 자주 떨어지면 Gazebo 화면이나 다른 프로그램을 꺼 주세요.

RViz 에서 직접 해보려면 상단 `Nav2 Goal` 버튼을 누르고 지도 위를 클릭-드래그(방향)하면 됩니다.

## 실행 방법 (4단계: 순찰)

```bash
# 터미널 1: 3단계와 같은 시뮬레이션 + Nav2
ros2 launch security_patrol nav.launch.py

# 터미널 2: 순찰 시작 (Ctrl+C 로 중단하면 진행 중인 이동도 취소)
ros2 run security_patrol patrol_node --ros-args -p use_sim_time:=true

# 자동 테스트: 2바퀴 돌고 실제 위치까지 확인 → PASS/FAIL
ros2 run security_patrol patrol_node --ros-args -p use_sim_time:=true -p laps:=2 -p check_truth:=true

# 순찰 상태 보기 (6단계 경보에서 사용할 토픽)
ros2 topic echo /patrol/status
```

순찰 지점은 `config/patrol_route.yaml` 에서 바꿀 수 있습니다.

## 실행 방법 (5단계: 이상 감지)

```bash
# 터미널 1: 시뮬레이션 + Nav2
ros2 launch security_patrol nav.launch.py
# 터미널 2: 이상 감지 (설정: config/anomaly_detector.yaml)
ros2 launch security_patrol detect.launch.py
# 터미널 3: 순찰
ros2 run security_patrol patrol_node --ros-args -p use_sim_time:=true

# 터미널 4: 침입자(사람 발 두 개 모양) 생성 → 터미널 2 에 '!!! 중형 물체 감지' 가 떠야 함
ros2 run ros_gz_sim create -world restricted_area -name intruder1 -x -1.0 -y 0.0 \
  -file $(ros2 pkg prefix security_patrol)/share/security_patrol/models/intruder_person.sdf
# 터미널 5: 침입자를 왕복시킴 → 터미널 2 에 '[긴급] 사건 #1 등급 상승' 이 떠야 함
ros2 run security_patrol intruder_walker
```

경보는 2단계입니다: 긴급(움직이는 대상), 주의(멈춰 있는 새 물체).
라이다 높이에서 다리 두 개 모양이 여러 번 보이면 `사람 후보`로 표시합니다. 탁자 다리도 같은 모양이라 단정하지 않고, 등급도 바꾸지 않습니다.
첫 알림 뒤에 사람 후보가 되면 `사람 후보 추가`로 한 번 더 알립니다. 폭 15 cm 미만의 작은 물체는
순찰 한 바퀴마다 묶어서 알리고, 확인하지 않은 사건은 5분마다 다시 알립니다.
알림은 `/anomaly/alerts` 로 나오고, `alert_notifier` 가 이를 텔레그램으로 보냅니다.
휴대폰에서 확인 버튼을 누르면 `/anomaly/ack` 로 전달되어 재알림이 멈춥니다.

처음 한 번만 설정합니다. 텔레그램 `@BotFather` 에서 `/newbot` 으로 봇을 만들어 토큰을 받은 뒤 실행합니다.
토큰은 저장소 밖 `~/.config/security_patrol/telegram.json` 에 저장되며, 저장소에 넣지 않습니다.

```bash
ros2 run security_patrol telegram_setup
```

설정 파일이 없으면 경고만 내고 전송 없이 실행됩니다. 이때 확인은 직접 보낼 수 있습니다:

```bash
ros2 topic pub --once /anomaly/ack std_msgs/msg/Int32 "{data: 1}"
```

판정 규칙은 ROS 없이 시험할 수 있습니다: `python3 -m unittest discover -s src/security_patrol/test`

### 7단계 통합 시험

시뮬레이션, 감지, 순찰을 켠 뒤 실행하면 `config/eval_scenario.yaml` 대로 4바퀴 동안 물체를 만들고 움직이며
감지 지연·위치 오차·오경보·알림을 기록합니다. 끝나면 결과표를 출력하고 `~/security_patrol_ws/eval_results/` 에 저장합니다.

```bash
ros2 run security_patrol eval_run --ros-args -p use_sim_time:=true
```

## 실행 방법 (8단계: 실제 로봇)

로봇(ESP32)은 휴대폰 핫스팟에 붙어 micro-ROS agent(UDP 8888)로 데이터를 보냅니다.
agent 는 가상머신 안에서 돌기 때문에, 맥이 로봇 패킷을 가상머신으로 넘겨 줍니다(펌웨어·VM 네트워크 변경 없음).

```
로봇 ──UDP──▶ 맥 (핫스팟 주소 = 펌웨어의 agent 주소) ──UDP──▶ VM 192.168.64.2:8888 (micro-ROS agent)
```

1. 휴대폰 핫스팟을 켭니다. ESP32 는 2.4 GHz 만 되므로 iPhone 은 `호환성 최대화`를 켭니다.
2. 맥을 핫스팟에 연결하고 Wi-Fi 의 IPv4 를 수동으로 펌웨어의 agent 주소로 지정합니다.
3. 맥 터미널에서 중계를 켭니다 (ROS 불필요):
   ```bash
   python3 tools/microros_udp_relay.py --vm 192.168.64.2
   ```
4. 가상머신에서 처음 한 번 micro-ROS agent 를 설치합니다: `bash tools/install_microros_agent.sh`
5. 로봇 전원을 켜고 연결을 확인합니다:
   ```bash
   ros2 launch security_patrol real_bringup.launch.py rviz:=true
   ros2 run security_patrol scan_check      # 다른 터미널: scan 약 6 Hz, odom 이 들어오는지
   ```
6. 지도 작성 + 순찰 지점 기록:
   ```bash
   ros2 launch security_patrol real_slam.launch.py
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.6
   ros2 run security_patrol route_recorder  # 순찰 지점에서 Enter
   mkdir -p ~/maps_real && ros2 run nav2_map_server map_saver_cli -f ~/maps_real/real_area
   ```
7. 순찰 + 감지 + 알림 (로봇을 지도 작성 때 출발한 자리·방향에 놓고 시작):
   ```bash
   ros2 launch security_patrol real_nav.launch.py
   ros2 launch security_patrol detect.launch.py use_sim_time:=false relink_seconds:=30.0 route_file:=$HOME/maps_real/patrol_route_real.yaml
   ros2 run security_patrol patrol_node --ros-args -p route_file:=$HOME/maps_real/patrol_route_real.yaml
   ```

실제 공간 지도와 순찰 지점은 저장소 밖 `~/maps_real/` 에 둡니다.
`robot_bridge` 는 ESP32 가 보내는 라이다 조각(`range_segment`)과 엔코더·자이로 값(`encoder_deltas_gyroz_bat_rssi`)을
시뮬레이션과 같은 `/scan`, `/odom`, TF 로 바꿔 주므로 나머지 노드는 그대로 씁니다.

RViz 에서 `Add → By topic → /anomaly/markers` 를 추가하면 감지된 물체가 색으로 보입니다
(소형 노랑 / 중형 주황 / 대형 빨강, 확정 전 후보는 회색).

## 자율 탐사 (로봇이 스스로 지도와 코스 만들기)

사람이 키보드로 몰며 지도를 만들고 지점을 찍던 일을 로봇이 대신합니다.
로봇이 아는 곳과 모르는 곳의 경계(프런티어)를 찾아 스스로 이동하며 지도를 채우고, 경계가 더 없으면
출발점으로 돌아와 지도를 저장한 뒤 그 지도로 순찰 코스까지 만듭니다(`explorer`, `explore_logic.py`).

```bash
# 시뮬레이션: 지도 ~/sentinel_maps/sim/explored.yaml, 코스 ~/sentinel_courses/sim/explored_course.yaml
ros2 launch security_patrol explore.launch.py
# 실제 로봇 (기존 지도를 덮어쓰지 않게 ~/maps_real/explored_area.yaml, patrol_route_explored.yaml 로 저장)
ros2 launch security_patrol explore.launch.py robot:=real
# 진행 상황 보기
ros2 topic echo /explore/status
```

- 목표 고르기: 경계 근처 아는 빈칸 중 벽에서 0.3 m 이상 떨어진 곳, '가깝고 큰 경계'부터. 가는 도중 경계가 이미 보이면 다음 목표로.
- 막힘 방지: 못 간 곳은 다시 고르지 않고, 서 있거나 가 본 자리에서도 안 보이는 경계는 다른 관찰 지점을 한 번 더 시도한 뒤 포기.
- 실제 로봇은 출발점에서 3 m 밖은 가지 않습니다(`max_radius`, 열린 문으로 복도에 나가지 않게). 모드를 바꿀 때 앱 서버가 정지 명령(속도 0)을 보냅니다.
- 탐사 지도의 원점은 탐사를 시작한 로봇 위치입니다. 그 지도로 순찰할 때도 같은 자리·방향에서 시작합니다.

## 관제 앱 (레이더 화면, 수동 주행, 코스 선택·자동 순찰)

휴대폰·노트북 브라우저로 로봇을 보는 웹 앱입니다. 설치가 필요 없고, 가상머신의 `app_server` 노드가 화면 파일과 실시간 데이터를 함께 보냅니다.

- 레이더 화면: 로봇 중심, 앞쪽이 위. 라이다 점, 지도 벽, 감지된 물체(사람 후보는 빨강)를 겹쳐 보여 줍니다.
- 지도 화면: 전체 지도 위 로봇 위치. 손가락·마우스로 끌어서 이동, +/- 로 확대.
- 위쪽: 연결 상태, 배터리, Wi-Fi 세기, 위치추정 상태. 아래쪽: 순찰 상태.
- 수동 주행: 조이스틱(위 = 앞으로, 좌우 = 제자리 회전), 컴퓨터에서는 방향키·WASD, 빨간 `정지` 버튼·스페이스.
  - 손을 떼거나, 앱 명령이 0.5초 끊기거나(Wi-Fi), 조종하던 화면의 연결이 끊기면 바로 멈춥니다.
  - 한 번에 한 화면만 조종합니다. 정지 버튼은 어느 화면에서나 누를 수 있습니다.
  - 차체 앞뒤 15 cm 안에 물체가 보이면 그쪽으로는 못 가고(빨간 구역 표시), 회전과 반대 방향은 됩니다.
  - 순찰(patrol_node)이 켜져 있으면 수동 주행을 받지 않습니다.
  - 최대 속도는 서버에서 정합니다: `max_linear`(기본 0.15 m/s), `max_angular`(기본 1.0 rad/s), `guard_gap`(기본 0.15 m, 0 = 끔).
- 자동 주행: 코스를 고르면 지도에 지점 번호와 경로가 보이고, 바퀴 수(계속 반복 / 1 / 3바퀴)를 골라 `순찰 시작`.
  - 진행 상황(몇 바퀴째, 몇 번째 지점, 막대)과 지금 가는 지점(주황색)이 보입니다. `순찰 정지`는 Ctrl+C 처럼 이동을 취소합니다.
  - `＋ 자동 만들기`: 지금 지도와 로봇 위치로 순찰 지점을 자동으로 뽑습니다(`course_planner.py`). 감시 거리
    (촘촘 1.5 / 보통 2.5 / 넓게 3.5 m)를 고르면 지도에 주황색으로 미리 보이고, 저장하면 목록에 들어갑니다.
    벽에서 35 cm 이상 떨어진 곳 중 라이다로 새로 보이는 공간이 가장 많은 곳부터 고르고, 벽을 돌아가는 실제 이동
    거리로 순서를 정합니다. 저장 위치: 시뮬레이션 `~/sentinel_courses/sim/`, 실제 로봇 `~/maps_real/`.
  - 앱 서버가 `patrol_node` 를 대신 켜고 끕니다. 코스 파일은 시뮬레이션이면 패키지 `config/`, 실제 로봇이면 `~/maps_real/`
    에서 찾습니다(`course_dirs` 파라미터로 바꿀 수 있음). 터미널에서 켠 순찰은 앱에서 멈추지 않고 그대로 둡니다.

명령 하나로 로봇과 앱 서버를 켭니다. 앱 서버가 '모드'를 켜고 끕니다:
처음에는 **순찰 모드**(기준 지도 + AMCL + Nav2 + 이상 감지), 앱의 `지도` 탭에서 `지도 만들기 시작`을 누르면
**지도 만들기 모드**(slam_toolbox + Nav2 + 자율 탐사)로 바뀌고, 탐사가 끝나면 `새 지도로 순찰 모드 시작`.

```bash
# 시뮬레이션
ros2 launch security_patrol app.launch.py
# 실제 로봇 (맥에서 tools/microros_udp_relay.py 를 먼저 켬, 회전 상한 1.8 등 실제 로봇 설정이 자동으로 들어감)
ros2 launch security_patrol app.launch.py robot:=real
# 선택: RViz 도 보기, 이상 감지 끄기, 탐사로 만든 지도로 시작
ros2 launch security_patrol app.launch.py rviz:=true detect:=false
ros2 launch security_patrol app.launch.py map:=$HOME/sentinel_maps/sim/explored.yaml
```

- `지도` 탭: 지금 모드와 지도, 탐사 중에는 알아낸 넓이·가 본 목표·남은 경계와 지도가 자라는 모습(지도 화면), 지금 가는 탐사 목표(주황 과녁).
  탐사 중에는 순찰·수동 주행·이상 감지가 멈춥니다. 탐사를 멈추면 지금까지 만든 지도는 저장되지 않습니다.
- 앱 서버를 끄면 앱이 켠 순찰과 모드(Nav2·감지·탐사)도 함께 꺼집니다.

맥 브라우저에서 `http://192.168.64.2:8080` (가상머신 주소) 을 엽니다.
ROS 없이 화면만 보려면 맥에서 `python3 tools/app_preview.py` 를 실행하고 `http://localhost:8765` 를 엽니다
(가짜 방·로봇·사람, 조이스틱 운전, 가짜 코스 순찰). `--patrol` 을 붙이면 '터미널에서 켠 순찰'을 흉내 냅니다.

## 패키지 구조

```
src/security_patrol/
├── launch/sim_world.launch.py      # Gazebo + 브리지 + RViz 실행
├── launch/slam.launch.py           # 시뮬레이션 + slam_toolbox 지도 작성
├── launch/nav.launch.py            # 시뮬레이션 + AMCL + Nav2 자율주행
├── config/nav2_params.yaml         # Nav2 설정 (소형 스키드 스티어용)
├── config/slam_toolbox_mapping.yaml
├── config/mapping_route.yaml       # 지도 작성 자동 주행 경로
├── config/patrol_route.yaml        # 순찰 지점 목록
├── config/anomaly_detector.yaml    # 이상 감지 설정
├── launch/detect.launch.py         # 이상 감지 실행
├── models/intruder_person.sdf      # 감지 테스트용 침입자 모델
├── maps/                           # 저장된 기준 지도
├── urdf/patrol_bot.urdf.xacro      # 로봇 모델 (실제/시뮬레이션 공용)
├── urdf/patrol_bot.gazebo.xacro    # Gazebo 전용 플러그인 (구동, 라이다)
├── worlds/restricted_area.world    # 8m x 6m 통제 구역
├── rviz/sim_view.rviz, slam_view.rviz
├── security_patrol/scan_check.py   # /scan, /odom 수신 점검 노드
├── security_patrol/motion_test.py  # 자동 주행 테스트 노드
├── security_patrol/auto_mapper.py  # 지도 작성용 웨이포인트 주행 노드
├── security_patrol/nav_test.py     # Nav2 자동 목표 주행 테스트
├── security_patrol/loc_monitor.py  # AMCL 위치추정 점검 노드
├── security_patrol/patrol_node.py  # 반복 순찰 노드
├── security_patrol/anomaly_detector.py  # 지도 비교 이상 감지 노드
├── launch/app.launch.py            # 관제 앱 한 번에 켜기 (로봇 바탕 + 앱 서버, 앱 서버가 모드를 켬)
├── launch/robot_base.launch.py     # 로봇 바탕만 (시뮬레이션 또는 실제 로봇 연결)
├── launch/mode_patrol.launch.py    # 순찰 모드 (map_server + AMCL + Nav2 + 감지)
├── launch/mode_explore.launch.py   # 지도 만들기 모드 (slam_toolbox + Nav2 + explorer)
├── security_patrol/mode_runner.py  # 앱 서버의 모드 켜고 끄기 (ros2 launch 묶음)
├── security_patrol/app_server.py  # 관제 앱 서버 노드 (ROS 데이터 → 브라우저)
├── security_patrol/patrol_runner.py  # 앱에서 patrol_node 켜고 끄기
├── security_patrol/course_planner.py # 지도에서 순찰 코스 자동 만들기
├── launch/explore.launch.py        # 자율 탐사 한 번에 켜기 (로봇 + SLAM + Nav2 + explorer)
├── security_patrol/explorer.py     # 자율 탐사 노드 (끝나면 지도·코스 저장)
├── security_patrol/explore_logic.py # 프런티어 찾기·목표 고르기
├── security_patrol/web_server.py  # 정적 파일 + WebSocket 서버 (표준 라이브러리만)
└── web/                           # 관제 앱 화면 (index.html, app.js, style.css)
```
