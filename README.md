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
- [ ] 3단계: Nav2 자율주행
- [ ] 4단계: 순찰 노드
- [ ] 5단계: 이상 감지 노드
- [ ] 6단계: 경보 노드
- [ ] 7단계: 시뮬레이션 통합 시험
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

## 패키지 구조

```
src/security_patrol/
├── launch/sim_world.launch.py      # Gazebo + 브리지 + RViz 실행
├── launch/slam.launch.py           # 시뮬레이션 + slam_toolbox 지도 작성
├── launch/nav.launch.py            # 시뮬레이션 + AMCL + Nav2 자율주행
├── config/nav2_params.yaml         # Nav2 설정 (소형 스키드 스티어용)
├── config/slam_toolbox_mapping.yaml
├── config/mapping_route.yaml       # 지도 작성 자동 주행 경로
├── maps/                           # 저장된 기준 지도
├── urdf/patrol_bot.urdf.xacro      # 로봇 모델 (실제/시뮬레이션 공용)
├── urdf/patrol_bot.gazebo.xacro    # Gazebo 전용 플러그인 (구동, 라이다)
├── worlds/restricted_area.world    # 8m x 6m 통제 구역
├── rviz/sim_view.rviz, slam_view.rviz
├── security_patrol/scan_check.py   # /scan, /odom 수신 점검 노드
├── security_patrol/motion_test.py  # 자동 주행 테스트 노드
├── security_patrol/auto_mapper.py  # 지도 작성용 웨이포인트 주행 노드
├── security_patrol/nav_test.py     # Nav2 자동 목표 주행 테스트
└── security_patrol/loc_monitor.py  # AMCL 위치추정 점검 노드
```
