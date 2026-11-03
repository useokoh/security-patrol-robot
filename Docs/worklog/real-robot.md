# 실제 로봇 (8단계)

관련 문서: [environment](environment.md), [navigation](navigation.md), [alerts](alerts.md)

## 2026-11-03 실제 로봇 연결 준비 (VM 미실행, 로봇 미연결)
- 한 일: 실제 로봇을 시뮬레이션과 같은 토픽으로 쓰기 위한 코드와 실행 파일을 만들었다.
  - `robot_bridge`: ESP32 라이다 조각 4개 → `/scan`, 엔코더·자이로 → `/odom`·TF·`/joint_states`. 계산은 `robot_bridge_logic.py`.
  - `tools/microros_udp_relay.py`: 맥에서 로봇 UDP 를 VM agent 로 넘기는 중계(표준 라이브러리만).
  - `tools/install_microros_agent.sh`: VM 에 agent 설치(apt 패키지가 없으면 소스 빌드).
  - `real_bringup` / `real_slam` / `real_nav` launch, `nav2_params_real.yaml`, `route_recorder`(순찰 지점 기록).
  - `detect.launch.py` 에 `use_sim_time`, `route_file` 인자 추가. `alert_notifier` 가 `route_file` 을 받는다.
- 결정과 이유:
  - 사용자가 네트워크 '가'(펌웨어 그대로, 맥이 agent 주소를 맡고 중계)와 범위 A(감지·휴대폰 알림까지)를 골랐다. 목표는 시연 영상(마감 다음 날).
  - 로봇에 올라간 펌웨어 사본(강의 자료의 publish_encoder_gyroz_bat_rssi_lidar_stamped_stable)을 읽어 형식을 맞췄다. 강의 PC 쪽 코드처럼 메시지 시각은 PC 시계를 쓰고, ESP32 시각은 간격 계산에만 쓴다.
  - 바퀴 값은 펌웨어 기준(기어비 298, 반지름 0.0325 m). URDF 의 0.0337 m 는 모양용이라 odom 계산에는 쓰지 않는다.
  - 스캔 시각은 한 바퀴 가운데로 당겼다(0.083초). 다 모인 시각을 쓰면 회전 중 약 6° 어긋난다.
  - AMCL 은 Differential 모델, odom 오차는 기본값 0.2. 시작 위치는 지도 원점(지도를 만들 때 출발한 자리).
  - 실제 공간 지도와 순찰 지점은 공개 저장소에 넣지 않도록 `~/maps_real/` 에 둔다.
- 확인 결과: 맥 단위 시험 103개 통과(pyyaml 환경에서 경로 파일 시험 3개 추가 통과). 가짜 ROS 로 `robot_bridge` 를 돌려 스캔 720점, odom·TF·바퀴 각도·배터리 값 확인. 중계기는 이 컴퓨터 안에서 왕복·재연결·agent 꺼짐을 시험했다.
- 미확인(실제 로봇에서 볼 것): 라이다 방향(앞에 물체를 두고 `scan_check` 의 가장 가까운 각도가 0° 근처인지), 실제 이동 거리와 odom 비율, 핫스팟에서 중계 지연, agent apt 패키지 존재 여부, 감지 기준값.

## 2026-10-27 연결은 8단계에서 하기로 결정
- 결정과 이유: 시뮬레이션으로 3~7단계를 먼저 완성한 뒤 한 번에 연결한다.

## 2026-10-27 연결 전 점검 (읽기 전용, 펌웨어 변경 없음)
- 확인 결과:
  - ESP32는 USB 시리얼로 정상 응답하지만 "Wi-Fi 연결 재시도 중..."을 반복한다.
  - 펌웨어의 WiFi 이름과 agent 주소가 휴대폰 핫스팟 기준으로 고정돼 있어, 집 공유기에 연결된 맥과 네트워크가 다르다.
  - VM은 UTM 공유 NAT라 로봇이 VM의 micro-ROS agent에 직접 닿을 수 없다.
- 다음 할 일 (각각 사용자 승인 필요):
  - 펌웨어 WiFi 설정 수정과 재업로드. 비밀번호는 사용자가 입력하고 공개 저장소에 올리지 않는다.
  - VM 네트워크를 브리지 모드로 변경.
  - 실제 로봇용 Nav2 설정 파일 작성(Differential 모델, RViz로 초기 위치 지정, `check_truth:=false`).
  - 실제 공간에서 지도를 만들고 2~7단계 재검증. 시뮬레이션에서 맞춘 감지 기준값은 다시 맞춰야 할 수 있다.
