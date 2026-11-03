#!/usr/bin/env bash
# 가상머신(Ubuntu 22.04 ARM64 + ROS2 Humble)에 micro-ROS agent 를 설치한다. 8단계에서 한 번만 실행.
#   bash ~/security_patrol_ws/tools/install_microros_agent.sh
# 1) 이미 있으면 끝  2) apt 패키지가 있으면 설치  3) 없으면 소스로 빌드 (~/microros_ws, 10~20분)
# sudo 비밀번호는 직접 입력한다.
set -e
source /opt/ros/humble/setup.bash

if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
    echo "[OK] micro_ros_agent 가 이미 설치되어 있습니다: $(ros2 pkg prefix micro_ros_agent)"
    exit 0
fi

sudo apt update
if apt-cache show ros-humble-micro-ros-agent >/dev/null 2>&1; then
    echo "[1/1] apt 패키지로 설치합니다"
    sudo apt install -y ros-humble-micro-ros-agent
    echo "[OK] 설치 완료. 새 터미널에서: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
    exit 0
fi

echo "[1/4] apt 패키지가 없어 소스로 빌드합니다 (~/microros_ws)"
sudo apt install -y python3-rosdep python3-colcon-common-extensions git
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

WS="$HOME/microros_ws"
mkdir -p "$WS/src"
cd "$WS"
if [ ! -d src/micro_ros_setup ]; then
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

echo "[2/4] micro_ros_setup 빌드"
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

echo "[3/4] agent 소스 받기"
ros2 run micro_ros_setup create_agent_ws.sh

echo "[4/4] agent 빌드"
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# 새 터미널에서도 agent 를 찾도록 등록 (이미 있으면 다시 넣지 않음)
LINE="source $WS/install/local_setup.bash"
grep -qxF "$LINE" "$HOME/.bashrc" || echo "$LINE" >> "$HOME/.bashrc"

ros2 pkg prefix micro_ros_agent
echo "[OK] 설치 완료. 새 터미널을 열고: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
