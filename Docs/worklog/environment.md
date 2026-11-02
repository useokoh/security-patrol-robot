# 개발 환경

관련 문서: [simulation](simulation.md), [real-robot](real-robot.md)

## 2026-10-27 저장소와 작업 방식
- 결정과 이유:
  - 공개 GitHub 저장소를 쓰고, 저장소 루트를 colcon 워크스페이스로 쓴다.
  - 커밋 이메일은 익명 noreply 주소를 쓴다. 개인 이메일을 공개하지 않기 위해서다.
  - 커밋과 푸시는 매번 사용자 승인을 받는다.
- 확인 결과: 맥(`~/projects/security-patrol-robot`)에서 수정하고, VM(`~/security_patrol_ws`)에서 `git pull` 후 빌드해 실행한다.
- 문제와 해결: VM에 키보드 입력을 보내면 글자가 깨진다. 명령은 사용자가 붙여 넣어 실행하고, Claude는 화면을 읽어 결과를 확인한다. 비밀번호는 항상 사용자가 직접 입력한다.

## 2026-10-27 Gazebo 렌더링
- 문제와 해결:
  - virgl은 OpenGL 3.3을 지원하지 않아 ogre2가 충돌했다. `MESA_GL_VERSION_OVERRIDE=3.3`을 줘도 똑같이 충돌했다.
  - ogre1은 실행되지만 라이다 빔이 약 5° 아래로 기울어 바닥을 찍는다(반경 1 m 원으로 보임). 쓰지 않는다.
  - 해결: Gazebo에만 `LIBGL_ALWAYS_SOFTWARE=1`을 주고 ogre2를 쓴다(`software_render` 기본 true, `9185d68`).
- 확인 결과: 시작점 (-3, 0)에서 기둥 모서리까지 0.95 m @ +39°로 계산값과 일치했다. scan 약 5~6 Hz.

## 2026-10-27 실행 환경 선택
- 결정과 이유:
  - 맥북 M1(arm64)의 UTM VM에 Ubuntu 22.04 + ROS2 Humble을 쓴다. 별도 Ubuntu PC가 없기 때문이다.
  - Gazebo Classic은 ARM64 패키지가 없어 Gazebo Fortress(ros_gz)를 쓴다.
