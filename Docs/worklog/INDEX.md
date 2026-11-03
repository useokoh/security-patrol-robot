# Worklog 색인

## 현재 상태
- 진행 중: 6단계 경보까지 VM 시험을 통과했다(코드 `86249d4`). 걷는 침입자 하나에 휴대폰 알림이 주의 1번, 긴급 1번만 갔다. 이 결과 worklog는 커밋 전이다.
- 5-3 사람 후보를 구현해 맥 시험 68개와 가짜 라이다 측정을 통과했다. VM 시험과 커밋 전이다 ([detection](detection.md)).
- 다음 할 일: 5-3 VM 시험 → 7단계 통합 시험 → 제출 서류와 증빙 캡처 → 8단계 실제 로봇
- 사용자 결정 대기: 5-3 코드를 GitHub에 올릴지 (6단계 결과 worklog도 함께)

## 주제
| 주제 | 문서 | 한 줄 요약 | 최근 갱신 |
| --- | --- | --- | --- |
| 개발 환경 | [environment](environment.md) | UTM VM, Gazebo Fortress, CPU 렌더링 결정 | 2026-10-27 |
| 시뮬레이션 | [simulation](simulation.md) | 1단계 구역·로봇 모델과 주행 시험 | 2026-10-27 |
| 지도 작성 | [mapping](mapping.md) | 2단계 자동 주행 SLAM 기준 지도 | 2026-10-27 |
| 자율주행 | [navigation](navigation.md) | 3단계 Nav2·AMCL, 위치 착각 해결 과정 | 2026-10-27 |
| 순찰 | [patrol](patrol.md) | 4단계 9개 지점 반복 순찰 | 2026-10-27 |
| 이상 감지 | [detection](detection.md) | 5단계 설계 결정, 5-1·5-2 검증, 5-3 사람 후보 | 2026-11-03 |
| 경보 | [alerts](alerts.md) | 6단계 텔레그램 알림, 확인 버튼, 사건 갈라짐 수정 | 2026-11-03 |
| 실제 로봇 | [real-robot](real-robot.md) | 8단계 연결 전 점검 결과와 할 일 | 2026-10-27 |
