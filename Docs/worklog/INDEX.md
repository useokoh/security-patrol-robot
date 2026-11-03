# Worklog 색인

## 현재 상태
- 진행 중: 7단계 통합 시험 첫 실행을 마쳤다. 4바퀴 동안 오경보 0건, 물체 4개 모두 감지, 물체마다 사건 1개 ([evaluation](evaluation.md)). 휴대폰 수신 문구도 결과표와 맞는 것을 확인했다. 결과 기록은 커밋 전이다. VM은 정상 종료했다.
- 다음 할 일: 결과 검토(첫 알림에 사람 후보가 빠진 경우 처리 여부) → 제출 서류와 증빙 캡처 → 8단계 실제 로봇
- 사용자 결정 대기: 다음 단계 선택, 결과 worklog를 GitHub에 올릴지

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
| 통합 시험 | [evaluation](evaluation.md) | 7단계 시나리오, 채점 도구, 첫 결과 | 2026-11-03 |
| 실제 로봇 | [real-robot](real-robot.md) | 8단계 연결 전 점검 결과와 할 일 | 2026-10-27 |
