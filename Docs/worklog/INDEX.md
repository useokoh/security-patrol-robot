# Worklog 색인

## 현재 상태
- 진행 중: 6단계 경보. 텔레그램 알림 코드를 만들어 맥 단위 시험 40개를 통과했고, 실제 텔레그램과 VM 시험은 아직이다. 커밋 전이다. 5-2는 VM 시험 3개를 통과했다(코드 `15224b5`).
- 다음 할 일: 6단계 커밋 → 봇 생성과 `telegram_setup` → 6단계 VM 시험 → 5-3 사람 후보 → 7단계 통합 시험 → 8단계 실제 로봇
- 사용자 결정 대기: 6단계 코드를 GitHub에 올릴지 (VM 시험에 필요, 5-2 시험 결과 worklog도 함께 커밋)

## 주제
| 주제 | 문서 | 한 줄 요약 | 최근 갱신 |
| --- | --- | --- | --- |
| 개발 환경 | [environment](environment.md) | UTM VM, Gazebo Fortress, CPU 렌더링 결정 | 2026-10-27 |
| 시뮬레이션 | [simulation](simulation.md) | 1단계 구역·로봇 모델과 주행 시험 | 2026-10-27 |
| 지도 작성 | [mapping](mapping.md) | 2단계 자동 주행 SLAM 기준 지도 | 2026-10-27 |
| 자율주행 | [navigation](navigation.md) | 3단계 Nav2·AMCL, 위치 착각 해결 과정 | 2026-10-27 |
| 순찰 | [patrol](patrol.md) | 4단계 9개 지점 반복 순찰 | 2026-10-27 |
| 이상 감지 | [detection](detection.md) | 5단계 설계 결정, 5-1·5-2 검증 | 2026-11-02 |
| 경보 | [alerts](alerts.md) | 6단계 텔레그램 알림과 확인 버튼 | 2026-11-02 |
| 실제 로봇 | [real-robot](real-robot.md) | 8단계 연결 전 점검 결과와 할 일 | 2026-10-27 |
