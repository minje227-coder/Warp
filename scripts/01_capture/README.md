# Quest 수집

현재 수집 경로는 **Quest raw v3**입니다. 볼륨 업으로 시작·종료하고 원본은 PC로 비동기 전송합니다.
설치·데이터 형식·수신기·전처리 worker는 [raw v3 안내](../../docs/RAW_CAPTURE.md)를 따릅니다.

| 경로 | 역할 |
|---|---|
| `systemd/` | lab6 원본 수신·전처리 user service |
| `build_unified_quest.py` | 통합 Quest 앱 빌드 |
| `smoke_camera/` | 별도 카메라 probe 도구 |
| `record_body.sh`, `validate_body.sh` | 이전 수집·검증 진입점 |
| `build_quest_rgb.sh`, `prepare_quest_rgb.py` | 이전 Camera2 companion 앱 |

이전 방식의 사용법은 [통합 수집 기록](../../docs/UNIFIED_CAPTURE.md)과
[companion 안내](../../docs/LEGACY_COMPANION.md)에 보존되어 있습니다.
[프로젝트 시작 안내](../../README.md)
