# Capture scripts

- `install_receiver.sh`: pinned XRT_devices[xr,recording]를 프로젝트 venv에 설치.
- `build_quest_rgb.sh`: Android Camera2 보조 앱을 빌드/서명. JDK + API34 SDK/build-tools 필요.
- `prepare_quest_rgb.py`: ADB로 연결된 Quest에 helper APK 설치·권한·local receiver 설정. 생성 token config는 Git 제외.
- `record_body.sh`: **Skeleton + physical Quest RGB 기본 녹화**. `--body-only`로만 RGB 제외.
- `validate_body.sh`: Skeleton, required RGB JPEG/timestamps/MP4 및 metadata 검증.

[전체 설치·실행](../../README.md) · [수집과 시간 기준](../../docs/CAPTURE_GUIDE.md)
