# Integration status — 2026-10-12

| 기능 | 현재 상태 | 검증 범위 |
|---|---|---|
| OS/Quest 설정, source bootstrap | 구현 | Linux/macOS/Windows CLI + 메뉴 tests |
| XRT Skeleton 수집 | 실제 Quest 검증 | 20.02초, 1,420 frames, 필수 SEW coverage 100%, writer drop 0 |
| Physical Quest RGB | Camera2 보조 APK + Python recorder 구현 | right camera 51, 1280×960; 5초 45 JPEG, MP4 45 frames decode |
| 기본 Skeleton + RGB | 실물 동시 episode PASS, missing/stalled RGB fail-closed | Skeleton 1,316 frames(18.42s 유효 시각), RGB 182 frames / 20.17s MP4, writer drop 0, RGB source sequence gaps 2 |
| RGB/skeleton timestamp 보존 | 구현 | 원본 sensor/Quest/PC clock 및 RTT bounds. UNKNOWN source에 거짓 exposure mapping 금지 |
| Hardware exposure sync | 미구현 | PC receive timing은 acquisition sync가 아님 |
| Camera2 calibration metadata | 저장 | raw intrinsics/lens pose; output crop 및 rack/Quest world calibration 미완료 |
| Object pose/contact labels | 미구현 | 추후 구현 |
| WARP/Gripper retargeting | upstream reference만 | robot-specific 구현/검증 필요 |
| Collision / policy / RL | 미구현 | 실제 robot action dataset 필요 |

오프라인 tests: setup, CSV/recorder mock, RGB fail-closed 및 clock transport 7개. Mock JPEG는 transport/clock 테스트용이며 실제 RGB 색/추적 정확도를 검증하지 않습니다.

실물 동시 episode: `rgb_smoke_20261012_191508`, metadata complete, strict SEW coverage 100%, JPEG/timestamp/MP4 검사 PASS. 20초 설정 창에서 body CSV의 유효 시각 범위는 18.42초로 RGB 전체 구간보다 짧습니다. 파일 검사 PASS는 하드웨어 acquisition synchronization 또는 모든 영상 시각의 body pose 존재를 뜻하지 않습니다.

Quest가 서로 다른 앱의 camera client를 퇴출하는 제약 때문에 XRT의 AprilTag용 HEADSET_CAMERA/CAMERA 접근을 비활성화하고 companion이 물리 right camera 51을 소유합니다. BODY_TRACKING 권한은 유지됩니다. `prepare_quest_rgb.py --restore-xrt-camera`로 원래 camera 권한을 복원할 수 있습니다. 이번 episode는 AprilTag/Object Pose를 포함하지 않습니다. [실행 안내](CAPTURE_GUIDE.md)
