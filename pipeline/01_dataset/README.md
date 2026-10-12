# Dataset stage

`record_quest_body.py`는 기본으로 XRT Skeleton CSV와 physical Quest Camera2 RGB를 같은 episode에 수집합니다. `quest_rgb.py`가 JPEG, 촬영/수신 clock sidecar, calibration metadata와 MP4를 기록합니다. RGB를 제외하려면 `--body-only`가 필요합니다.

`quest_rgb_android/`는 오른쪽 전면 카메라를 사용하는 Android foreground service입니다. 카메라 source와 eye는 Meta vendor tags로 식별하며 rendered VR view를 녹화하지 않습니다. Companion 모드에서는 XRT AprilTag 카메라 접근을 끄고 BODY_TRACKING을 유지합니다. 원래 카메라 권한은 private config에 저장되며 복원 명령을 제공합니다.

`validate_episode.py`는 CSV 및 필요한 RGB artifact를 함께 검사합니다. 원본 timestamps를 보존하며 UNKNOWN sensor clock에는 PC exposure time을 만들어 넣지 않습니다. Hardware RGB/skeleton synchronization은 아직 미구현입니다.

향후 작업: precise acquisition sync, output intrinsics/crop 보정, Quest/rack calibration, object/contact label, robot retargeting, collision validation, LeRobot conversion.

[실행 안내](../../docs/CAPTURE_GUIDE.md)
