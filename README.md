# Warp

Quest에서 사람의 움직임과 실제 전면 RGB를 수집하고 로봇 retargeting과 학습으로 확장하는 프로젝트입니다.

## 통합 수집

기존 XRT-Client에 RGB를 추가한 한 앱에서 **RGB + Skeleton + AprilTag**를 기록합니다.
한 카메라 프레임을 RGB 저장·AprilTag 검출에 공유하고, Skeleton을 그 프레임의 원본 타임스탬프에 정렬합니다.

Quest 안에서는 **볼륨 업 한 번으로 시작, 다시 한 번으로 종료**합니다. 화면 녹화 버튼은 없습니다.
작은 영어 상태 표시만 `Preparing…` → `Ready` → `Recording · 00:23` → `Saving…` → `Saved`로 바뀝니다.
녹화 중 메뉴·Skeleton 구체·AprilTag 축을 숨깁니다. 서버가 저장 검증을 끝내야 `Saved`가 표시됩니다.

현재 통합 버전은 **소스 구현 및 실제 WebRTC를 사용한 소프트웨어 테스트 단계**입니다.
Unity APK 빌드·Quest 설치·볼륨 키·물리 영상 정렬 검증은 아직 완료되지 않았습니다.
이전 companion 방식의 실물 녹화 성공을 통합 버전의 검증으로 간주하지 않습니다.

```bash
bash scripts/bootstrap.sh capture
bash scripts/01_capture/install_receiver.sh
# 통합 XRT APK 설치 후 lab6에서:
bash scripts/01_capture/record_body.sh --session take_0001 --rgb-fps 10 --wait-seconds 300
# Quest: volume up → start, volume up → stop
bash scripts/01_capture/validate_body.sh local_data/raw_human/take_0001 --strict-sew
```

공통 시간은 Quest 카메라 SDK의 UTC ticks입니다. Skeleton의 원래 provider 조회 시각을 함께 보존하고,
앞뒤 샘플 사이에서 보간하며 외삽하지 않습니다. 카메라와 Skeleton 센서의 하드웨어 동시 노출을 뜻하지 않습니다.

설치·권한 복원·실제 tag 크기·APK 서명 이전 및 데이터 형식은 [통합 수집 안내](docs/UNIFIED_CAPTURE.md)를 참고하세요.
[이전 실물 결과](docs/STATUS.md)와 [기존 companion 안내](docs/LEGACY_COMPANION.md)도 보존합니다.
`--body-only`는 기존 Skeleton 전용 수집기를 명시적으로 사용합니다.

## 이후 파이프라인

- Dataset: 좌표계·물체 pose calibration, base/waist/arms 및 gripper retargeting, collision/grasp 검증.
- Policy: Whole-Body policy 학습.
- RL: fine-tuning 및 로봇 검증.

현재 캡처 파일 검증은 로봇 grasp 성공이나 학습 데이터 적합성의 증명이 아닙니다.
[Upstreams](docs/UPSTREAMS.md), [설정 선택](docs/SETUP_OPTIONS.md), [상태](docs/STATUS.md)를 참고하세요.
