# Warp — Meta Quest × WARP × RoboWheel

Meta Quest의 Human Skeleton과 **실제 전면 RGB 카메라 영상**을 모으고, 이후 로봇 retargeting → Whole-Body Policy → RL로 확장하는 프로젝트입니다.

## 기본 수집: Skeleton + Quest RGB

`record_body.sh`는 이제 **Skeleton CSV + Quest 오른쪽 전면 RGB**를 기본으로 기록합니다. RGB가 준비되지 않거나 중간에 멈추면 성공으로 처리하지 않습니다. Skeleton만 수집하려면 `--body-only`를 명시하세요.

- Skeleton: 공식 XRT-Client APK → 고정 commit의 XRT_devices WebRTC receiver.
- RGB: 이 저장소의 **Warp Quest RGB** 보조 앱 → 공식 Android Camera2/Passthrough API → lab PC.
- 한 episode에 한쪽 카메라를 기록합니다. 기본은 **right**: 기존 XRT 앱의 왼쪽 카메라/AprilTag 처리와 분리됩니다.
- RGB는 VR 화면 캡처가 아닙니다. UI 없는 물리 카메라 JPEG와 MP4를 저장합니다.
- RGB 타임스탬프는 원본 sensor clock과 PC receive clock을 모두 보존합니다. **하드웨어 동기화는 구현되지 않았습니다.**

## 설치와 첫 실행

PC에는 Git, Python 3.10+, Bash가 필요합니다. Windows는 Git Bash 또는 WSL을 사용합니다.

```bash
bash scripts/setup.sh --os linux --quest quest3 --streams body,hand,rgb_right --modules capture
bash scripts/bootstrap.sh capture
bash scripts/01_capture/install_receiver.sh
```

Quest에 [XRT-Client APK](https://github.com/yunho-c/XRT-Client/releases)를 설치하고 개발자 모드·USB 디버깅을 허용하세요. Quest 3에서 실측 검증하며, 3S 등 다른 모델은 별도 확인이 필요합니다.

RGB 보조 앱 빌드에는 JDK 및 Android API 34/build-tools 34.0.0이 필요합니다. `WARP_ANDROID_SDK`로 SDK 위치를 지정할 수 있습니다. 개인 debug signing key와 APK는 `.venv/android-build/`에만 저장합니다.

```bash
bash scripts/01_capture/build_quest_rgb.sh
python3 scripts/01_capture/prepare_quest_rgb.py \
  --serial YOUR_QUEST_SERIAL --receiver-host RECEIVER_LAN_IP \
  --apk .venv/android-build/warp-quest-rgb.apk
```

이 준비 명령은 **Quest가 ADB로 연결된 컴퓨터**에서 실행합니다. USB 연결 PC와 수신 PC가 다르면 생성된 `configs/quest_rgb.json`을 수신 PC의 동일 경로에 복사하세요. 이 파일은 private token을 포함하며 Git에서 제외됩니다.

준비 명령은 Quest의 XRT AprilTag 카메라 접근을 비활성화하고 원래 권한을 private config에 저장합니다. 두 앱이 동시에 카메라를 열면 Quest가 보조 카메라를 퇴출하기 때문입니다. **Body/Hand 권한은 유지**됩니다. 이후 Quest의 **Warp Quest RGB** 앱을 열어 보조 카메라를 시작하고 XRT-Client로 돌아옵니다. 보조 앱은 2분 동안 PC 요청이 없으면 자동 종료됩니다. 초기 설정은 앱 내부에 보존됩니다.

```bash
# RGB source와 Python 의존성 확인; 아직 Skeleton 연결 검사가 아닙니다.
bash scripts/01_capture/record_body.sh --check

# 첫 valid pose부터 20초 녹화
bash scripts/01_capture/record_body.sh --session demo_0001 --wait-seconds 300 --duration 20
```

수신기가 켜진 후 XRT-Client의 **IP Address Panel 맨 위 칸**에 PC LAN IP만 입력·적용하고 **가운데 전원 버튼**을 누르세요. URL은 `http://PC_IP:8080/offer`입니다. `Streaming Connection`은 별도 영상 수신 메뉴입니다.

```bash
bash scripts/01_capture/validate_body.sh local_data/raw_human/demo_0001 --strict-sew
# RGB 없이 진행할 때만 명시:
bash scripts/01_capture/record_body.sh --body-only --session pose_only_0001 --duration 20
```

### XRT 카메라 복원

AprilTag 카메라가 다시 필요하면 USB 연결 PC에서 복원합니다. 이 경우 RGB companion은 종료되며 동시 모드는 사용할 수 없습니다.

```bash
python3 scripts/01_capture/prepare_quest_rgb.py \
  --serial YOUR_QUEST_SERIAL --receiver-host RECEIVER_LAN_IP --restore-xrt-camera
```

## 저장 파일

```text
local_data/raw_human/demo_0001/
  body.csv
  metadata.json
  rgb/
    frame_000000.jpg ...     # raw physical camera JPEGs
    video.mp4               # capture interval PTS, verified by decoding
    timestamps.csv          # original sensor/Quest/PC clocks and RTT bounds
    calibration.json        # Camera2 source/eye/size/intrinsics/pose metadata
```

`metadata.json.status=complete`와 episode 검사 PASS를 함께 확인하세요. MP4의 모든 프레임이 다시 디코딩되는지도 검사합니다. CSV의 SEW coverage는 랜드마크 존재율이며 추적 정확도나 로봇 파지 성공을 뜻하지 않습니다.

### 시간과 calibration 한계

Skeleton CSV는 **PC pose-packet receive time** 기준입니다. `body_time_origin_pc_monotonic_ns`로 PC 단조시계에 연결됩니다. RGB는 Camera2 원본 `sensor_timestamp_ns`와 HTTP request/receive 시각을 보존합니다.

`timestamp_source=REALTIME(1)`이면 Quest elapsed clock과 PC clock의 HTTP midpoint offset 및 RTT/2 불확실성으로 capture time을 추정합니다. `UNKNOWN(0)`이면 sensor clock을 PC exposure 시각으로 바꾸지 않고 해당 추정 칸을 비웁니다. 이 경우 **PC 수신 시각 기준 비교만 가능**하며 exposure 동기화는 추가 개발이 필요합니다.

Camera2 intrinsics/extrinsics는 raw metadata입니다. Crop/resize 보정 또는 Quest tracking ↔ rack 좌표계 calibration이 완료됐다고 주장하지 않습니다.

## 검증 및 구현 상태

```bash
PATH="$PWD/.venv/bin:$PATH" bash scripts/test_capture.sh
```

- 실제 Quest Skeleton: 2026-10-12, 20.02초 / 1,420 frames, 필수 SEW coverage 100%, writer drop 0.
- 실제 Quest Camera2 RGB: right camera 51, 1280×960, JPEG/MP4 수신·디코딩 확인.
- 실제 RGB/Skeleton 동시 episode: Skeleton 1,316 frames / 18.42초의 유효 body 시각, RGB 182 frames / 20.17초 MP4; episode validation PASS. 자세한 범위는 [STATUS](docs/STATUS.md).
- 오프라인 테스트는 기본 RGB 필수 동작, 누락 시 fail-closed, sensor clock 보존 및 UNKNOWN clock의 잘못된 정렬 방지를 확인합니다.

## 이후 연구 단계

1. Dataset: Quest RGB/Skeleton → object pose/좌표계 calibration → WARP-inspired base/waist/arms + RoboWheel-inspired gripper mapping → collision/grasp 검증.
2. Whole-Body Policy: validated robot observation/action으로 BC/ACT/Diffusion/Flow Matching 학습.
3. RL Fine-Tuning: simulation에서 충돌 회피·접촉/파지·recovery 개선.

Retargeting, robot action labels, collision validation, policy 및 RL은 아직 미구현입니다. 이 프로젝트는 공식 WARP 전체 구현이 아닙니다. 외부 코드는 `third_party/`의 7개 pinned Git submodule로 참조하며 원 라이선스를 유지합니다.

[Upstreams](docs/UPSTREAMS.md) · [수집 안내](docs/CAPTURE_GUIDE.md) · [설정](docs/SETUP_OPTIONS.md) · [WARP](https://warp-retargeting.github.io/) · [RoboWheel](https://arxiv.org/abs/2512.02729) · [Meta Camera2 API](https://developers.meta.com/vr/documentation/unity/unity-pca-overview/)

GitHub: [minje227-coder/Warp](https://github.com/minje227-coder/Warp). Raw human data, 영상, token, signing key, APK 및 비공개 robot model은 공개 저장소에 넣지 않습니다. 프로젝트 자체의 별도 공개 라이선스는 아직 설정하지 않았습니다.
