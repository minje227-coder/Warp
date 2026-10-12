# Warp — Humanoid Box Retrieval (Meta × WARP × RoboWheel)

> **현재 상태 (2026-10-12): 기본 Human Skeleton CSV 수집 스크립트까지 작성 / 실제 Meta Quest 장비 연결은 아직 미검증.**
>
> `scripts/setup.sh`에서 `body,hand,rgb_left,rgb_right`를 모두 선택할 수 있지만, **지금 실행되는 Recorder는 Body/Hand Bone CSV만 기록**합니다. 선택 메뉴가 RGB 녹화 프로그램의 완성을 뜻하지 않습니다. **WARP/RoboWheel Retargeting, Whole-Body Policy, RL은 아직 실행 불가**합니다.

이 저장소는 다음 3단계를 위한 연구 프로젝트입니다.

1. **Dataset Generation:** Meta Quest Human Demonstration → WARP-inspired Base/Waist/Arms Retargeting → RoboWheel-inspired Gripper Mapping → Collision/Grasp 검증
2. **Whole-Body Policy:** Box 위치/Robot State 기반 Grasp-and-Lift Policy 학습
3. **RL Fine-Tuning:** Simulation 기반 충돌 회피·접촉/파지 안정성·Recovery 향상

**현재 첫 번째 마일스톤: Quest에서 Human Body/Hand 데이터를 `body.csv`로 저장하고 정상적으로 재생/검증 가능한지 확인.** 로봇 제어 없이 시작합니다.

## 0. 준비 상태 — 무엇까지 가능한가?

| 구성 | 상태 | 실제 의미 |
|---|---|---|
| Git 프로젝트 및 7개 upstream commit pin | ✅ 작성됨 | 소스는 submodule로 *참조*만 등록; 네트워크 Clone 필요 |
| Linux/macOS/Windows 및 센서 선택 메뉴 | ✅ 로컬 테스트 통과 | `setup.sh`는 선택 내용을 기록하는 단계 |
| Quest Body/Hand PC 수신 | 🟡 코드 작성됨 | `XRT_devices` 설치 + Quest APK + Wi-Fi가 필요. 장비 테스트 전 |
| Human Bone CSV/metadata 저장기 | 🟡 모의 테스트 통과 | `scripts/01_capture/record_body.sh` + Python wrapper 추가 |
| CSV 누락/SEW Coverage 검사 | ✅ 모의 테스트 통과 | 실제 Data 정확도 검증은 아직 |
| 좌/우 Egocentric RGB 동시 녹화 | ❌ 미구현 | Meta Camera API 예제만 등록; 동시 수집/Sync 구현 필요 |
| Object Pose/AprilTag/Camera Calibration | ❌ 미구현 | 수집된 Skeleton과 Box를 동일 World Frame에 두는 과정 |
| Human → Robot WARP / Gripper Retargeting | ❌ 미구현 | 4-DoF Waist + Gripper 맞춤화 필요, 일부 Solver 라이선스 이슈 |
| Simulation Validation / Policy / RL | ❌ 미구현 | 다음 단계 |

**아래 명령은 Git 소스에 접근 가능한 개인/랩 PC에서 실행**합니다. 이 프로젝트 아카이브에는 `.git` 메타데이터가 포함되어 있으나 **외부 저장소 소스 자체는 포함되어 있지 않습니다**.

## 1. 처음 사용하는 PC에서 준비 (OS 선택 가능)

- **Linux:** Git, Python 3.10+ (`venv`, `pip` 포함), Bash. Quest에 APK를 설치할 때 Android Platform Tools(`adb`) 권장.
- **macOS:** Git, Python 3.10+, Bash, `adb`. Meta Quest Developer Hub(MQDH) 사용 가능.
- **Windows:** Git for Windows + **Git Bash**, Python 3.10+, Android Platform Tools(`adb`) 및 필요 시 Oculus ADB Drivers. MQDH 사용 가능. **WSL에서 스크립트를 실행한다면 `--os linux`**를 선택하고 WSL의 네트워크/USB 접근 문제를 별도로 확인.
- **Meta Quest:** Quest 3를 우선 검증 대상으로 권장. XRT-Client의 공식 문서는 Quest 3 지원을 명시하며 Quest Pro 등 다른 기종의 동일 기능은 확인이 필요합니다.

참고: [Meta Developer Mode 및 USB Debugging](https://developers.meta.com/vr/documentation/native/android/mobile-device-setup/) · [Meta Quest Developer Hub](https://developers.meta.com/vr/documentation/unreal/ts-mqdh-getting-started/)

```bash
# 본 프로젝트 폴더에서 실행
bash scripts/setup.sh
# 또는 비대화식 설정 (원하는 OS/Quest/stream/module 선택)
bash scripts/setup.sh --os linux --quest quest3 --streams all --modules capture,video

# 현재 준비 상태 확인
bash scripts/doctor.sh

# 먼저 Meta 수집에 필요한 외부 코드 다운로드 (인터넷 연결 필요)
bash scripts/bootstrap.sh capture

# 프로젝트 전용 Python 가상환경 생성 및 XR/CSV Receiver 설치
bash scripts/01_capture/install_receiver.sh

# 설치 확인 (Quest 연결 테스트 아님)
bash scripts/01_capture/record_body.sh --check
```

Windows Git Bash에서는 `--os windows`, macOS에서는 `--os macos`. 수집을 먼저 할 때는 `capture`만 Clone하면 됩니다. `video`, `retarget`, `gripper`는 필요할 때 추가: `bash scripts/bootstrap.sh video retarget gripper`.

## 2. Quest에 수집 앱 설치하기 (최초 1회)

1. Meta 개발자 계정/팀 및 계정 인증을 준비하고, 휴대폰 **Meta Horizon 앱 → 헤드셋 설정 → Developer Mode**를 활성화.
2. USB-C 데이터 케이블로 Quest와 PC 연결 → 헤드셋에서 **USB Debugging 허용**.
3. [XRT-Client Releases](https://github.com/yunho-c/XRT-Client/releases)에서 배포된 `xrt_client.apk`를 내려받아 설치. 확인된 배포 예: [v0.1.4](https://github.com/yunho-c/XRT-Client/releases/tag/v0.1.4). 최신 버전은 upstream 변경과 호환성을 확인해야 합니다.
4. `adb devices`에 `device`가 표시되는지 확인하고, `adb install -r /path/to/xrt_client.apk` 실행. Windows/macOS에서는 MQDH로 APK 설치 가능.
5. Quest와 Receiver PC를 **동일 LAN/Wi-Fi**에 연결. Guest Wi-Fi의 기기 간 차단 기능이 있다면 연결되지 않을 수 있음. 실험 동안 VPN/방화벽 정책을 점검.

참고: APK 앱이 이미 설치되어 있으면 초기 개발 환경에서 다시 빌드할 필요는 없습니다. Meta Passthrough RGB 수집 코드를 앱에 **추가하는 순간에는 Unity/Android 개발 빌드**가 별도로 필요합니다.

## 3. Body/Hand 첫 Episode 실제 기록

### PC에서 Recorder 시작

```bash
# Quest가 접속할 서버는 PC에서 열립니다.
# 90초 내 첫 Body Pose가 들어오면 그 시점부터 약 20초 기록.
bash scripts/01_capture/record_body.sh \
  --session rack_demo_0001 \
  --port 8080 \
  --wait-seconds 90 \
  --duration 20 \
  --min-frames 30
```

- PC LAN IP를 확인: Linux `hostname -I`, macOS `ipconfig getifaddr en0`(인터페이스 다르면 변경), Windows `ipconfig`.
- Quest에서 **XRT-Client** 실행 → PC LAN IP 및 Port `8080`을 입력 → 연결.
- 랙 앞에서 **몸 숙이기 → 팔 뻗기 → Box 잡기 → 들기 → 내려놓기** 순서의 짧은 Demo를 수행.
- `--duration 0`으로 실행하면 `Ctrl+C`를 누를 때까지 기록하며 파일을 Flush합니다.
- 기본 서버 bind 주소 `0.0.0.0`은 **PC가 수신하는 인터페이스**를 뜻하고 Quest 앱에 입력하는 주소는 아닙니다. Quest에는 PC 실제 사설 IP를 입력하세요.

**출력 파일:**

```text
local_data/raw_human/rack_demo_0001/
  body.csv       # 원시 Skeleton/Bone rows (PC 수신 시간 기준)
  metadata.json  # 시각, frame 수, drop 통계, 수집 완료 여부
```

CSV 기록 시 동일 Session 이름의 기존 폴더는 덮어쓰지 않고 오류를 반환합니다. 원본 RGB/Body/Hand 데이터는 `local_data/` 아래에 두며 Git에서는 제외합니다.

### 결과 검증 (필수)

```bash
bash scripts/01_capture/validate_body.sh \
  local_data/raw_human/rack_demo_0001 \
  --min-frames 30 --strict-sew
```

결과가 `PASS`여도 **CSV 구조 및 필수 Shoulder/Elbow/Wrist 랜드마크가 존재한다는 뜻**이지, 절대 위치 정확도나 Box Pose, RGB 동기화, 로봇에서의 파지 성공을 보장하지는 않습니다. `metadata.json`의 `status: complete`, `recorder_dropped_frames`, 누락 프레임과 Gap 경고를 확인하세요.

**중요:** `body.csv`는 Raw Human Skeleton만 제공합니다. RGB, Box Pose, Robot Joint Action, Ground Truth 접촉/파지 성공은 아직 들어 있지 않습니다. 또한 XRT_devices의 기본 CSV는 **PC 수신 elapsed time**을 기록하며 Quest Camera Exposure Timestamp는 포함하지 않습니다.

## 4. 장비 없이 할 수 있는 자동 테스트

```bash
bash -n scripts/*.sh scripts/01_capture/*.sh
bash scripts/test_capture.sh              # 아래 3개 검사 묶음
# 또는 각각 실행:
bash scripts/test_setup.sh
python3 -m unittest discover -s pipeline/01_dataset/tests -p 'test_*.py' -v
bash scripts/bootstrap.sh --dry-run capture video retarget gripper
bash scripts/doctor.sh
```

- `test_setup.sh`: OS 세 가지·Quest/Stream/Module 설정의 CLI/메뉴 검사.
- `unittest`: **가짜 Skeleton CSV로 Recorder Wrapper 저장 흐름과 Validator를 검증**. Quest/실제 WebRTC/Tracking/Gripper를 테스트하지는 않습니다.
- 현재 실행 환경은 GitHub `git clone` 네트워크에 접근할 수 없으므로 **외부 소스 다운로드, pip 설치, Quest 연결은 수행하지 못했습니다**.

## 5. 실제 수집 성공 후 해야 할 일

- [ ] Quest 첫 CSV에서 Elbow/Wrist/Root 추적 정확도 및 가려짐 상태 평가.
- [ ] Quest 영상까지 사용할지 확정: 현재 `rgb_left/right` 옵션은 **계획용 설정**이고 실제 수집 기능은 없음.
- [ ] Meta Passthrough Camera API와 XRT Unity 앱 통합 → **RGB + Skeleton 공통 Capture Timestamp** 저장.
- [ ] Box 초기 6D Pose 확보 방식(알려진 Rack/Box 좌표, AprilTag, 외부 RGB-D 등)과 Camera–Rack–Quest Frame Calibration 결정.
- [ ] 사람이 Grasp/Lift한 순간의 Event Label과 Tracking Confidence 기록.
- [ ] Human → Robot Retargeting: 7-DoF 양팔, 4-DoF Waist(X/Y/Pitch/Yaw), Base, Parallel-Jaw Gripper TCP.
- [ ] Rack/Box/Self-Collision 및 실제 Grasp/Lift Success 기반 시뮬레이션 데이터 선별.
- [ ] Whole-Body BC/ACT/Diffusion → RL Fine-Tuning 순서로 확장.

구체적인 실행 오류/OS별 차이는 **[수집 상세 가이드](docs/CAPTURE_GUIDE.md)** 와 **[현재 구현 상태](docs/STATUS.md)** 참고.

## 6. Repository 구조와 외부 코드

```text
third_party/
  capture/xrt-client/                # Meta Unity Movement/WebRTC  (submodule)
  capture/xrt-devices/               # XRDevice + CSVRecorder      (submodule)
  capture/meta-passthrough-samples/  # Quest RGB API 샘플         (submodule)
  retarget/rby1-teleop/              # RB-Y1용 SEW 참고; 라이선스 solver 별도
  retarget/stereo-sew/               # SEW IK 수학 및 MATLAB 예제
  retarget/ik-geo/                   # IK-Geo 참고 코드
  gripper/robowheel-toolkits/        # RoboWheel 공개 자료 (완전한 Retargeter 아님)
pipeline/
  01_dataset/record_quest_body.py    # CSV Receiver Wrapper
  01_dataset/validate_csv.py        # CSV 검사 및 SEW coverage
  01_dataset/tests/                 # No-hardware Mock Test
  02_policy/                        # 미구현
  03_rl/                            # 미구현
scripts/
  setup.sh / bootstrap.sh / doctor.sh / test_setup.sh / test_capture.sh
  01_capture/install_receiver.sh    # Python venv에 XRT_devices 설치
  01_capture/record_body.sh         # Body/Hand Recording 실행
  01_capture/validate_body.sh       # OS 공통 CSV 검사 래퍼
  02_convert/ ... 04_rl/            # 예약: 미구현
configs/                            # OS/Quest 선택 (local.env ignored)
robot/                              # 향후 URDF/MJCF, Gripper TCP 정의
local_data/                         # 사람 영상/동작, Git 업로드 금지
```

관련 문서: [Upstream Commit과 경고](docs/UPSTREAMS.md) · [설정 옵션](docs/SETUP_OPTIONS.md) · [설계 결정](docs/DESIGN_DECISIONS.md)

이 프로젝트는 **공식 WARP 전체 구현이 아닙니다.** `rby1_teleop`의 기하학적 c-SEW Solver는 별도 라이선스가 필요하며 공개 MINK Fallback으로 WARP 전체 성능이 재현되지는 않습니다. RoboWheel Toolkit도 논문에 기술된 Hand→Gripper Retargeting 전체 구현이 공개됐다고 확인할 수 없습니다. **Submodule 원 저작권과 라이선스를 유지**하세요.

원문: [WARP](https://arxiv.org/abs/2606.29940) · [WARP 프로젝트](https://warp-retargeting.github.io/) · [RoboWheel](https://arxiv.org/abs/2512.02729)

## 7. Public GitHub 배포 (Warp)

이 프로젝트의 GitHub 저장소 이름은 **Warp**로 설정합니다. 현재 이 아카이브에는 소스와 **초기 Git 커밋**만 있으며 GitHub 원격에 자동 게시된 것은 아닙니다.

```bash
# GitHub CLI 설치와 인증(최초 1회): https://cli.github.com/
gh auth login

# Public repo 생성 + 최초 push (실행 전에 공개 저장소로 올릴 코드인지 확인)
bash scripts/publish_public.sh
```

또는 프로젝트 루트에서 `gh repo create Warp --public --source=. --remote=origin --push`를 실행할 수 있습니다.

공개 저장소에는 **스크립트·문서·외부 소스 Git Submodule 링크만** 포함됩니다. `local_data/`의 Human Motion, RGB 영상, `configs/local.env`, 인증 파일, Robot-specific 비공개 모델은 Git에 넣지 않습니다. `.gitignore`가 무시하더라도 `git add -f`로 강제로 올리지 마세요.

**라이선스 주의:** Public 저장소라는 사실은 third-party 코드를 재배포하거나 WARP의 라이선스가 필요한 Solver를 사용할 권리를 의미하지 않습니다. 별도 Public 라이선스는 이 프로젝트에 설정하지 않았습니다.
