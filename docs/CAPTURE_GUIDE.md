# Meta Quest Human Skeleton 데이터 수집 — 실행/테스트 가이드

> 2026-10-10 현황: Body/Hand Skeleton CSV Recording Script **작성됨, 실제 Quest 미검증**. RGB 동시 수집, Object Pose, WARP Retargeting **미구현**.

## A. 공통 사전 조건

- Meta Quest 3 권장(XRT-Client 업스트림이 지원 명시). Quest 3S, Pro, 2는 해당 앱의 Body Tracking 동작 여부부터 확인.
- Meta 개발자 계정 인증, 개발자 모드, USB Debugging 허용. `adb devices`에 `device` 확인.
- 수집 PC: Linux/macOS/Windows Git Bash 중 하나 + Git, Python 3.10 이상, Bash, 네트워크 접근. Windows에는 Oculus ADB Driver가 필요할 수 있음.
- Quest/PC 동일 LAN의 기기 간 접속 허용. `--host 0.0.0.0`는 모든 인터페이스에 바인딩되므로 신뢰하는 실험실 LAN에서만 사용하고 인터넷에 Port Forwarding하지 말 것. PC IP는 VPN/가상 네트워크 주소가 아닌 Quest에서 접근 가능한 LAN IP.
- APK는 [XRT-Client 공식 Releases](https://github.com/yunho-c/XRT-Client/releases)에서 가져옴(확인한 v0.1.4의 APK 배포는 약 239MB). 이 저장소의 ZIP에는 APK 없음.

## B. 설치 절차 — PC

```bash
bash scripts/setup.sh                  # OS, Quest, stream, modules 지정
bash scripts/bootstrap.sh capture       # 외부 GitHub 코드 Clone
bash scripts/01_capture/install_receiver.sh
bash scripts/01_capture/record_body.sh --check
bash scripts/doctor.sh
```

`--check` 성공 = **Python Import 성공**. Quest 연결 성공이 아님.

`install_receiver.sh`는 프로젝트 전용 `.venv`를 생성하고, 고정 upstream의 `xrt-devices[xr,recording]`을 Editable Install합니다. 의존성을 내려받으므로 네트워크가 필요합니다. 시스템 Python 대신 `.venv`의 Python을 사용합니다.

| OS | 추가로 확인할 내용 |
|---|---|
| Linux | `python3-venv`, Python 3.10+, Git, `adb` Platform Tools. Meta Quest Developer Hub는 Windows/macOS 중심이므로 Linux에서는 `adb install` 사용. |
| macOS | Python 3.10+, Git, Android Platform Tools, MQDH 또는 `adb`. Wi-Fi Interface가 `en0`가 아닐 수 있음. |
| Windows Git Bash | Python 3.10+가 PATH에서 동작해야 함. Android Platform Tools, 필요 시 Oculus ADB Drivers. WSL은 선택값을 Linux로 입력해야 함. |

## C. APK 설치 절차 — Quest

1. 휴대폰 Meta Horizon 앱에서 헤드셋과 페어링, 개발자 모드 활성화.
2. USB-C 데이터 케이블 연결 후 헤드셋 안에서 USB Debugging 요청 허용.
3. PC에서 `adb devices` → `device`인지 확인. `unauthorized`라면 헤드셋 내부 승인 재확인.
4. 공식 Release의 `xrt_client.apk` 파일을 준비하고 `adb install -r /ABS/PATH/xrt_client.apk` 실행(Windows/macOS에서는 MQDH로 설치 가능).
5. Quest Library의 Unknown Sources에서 XRT-Client 실행.

[Meta 공식 Developer Mode](https://developers.meta.com/vr/documentation/native/android/mobile-device-setup/) · [ADB 설치](https://developers.meta.com/vr/documentation/unity/unity-env-device-setup/)

## D. 첫 Smoke Test

1. Quest에서 볼 수 있는 PC LAN IP 확인. Linux `hostname -I`, macOS `ipconfig getifaddr en0`, Windows `ipconfig`. 어느 주소를 사용할지 애매하면 PC 라우팅/네트워크를 점검.
2. PC 터미널에서 실행:

   ```bash
   bash scripts/01_capture/record_body.sh --session smoke_0001 --wait-seconds 90 --duration 20
   ```

3. Quest XRT-Client에 **PC LAN IP와 8080** 입력 후 연결. 먼저 PC Recorder를 실행해야 함.
4. 사람 손을 뻗고 Elbow, Wrist, Torso 자세를 바꿔가며 20초 촬영. 안전한 자세로, 우선 Box를 들지 않아도 됨.
5. 20초 후 종료되면 `local_data/raw_human/smoke_0001/body.csv`, `metadata.json` 확인.
6. 다음 명령으로 Skeleton Frame/필수 랜드마크 기록 여부 확인:

   ```bash
   bash scripts/01_capture/validate_body.sh local_data/raw_human/smoke_0001 --min-frames 30 --strict-sew
   ```

7. `PASS`, `metadata.json.status=complete`, 드롭 프레임 및 SEW Landmark Coverage 확인. 좌/우 손이 보이지 않는 구간을 별도 측정하면 이후 Retargeting 품질 판단에 도움.

**예상 CSV:** `time_elapsed,data_type,id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w`. `bone` Row는 관절당 하나이며 동일 timestamp의 여러 Row가 하나의 Skeleton Frame에 해당. `time_elapsed`는 PC가 Pose Packet을 받은 단조시계 기준 경과 시간으로 카메라 프레임 노출 시각과 다름.

**주의:** `metadata.json`의 `observed_unique_pose_sequences`는 `XRDevice.get_frame`에 의해 처리된 Snapshot 수, `recorded_skeleton_frames`는 CSV에서 실제 읽힌 Bone Frame 수. 전자가 후자보다 적을 수도 있으므로 둘 다 확인. `--session` 이름은 덮어쓰기 방지를 위해 재사용 불가.

## E. 자주 발생하는 문제

| 증상 | 점검 및 해결 |
|---|---|
| `xrt_devices` import 실패 | `bash scripts/bootstrap.sh capture`와 `bash scripts/01_capture/install_receiver.sh` 완료 여부, `.venv` 확인 |
| `Submodule` 앞에 `-`가 표시됨 | 외부 소스 미다운로드 상태. 인터넷 연결 후 `bootstrap.sh capture` 실행 |
| `No valid pose within wait timeout` | Quest 앱 실행 상태, PC LAN IP, 포트 8080, 동일 Wi-Fi, AP 격리 및 VPN 확인. 앱의 Streaming이 시작됐는지도 점검 |
| Port already in use | 다른 Receiver 종료하거나 `--port 8081` 등으로 변경하고 Quest도 같은 포트로 설정 |
| `adb devices`가 `unauthorized` | Quest 내부의 USB Debugging 확인 및 승인. Windows 드라이버 확인 |
| Body CSV 비었음 | 앱/권한, `body_pose` 채널, 연결·Tracking 오류를 확인. `metadata.json`에 실패 상태가 남음 |
| `--strict-sew` 실패 | 해당 관절 Bone ID가 누락된 Frame이 많음. Tracking / SDK 버전 / Schema 차이 확인. 위험한 데이터를 무조건 통과시키지 말 것 |
| 많은 drop/gap 발생 | LAN 품질, Wi-Fi 혼잡, PC CPU/디스크 부하 점검. CSVRecorder 큐 초과도 기록됨 |
| Quest RGB 파일이 없음 | **정상:** 아직 RGB Camera API 통합 전. `setup.sh --streams all`은 기능 활성화가 아님 |
| Windows PowerShell에서 `.sh` 실행이 안 됨 | Git Bash에서 `bash scripts/...` 사용. WSL은 Linux 환경으로 재설정 |

## F. 실제 장비 없이 실행할 테스트

```bash
bash scripts/test_capture.sh
# 포함 항목: setup CLI, Python Mock Recorder/CSV, bootstrap --dry-run
```

이 테스트에는 **가짜 XRTDevice/CSVRecorder와 합성 Bone Row**만 사용. Quest APK, 실제 Body Tracking, 비디오, PC–Quest 네트워크를 검증하는 테스트가 아님.

## G. 수집 후 아직 필요한 구현

- Quest Passthrough Camera API와 Movement SDK 동시 실행, RGB 프레임과 Body/Hand Skeleton에 공통 또는 보정 가능한 Capture Timestamp 부여.
- Headset Tracking 좌표계 ↔ Rack World 좌표계 Calibration.
- Object Pose/Grasp Contact Event 및 Occlusion 발생 시 품질 Flags.
- WARP-inspired Whole-Body Retargeting: 14-DoF Arms + 4-DoF Waist + Mobile Base.
- RoboWheel-inspired Hand→Parallel-Jaw Gripper Mapping과 TCP 제약 보정.
- Sim Replay, Allowed-Contact-aware Collision Checking 및 학습용 LeRobot Conversion.

## H. 개인정보/보존

Human Body Pose 및 Egocentric 영상에는 사람과 작업 환경 정보가 포함될 수 있음. 저장 경로는 **Git Ignore된 `local_data/`**, 향후 업로드 전에 촬영 동의·접근 권한·보존 기간을 별도로 정할 것. 원본 파일을 덮어쓰지 않고 Raw/Processed를 구분할 것.
