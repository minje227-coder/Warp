# Quest Skeleton + physical front RGB capture

기본 Recorder는 `body.csv`와 물리 Quest 전면 RGB를 같은 episode에 기록합니다. 현재 기본 RGB source는 **right camera**입니다. 자세한 설치·실행 절차는 [README](../README.md)를 따르세요.

## 준비

1. Quest 3의 developer mode/USB debugging 및 XRT-Client APK.
2. 수신 PC의 `bootstrap.sh capture`와 `.venv` receiver 설치.
3. `build_quest_rgb.sh`로 Camera2 보조 APK 생성.
4. Quest가 USB로 연결된 PC에서 `prepare_quest_rgb.py --serial ... --receiver-host ... --apk ...`.
5. 준비 명령은 XRT의 AprilTag 카메라 접근만 끄고 복원 상태를 config에 저장합니다. Body/Hand 권한은 유지합니다. `--restore-xrt-camera`로 복원 가능하며 복원 시 companion은 종료됩니다.
6. 별도 수신 PC라면 private `configs/quest_rgb.json`을 복사.
7. Quest에서 Warp Quest RGB 앱을 열고 XRT-Client로 복귀. 보조 앱은 설정을 저장하며 PC 요청이 2분 없으면 종료.
8. `record_body.sh --check`로 import 및 **fresh physical RGB source** 확인.

## 녹화

```bash
bash scripts/01_capture/record_body.sh --session demo_0001 --wait-seconds 300 --duration 20
```

Quest에서 IP Address Panel **맨 위 칸에 수신 PC IP만** 입력·적용합니다. 기본 signaling port는 8080이며 URL은 `http://PC_IP:8080/offer`입니다. **가운데 전원 버튼**을 누르면 body WebRTC 연결이 시작됩니다. Streaming Connection은 영상 수신용 메뉴입니다.

첫 body pose가 들어오면 RGB JPEG 저장도 활성화합니다. JPEG를 받을 때까지 dummy RGB를 만들거나 화면 캡처로 대체하지 않습니다. 녹화 종료 시 capture interval PTS의 `rgb/video.mp4`를 만들고 프레임 수를 다시 디코딩해 검사합니다.

```bash
bash scripts/01_capture/validate_body.sh local_data/raw_human/demo_0001 --strict-sew
```

기본 RGB가 없으면 Recorder는 실패합니다. 실제로 Skeleton만 필요한 경우:

```bash
bash scripts/01_capture/record_body.sh --body-only --session body_0001 --duration 20
```

## 데이터와 시각

- `body.csv`: 원시 bone pose, PC monotonic packet receive 경과 시각.
- `metadata.json`: complete/failed, skeleton/RGB frame 수, writer drop, source sequence gap, PC pose origin.
- `rgb/frame_*.jpg`: UI 없는 실제 전면 카메라 영상.
- `rgb/timestamps.csv`: sensor timestamp, Quest send/encode clock, PC request/receive clock, midpoint clock estimate 및 RTT/2 uncertainty.
- `rgb/calibration.json`: vendor-identified Camera2 source/eye, 크기, timestamp source, raw sensor intrinsics 및 lens pose.

Camera2 timestamp source가 UNKNOWN이면 exposure 시각을 PC clock으로 변환하지 않습니다. 동일 episode 저장이 **하드웨어 동기화**를 뜻하지 않습니다. PC receive 기반 근사 정렬과 원본 camera interval 분석은 가능하지만 precise RGB/Skeleton acquisition sync는 추가 개발 대상입니다.

## 문제 해결

| 증상 | 확인 |
|---|---|
| Required Quest RGB unavailable | Warp Quest RGB 앱 시작, token config, Quest Wi-Fi IP, physical camera status 확인 |
| Camera2 busy/error | 동시 모드에서는 companion camera 51(right)만 열려 있어야 합니다. XRT camera 50이 재개되면 companion이 퇴출됩니다. 상태를 `adb shell dumpsys media.camera`로 확인. 다른 카메라 앱을 무작정 종료하지 말 것 |
| RGB source stalled | 헤드셋 착용/카메라 허용 상태, LAN, 앱 오류 확인. stale frame의 시각을 수정하지 않음 |
| No valid pose | XRT 가운데 전원 버튼, LAN IP/8080, body permissions 및 WebRTC 로그 확인 |
| Server URL에 8080만 들어 있음 | IP 칸에는 PC IP만 입력; 포트는 앱이 자동 추가 |
| MP4 검증 실패 | raw JPEG/timestamps를 보존하고 codec/PTS 검사. 성공으로 바꾸지 않음 |
| `--strict-sew` 실패 | 원본 bone ID와 필수 관절 존재율 확인 |

`PASS`는 파일/시각/필수 관절 및 영상 디코딩 무결성입니다. 절대 3D 정확도, world calibration, object pose, RGB exposure sync, robot grasp success는 별도 검증 대상입니다.
