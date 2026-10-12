# Integration readiness — audited 2026-10-10

| Module | 소스/구현 상태 | Test | 현재 한계 |
|---|---|---|---|
| OS/Quest/Stream 선택 | `scripts/setup.sh` 작성됨 | 세 OS CLI + 대화형 모의 테스트 PASS | 설정을 저장할 뿐 실제 Quest 기능 활성화 안 함 |
| Quest Body/Hand Client | `yunho-c/XRT-Client`를 pinned submodule로 등록 | 실제 Quest 미테스트 | APK 설치·권한·IP/port 연결 필요 |
| Python XR Receiver + CSV | `XRT_devices` pinned submodule + `record_quest_body.py` | **Mocked Recorder/CSV PASS** | Git Clone·pip install·실제 WebRTC 미검증 |
| CSV 검사 | `validate_csv.py` | 필수 SEW, 유효/비유효 Row에 대한 단위 테스트 PASS | 추적 3D 정확도는 검사 못함 |
| Egocentric RGB | Meta Passthrough API Sample pinned | 미구현 | Body Stream과 통합 앱·Timestamp/인코딩 필요 |
| Box/Rack/Object Pose | 미구현 | 미구현 | Tracker/Calibration/Contact Event 필요 |
| WARP analytic c-SEW | `rby1_teleop`, `stereo-sew`, `ik-geo` 참고만 등록 | 우리 로봇 미테스트 | `geo_kin` licensed solver, 별도 4D Waist 구현 |
| RoboWheel gripper mapping | 공개 RoboWheel Toolkit 참조만 등록 | 미구현 | 논문의 전체 Hand→Gripper 코드가 Toolkit에 있다고 확인되지 않음 |
| Collision-aware validation | 폴더만 예약 | 미구현 | Rack/Box Geometry, self-collision, allowed contact |
| Whole-Body BC/Policy | 폴더만 예약 | 미구현 | Robot Joint Action labels가 아직 없음 |
| RL fine-tuning | 폴더만 예약 | 미구현 | 시뮬·Reward·안전 계층 미구현 |

## 실제 테스트 기록

- Bash CLI: `bash scripts/test_setup.sh` — OS/stream/module 설정 PASS.
- Python: `python3 -m unittest discover -s pipeline/01_dataset/tests -p 'test_*.py' -v` — Synthetic CSV Validation과 Mock XRDevice Recording Lifecyle PASS.
- Shell: `bash -n scripts/*.sh scripts/01_capture/*.sh` — Syntax PASS.
- `git ls-remote ...`: 이 실행 환경에서는 `Could not resolve host: github.com` → **소스 Clone/Python 설치/Quest 앱 실연결 테스트 불가**.

**장비가 준비됐다는 판단 기준:** `body.csv`가 실제 Quest로부터 생성되고 Validator `PASS`, `metadata.json.status=complete`, 필요 Landmark Coverage 및 Drop Rate 확인. 이것까지 끝난 상태는 아님.

## Upstream 주의사항

- `rby1_teleop`은 RB-Y1/XHand용, 핵심 c-SEW 솔버는 라이선스 필요. 공개 MINK fallback은 WARP TCP/c-SEW를 재현하지 못함.
- `stereo-sew`는 주로 MATLAB IK 참고. Robot URDF에 곧바로 적용되는 Python 라이브러리가 아님.
- `Robowheel-Toolkits`가 Paper 전체 Retargeting을 제공하는지 확인되지 않음.
- RGB/Body 선택 옵션은 계획을 기록하는 설정값이며 실제 센서 동시 저장 기능 아님.

[실물 Quest 준비 절차](CAPTURE_GUIDE.md) · [커밋 고정 업스트림](UPSTREAMS.md)
