# Warp

## Retargeting standard — 2026-10-22 채택

WiLoR→KUAVO 계산·검증·4분할 영상은 채택된 **wilor-kuavo-standard**를 사용합니다.
날짜별 실험 solver/renderer를 자동 선택하지 않습니다.

- 랙 태그 중심 높이: **0·4·1 = 1.615m**, **2·5·3 = 1.000m**. 녹화 좌표의 공통 Z 이동으로 맞추며, 로봇 카메라·TF로 높이를 다시 계산하지 않습니다. 녹화 XY·yaw·형상은 유지하고 태그별 잔차를 기록합니다.
- 양손 TCP: 기본 **3cm**, 각 박스 태그의 **-Y 아래축**으로 전체 궤적에 적용합니다. 고정 좌표계 수직 보정·대체 경로는 없습니다. 원본 입력에서 매번 계산하므로 누적되지 않습니다.
- 그리퍼: 보간한 열림 값 **40% 이하 → 완전히 닫기**. 초과 값은 유지하며 첫 자세·재생·영상에 동일하게 적용합니다. 힘·속도를 높이는 설정은 아닙니다.
- 영상: 닫힘 구간에는 **그리퍼 손가락 접촉부만 빨간색**, CLOSE 문구 없음. 우하단은 **측면 뷰**, 그 창의 사람 스켈레톤은 숨깁니다. 빨간색은 닫힘 명령 표시이며 실제 접촉 검출은 아닙니다.
- 실시간 head 영상은 대시보드와 **동일한 최신 버퍼/엔드포인트**를 사용합니다. 태그 실패로 영상이 멈추지 않습니다. RWH 태그 하나로도 박스를 투영하지만, 로봇 정렬·이동 허용에는 사용하지 않습니다.

```bash
cd /home/lab6/Warp
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --prepared /path/to/prepared.json
```

녹화별 입력 준비 후 **native IK → constrained C2 B-spline → TOPPRA → dense/MoveIt 검증 → 4-window 렌더링**을 수행합니다.
대시보드 계산은 고정 높이·박스축 보정을 준비한 뒤 이 진입점을 호출합니다. CLI에 원본 prepared를 직접 넘기면 UI 전처리를 자동 적용하지 않으므로 아래 문서를 따르세요.
기존 lazy base·몸통/양팔·고정 head와 공통 시간축을 유지하며 Kalman/jerk는 사용하지 않습니다.
과거 **13.35초는 준비된 입력의 IK+시간 재조정만**이며 전체 처리 시간이 아닙니다.

[실행·입력 준비·전체 합의](docs/RETARGETING_STANDARD.md) · [검증·실패 기록](docs/RETARGETING_VALIDATION.md) · [채택 코드/정책](pipeline/01_dataset/kuavo_standard) · [대시보드 설치/재생](scripts/05_execute/README.md)

Quest에서 사람의 움직임과 실제 전면 RGB를 수집하고 로봇 retargeting과 학습으로 확장하는 프로젝트입니다.

## 현재 수집: Quest raw v3

- 기본 1280×1280·30fps 요청, Camera2 하드웨어 H.264 녹화.
- 영상과 Skeleton/Hand 원본을 독립 저장하고 PC에서 AprilTag·정렬·QC 처리.
- 볼륨 업으로 시작/종료. 로컬 저장 후 다음 녹화 가능, 전송은 비동기 큐 30개.
- 별도 아래쪽 HUD에 `Upload pending · N/30`. PC 확인된 오래된 백업만 교체.
- 실제 시각·카메라 보정값·추적 유효성·파일 체크섬 보존. LeRobot 변환은 추후.
- 수집 원본 저장 성공과 학습 데이터 적합성은 별도 판정.

실행·스키마·시계 검증·복구 정책은 [raw v3 안내](docs/RAW_CAPTURE.md)를 참고하세요.
아래 v1/v2 설명은 이전 방식의 기록이며 raw v3 동작 설명이 아닙니다.

## 이전 통합 수집 (v1/v2)

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

---

## WARP × RoboWheel 데이터 생성: GAP 분석 및 로드맵 (2026-10-13)

**최종 목표:** Meta Quest human demonstration (physical RGB, Skeleton, Hand, AprilTag) → Human Data Processing → **WARP-inspired Whole-Body Retargeting + RoboWheel-inspired Gripper/Contact** → Robot Action Dataset (LeRobot) → Whole-Body Policy → RL Fine-tuning.

**현재 결론:** Quest 수집 및 일부 실물 검증은 진행되었지만, **human→robot retargeting, robot-specific gripper mapping, action dataset export, policy 학습 및 RL은 미구현**입니다. Submodule을 연결한 것과 원본 논문 알고리즘의 전체 구현은 다릅니다.

### 1. 기능별 구현 상태 및 검증 범위

| 구간 | 필요한 기능 | 현 상태 |
| --- | --- | --- |
| Capture | Physical Quest RGB + Skeleton | 구현; 기존 companion/초기 통합 앱 실물 검증 |
| Capture | Hand/Finger 전 관절 | 수집 경로는 존재하지만 개별 joint coverage·정확도 검증 필요 |
| Capture | RGB/Skeleton/AprilTag 원본 time 정렬 | 통합 구현, software tests 통과; 최신 v2 Quest 실물 재검증 필요 |
| Capture | 30Hz MP4 | 구현; **출력 30Hz ≠ 실제 unique camera frame 30fps** |
| Calibration | Camera intrinsic/extrinsic, Quest↔rack↔robot world | metadata 일부 확보; 실제 좌표계 calibration 미완료 |
| Perception | Object 6D pose, tracking, contact phases | 미구현 |
| Retargeting | Skeleton → robot arm/waist/base, TCP | 참조 upstream만, robot-specific solver 미구현 |
| Gripper | Hand/object-relative grasp → TCP, closing-axis, open/close | 미구현 |
| Verification | Joint/velocity limit, collision, grasp/lift, replay | 미구현 |
| Dataset | Robot state/action schema, sync, LeRobot export | 미구현 |
| Learning | BC/ACT/Diffusion/Flow Matching, RL | 미구현 |

**검증 관련 주의사항**
- 기존 two-app companion에서 실물 수집 성공했다고 최신 통합 **v2 APK**까지 검증된 것은 아닙니다.
- 초기 통합 앱의 71 shared frames/AprilTag ID 2 검출은 확인됐지만 첫 영상 상하 반전 문제가 있어 수정 후 재검증이 필요합니다.
- 이전 실물 baseline은 **175 unique input frames / 15.07초 ≈ 11.55 source fps**였습니다. 최신 비동기 AprilTag 및 native body pose time 수정 이후 재측정해야 합니다.
- 반복 프레임으로 만든 30Hz CFR MP4만 보고 source FPS를 판단하면 안 됩니다. source_frames, source_fps, repeated_frames, coalesced_frames를 함께 확인합니다.
- 파일 검사 PASS는 hardware exposure synchronization, object pose 정확도, robot grasp 성공을 보장하지 않습니다.
- 근거: [통합 수집 안내](docs/UNIFIED_CAPTURE.md), [Unity 실물 빌드 기록](UNITY_BUILD_README.md), [기존 Capture 상태](docs/STATUS.md).

### 2. WARP/RoboWheel에서 가져온 것과 실제 GAP

| 원본 | 현재 저장소에서 참조한 것 | 별도 작업 |
| --- | --- | --- |
| [WARP](https://warp-retargeting.github.io/) | rby1_teleop, stereo-sew, ik-geo Git submodules | 원본 WARP 전체 end-to-end 구현이 아님. 대상 로봇 Human→Robot IK/SEW retargeting, torso/base allocation, TCP, joint/collision 검증을 직접 통합해야 함. 일부 geometric solver에는 별도 라이선스 필요 가능 |
| [RoboWheel](https://github.com/zhangyuhong01/Robowheel-Toolkits) | 공개 dataset/replay, HDF5→LeRobot 변환, training/visualization 참고 | 공개 toolkit이 논문의 전체 HOI reconstruction, contact refinement, 범용 hand→gripper retargeting을 제공하는 것은 아님. Object-relative grasp와 gripper command 생성 필요 |

원본 논문 또는 toolkit이 그대로 통합 실행된다고 가정하지 않습니다. Revision 및 라이선스 관련 주의: [Upstreams](docs/UPSTREAMS.md).

### 3. 세부 TODO

**A. Quest 수집·전처리**
- [ ] 최신 Unity APK 빌드/설치, volume-up 녹화·반복 take·Saved·영상 상하 방향을 실제 Quest에서 검증.
- [ ] Hand/Finger 및 Shoulder–Elbow–Wrist·Spine bone coverage와 coordinate convention 확인.
- [ ] 카메라 source FPS, tag latency, repeated frame ratio, tracking dropout 실측.
- [ ] Camera intrinsic/crop/extrinsic 및 Quest↔rack↔robot base calibration.
- [ ] Timestamp alignment, invalid frame/episode quality filter.

**B. WARP-inspired Whole-Body Retargeting**
- [ ] Quest skeleton → retargeter 입력 변환 및 Human/Robot coordinate transform.
- [ ] 대상 로봇 URDF/MJCF, arm/waist(4-DoF)/mobile base joint limits, TCP/closing-axis 정의.
- [ ] **팔 IK/SEW부터** 구현한 다음 bilateral arms → waist/base allocation 확장.
- [ ] Joint position/velocity/acceleration limits 및 self/environment collision 검사.
- [ ] TCP position/orientation error 리포트와 simulation trajectory replay.

**C. RoboWheel-inspired Gripper/Contact**
- [ ] Object 6D pose 추정(첫 pilot은 known/fixed box pose 허용).
- [ ] Human hand–object relative pose 및 approach/contact/grasp/lift/release phase 추출.
- [ ] Human grasp → robot TCP, gripper approach axis·closing axis 및 binary open/close 매핑.
- [ ] Contact/penetration/grasp feasibility 체크 및 필요시 refinement.

**D. Robot Dataset / Training**
- [ ] Robot observation/action (q_t, a_t: base·waist·arms·grippers), RGB, object pose, timestamps schema 확정.
- [ ] Raw source timestamps + output fixed timestep 모두 보존; repeated frames 별도 표기.
- [ ] Simulation open-loop replay, collision/contact/grasp/lift success 검증.
- [ ] LeRobot export, schema validation, train/val split 및 데이터 품질 통계.
- [ ] Whole-Body policy BC/Flow Matching 등의 baseline 후 RL fine-tuning.

**시점 차이 주의:** Quest egocentric RGB를 그대로 robot policy에 투입하면 viewpoint gap이 생길 수 있습니다. 실제 로봇 카메라 시점 또는 simulation-rendered observation 사용 여부를 별도 결정해야 합니다.

### 4. 구현 우선순위 / Definition of Done

| 우선순위 | 작업 | 완료 기준 |
| --- | --- | --- |
| **P0: 최소 1 episode** | 최신 Quest capture → 고정 box pose → **팔 중심** retargeting → binary gripper → simulation replay | 유효 robot joint/action trajectory 1개 생성, joint/collision 검사 PASS, box grasp/retrieval replay 성공, 시간 정렬된 dataset 출력 |
| **P1: Whole-Body** | 양팔, waist/base, dynamic object tracking, bimanual contact, collision-aware refinement | 다양한 상자 위치/초기 자세에서 action 유효성 및 성공률 평가 |
| **P2: 반복 생성/학습** | Batch conversion, quality filtering, LeRobot, policy, RL | 재현 가능한 training/evaluation scripts와 held-out evaluation metrics |

**P0의 핵심:** 처음부터 WARP/RoboWheel 전체를 재현하지 말고 **base 고정 → 한 팔 TCP + 알려진 box pose + 단순 gripper open/close → 로봇 관절 trajectory → 시뮬레이션 상자 꺼내기** 순서로 검증합니다. 이후 양팔/waist/base 및 복잡한 contact로 확장합니다.

추천 품질 지표: TCP pose error, joint/velocity violation, collisions/penetrations, grasp/lift success, unique source FPS 및 repeated-frame ratio.

### 5. 권장 구조 (아직 생성되지 않은 계획)

- pipeline/01_dataset/capture/ — 현재 수집 코드 분리 후보
- pipeline/01_dataset/preprocess/ — timestamps, calibration, quality filter
- pipeline/01_dataset/object_tracking/ — 6D object pose, contact
- pipeline/01_dataset/retarget/whole_body/ — WARP-inspired SEW/IK, base/waist/arms
- pipeline/01_dataset/retarget/gripper/ — RoboWheel-inspired grasp mapping
- pipeline/01_dataset/validation/ — collision, replay, QA
- pipeline/01_dataset/export/ — LeRobot conversion
- scripts/01_capture/, 02_convert/, 03_train/, 04_rl/ — 실행 entrypoints

**주의:** 현 시점에 scripts/02_convert/, scripts/03_train/, scripts/04_rl/은 안내용 README placeholder이며, 실행 가능한 변환·학습 코드가 아닙니다. 기존 capture 경로를 재구성할 때도 현재 shell entrypoint 호환성을 유지해야 합니다.
