# Retargeting standard — Quest → KUAVO S63

2026-10-22 사용자가 현재 완성된 흐름을 standard로 채택했습니다. solver profile의 최초 채택일은 2026-10-17입니다.
채택 runtime은 `/home/lab6/Warp-review/wilor-kuavo-standard`, Git 소스/정책은
[`pipeline/01_dataset/kuavo_standard`](../pipeline/01_dataset/kuavo_standard)입니다.
날짜별 실험 폴더나 다른 IK/renderer를 자동 선택하지 않습니다.
`STANDARD.json.workflow`가 이번 합의를 기록합니다. `CURRENT.json`은 개별 검증 결과의 선택이며 정책 채택만으로 교체하지 않습니다.
원본 녹화·사람별 프레임·생성 궤적·영상은 Git에 넣지 않습니다.

## 실행과 입력 계약

```bash
cd /home/lab6/Warp
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --dry-run
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --prepared /path/to/prepared.json --run-name unique-name
```

공식 진입점은 native Drake/SNOPT IK → constrained C2 B-spline refinement → TOPPRA → dense Drake/FK/clearance → MoveIt → 4분할 렌더링입니다.
결과는 runtime의 고유 `results/RUN_NAME/` 아래 생성하며 기존 디렉터리를 덮어쓰지 않습니다. 실패한 IK/refinement/검증은 실행 가능 결과로 승인하지 않습니다.
`--no-render`는 명시적으로 영상만 생략합니다. 원본 WiLoR 추론·Meta 융합·태그 world 준비는 별도입니다.

```json
{
  "episode": "/path/to/original/episode",
  "reference": "reference.jsonl",
  "world": "world.json",
  "reference_config": "reference-config.json",
  "display_source": "/path/to/this-episodes-render-provider"
}
```

상대 경로는 descriptor 폴더 기준입니다. 기본 descriptor는 `RUNTIME/prepared/<episode-name>/prepared.json`입니다.
`reference`에는 원본 시간, 융합 TCP/방향, 사람 점, 그리퍼, lazy base/몸통 기준이 있어야 합니다.
`world`에는 고정 랙과 시간별 박스의 옆면/바닥 충돌 셸이 필요합니다. 다른 녹화의 입력·world·정렬을 재사용하지 않습니다.
`display_source`의 `results/<episode-name>/`에는 원본 영상·프레임 매핑·manifest·MuJoCo 모델,
`data/frame-scenes.jsonl`에는 카메라/물체 변환이 있어야 합니다. provider 구현은 snapshot의 `review_provider/`에 보존합니다.

대시보드의 새 계산은 원본 `input_prepared`를 출발점으로 팔 초기 seed를 standard nominal로 보정하고 녹화 마지막 2초를 제외한 뒤 아래 고정 높이/박스축 전처리를 적용합니다.
대기열은 2개 병렬이며 각 job에 calibration·보정값·입력 descriptor를 기록합니다. 기존 job의 값은 나중에 UI를 바꿔도 유지됩니다.
CLI wrapper에 원본 prepared를 직접 전달하면 이 대시보드 전처리를 자동 실행하지 않습니다. UI와 동일한 계산은 고정 높이 calibration을 만들고 다음처럼 명시합니다.

```bash
/home/lab6/Warp/.venv-retarget/bin/python scripts/02_convert/prepare_base_link.py \
  --prepared /path/to/original/prepared.json \
  --calibration /path/to/fixed-height-calibration.json \
  --output /path/to/new-input-directory --tcp-lowering-cm 3
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode \
  --prepared /path/to/new-input-directory/prepared.json \
  --run-name fixed-height-box-axis-3cm
```

`--base-link-*`와 `T_base_recording`은 파일 호환을 위한 기존 이름입니다. 현재 standard의 의미는 `fixed_rack_height`이며 로봇 관측으로 얻은 base_link 측정값이 아닙니다.
반복 계산은 원본에서 시작해 변환과 3cm가 누적되지 않습니다. 실패/취소 시 이전 선택 결과와 최신 진단을 보존합니다. 성공적으로 교체된 대시보드 결과/영상/로그는 latest-only retention을 적용하며 원본과 CURRENT는 보존합니다.

## 좌표와 grasp 위치

- 실측 태그 중심 높이: **0/4/1 = 1.615m**, **2/5/3 = 1.000m**.
- 녹화의 여섯 중심과 실측값의 공통 Z 차이를 최소제곱 평균으로 구합니다. 회전은 identity이고 XY/yaw와 물체 형상은 유지합니다.
- 기존 모델의 줄 간격은 0.610m이고 실측은 0.615m이므로 모든 태그가 동시에 정확히 일치하지 않습니다. 태그별 잔차·공통 이동량을 저장하고 최대 잔차 2cm 초과는 거부합니다. 두 줄의 실측값 자체를 임의로 바꾸지 않습니다.
- 높이 준비에는 로봇 head 영상·현재 태그·TF를 쓰지 않습니다. `robot_observation_used=false`입니다. 실제 로봇에 대한 실행 시점 정렬은 별도입니다.
- 양손 TCP position은 각 시점에 연관된 **박스 태그 bundle의 -Y축으로 기본 3cm** 이동합니다. 0–30cm 명시 입력을 허용하며 0은 비활성화입니다.
- 박스 선택에는 거리/30mm hysteresis와 기존 관측 필터를 사용합니다. 축이 없으면 준비를 거부합니다. 고정 world/base 수직 방향과 그 fallback은 없습니다.
- 원본 손/박스/몸통과 TCP orientation은 유지합니다. 보정된 TCP가 IK 목표이며, 기존 grasp 목표보다 박스 태그 아래축으로 3cm 이동한다는 의미입니다. robot base_link의 수직 깊이가 항상 3cm 변한다는 뜻은 아닙니다.

## 태그·물체·실시간 영상

녹화 입력은 WiLoR 손가락 + Meta 손목 position/orientation 및 몸통을 사용합니다. thumb/index tip 중점이 원래 grasp TCP이고 두 점 거리가 열림 값입니다.
랙 tag36h11 ID는 위 `0—4—1`, 아래 `2—5—3`, 검은 변 40mm·흰 여백 5mm, 양끝 중심 간격 760mm입니다.
박스는 `100/101`, `110/111`의 두 A13 객체, 검은 변 60mm·흰 여백 5mm, 외형 265×185×222mm입니다.
100/101에는 승인된 고정 상대 배치 [box_tag_calibration.json](../pipeline/01_dataset/kuavo_standard/box_tag_calibration.json), 110/111에는 nominal slot 배치를 씁니다.
기존 ±0.3초 pose smoothing을 유지하고 각 박스의 마지막 유효 pose를 재관측까지 보관하며, 첫 관측 이전을 미래 관측으로 채우지 않습니다. held pose에는 관측 age를 기록합니다.
충돌 world에는 관측된 두 박스의 1mm 옆면 4개와 바닥을 유지하며 윗뚜껑은 없습니다.

실시간 태그는 대시보드 태그 탭과 같은 Up apt `apriltag_ros`의 **`/rwh/tag_detections`**를 사용합니다.
수신 프로세스/버퍼는 별도이므로 같은 버퍼라고 주장하지 않습니다. 이미지 시각에 맞는 camera/base TF를 사용합니다.
RWH 출력은 pose이므로 픽셀 RMS는 없고 null로 남깁니다. 다중 태그 정렬은 수평으로 떨어진 두 개 이상·잔차 5cm 이내를 요구합니다.
태그 **하나만 보일 때도** 태그 방향+중심으로 영상의 랙/박스 투영은 허용하지만 `projection_ok=true`, 로봇 정렬 허용 `ok=false`로 구분합니다.
사용 개수는 실제 fit에 사용한 태그 수입니다. 실패한 fit/오래된 관측/TF는 투영과 제어를 각각 제한합니다. 모든 검출 끊김의 투영 깜빡임이 해결됐다는 보장은 없습니다.

head RGB는 **`common.LINK.frame('head')`와 `/kuavo/cam/head`**를 대시보드와 공유합니다.
retargeting 관측기는 Image 재구독/JPEG 재인코딩 없이 tag·camera_info·TF만 수신합니다.
태그 매칭 실패가 최신 RGB 갱신을 막지 않으며 존재하지 않는 head stamp 비교는 제거했습니다.
상위 RWH detector가 중단되면 태그도 끊기므로 `rwh_marker_detection marker_detection.launch`가 실행 중이어야 합니다.
녹화 종료 시 detector가 함께 내려갔던 문제는 기존 detector를 독립 실행해 복구했으며 자동 재시작 service를 추가한 것은 아닙니다.
새 offline 태그 전처리도 Up의 apt ROS detector와 검증된 intrinsics/RWH ID별 크기를 사용합니다. 예전 OpenCV PnP/corner helper의 픽셀 기준을 현재 ROS pose의 검증값으로 간주하지 않습니다.

## IK·시간·검증

기존 original lazy base(XY deadband 50mm, yaw deadband 0.1rad), 몸통+양팔의 구조를 유지하고 head 관절은 고정/제외합니다.
공통 source phase와 1x execution clock을 유지합니다. 원본 시각과 TOPPRA 실행 시각은 구분합니다.
Down cone 0–60°, 양손 parallel 10°, mount-aware wrist-camera anti-flip을 유지합니다.
박스 옆면/바닥 거리 100mm 이하 TCP 허용 10mm, 200mm 이상 20mm, 사이 smooth transition입니다.
충돌 clearance 10mm이며 랙·박스·팔/몸통/wrist link7을 유지하고 기존 8개 gripper collision shape 제외 정책을 유지합니다.
몸통/팔 continuity 비용 1000/100, prediction 300은 최적화 비용입니다.
C2 curve는 joint control point bounds와 sampled TCP/orientation/collision 제약으로 다듬고, source phase 240Hz 독립 확인과 최종 exact timed curve/MoveIt 검증을 거칩니다.
위반 phase를 다시 넣어 제한된 횟수 재최적화하며 소진하면 거부합니다. 이는 수학적 연속 충돌 증명이 아닙니다.
TOPPRA와 검증은 같은 exact spline을 사용하며 원본 IK 행은 보존합니다. Kalman과 jerk는 사용하지 않습니다.

| 좌표 | 속도 | 가속도 |
|---|---:|---:|
| Base X/Y | 0.8m/s | 0.9m/s² |
| Base yaw | 1.2rad/s | 1.3rad/s² |
| Knee/leg/waist pitch/yaw | 3.14rad/s | 5.14rad/s² |
| 양팔 14관절 | 5.14rad/s | 25rad/s² |

선택된 S63 reference-generator 설정이며 하중별 실물 인증은 아닙니다. 출처와 상세 설정은 `motion_policy.json`에 있습니다.
Dense 검증은 기본 최대 4개의 독립 spawn 프로세스를 사용하며 `KUAVO_VALIDATION_WORKERS=1..4`로 줄입니다.
샘플 grid와 승인 기준을 유지하고 worker 오류 시 승인하지 않습니다. 동일 phase/scene/base에 한정한 캐시만 재사용합니다.

## 그리퍼와 4분할 영상

보간한 `open_fraction <= 0.40`이면 **0(완전 닫힘), claw position 100**으로 보냅니다. 40% 초과는 그대로입니다.
선택 첫 자세와 매 replay sample의 보간 **후** 처리하고 영상에도 같은 threshold를 적용합니다.
`gripper_policy.close_at_open_fraction=0.40`을 payload에 저장합니다. 과거 20/30% 명시 payload는 그 값을 따르고 정책이 없는 과거 payload는 기존 의미를 유지합니다.
원본 gripper/몸통/팔 IK·궤적·시간은 재작성하지 않습니다. 기존 100Hz replay 중 반복 명령하며 끝점 최종 명령과 feedback completion 흐름을 유지합니다. 힘/속도를 올리는 변경은 아닙니다.

1. 위 왼쪽: external RGB + 관측 사람/손.
2. 위 오른쪽: My view(Quest RGB) + 물체/손.
3. 아래 왼쪽: whole body + Kuavo + 전체 사람 skeleton.
4. 아래 오른쪽: **측면 camera**, 이 창의 사람 몸/손 skeleton OFF.

측면 camera는 azimuth -90° + base yaw, elevation -8°, distance 2.65m,
lookat base에서 전방 0.4m·z 0.95m입니다. 더 이상 robot head camera를 네 번째 기본 창으로 쓰지 않습니다.
1920×1200 출력, robot panel 30Hz이며 공통 source phase로 RGB·사람·물체·로봇·그리퍼를 맞춥니다.
닫힘에는 `l_f_finger/l_b_finger/r_f_finger/r_b_finger` 접촉 mesh만 빨간색으로 하고 열리면 원래 재질을 복구합니다.
CLOSE/threshold 문구는 없습니다. 반투명 박스 합성에 색이 묻히지 않도록 robot-only render의 보이는 finger red pixel만 최종 합성에 복구합니다.
이 표시는 **40% 규칙에 따른 닫힘 명령**이며 실제 물체 접촉/파지 성공 센싱이 아닙니다.
재렌더는 새 media bundle/manifest를 만들고 원본 궤적 hash와 source phase 매핑을 검증합니다. 화면 translation으로 IK 오차를 숨기지 않습니다.

## 재생·배포·검증 범위

실행은 **첫 자세 이동 → 측정된 도착 확인 → replay** 순서이며 연결했다는 이유만으로 도착으로 간주하지 않습니다.
태그 한 개 투영은 이동 허용과 분리합니다. 이 문서의 offline 성공은 실물 파지 성공을 뜻하지 않습니다.
소스/backend는 `scripts/05_execute`, 기존 tab dashboard 설치 파일은 그 아래 `dashboard_adapter`에 함께 보존합니다.
설치기는 기본 dry-run, `--apply` 시 기존 파일 backup, 재시작/로봇 연결/이동은 하지 않습니다.

```bash
python3 scripts/05_execute/install_dashboard_adapter.py --dashboard-root /home/lab6/humanoid
# 실제 파일 반영: 같은 명령에 --apply (기존 dashboard 구조 필요)
python3 -m unittest discover -s scripts/05_execute/tests
```

`SNAPSHOT_SHA256.json`으로 frozen source/config를 확인합니다. ROS2 Humble/MoveIt2 validator와 공식 S63 URDF/meshes 및 녹화별 입력은 별도로 준비해야 합니다.
`STANDARD.json`의 lab6 runtime/validator 경로는 다른 host에서 설정해야 합니다.
[검증 기록](RETARGETING_VALIDATION.md)에 3cm 실패와 기존 결과 유지 사실, 과거 benchmark 범위를 구분합니다.
