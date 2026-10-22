# Retargeting 검증 기록

이 문서는 특정 입력·시점의 검증과 측정 결과를 보존합니다.
현재 실행 방법과 정책은 [standard 안내](RETARGETING_STANDARD.md)를 따릅니다.

## 2026-10-17: 준비된 입력의 offline retargeting


For the adopted 20-second demo, using already prepared input:

| Stage | Wall time |
|---|---:|
| Native IK | **11.72s** |
| TOPPRA timing | **1.63s** |
| IK + timing subtotal | **13.35s** |
| Dense Drake verification | **45.55s** |
| MoveIt verification | Additional; not separately timed |
| Four-window rendering | **16.87s** |
| Duration of the resulting motion | **10.09s** — playback time, not calculation time |

**13.35s excludes WiLoR inference, input fusion, tag/world preparation, validation
and video rendering.** Existing saved trajectories do not need IK again for replay.
The robot has not physically executed this offline result.

Verification: **597 valid knots**, **6,257 dense states**, MoveIt collision/bounds
failures **0**; the standard entry point reproduced all 600 source rows' validity,
base and joint solutions exactly. Minimum sampled clearance **10.89mm**. The first
three observations were unavailable; 597 is not a claim of 600 solved inputs.

Other recording: 417 prepared states solved, no MoveIt collisions; 70 dense states
at source 14.18–14.48s exceeded TCP allowance. That result was rejected, not promoted.
Its 23 missing/failed-reference rows and 10 unobserved-box rows were outside that
prepared-input test. This is not a full success claim for arbitrary recordings.


## 2026-10-19: 공동 태그 추정 도입, 박스 보정 전

같은 0~20초 입력에서 랙의 다중 검출 115프레임 중 114프레임이 공동 추정을 통과했고,
1프레임은 단일 추정으로 복귀했습니다. 박스는 당시의 19.5cm·같은 높이·같은 방향 가정으로
공동 기준을 통과하지 못해, 다중 검출 486프레임 모두 단일 추정으로 복귀했습니다.
이 배치는 아래 승인된 보정으로 대체되었습니다.

## 2026-10-19: 박스 태그 보정 채택

현재 박스 100·101의 상대 배치를 영상에서 추정하고, 실제 원본에 투영한 비교를 검토한 뒤 채택했습니다.
사용한 고정 후보는 0~7초의 60프레임에서 추정한 `early_to_late`입니다.
별도 13~20.1초 구간에서는 148프레임 중 147프레임이 픽셀 기준을 통과했습니다.
이 값은 영상 기반 상대 태그 보정이며 물리 치수의 실측이나 카메라 보정을 대체하지 않습니다.

| 같은 0~20초 입력의 world 생성 | 결과 |
|---|---:|
| 입력 paired frames | 582 |
| 박스 관측 프레임 | 580 |
| 두 태그 검출 중 공동 추정 통과 | 484 / 486 |
| 공동 추정 실패 후 단일 복귀 | 2 |
| 한 태그만 검출되어 단일 추정 | 94 |
| 선택된 박스 관측 RMS 중앙값 | 약 0.608px |
| 태그 자세 테스트 + episode CLI 테스트 | 12 + 5 통과 |

정확한 보정값·검토 영상 SHA256·검증 출처는
[box_tag_calibration.json](../pipeline/01_dataset/kuavo_standard/box_tag_calibration.json)에 있습니다.
랙 63개 부분집합, 박스 공동 8코너, 3태그 12코너, 이상 태그/코너 제외,
단일 추정 호환성과 회전된 오른쪽 태그의 단독 추정을 테스트했습니다.
기존 prepared world와 CURRENT의 로봇 결과는 보존했으며, 보정은 새 world 준비부터 적용됩니다.

## Constrained path regression (2026-10-20)

User-approved native Drake/SNOPT constrained C2 B-spline refinement was tested
against the three rejected replay trajectories without recomputing their accepted
IK rows. Original input/source IK files and CURRENT.json remain unchanged.

| Recording | Old invalid states | New invalid states | Final checked states | Duration |
|---|---:|---:|---:|---:|
| 13-42-32 | 14 | 0 | 7302 | 13.377087 s |
| 13-43-02 | 9 | 0 | 6256 | 11.405622 s |
| 13-43-31 | 19 | 0 | 5664 | 10.908699 s |

All three passed independent FK/clearance, velocity/acceleration and MoveIt
checks. Eleven focused numerical/acceptance tests passed. Exact curve artifacts
round-trip with their first/second derivatives; historical natural-cubic artifacts
keep their old geometry. New four-window previews were rendered and bound to the
new trajectory hashes. Pure replay package construction passed for all three.
The preview includes the full retimed endpoint, avoiding an artificial moving
cut at 13 seconds. No robot motion or mode setter was executed. Jerk/Kalman off.
Path refinement alone took approximately 25–34 seconds in these concurrent runs;
this is additional offline computation, not replay duration.

Detailed report: /home/lab6/warp-constrained-path-stage-20261020/REPORT.json.
Previous source/catalogue backup:
/home/lab6/Warp-review/backups/standard-before-constrained-path-20261020.

## Validation performance regression (2026-10-20)

Three matched concurrent offline runs, existing accepted IK reused, video excluded:

| Recording | Prior geometry+timing+verification | Optimized | Prior dense check | Optimized dense check |
|---|---:|---:|---:|---:|
| 13-42-32 | 120.53 s | 48.84 s | 73.03 s | 18.18 s |
| 13-43-02 | 103.92 s | 44.22 s | 62.44 s | 18.57 s |
| 13-43-31 | 92.44 s | 40.64 s | 55.37 s | 16.72 s |

All 19,222 per-state records and all source-knot checks matched exactly. P/V/A
arrays were bit-identical. MoveIt inputs and result summaries matched exactly.
All 42 originally rejected states remained rejected. Twenty focused unit tests
passed; a malformed worker input propagated an exception without acceptance.

A separate fresh 13-43-02 run, from prepared inputs through NEW IK and final
MoveIt verification, passed in 54.58 seconds (IK 14.04 s), excluding raw input
preparation and video rendering. That isolated run's dense check took 10.93 s.

Original validated dashboard results/videos and CURRENT.json are retained because
their trajectories are identical. New conversions use the optimized implementation.
Detailed evidence: /home/lab6/warp-validation-speed-stage-20261020/.
No robot commands or mode setters were executed; jerk/Kalman remain off.


## 2026-10-22: 고정 높이·박스축 3cm·40% 닫힘·측면/빨간 접촉부 채택

고정 높이는 0/4/1=1.615m, 2/5/3=1.000m를 공통 Z 이동으로 맞췄습니다.
기록 모델의 610mm 간격과 실측 615mm 차이 때문에 잔차가 있으며 현재 녹화들에서 최대 약 3.7–4.8mm입니다.
로봇 관측으로 높이를 재추정하지 않았습니다. 박스 태그 -Y 3cm 계산은 7개 중 5개 통과, 2개 실패였습니다.

| 녹화 | 3cm 계산 | 현재 표시 결과 |
|---|---|---|
| 13-42-32 | 통과 | 검증된 3cm 궤적 |
| 13-43-02 | 통과 | 검증된 3cm 궤적 |
| 13-43-31 | 실패, IK 5프레임 | 이전 검증된 0cm 궤적 유지 |
| 13-43-54 | 실패, refinement SNOPT 41 | 채택 결과 없음 |
| 13-44-20 | 통과 | 검증된 3cm 궤적 |
| 13-44-52 | 통과 | 검증된 3cm 궤적 |
| 13-45-15 | 통과 | 검증된 3cm 궤적 |

현재 존재하는 6개 영상은 40% 닫힘·측면 뷰·접촉 finger 빨간색으로 재렌더했고 source/trajectory artifact 불변을 확인했습니다.
6개가 전부 3cm 계산 통과라는 뜻은 아닙니다. 결과 manifest:
`/home/lab6/humanoid/data/retargeting/media/gripper-snap40-redtips-visible-side-20261022/RESULT.json`.
영상의 빨간색은 실제 접촉 검출이 아닙니다. head 공유/단일 태그 projection과 별도로 검출 끊김에 의한 투영 flicker는 남을 수 있습니다.
이번 standard 채택/문서/저장소 통합에서 로봇 명령은 보내지 않았습니다.


### Main 병합본 확인

- 실행 backend 98 + 입력 준비 3 + episode CLI 7 + exact path/validation 20: **128개 통과**.
- 기존 retarget/hand mapping/gripper pytest: **24개 통과, 14개 skip** (환경 조건).
- frozen source/config 52개 SHA256 일치, active runtime 핵심 계산/설정 14개 일치, 변경 Python AST·dashboard JavaScript 문법 확인.
- 기존 dashboard adapter 설치 계획 dry-run 성공. 설치/서비스 재시작/로봇 동작은 실행하지 않음.
- 서로 분기됐던 로컬 retarget 이력과 원격 main 이력을 병합해 보존. 별도 수집 앱의 미커밋 수정과 생성 영상은 추가하지 않음.

## 2026-10-23: squared-distance normalization; no constraint relaxation

The 2026-10-22 failures above are historical. The position constraint in the
constrained C2 path stage is now normalized by its native squared radius.
Ablation kept the original initial guess and 200 major iterations: normalization
alone changed the 13-43-54 source-phase audit from 63 invalid states to 2, then 0.
Its unchanged input IK rows, reference, collision world, base curve, tolerances,
policy and original source timestamps are retained. The final patch uses the
native upper bound rather than duplicating the physical radius.

The final patched code passed all 3,622 source-phase states, all 396 source IK
knots and 4,911 timed samples, including MoveIt full-robot/environment checks.
Minimum sampled clearance was 10.00965 mm; maximum sampled velocity/acceleration
ratios were 0.26491 / 0.99146. Execution duration was 8.9417319 s. No robot
commands were sent. Dashboard selection/media and CURRENT.json were not changed.

Twenty-three focused tests pass, including unchanged inside/outside physical
boundaries at 9.8/19.6-mm solver radii and native AutoDiff/finite-difference checks.
The previously passing 13-43-02 3cm recording also passed the final code:
6,383 timed samples and MoveIt checks, with all original input hashes unchanged.
See normalization-validation-20261023.json for exact results and artifact hashes.

13-43-31 is not recovered. Its original five failed frames span source
12.7416504--12.8747084 s. More iterations and neighboring/0cm warm starts did not
recover them. Position/cost rescaling plus deterministic perturbed starts
recovered frame 386 only; 382--385 still failed. In a separate depth-continuation
diagnostic, successful isolated configurations reached 24--27 mm, while the
next tested depth produced right wrist link7/box_100_A13_right and right-rack-
pillar clearances below 10 mm (approximately 6.7--9.6 mm), sometimes also exceeding
10-degree mutual tool orientation. These failed candidates identify a geometric
bottleneck, not a proof that no feasible configuration exists. No smaller depth,
collision exclusions or relaxed parallelism have been adopted.

Detailed diagnostic artifacts remain on lab6 under
`/home/lab6/Warp-review/wilor-kuavo-standard/results/diagnose-box3cm-20261022-235539/`.

## Box-relative grasp policy adoption (2026-10-23)

User-approved common settings are recorded in `STANDARD.json.workflow.gripper`
and RETARGETING_STANDARD.md: close <=70% with box-depth gating; release >50%
with relative opening/withdrawal, including the documented 5mm/40ms settings
and persistence/regrasp gates. These are project settings, not empirically
calibrated hardware thresholds. No per-recording release timestamp is used.

Validation on the latest main-based promotion:
- Execution test suite: 115 tests passed, including 17 box-grasp regression tests.
- Standard pipeline test suite: 23 tests passed.
- Delivered preview state traces match the promoted implementation for all six
  computed episodes (13-42-32, 13-43-02, 13-43-31, 13-44-20, 13-44-52, 13-45-15).
- Synthetic fixture checks cover recording/base transforms, bilateral independence,
  missing-observation holds, relative movement, short pulses, regrasp, discrete
  seeking, first-state/package/preview consistency and unchanged body P/V/A/files.
- Replay packaging on the six existing episodes passed the existing artifact gates
  in the prior offline contract check; rejected trajectories retain their status.

Only gripper policy/preview behavior is promoted here. Existing IK/time-assignment
results and CURRENT.json remain selected as before. No robot motion or service
restart was performed for this promotion; physical grasp validation remains open.
