# WiLoR → KUAVO adopted standard

이 디렉터리는 녹화별 준비된 WiLoR/Meta 입력을 KUAVO S63 궤적으로 변환하는 lab6 채택 runtime입니다.
**계산·검증·4분할 렌더링은 이 standard를 사용합니다.**

## 실행

lab6의 `/home/lab6/Warp`에서 해당 녹화의 입력을 준비한 뒤 실행합니다.

```bash
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --dry-run
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode
```

입력은 해당 episode의 `prepared.json`, reference, world, reference config와 rendering provider입니다.
원본 WiLoR·Meta·태그 전처리는 별도 단계입니다. 결과는 lab6 runtime의 `results/RUN_NAME/`에 저장됩니다.
`CURRENT.json`은 채택된 검증 결과를 선택하며 새 실행으로 자동 교체되지 않습니다.

## 파일 안내

| 파일 | 역할 |
|---|---|
| [standard.py](standard.py) | `status`, `run`, `verify`, `render` 진입점 |
| [STANDARD.json](STANDARD.json), [standard_config.json](standard_config.json) | 채택 solver profile·IK 설정 |
| [motion_policy.json](motion_policy.json) | 시간 재조정·속도·가속도 정책 |
| [object_overlay_config.json](object_overlay_config.json), [tag_policy.json](tag_policy.json) | 랙·박스 추정과 태그 배치 |
| [box_tag_calibration.json](box_tag_calibration.json) | 승인된 고정 박스 태그 보정·출처 |
| [tools/build_tag_world.py](tools/build_tag_world.py) | 해당 녹화의 태그 관측으로 world 준비 |
| [SNAPSHOT_SHA256.json](https://github.com/minje227-coder/Warp/blob/main/pipeline/01_dataset/kuavo_standard/SNAPSHOT_SHA256.json) | 소스·설정 파일 해시 |

배포 소스 저장소에는 녹화·생성 궤적·영상을 포함하지 않습니다.

## 상세 문서

- [실행 방법·입력 계약·현재 정책](https://github.com/minje227-coder/Warp/blob/main/docs/RETARGETING_STANDARD.md)
- [검증 결과·측정 범위](https://github.com/minje227-coder/Warp/blob/main/docs/RETARGETING_VALIDATION.md)
- [프로젝트 시작 안내](https://github.com/minje227-coder/Warp/blob/main/README.md)

## Two box instances and missing observations (2026-10-19)

Track A13 objects `box_100_A13` (tags 100/101) and `box_110_A13` (110/111)
independently. Keep the existing 100/101 fixed calibration. The 110/111 instance
uses the nominal A13 paper-edge slot layout; it does not reuse the 100/101 image
calibration. Both use the standard 265 x 185 x 222 mm open-top box geometry.
Read tag observations over the full episode. Preserve the existing PnP/head-pose
acceptance gates and +/-0.3 s observation smoothing, then hold the last valid pose
for each object separately until reacquisition. Before its first observation an
object is unknown; never fill its prefix from the future. Missing observations
do not abort extraction. Prepared collision worlds must contain both observed
objects over their requested interval, each as five 1 mm plates. A held pose is
recorded as held with its observation age, not claimed to be a new measurement.
Full-episode observations and object states are retained in the preparation data.
Existing validated trajectories and CURRENT.json are preserved.

## Constrained path revision (2026-10-20)

The user-approved path stage is now native Drake/SNOPT constrained C2 B-spline
refinement between the existing IK and TOPPRA stages. The base curve, IK reference
rows, source timestamps, tolerance thresholds, collision model, V/A limits,
renderer and CURRENT.json selection are preserved. The optimizer bounds every
joint control point and constrains TCP position/orientation and collision at
sampled phases. Independent 240-Hz source-phase checks and the existing exact
curve execution-time/MoveIt checks remain mandatory; discovered failures feed
back into refinement. Finite retry exhaustion rejects the result.

`pipeline/path_tools.py` serializes the exact optimized curve for both TOPPRA and
verification. It also reads historical natural-cubic parameterizations without
changing their geometry. Historical results are not rewritten. To reprocess
previously solved IK into a new result without running IK again:

```bash
/home/lab6/Warp/.venv-retarget/bin/python standard.py refine NEW_RUN_NAME \
  --source-result /absolute/path/to/previous/result
```

Four-window rendering is the default for this command; `--no-render` explicitly
skips it. This is offline computation and sends no robot commands. The historical
13.35-second IK+timing benchmark does not include this new path stage.

## Validation performance revision (2026-10-20)

Dense checking uses up to four isolated spawned processes per trajectory. Each
process owns its Drake scene and model cache. Ordered chunk merging preserves
sample indices, coverage, all original pose/joint/collision checks and MoveIt
input. Worker exceptions propagate and prevent acceptance. Set
KUAVO_VALIDATION_WORKERS=1..4 to lower concurrency; CPU affinity also caps it.

Repeated environment interpolation reuses the exact same two-keyframe Slerp and
arithmetic. A scene update is reused only for the identical scene context, base
bytes and source phase. Refinement reuses a check only for bit-identical
coordinates at the same source phase during that immutable-input run. The final
dense pass still checks every state and keeps the existing source/execution grids.
The IK solver, curve optimizer, P/V/A output, tolerances and renderer are unchanged.

Three matched offline regressions reduced geometry+retiming+validation from
120.53/103.92/92.44 seconds to 48.84/44.22/40.64 seconds. Both sets were three
concurrent runs on lab6; these timings exclude IK and video. All 19,222 sample
records and the source-knot checks matched exactly; P/V/A arrays, MoveIt inputs
and MoveIt summaries were also identical. All 42 previously rejected sample
states remained rejected. Twenty focused unit tests passed.


## 2026-10-22 current workflow

The adopted workflow is recorded in `STANDARD.json.workflow` and
[the current guide](../../../docs/RETARGETING_STANDARD.md).
Surveyed rack centers 0/4/1=1.615m and 2/5/3=1.000m replace robot-observed offline height calibration.
New dashboard jobs default to 3cm along the associated box tag -Y, with no world-vertical fallback.
Interpolated gripper openings <=40% close fully. Preview marks contact fingers red only,
removes CLOSE text and uses a lower-right side camera with that window's human skeleton off.
Live retargeting RGB shares the dashboard head buffer; single-tag fits allow projection only.
The original IK/refinement/TOPPRA/verification profile remains in force.
Policy adoption does not rewrite CURRENT or certify physical execution; consult the validation guide for failed 3cm recordings.

## Position-constraint numerical normalization (2026-10-23)

The adopted constrained-path solver now expresses the native squared TCP
position constraint as `distance_squared / radius_squared <= 1`. The radius
(including its existing 0.98 reserve), lower bound, native AutoDiff evaluator,
objective, initial guesses, 200-major-iteration budget and all independent
acceptance checks remain unchanged. No collision, joint, orientation or V/A
limit is relaxed. Other IK search experiments are not part of this adoption.

The 13-43-54 fixed-height/box-axis-3cm input previously stopped with SNOPT 41.
Normalization alone recovers it: 240-Hz invalid states 63 -> 2 -> 0, followed
by complete timed Drake and MoveIt validation. The 13-43-31 frame-IK issue is
separate; this patch does not claim to resolve that recording.

## Box-relative gripper policy (adopted 2026-10-23)

`STANDARD.json.workflow.gripper` records the common rule approved after reviewing
six previews: depth-gated close <=70%; opening >50% with a >=12pp increase and
5mm relative withdrawal confirmed for 40ms in source time. Supporting association,
speed, persistence and regrasp gates are documented in
[RETARGETING_STANDARD.md](../../../docs/RETARGETING_STANDARD.md).
The renderer loads `pipeline/source/retarget/box_grasp.py`; the execution backend
carries the identical implementation. Regression tests check code/policy parity,
closed-state retention and selected-start/replay consistency. Raw recordings,
IK/time assignments and CURRENT.json are preserved; preview adoption does not
claim physical grasp validation.
