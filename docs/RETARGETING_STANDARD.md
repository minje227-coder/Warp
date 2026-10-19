# Retargeting standard — Quest → KUAVO S63

> **MUST / 반드시 standard 사용:** WiLoR→KUAVO 변환·계산·4분할 렌더링은
> 채택된 `wilor-kuavo-standard/standard.py`와 그 설정을 사용한다. 날짜가 붙은
> 실험 폴더나 별도 IK/렌더러를 임의로 선택하지 않는다. 공식 실행은 아래
> `convert_episode_standard.sh EPISODE_PATH`이며 `CURRENT.json`이 채택 결과다.

Adopted **2026-10-17**. Runtime: `/home/lab6/Warp-review/wilor-kuavo-standard`.
The adopted solver, configuration, validation and four-window renderer are frozen in
[`pipeline/01_dataset/kuavo_standard`](../pipeline/01_dataset/kuavo_standard).
Recordings, reconstructed hand data, trajectories, models converted for a particular
machine and rendered videos stay outside Git.

## Run with an episode path

On **lab6**, after the episode's inputs have been prepared:

```bash
cd /home/lab6/Warp
bash scripts/02_convert/convert_episode_standard.sh \
  /home/lab6/Warp_data/2026-10-16/17-13-10_7ef9211ae173462d9952a35ebc1014fa
```

This runs **native Drake IK → constrained C2 B-spline refinement → TOPPRA → dense verification → MoveIt verification →
the same four-window rendering**. It uses a unique output directory under
`$KUAVO_STANDARD_RUNTIME/results/` and refuses to overwrite existing results.
A failed IK/dense check is not accepted as an executable trajectory. The selected
adopted result in `CURRENT.json` is not silently replaced by a new recording.

Check the exact resolved inputs without computing:

```bash
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --dry-run
```

Use another prepared bundle or runtime:

```bash
KUAVO_STANDARD_RUNTIME=/home/lab6/Warp-review/wilor-kuavo-standard \
KUAVO_STANDARD_PYTHON=/home/lab6/Warp/.venv-retarget/bin/python \
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode \
  --prepared /path/to/prepared.json --run-name take-review-01
```

**Input preparation is separate.** This command does not run WiLoR inference or
estimate tags from raw RGB. It requires a prepared bundle belonging to that exact
episode and stops with an explanation if none exists. Default descriptor:
`RUNTIME/prepared/<episode-directory-name>/prepared.json`.
The current recording already has this descriptor on lab6. For a new recording,
prepare the following data before invoking the command:

```json
{
  "episode": "/path/to/the/original/episode",
  "reference": "reference.jsonl",
  "world": "world.json",
  "reference_config": "reference-config.json",
  "display_source": "/path/to/that-episodes-prepared-rendering-provider"
}
```

Relative file paths resolve against the descriptor directory. `reference.jsonl`
contains original timestamps, fused targets, human points, gripper signals,
take-specific IK seeds and the existing lazy-base reference. `world.json` contains
the fixed rack and time-indexed 1mm box side/bottom collision shells. The reference
configuration supplies that recording's alignment and initialization. The input
range is the range prepared in the reference; the adopted example is 0–20 seconds.
Do not reuse another recording's joint seed, alignment or world.

`display_source` is the prepared rendering provider used by the verified renderer:
its `results/<recording-name>/` contains the source review video, source frame
mapping, manifest and MuJoCo overlay model, and its `data/frame-scenes.jsonl`
contains that episode's camera/object transforms. Matching provider implementation
is preserved in `kuavo_standard/review_provider/`; this is a data contract, not an
automatic raw-data preprocessing stage. See the frozen renderer for exact inputs.
`--no-render` explicitly skips video; four-window rendering is the default.

## 처리 단계


| Stage | Used | Purpose |
|---|---|---|
| Capture | Unity XRT-Client Warp / Meta Quest | RGB, Meta body/hand tracking, timestamps and calibration; volume-button recording and asynchronous upload |
| Additional hand view | External RGB + WiLoR | Reconstruct both hands; compare with Meta to expose occlusion/tracking issues |
| Input fusion | WiLoR fingers + Meta wrist position/orientation; Meta body | Build common-world targets; midpoint of thumb/index tips is the grasp TCP, their separation drives gripper aperture |
| Objects | AprilTag tag36h11 + OpenCV PnP | Estimate rack/box poses; fixed rack and centered ±0.3s box pose averaging |
| Retargeting | Native Drake InverseKinematics + SNOPT | Preserve the lazy-base reference; solve torso and both arms with collision and continuity constraints/costs |
| Time assignment | TOPPRA | Retiming of the common 21-coordinate path using official S63 reference-generator velocity/acceleration values |
| Verification / review | Drake FK/distance, MoveIt2 PlanningScene; MuJoCo + PyAV | Verify the path and render four synchronized views |

MoveIt은 충돌·관절 범위·FK를 검증하고, Drake/SNOPT는 IK를, TOPPRA는 시간 재조정을 담당합니다.

## 물체·태그 추정

`tools/build_tag_world.py`는 별도의 새 출력 디렉터리에 해당 녹화의 world를 준비합니다.
OpenCV와 NumPy/SciPy가 필요하며, lab6에서는 기존 box_detection의 cv2와
`.venv-retarget`의 NumPy/SciPy를 사용합니다.

- **랙:** tag36h11, 검은 한 변 40mm·흰 여백 5mm. 정면 위쪽 `0 — 4 — 1`, 중간 `2 — 5 — 3`.
  양끝 종이 옆변은 기둥 안쪽 경계에, 중심 높이는 선반 앞면 두께 중앙에 맞춥니다.
  양끝 중심 간격 760mm, 두 줄 높이 차이 610mm입니다. 선택 구간의 관측으로 고정 자세 하나를 계산합니다.
- **박스:** 현재 A13 100·101, 검은 한 변 60mm·흰 여백 5mm.
  박스 외형은 265×185×222mm이며, 100의 원래 기준 위치를 유지한 고정 상대 배치 보정을 사용합니다.
  정확한 위치·회전과 출처는 [보정 설정](../pipeline/01_dataset/kuavo_standard/box_tag_calibration.json)에 있습니다.
  시간 구간별 보정 교체 없이 같은 배치를 사용하며 박스 pose는 기존 ±0.3초 평균을 적용합니다.
- **공동 추정:** 두 개 이상 보이면 알려진 상대 위치·평면 회전으로 코너를 모아
  planar PnP/RANSAC과 LM 재최적화를 수행합니다. 태그 단위로 이상치를 제외하고,
  공동 결과에는 온전한 태그 두 개 이상이 필요합니다.
- **선택 기준:** 태그별 RMS 2px, 코너 오차 3px. 한 개만 보이거나 공동 추정이 실패하면
  단일 추정으로 처리합니다. 공동 실패는 `single_tag_fallback`으로 구분하고,
  관측·사용·제외 ID, 사용 코너 수와 오차를 기록합니다.

태그 크기·배치는 [tag_policy.json](../pipeline/01_dataset/kuavo_standard/tag_policy.json),
추정 설정은 [object_overlay_config.json](../pipeline/01_dataset/kuavo_standard/object_overlay_config.json)이 기준입니다.
실제 배치가 다른 녹화에 다른 recording의 world를 재사용하지 않습니다.

## 채택 IK·시간·렌더링 정책


- Original grasp point, world offset **0**. Down cone **0–60°**, bimanual parallel
  tolerance **10°**, mount-aware wrist-camera anti-flip; head joints remain fixed.
- Existing lazy base: XY deadband **50mm**, yaw deadband **0.1rad** and damping.
  No recording-specific 9–16s hold. Prepared reference base is reused exactly.
- Target-to-open-box-side/bottom distance ≤**100mm**: TCP allowance **10mm**;
  ≥**200mm**: **20mm**; smooth transition between. No imaginary top lid.
- Collision clearance **10mm**. Rack, **1mm** box sides/bottom, arm/torso/wrist
  link7 stay enabled; eight gripper collision shapes are excluded. Box top is open.
- Effective continuity weights: torso **1000**, arms **100**, prediction **300**.
  These are objective costs, not physical rate limits. Weak chest/elbow references
  avoid unnecessary torso following. Solve reserves are stricter than final checks.
- Preserve source timestamps. Bounded missing-input interpolation and an isolated
  hand-target/aperture spike rule are explicit; raw recordings and observed skeletons
  remain unchanged. Missing initial observations are not extrapolated.
- Constrained C2 B-spline path + TOPPRA (2026-10-20 revision). Official velocity/acceleration values below; jerk and
  low-level Kalman filtering are off. Rendering uses the same source phase for RGB,
  human, grippers, objects and robot. Source timestamps differ from execution time.

| Coordinates | Velocity | Acceleration |
|---|---:|---:|
| Base X/Y | 0.8m/s | 0.9m/s² |
| Base yaw | 1.2rad/s | 1.3rad/s² |
| Knee / leg / waist pitch / waist yaw | 3.14rad/s | 5.14rad/s² |
| Both arms, 14 joints | 5.14rad/s | 25rad/s² |

Source: [official Kuavo S63 task.info](https://github.com/LejuRobotics/kuavo-ros-opensource/blob/fa134789af6d6846da10d11ff656e0a84293a02c/src/humanoid-wheel-control/humanoid_wheel_interface/config/kuavo_s63/task.info),
`referenceKinematicLimit` and selected MPC yaw box. These are the selected official
reference-generator settings, not a load-specific hardware execution certificate.

## Four-window rendering is fixed

1. **External RGB** with observed human/hand overlay.
2. **My view** (Quest RGB) with objects and human hands.
3. **Whole body + Kuavo**, in the same world frame.
4. **Kuavo head view**, attached to the robot head; down **20°**, FOV **55°**.

1920×1200 output, robot panels rendered at 30Hz from the 100Hz trajectory. Keep
one full-body skeleton, body lines 7px / hand lines 3px, gold filled box sides/bottom,
open top and green rack. Do not apply a display translation to hide target errors.

## 소스·환경


`SNAPSHOT_SHA256.json` freezes the implementation/configuration. The lab6 runtime
keeps the validated recording artifacts and `CURRENT.json`; the source snapshot
contains no raw capture, per-frame human data or videos. It also preserves native
MoveIt validator C++/CMake and the pinned Python dependency versions. On a new
machine install ROS2 Humble/MoveIt2, build that validator, supply the official S63
URDF/meshes and prepare the episode inputs. `STANDARD.json` records the current
lab6 validator binary path/hash and `motion_policy.json` records rate limits.
Those host-dependent paths must be configured for a different machine.

상세 수치·녹화별 성공/실패·시간 측정은 [검증 기록](RETARGETING_VALIDATION.md)에 모았습니다.
이전 수집 방식과 초기 로드맵은 [README 보존본](history/README_BEFORE_CLEANUP_20261019.md)에 있습니다.

## Constrained path refinement (2026-10-20)

The existing IK output is the initial guess for native Drake/SNOPT B-spline
optimization. TCP position/orientation, joint interior bounds and collision
clearance constrain the connecting curve. Only affected body/arm control points
are optimized; the original lazy-base curve is fixed. All joint control points
obey the existing bounds, which bounds the entire B-spline. Nonlinear constraints
are sampled and independently checked at 240 Hz in source phase, then again on
the exact time-parameterized curve and delivered samples. Failed phases are added
for bounded refinement; exhausted retries reject the result. Sampled nonlinear
checks are not a mathematical continuous-collision certificate.

The original IK rows and raw recordings are retained. An exact spline artifact
is shared by TOPPRA and validation; validation never rebuilds a different curve
from the output points. The original renderer and CURRENT.json selection remain.
Existing results can be reprocessed with standard.py refine NEW_NAME
--source-result OLD_RESULT; this creates a new result and defaults to rendering.
Jerk and Kalman remain off. The historical 13.35-second benchmark predates this
stage and must not be presented as its end-to-end compute time.

## Validation execution (2026-10-20 performance revision)

Dense checks use four isolated spawned processes by default, bounded by CPU
affinity. KUAVO_VALIDATION_WORKERS=1..4 can reduce concurrency. Ordered chunks
retain every original sample and worker errors prevent acceptance. The two-slot
dashboard compute queue is unchanged.

Environment interpolation reuses identical source phases with the original
two-keyframe Slerp/arithmetic. Scene updates additionally require the exact same
base bytes and scene context. Refinement reuses only bit-identical coordinates
at the same phase within one immutable-input run. The final dense stage still
checks all original states. IK/geometry/PVA, tolerances, collision models,
source timestamps, rendering and CURRENT.json selection are unchanged.
