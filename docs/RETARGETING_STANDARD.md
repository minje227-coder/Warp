# Retargeting standard — Quest → Kuavo S63

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

This runs **native Drake IK → TOPPRA → dense verification → MoveIt verification →
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

## Implementation from the beginning

| Stage | Used | Purpose |
|---|---|---|
| Capture | Unity XRT-Client Warp / Meta Quest | RGB, Meta body/hand tracking, timestamps and calibration; volume-button recording and asynchronous upload |
| Additional hand view | External RGB + WiLoR | Reconstruct both hands; compare with Meta to expose occlusion/tracking issues |
| Input fusion | WiLoR fingers + Meta wrist position/orientation; Meta body | Build common-world targets; midpoint of thumb/index tips is the grasp TCP, their separation drives gripper aperture |
| Objects | AprilTag tag36h11 + OpenCV PnP | Estimate rack/box poses; fixed rack and centered ±0.3s box pose averaging |
| Retargeting | Native Drake InverseKinematics + SNOPT | Preserve the lazy-base reference; solve torso and both arms with collision and continuity constraints/costs |
| Time assignment | TOPPRA | Retiming of the common 21-coordinate path using official S63 reference-generator velocity/acceleration values |
| Verification / review | Drake FK/distance, MoveIt2 PlanningScene; MuJoCo + PyAV | Verify the path and render four synchronized views |

**MoveIt checks collision/bounds/FK; Drake/SNOPT solves IK; TOPPRA changes timing.**
The final candidate does not use MoveIt's KDL as its main IK solver. The 30-frame
recovery experiment was an earlier continuity investigation; the adopted native
runner uses previous-pose/reference seeds and the existing path-collision retry.
A longer-window recovery is not silently claimed for every failed new recording.

## Adopted policy

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
- Natural cubic path + TOPPRA. Official velocity/acceleration values below; jerk and
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

## Measured time: calculation is not total episode processing

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

## Source snapshot and existing runtime

`SNAPSHOT_SHA256.json` freezes the implementation/configuration. The lab6 runtime
keeps the validated recording artifacts and `CURRENT.json`; the source snapshot
contains no raw capture, per-frame human data or videos. It also preserves native
MoveIt validator C++/CMake and the pinned Python dependency versions. On a new
machine install ROS2 Humble/MoveIt2, build that validator, supply the official S63
URDF/meshes and prepare the episode inputs. `STANDARD.json` records the current
lab6 validator binary path/hash and `motion_policy.json` records rate limits.
Those host-dependent paths must be configured for a different machine.

The previous runtime is preserved at
`/home/lab6/Warp-review/backups/standard-before-lazybase-collision-toppra-20261017`.

## 랙 태그 배치 (2026-10-19 채택)

랙 정면 기준 위쪽 선반은 `0 — 4 — 1`, 중간 선반은 `2 — 5 — 3`.
4는 0·1 중심 사이의 정중앙, 5는 2·3 중심 사이의 정중앙이며 각 줄은 같은 높이와 방향이다.
태그는 tag36h11, 검은 외곽 한 변 40mm, 사방 흰 여백 5mm, 종이 전체 50mm.
양끝 종이 옆변은 기둥 안쪽 경계에 맞추고, 태그 중심은 선반 앞면 두께 중앙에 맞춘다.
좌우 끝 중심 간격 760mm, 중앙과 양끝 간격 각각 380mm, 두 줄 높이 차이 610mm.
geometry tag 좌표계에서 4의 중심은 `[0, 0, 0.05]`m, 5는 `[0, -0.61, 0.05]`m.
기존 0~3의 위치와 geometry 기준은 유지한다. 새 배치는 이후 녹화의 태그/world 준비에 적용하며,
과거 녹화의 prepared world와 검증 결과는 당시 배치를 그대로 사용한다.

## 랙·박스 공동 태그 자세 추정 (2026-10-19)

새 녹화의 world 준비는 `tools/build_tag_world.py`와 `tools/multi_tag_pose.py`를 사용한다.
랙은 0~5, 현재 A13 박스는 기존 좌상단 100·우상단 101의 알려진 위치를 사용한다.
한 개만 보이면 기존 IPPE_SQUARE를 사용하고, 두 개 이상이면 보이는 태그의 코너를
공통 물체 좌표계에 모아 공동 planar PnP/RANSAC과 LM 재최적화를 수행한다.
태그 단위로 이상치를 제외하며 공동 결과는 온전한 태그 두 개 이상을 설명해야 한다.
태그별 RMS 2px 및 코너 3px 기준을 통과하지 못하면 기존 최선 단일 추정으로 복귀하되
`single_tag_fallback`과 실패 이유를 기록한다. 이를 공동 추정 성공으로 집계하지 않는다.
원래 관측 ID, 사용/제외 ID, 사용 코너 수, 태그별 오차를 프레임별로 남긴다.
랙은 선택 구간에서 고정 자세 하나로 통합하고, 박스는 기존 ±0.3초 평균을 유지한다.
기존 부착 치수, 원본 영상/타임스탬프, prepared world, CURRENT 결과와 IK 정책은 보존한다.

동일한 기존 0~20초 입력으로 world 생성 검증: 랙의 다중 검출 115프레임 중 114프레임에서
공동 추정 통과, 1프레임은 명시적 단일 복귀. 박스의 다중 검출 486프레임은 모두 공동
오차 기준을 통과하지 못해 단일 복귀했으며 박스 위치 결과는 기존과 같다.
이는 박스 공동 추정의 실영상 정확도 향상을 입증한 결과가 아니다. 기존 좌상단/우상단
배치를 유지한 채 부착 기하와 관측의 불일치 원인을 별도로 확인해야 한다.
합성 검증은 랙 63개 부분집합, 박스 공동 8코너, 3태그 12코너, 이상 태그/코너 제외,
단일 추정 호환성 및 잡음 조건을 포함한다. 실제 박스 기준으로 치수를 임의 보정하지 않는다.

태그/world 준비에는 OpenCV가 추가로 필요하다. lab6에서는 기존 box_detection의 cv2와
`.venv-retarget`의 NumPy/SciPy를 사용하며, IK 실행 환경을 새로 설치하거나 변경하지 않는다.
원본 전처리와 IK 실행의 경계는 그대로다. `--root`는 별도의 새 world 출력 디렉터리로 지정한다.
