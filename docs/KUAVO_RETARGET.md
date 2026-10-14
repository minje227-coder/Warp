# KUAVO S63 offline retargeting

This implementation is WARP-inspired numerical retargeting, with no geo_kin dependency.
It does not reproduce c-SEW's analytic uniqueness guarantee. It never starts ROS,
publishes joint commands, or declares an episode training-ready.

## Paths and installation on lab6

- Source: `/home/lab6/Warp/pipeline/01_dataset/retarget/`
- Configuration: `/home/lab6/Warp/configs/kuavo_retarget.json`
- URDF: `/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf`
- Recordings: `/home/lab6/Warp_data/YYYY-MM-DD/UUID/`
- Generated results: `/home/lab6/Warp/local_data/retarget/`
- Separate environment: `/home/lab6/Warp/.venv-retarget/`

On this lab6 image `ensurepip` is absent. The existing capture environment's pip can
populate the separate environment without changing capture dependencies:

```bash
cd /home/lab6/Warp
python3 -m venv --without-pip .venv-retarget
.venv/bin/python -m pip --python .venv-retarget/bin/python install -r pipeline/01_dataset/requirements-retarget.txt
```

## Convert and view

```bash
cd /home/lab6/Warp
bash scripts/02_convert/retarget_kuavo.sh \
  /home/lab6/Warp_data/2026-10-13/15f3f492a86044d5afcfa9bbef55265f \
  /home/lab6/Warp/local_data/retarget/new-take1 \
  --urdf /home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf

bash scripts/02_convert/preview_kuavo.sh \
  /home/lab6/Warp/local_data/retarget/new-take1 \
  /home/lab6/Warp/local_data/retarget/new-take1.mp4
```

Use a fresh output name; existing results cannot be overwritten. Conversion verifies
every raw-manifest checksum. It creates a `.partial` directory and renames it only
after trajectory, quality and manifest files are complete. Raw and processed capture
files are never modified. Failed stages remain inspectable under `.partial`.

The video has three panels: original first-person Quest RGB, the recorded 3D human
skeleton, and the actual URDF visual meshes rendered by MuJoCo. It is a kinematic
visualization, not a physics rollout. Head joints are fixed at zero; head pose moves
with the torso. Gripper linkage joints remain neutral until actuator/width calibration.
Human pinch width and normalized open fraction are exported separately.

Green frames pass TCP/FK, configured position/velocity/acceleration and modeled
collision checks. Red diagnostic poses are geometry-only solutions that failed the
continuous trajectory criteria. Red held frames retain the last displayed robot pose
when input/geometry is invalid. Neither red diagnostic poses nor visual holds become
valid actions. Each rejected frame breaks the accepted trajectory into a new segment;
there is no validated transition from an arbitrary robot start state.

## Input and coordinates

Native numeric body IDs are authoritative; serialized enum names overlap.
Body IDs 10/11/19 and 15/16/45 are shoulder/elbow/wrist landmarks.
XR hand IDs 3/5/7/10 define thumb MCP/tip and index MCP/tip, with wrist ID 1.

Both hands must bracket a body source timestamp within 50ms, with no extrapolation,
invalid source samples or tracking discontinuity. Raw line numbers, timestamp pairs
and interpolation weights are retained. Position-based XR/body wrist agreement must
be within 3cm. This is a consistency gate, not an independent calibration.

A single provisional Unity-position to robot-world transform aligns first-body yaw
and pelvis XY. Human translation is retained thereafter; TCP targets are never
re-centered every frame or arm-length-scaled. Native body quaternions are not reused.
The shared body/hand position basis and Quest-to-robot extrinsics remain unverified.

TCP is the thumb/index fingertip midpoint. The approved default `mcp_pinch`
constructs orientation from wrist and thumb/index MCPs, independent of fingertip
flexion (HumanEgo Eq.6 basis with a constant tool-axis transform).
`--hand-mapping legacy_pinch` retains the old index-MCP-to-TIP orientation for
controlled comparisons. Degenerate frames and large jumps are rejected. Both robot
TCP frames are `zarm_l7_endeffector_center` / `zarm_r7_endeffector_center`.
The fixed correction aligns robot -Z with the MCP-frame forward direction and robot X with opening
direction. Its robot/human rest-pose calibration still needs confirmation.

## Body allocation

Base SE(2) uses WARP's 5cm/0.1rad deadband, 1.5Hz critically damped filter.
The default geometry allocation first derives a two-arm centroid placement from
human SEW directions and nominal URDF arm lengths, filters the task-space torso
target, and projects it onto KUAVO's four torso joints. The represented torso
pitch/yaw orientation is constrained; unrepresentable roll is reported. Torso
state persists through arm failures. Arms solve numerically with the prescribed
torso frozen, so EE IK cannot trade torso orientation for hand tracking.
KUAVO's displaced wrist axes make the geometric prior approximate; this is not
analytic c-SEW. `--allocation joint` retains the previous coupled baseline.
Both TCP poses
are hard constraints (5mm Euclidean position and approximately 2deg orientation).
Chest/shoulder-center and elbow goals are soft posture references.

The lazy base follows implied upper-body relocation. Initial robot base placement
is provisional because no robot start state was captured; human/TCP targets remain
fixed in the source world. Joint, velocity, acceleration and collision checks
remain active. Accepted segments are not automatically executable across gaps.
Full prescribed-torso variation includes rejected arm frames; the historical
accepted-segment metric excludes such gaps and must not be used alone to describe
the whole preview. Invalid diagnostic arm solutions are hidden by default; use
`--show-diagnostics` only for explicitly labeled debugging previews.
Lazy behavior is an added KUAVO design; it is not an original WARP lazy-torso module.

If a preferred lazy base fails, a rate-bounded relocation candidate is tried. TCP
targets are not moved to conceal failure. Final joints are never low-pass-filtered
without re-solving. An experimental configurable multi-frame horizon exists but is
disabled by default; it did not improve the initial recordings under these bounds.

## Limits and research evidence

There is no universal set of IK limits; published examples differ:

- [WARP](https://arxiv.org/html/2606.29940v1) compares joint-limits-on/off and reports
  self-collision in simulation with neutral fingers. Its public paper does not specify
  a common hard per-joint velocity/acceleration cap or universal collision clearance.
- [GMR source](https://github.com/YanjieZe/GMR/blob/master/general_motion_retargeting/motion_retarget.py)
  uses `3*pi rad/s` when its optional velocity limit is enabled. Its internal IK dt
  must not be confused with recorded trajectory sample dt.
- [MoveIt generated defaults](https://moveit.picknik.ai/humble/api/html/srdf__config_8cpp_source.html)
  use velocity/acceleration scaling 0.1. This is a relative scale, not rad/s.
- [Mink G1 example](https://github.com/kevinzakka/mink/blob/main/examples/humanoid_g1.py)
  uses 5mm minimum collision separation and 15cm detection distance for selected
  hand/table/thigh pairs. [Mink tutorial](https://github.com/kevinzakka/mink/blob/main/docs/tutorial/tasks_and_limits.rst)
  includes a 5cm example. These are example-specific values, not a required standard.
- [KUAVO S63 task.info](https://github.com/robots-oh/kuavo-ros-opensource/blob/lab-dev/src/humanoid-wheel-control/humanoid_wheel_interface/config/kuavo_s63/task.info)
  uses 10mm self-collision distance. It lists full per-joint velocity bounds and
  has `safeAccLimit.enable=false`, `realTimeAccEnable=false`.
- [Existing GraspSystem settings](https://github.com/robots-oh/kuavo_ros_application/blob/lab-dev/data_collection/GraspSystem/config/grasp_system.yaml)
  use arm speed 1.5rad/s, acceleration 8rad/s^2 and 10mm collision margin.

User explicitly selected the high speed upper bounds. This converter therefore uses
the complete declared S63 URDF limits: body 25.33rad/s; each arm
`[13, 8, 7.5, 8, 17.5, 17.5, 17.5] rad/s`. These declared limits are not measured
safe operating speeds. Position bounds come from the exact URDF with a provisional
0.02rad interior margin. Arm acceleration is 8rad/s^2 from GraspSystem; torso
acceleration is a separate provisional offline experiment value, 2rad/s^2.
The disabled S63 safeAccLimit table is not treated as a validated operational bound.

Position IK bounds include the source-time finite-difference velocity interval and
acceleration interval from the last two accepted states. Final FK checks enforce the
same limits independently. Modeled collision separation is 10mm, with interpolated
joint samples spaced by at most 0.02rad checked too (matching GraspSystem sampling).
This sampled check is not a proof for every instant of a physical trajectory.

All source collision primitives and finger mesh convex hulls are retained. Wheels,
head and neutral gripper joints are fixed. Two adjacent segment pairs per arm are
explicitly excluded because the simplified cylinders overlap at their structural
junctions; torso/arm and cross-arm pairs remain checked. A broad ground slab handles
ground clearance; Cylinder/HalfSpace signed distance is unsupported by Drake.
URDF links lacking collision geometry are listed in each manifest. No rack/object
scene, gripper swept-volume collision or torque/balance guarantee is claimed.

## Quality, timing and tests

`trajectory.jsonl` retains valid/invalid masks, segment IDs, exact source times,
TCP targets, q/base and diagnostics. `quality.json` summarizes failure reasons,
TCP errors and solve times. `manifest.json` hashes raw files, config, source code,
URDF and mesh assets. `dataset_ready=false` and `trajectory_ready_for_robot=false`
remain unconditional until clock/extrinsics, axis/gripper, environment and physical
execution validation are completed.

Video camera alignment uses median callback UNIX minus callback uptime as a
provisional offset to SENSOR_TIMESTAMP. It is labeled estimated throughout. A 30fps
video is only a presentation format; it neither changes source timestamps nor proves
sensor synchronization. `--time-scale 2` explicitly stretches execution and playback
time to half speed while keeping source times unchanged; default is original speed.

```bash
cd /home/lab6/Warp
.venv/bin/python -m pip --python .venv-retarget/bin/python install pytest
PYTHONPATH=pipeline/01_dataset \
KUAVO_RETARGET_TEST_URDF=/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf \
.venv-retarget/bin/python -m pytest pipeline/01_dataset/tests/test_retarget.py -q
```

Tests include invalid/gapped interpolation, handedness and pinch degeneracy, world
translation preservation, lazy base/torso behavior, real URDF IK, impossible targets,
mechanical/acceleration rejection and independent MuJoCo/Drake TCP FK agreement.

Geometry arm lookahead (2026-10-14): `horizon_frames=5` now also applies to
geometry allocation. Future base and torso placement are predicted using copies
of the same filters and URDF torso projector, without committing those states.
Each frame's torso is prescribed; the arm optimizer cannot move it to buy TCP
accuracy. The optimizer constrains actual timestamp-based finite-difference
velocity and acceleration across the window and independently validates every
planned frame, including sampled interframe collisions. Only the first solution
is committed, with the remaining solution shifted as the next warm start.

If the requested window is infeasible or fails convergence/validation, lengths
3, 2 and 1 are attempted. This preserves strict current-frame acceptance without
claiming a longer feasible continuation. The successful window length and failed
attempts are stored. Up to three deterministic seeds (previous, velocity-predicted,
configured reference pose) are attempted; this is numerical SEW-guided IK and does
not reproduce WARP's analytical unique-solution guarantee. Rejected input still
splits segments, and no states are connected across those gaps.

`retarget.preview --binary-gripper` animates only the custom URDF's actual
revolute linkage joints using episode P20/P80 closing-score calibration and the
0.35/0.65 hysteresis thresholds. This is display-only. The arm collision model
still freezes grippers at neutral; opened and swept gripper collision is unverified.
