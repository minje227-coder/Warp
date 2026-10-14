# KUAVO retargeting continuity research — 2026-10-14

Scope: source 25–46 seconds of episode 4b9f055a34a042af970f1b4f9c2764eb.
No geo_kin, no ROS commands, no source recording changes.

## Primary sources and implications

- Pink acceleration limit: https://raw.githubusercontent.com/stephane-caron/pink/main/pink/limits/acceleration_limit.py — combines acceleration bounds with braking distance to joint position limits. Our experiment checks directional endpoint stopping distance plus a conservative half-step allowance, using one vector constraint per frame to reduce Python callback overhead.
- WARP: https://arxiv.org/html/2606.29940 — geometric SEW selection and lazy body allocation reduce ambiguity. The analytic assumptions and licensed implementation cannot be presumed compatible with this S63 numerical solver. It does not supply our continuous dynamic feasibility guarantee.
- RoboWheel: https://arxiv.org/abs/2512.02729 — relevant cross-embodiment retargeting case; no evidence that changing our hand mapping alone will repair all continuous failures.
- cuRobo: https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html and https://arxiv.org/abs/2310.17274 — parallel seeds, trajectory optimization and explicit time dilation. Promising GPU candidate generation; published benchmarks are not measurements on this S63 episode.
- TOPPRA: https://hungpham2511.github.io/toppra/index.html and https://hungpham2511.github.io/toppra/auto_examples/plot_kinematics.html — separate geometric path from velocity/acceleration-limited timing. It cannot repair an infeasible or colliding geometric path.
- MoveIt Noetic: https://moveit.github.io/moveit_tutorials/doc/time_parameterization/time_parameterization_tutorial.html — TOTG can move waypoints within tolerance; recheck collisions and TCP precision after path-changing smoothing.
- Tesseract/Descartes: https://tesseract-robotics.github.io/tesseract_nanobind/user-guide/planning/ — graph search for Cartesian waypoint planning. Candidate selection across a full segment is a better long-term direction than committing a greedy single-frame fallback. A graph of position-only IK candidates does not itself enforce our acceleration constraints.
- Ruckig: https://docs.ruckig.com/tutorial.html — future feasibility of velocity/acceleration/jerk states matters. Endpoint interpolation is not a replacement for collision- and Cartesian-constrained retargeting.

## Implemented experiments

`configs/kuavo_continuity.json` enables directional joint stopping-distance constraints on body and arms, and minimum horizon length 2 whenever at least two valid future targets exist. Original TCP tolerances, joint limits, collision margins and acceleration limits remain unchanged. The default `kuavo_retarget.json` remains unchanged. `--end-seconds` supports explicit source windows; `--time-scale 2` is a separate execution clock, not changed raw tracking timestamps.

A 90-frame 1x diagnostic passed 85 frames in 96.42 s. The 2x full-window experiment is assessed separately; slower execution must not be described as faster source capture or real-time operation.

Temporary path/TOPPRA experiments first solve static poses and then construct cubic joint/base paths. The initial fit was rejected after midpoint validation. A stricter fit (2 mm, 0.01 rad, 12 mm collision margin and extra 0.01 rad joint margin) reserves interpolation error. Dense path checks include TCP, joint bounds, self-collision and interpolated torso anchors. These checks remain finite sampling, not a proof of continuous collision freedom. TOPPRA uses conservative XY component bounds for planar speed/acceleration norms. Failed paths and tracking gaps are not silently connected.

Measured geometry-plus-retiming experiment: 10.77 s, 582 static poses. Initial path checks admitted 540 frames, but independent 21-sample-per-source-interval checks admitted only 450. Therefore this experiment is not promoted to the default. Geometry-only success must not be compared with full continuous trajectory success.

## Next engineering direction

Generate multiple accurately verified IK candidates at each waypoint, select a consistent elbow/wrist branch across an entire valid tracking segment, and refine failing Cartesian intervals before time parameterization. Preserve margin for interpolation, then independently validate the actual timed path. GPU batch candidate generation plus short local refinement can reduce repeated 90-variable CPU solves, but requires model/TCP/collision equivalence checks before adoption. Do not hide tracking gaps or relax pose tolerances to manufacture continuity.
