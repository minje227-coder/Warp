# KUAVO topic execution

Implementation: kuavo_replay.py. This script owns artifact packaging/validation,
mode and receiver selection, first-state preparation, current-clock replay,
feedback completion and cancellation. It is importable without ROS.
The executor consumes the final artifacts produced by the adopted retargeting pipeline.

The dashboard selects an episode/range, sends the script contents to Up through
RobotLink, feeds configure/vision/heartbeat messages, and exposes stop/status.
It does not implement another robot controller. The compatibility path
humanoid/robot/agent/retargeting_align.py is a symlink to this script.
Override the dashboard's WARP_ROBOT_EXECUTOR environment variable only to select
an explicit compatible executor file; there is no fallback to an older worker.

## Preparation
- Lock the initial reliable rack observation into a local odom goal.
- Select MPC 3, arm 2, ROS arm transport 2, quickMode 0.
- Send one relative /cmd_pose Twist: forward/left metres and yaw radians.
  Require fresh /lb_cmd_pose_reach_time acknowledgement, then measured odom arrival.
  The native controller owns base speed/acceleration; dashboard 5cm/s limits
  from the old streaming alignment path do not apply.
- Publish /cmd_lb_torso_pose from URDF-derived four-joint FK: linear.x/z metres,
  angular.y pitch and angular.z yaw radians. The torso retains its slow profile.
- Publish fourteen arm degree positions on /kuavo_arm_traj over three seconds.
  Three seconds describes the transmitted profile; native MPC and measured
  arrival may take longer. Do not infer physical completion from elapsed time.
- Prepare the two percent-valued claws after joint arrival, send the final target
  once, then wait for fresh hardware position and reached/grabbed feedback.
- Head commands are excluded. Completion requires both odom and joint/claw arrival.
  A cancelled preparation cancels its own relative goal and holds measured posture.

## Replay
- Read the final adopted 100Hz NPZ, source claw timing and validation report.
  An explicit existing per-selection standard override remains labelled as such.
- Publish the sample for actual monotonic elapsed time. Missed time slots are skipped,
  never flushed as a queue of old targets, and >50ms execution gaps still abort.
- Base /cmd_vel plus joint /lb_leg_traj and /kuavo_arm_traj use the existing
  MPC 3 / arm 2 / quickMode 3 direct-joint contract after current-pose priming.
- Original P/V/A arrays and 1x clock are preserved in the requested samples/trace.
  JointState has no acceleration field. The installed controller can filter/limit
  targets and replace velocity with position differences, so controller_pva_exact
  is false; requested trajectory timing is not a claim of identical hardware P/V/A.
- No Kalman filter is enabled.

## Protocol and verification
RobotLink transmits this single file as source; no Warp checkout is needed on Up.
The runtime requires the existing ROS1 workspace and current /robot_description.
No motion starts without an explicit configure packet, fresh page/vision leases,
sensor feedback, controller readiness and exclusive command ownership.

On a ROS-enabled host, --check-runtime performs imports, message serialization,
URDF parsing and service type-header checks only. It creates no actuator publisher
and calls no controller setter. Ordinary imports also create no ROS interface.
No daemon or new ROS service is installed by this change.

Pure tests:
    python -m unittest discover -s scripts/05_execute/tests
Full virtual ROS/main-loop integration tests remain in the dashboard repository:
    PYTHONPATH=.:tests python -m unittest discover -s tests -p 'test_retargeting*.py'

Do not use a successful virtual test as proof of live trajectory tracking.
