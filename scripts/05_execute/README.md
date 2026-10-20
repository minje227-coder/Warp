# KUAVO topic execution

Implementation: kuavo_replay.py. This script owns artifact packaging/validation,
mode and receiver selection, first-state preparation, current-clock replay,
feedback completion and cancellation. It is importable without ROS.
The adopted retargeting/IK/timing pipeline is unchanged.

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

## Automatic replay data collection

The dashboard runs record_replay.py on Up and waits for all three RGB-D cameras,
CameraInfo, sensor/claw state, odom, TF and rosbag subscriptions before starting
the existing executor. Missing or stale required streams prevent replay; stream
loss during replay stops that run. It uses the existing rosbag backend and reads
rosbag_data.sh --list without changing the team recorder or manual recording UI.

Raw bags remain on Up at ~/kuavo_dataset/rosbags/YYYY-MM-DD/warp_replay_<run-id>.bag.
Matching .urdf and .qc.json files retain robot_description/hash and capture QC.
The dashboard stores the same run ID, remote host/path, bag hash, selected source
artifact hashes and a local trace.jsonl/manifest.json under data/retargeting/runs/.
This does not download raw bags to lab6 or create MP4/LeRobot exports.

The executor adds original ROS nanoseconds, sensor/claw stamps, head frame stamp
and worker monotonic/execution seconds to the trace. Start/end and periodic clock
anchors are also recorded in the bag's own /warp/replay/events/<run-id> topic.
Original image/header stamps and requested P/V/A samples are preserved.

After the executor stops, only this run's bag process receives SIGINT. QC checks
normal closure, required topic counts, source image timestamp order, clock offset,
coverage, gaps and minimum rates (images 2 Hz, states 5 Hz, command streams 20 Hz).
The final claw setpoint remains held while arrival is verified: command coverage
ends at the planned endpoint, whereas camera/state coverage includes settling.
Disk readiness requires 10 GiB free on Up. Invalid/partial files are kept.

History displays execution completion, collection QC and manual task success as
separate fields. After saving, the operator can label success/failure/unlabeled
and add a note; executing the trajectory alone never marks the task successful.
The recorder creates only its own event publisher and read-only subscriptions;
all actuator commands remain in kuavo_replay.py. No live movement is exercised
by tests. The new lifecycle needs a dashboard restart to load.

## Application ownership

All retargeting application/backend behavior lives in this Warp directory:

- `kuavo_replay.py`: robot-side preparation/replay and package/PVA contract.
- `record_replay.py`: robot-side full-data recording, readiness and bag QC.
- `observe_replay.py`: read-only timestamped head RGB/TF observations.
- `kuavo_execution/session.py`: ownership/leases, complete start/stop lifecycle,
  capture-before-motion ordering, trace/manifest writing and tracking statistics.
- `kuavo_execution/capture.py`: bounded recorder protocol and shutdown/QC lifecycle.
- `kuavo_execution/history.py`: history/file lookup and manual task-result labels.
- `kuavo_execution/catalog.py`, `first_state.py`, `package.py`, `timeline.py`,
  `validation.py`: episode/selection state, first-state/readiness gates, adopted
  package validation, video-time mapping and existing report summaries.
- `kuavo_execution/head_stream.py`, `standard_rack*.py`: read-only observation
  transport and the same adopted tag-estimator bridge.
- `kuavo_execution/compute*.py`: the existing persistent two-slot offline queue
  and camera/video preparation. Native IK/TOPPRA/validation/rendering still run
  through the adopted standard wrapper; no solver or selection policy changes.

The dashboard retains HTTP token checks, request/response routing, UI and tiny
resource-binding/compatibility imports. It supplies RobotLink, shared controller
ownership state, data-directory resolution, ROS setup and read-only inspection of
other controls. It contains no alternate session, manifest, statistics, recorder,
history, selection/readiness or trajectory implementation. Compatibility imports
reference the same module objects, so there is one set of session locks and state.
No backend module imports `tabs`, `core` or the dashboard `datadir` module.

`WARP_EXECUTION_ROOT` selects this directory for a dashboard deployment. By default
it is the sibling `Warp/scripts/05_execute`; an explicitly configured
`WARP_ROBOT_EXECUTOR` also selects its parent. Missing/mismatched packages fail
rather than falling back. Keep the existing executor/recorder override paths in
that same root. Restart the dashboard after changing roots.

Other hosts can import `kuavo_execution` directly. Bind `catalog.configure()` to
transport/storage, then `session.configure()` to that catalog, the control store,
storage, ROS setup and optional callbacks that report other controllers. Importing
these modules starts no process, network connection, recorder or actuator. The
standalone history provider takes an explicit `DataDirectory(root)` or
`WARP_EXECUTION_DATA_ROOT`; the compute CLI uses `KUAVO_DATA` or that root, with
`~/Warp_data` as the standalone default. Spawned compute workers inherit the
explicit data root. Existing live data and adopted CURRENT are never relocated.

Standalone tests (including host-import isolation and complete fake lifecycle):

    python -m unittest discover -s scripts/05_execute/tests

### Dashboard adapters are versioned in Warp

`dashboard_adapter/` contains only the small host bindings/compatibility launchers
and their README. `install_dashboard_adapter.py` copies those files and creates
relative observer/executor compatibility links, with a dated backup of every
replaced file. This lets the existing dashboard connect without publishing its
unrelated local commits or copying application implementation back out of Warp.
The host must already use the tab-based dashboard/retargeting HTTP UI; the older
Gradio-only dashboard is intentionally not migrated by this installer.

Preview, then apply while the dashboard's recording/motion sessions are idle:

    python scripts/05_execute/install_dashboard_adapter.py --dashboard-root /path/to/humanoid
    python scripts/05_execute/install_dashboard_adapter.py --dashboard-root /path/to/humanoid --apply

The installer does not restart services or launch/connect to any robot. Restart
the dashboard after installation. An idle compute supervisor also needs a restart
to load the relocated queue implementation; retain its existing data environment.
