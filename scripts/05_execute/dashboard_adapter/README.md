# Warp-backed retargeting dashboard

All preparation, replay, data collection and offline retargeting backend logic is
owned by `/home/lab6/Warp/scripts/05_execute`. The dashboard retains its UI,
HTTP token checks/routing and thin adapters that bind transport/storage and import
the matching Warp modules. There is no copied lifecycle, manifest/statistics,
history/label, selection/readiness, validation or compute implementation here.

`warp_backend.py` loads the package once from `WARP_EXECUTION_ROOT` (default:
sibling `Warp/scripts/05_execute`). Existing `WARP_ROBOT_EXECUTOR` and
`WARP_REPLAY_RECORDER` overrides remain supported; missing/mismatched roots fail.
The compatibility modules resolve to the same Warp objects and shared locks.

Start/stop, page leases, controller ownership, first-state validation, recording
readiness, full-data QC and manual result labels keep their existing API/behavior.
No Kalman is enabled; recorded requested P/V/A and 1x time are preserved. Physical
controller tracking is distinct from requested P/V/A. Head joints are excluded.

Raw bags remain on Up under `~/kuavo_dataset/rosbags/YYYY-MM-DD/`. Lab6 keeps
`data/retargeting/runs/<run-id>/trace.jsonl` and `manifest.json`, including the bag
host/path/hash, raw clocks, tracking errors and collection QC. A completed replay
never labels the physical task successful automatically. Operators label task
success/failure only after saving. Manual/team recording and VR nodes are unchanged.

The read-only head observer and compatibility executor paths in `robot/agent/`
point to Warp. Offline computation is a thin CLI launcher as well; it invokes the
same adopted standard through Warp's queue and keeps the existing live data root.
The four-window renderer, adopted pipeline, raw input and CURRENT remain unchanged.

See Warp's `scripts/05_execute/README.md` for implementation and protocol details.

    cd webpage
    ../.venv/bin/python -m unittest discover -s tests -p 'test_retargeting*.py'

Tests use virtual ROS and fake processes; no physical robot execution is implied.
