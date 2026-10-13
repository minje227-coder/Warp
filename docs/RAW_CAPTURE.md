# Quest raw capture v3

The capture package is `com.GTSafeRoboticsLab.xrteleopclient.warp`.
Volume up starts/stops local recording. Capture readiness does not depend on PC signaling.
The app records the right passthrough camera at 1280x1280, requested 30 fps,
using Camera2 -> MediaCodec H.264 Surface -> MediaMuxer MP4 (20 Mbps, 1 s keyframe interval).
No JPEG conversion, live AprilTag detection, skeleton interpolation, or artificial
fixed-FPS padding is in the recording path. The idle square preview shows the
entire PCA output, then releases the camera before native capture starts.

## Independent streams and provenance

Quest stores `WarpRawEpisodes/EPISODE_UUID/` under its external app files directory:

- `camera.mp4`: native encoded camera observations; no user-view/UI overlays.
- `camera_frames.jsonl`: original SENSOR_TIMESTAMP, camera frame number, exposure,
  crop, and callback clock samples. Callback times are not exposure times.
- `encoded_frames.jsonl`: native encoder PTS, flags, sample sizes and index.
- `tracking.jsonl`: independent body samples, head/status samples and any available
  left/right OVRSkeleton hand observations, with joint IDs/names and validity.
- `calibration.json`: Camera2 intrinsics, active array, extrinsics, distortion,
  sensor orientation and timestamp source.
- `events.jsonl`: start/stop, pause, unmount, recenter and clock/camera failures.
- `manifest.json`: schema v3, original file SHA-256/size, counts and settings.

Body values remain in the original MetaSourceDataProvider coordinate convention;
provider transform and tracking origin are retained. Head/hand Transform values
are Unity world metres with xyzw quaternions. A reported valid provider sample
is not a proof of per-joint tracking accuracy. Missing hand streams are reported
by QC rather than synthesized. Camera extrinsics require verification before
camera-relative tag poses can be interpreted in world/robot coordinates.

Camera2 reports timestampSource UNKNOWN on the tested Quest. Native sensor PTS
are retained; body target time, OVR clock, query UTC brackets are also retained.
Do not claim hardware synchronization or substitute PC receive time. An explicit,
independently validated camera-to-UTC offset is required for offline alignment;
offset drift must also be checked. No valid offset is guessed from callback latency.

## Capture and upload states

Main status: Ready -> Starting -> Recording -> Saving -> Ready.
Saving waits for in-flight camera metadata, encoder EOS and local logs to close.
Files are sealed with a manifest before upload. Separate lower HUD text shows
`Upload pending · N/30` and transfer progress/retry status. Failed partial recordings
remain protected and can occupy queue capacity; they are not silently uploaded.

A single native uploader processes sealed episodes oldest first, streams a ZIP
from disk, and retries with exponential backoff. Its queue is reconstructed from
disk on app startup. Network or signaling failure does not stop recording.
Native threads can continue while Unity pauses if Android keeps the process alive;
force-stop/process termination pauses work, which resumes on next app launch.
This release does not claim an always-running OS-scheduled background service.
The active recording has priority: hashing/packaging/upload yields between chunks.

At most 30 raw episode directories are retained. When full or below the free-space
reserve, only the oldest PC-confirmed backup can be evicted; unconfirmed, partial
and currently-uploading episodes are protected. If none can be evicted, a new
recording is refused. The active recording stops safely if space becomes low.
Previously recorded WQL1 backups are retained in their separate legacy directory.

PC acknowledgement means all original files and hashes were durably stored and
video header/dimensions checked, not that perception or learning QC passed.
Original MP4 bytes are preserved. Duplicate IDs with different bytes are rejected.
An interrupted receiver receipt commit can be recovered from durable transfer data.
Hand names are qualified by skeleton type and numeric joint ID, avoiding enum aliases;
provider_valid preserves the SDK flag while uninitialized all-zero hands are excluded
from effective valid coverage and shown as unavailable in the HUD.

## PC receiver and offline processing

```bash
cd /home/lab6/Warp
.venv/bin/pip install -r pipeline/01_dataset/requirements-raw.txt
# Existing offer/legacy-upload support plus raw-v3 /upload_raw:
.venv/bin/python pipeline/01_dataset/record_unified_quest.py --wait-seconds 86400
# Or raw-only standalone receiver:
.venv/bin/python pipeline/01_dataset/raw_receiver.py --port 8080
```

The receiver and the separate serial processing worker are installed as user services:

```bash
mkdir -p ~/.config/systemd/user
cp scripts/01_capture/systemd/*.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now warp-raw-receiver warp-raw-processing
```

The default worker detects tag pixels without metric pose until an actual printed tag size is configured.
Add `--tag-size-m` to its ExecStart or use the manual processor below for calibrated pose.
Persistent `.raw_processing/ID.json` jobs report running/complete/error; interrupted jobs recover.

Use only one listener on a port. Incoming raw episodes live under
`~/Warp_data/YYYY-MM-DD/UUID/`; local timestamps need not be 30-Hz uniform.
The original video, logs, manifest and transfer receipt stay at the episode root.
The worker stages results in `processed.partial/` and atomically publishes
`processed/` only when processing succeeds:

```text
Warp_data/YYYY-MM-DD/UUID/
  camera.mp4
  camera_frames.jsonl, encoded_frames.jsonl, tracking.jsonl, events.jsonl
  calibration.json, manifest.json, transfer.json
  processed/
    quality.json, processing_manifest.json
    apriltag.jsonl, decoded_frames.jsonl, aligned_body.jsonl
```

Existing `raw/YYYY-MM-DD/UUID/` archives and `derived/YYYY-MM-DD/UUID/`
results remain supported in place, including upload retries and pending jobs.

```bash
.venv/bin/python pipeline/01_dataset/process_raw_episode.py \
  EPISODE EPISODE/processed --tag-size-m ACTUAL_PRINTED_TAG_SIZE_METRES
```

Nested output is restricted to the separate `processed/` directory (or its worker
staging directory); original files cannot be used as output. Default processing checks
source hashes, full video decode, PTS sequence/frame counts, tag36h11 detections,
hand/body coverage and sensor gaps. It permits a constant MP4 PTS normalization
shift only after full-sequence verification (1 ms max container residual).
Encoder-to-sensor association uses a separate 2 us tolerance. Pixel/tag detections
remain available without trusted output intrinsics; pose estimation requires a
verified full uncropped calibration or an explicit calibration override.

`--camera-to-utc-offset-ns` enables body alignment only with the supplied offset;
maximum bracket span is 50 ms, no extrapolation, quaternion SLERP, and recenter/
tracking-discontinuity filtering. The operator must validate that offset and drift.
Default quality is `needs_clock_validation`; frame inconsistencies take precedence
as `needs_frame_validation`. No output is automatically labelled dataset-ready.
LeRobot, robot state/action mapping and robot retargeting remain later stages.

## Build and verification

Resolution and Java plugin copying are in tracked `build_unified_quest.py`, not
only the generated `.venv` scene. WarpBuild sets ARM64/IL2CPP and the raw app version.
Do not modify vendor SDK files or overwrite the upstream XRT package.

```bash
.venv/bin/python -m unittest discover -s pipeline/01_dataset/tests -v
```

Camera smoke-test tooling in `scripts/01_capture/smoke_camera/` builds a separate
probe APK from the production native recorder, injects no fake skeleton samples,
and preserves interrupted camera-only results for inspection. Probe artifacts
prove the camera/encoder path; the production Unity lifecycle and wearer hand/body
quality need distinct evidence. Record actual results in release verification notes.

The 3.0.1 raw build guards native start admission while PCA preview is releasing;
an old native Ready state cannot restart preview during that transition. A pause
or unmount cancels a pending start. Native stop drains only a session with an
active repeating request. Upload pending counts sealed, unconfirmed episodes;
interrupted originals remain protected and consume local capacity separately.

On 2026-10-13, three consecutive approximately 3-second production recordings
passed with the receiver paused for the first two takes and resumed during the
third. Start admission took 0.60–0.61 seconds; local finalization took 0.34–0.46
seconds. The queue reached two pending episodes, recording continued during
transfer, and all three receipts arrived with zero pending uploads. Sensor,
encoded, and decoded counts agreed at 102/103/103 frames; observed FPS was 29.85,
maximum sensor gap 50 ms, with no unmatched frames or duplicate decoded PTS.
This unworn test verifies capture and transfer; wearer tracking and camera/body
clock calibration remain separate checks.
