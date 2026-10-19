# Quest raw capture v3

The capture package is `com.GTSafeRoboticsLab.xrteleopclient.warp`.
Volume up starts/stops local recording. Before starting, the preview disappears and
the digits 3, 2, 1 appear just above the existing Ready/status text for one second each,
without a background panel. The digits disappear before native capture starts.
Pausing the app cancels a pending countdown; removing the headset also cancels it in HEAD mode.
Volume down switches HEAD/NECK mounting while idle. The lower-right HUD shows
`Mount: HEAD | Vol -` or `Mount: NECK | Vol -`; NECK is cyan. The selection persists
across app restarts. Countdown, recording and saving lock the mode for that take.
NECK ignores the app's HMD-unmounted stop event and requests Meta's proximity
close override every two seconds while NECK is selected, with a six-second
lease. A temporary Activity stop/VR-focus loss retains this override; HEAD,
Activity finish and Activity destruction release it.
The selected mode is also mirrored to native preferences so NECK can request the
override during Activity startup before Unity resumes. No global sleep settings,
secure settings permissions or separate keep-awake service are changed.
The broadcast hook is firmware-dependent; APK compilation is not proof that the
OS accepts it. Verify neck-mode no-sleep behavior and HEAD restore on the headset.
Other Android/OpenXR pauses still stop recording safely.
This setting does not alter Meta's skeleton inference
or fabricate an anatomical head/neck offset.
Each raw sample and episode manifest records `mount_mode` and `hmd_pose_role`.
In NECK mode the legacy `head_pose` field is the neck-mounted device pose, not the
human head pose; `mount_calibration` and neck body accuracy remain unverified.
Capture readiness does not depend on PC signaling.
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

Main status: Ready -> Starting (3, 2, 1 countdown) -> Recording -> Saving -> Ready.
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

## NECK idle return (3.0.6)

NECK returns to HEAD after 20 minutes of idle time measured with Android's
monotonic elapsed-realtime clock, starting when NECK is entered. Countdown,
camera startup, recording and native drain protect the proximity override;
manual mode changes and the idle return cannot release it during capture.
Native drain completion after camera recording starts a fresh 20-minute idle
period, including a recording interrupted by failure. Canceling countdown or
rejected startup resumes the original idle deadline without resetting it. Upload and preview activity do not reset the period. The Android timer
runs through temporary Unity/VR focus loss, and Unity synchronizes its HUD
and persisted mode to Android. Loading persisted NECK on a new Activity
starts a new idle period. The six-second lease still limits process-crash
residue. No tracking calibration or data-format changes are implied.

## External USB cameras and D435 restart fix (3.0.11)

The capture app supports an optional external UVC camera alongside the Quest
camera. Supported transports are MJPEG over isochronous USB and D435/D435i RGB
YUY2 over bulk USB. Depth and infrared interfaces are not selected. The external
camera uses its largest advertised RGB mode at 30 fps: the tested D435 exposes
1920x1080 on USB 3 and 640x480 on USB 2. Lower-rate modes are not silently used
to satisfy a larger resolution request.

Idle preview and recording own the USB connection sequentially. Recording waits
for preview shutdown before opening its own stream. Version 3.0.11 clears the
bulk endpoint before a new probe/commit and after the native worker has canceled
and reaped outstanding transfers. This supplies the bulk-stream stop operation
that SET_INTERFACE(0) does not provide, fixing `bulk_status_-71` when a working
1080p preview transitions into recording. Shutdown failures remain explicit.
The native transport retains complete frames with their original arrival times;
preview JPEG conversion does not change recorded YUY2 pixels.

External files are `external_camera.json`, `external_camera_frames.jsonl`, and
either `external_camera.yuyv` or `external_camera.mjpeg`. The receiver validates
the declared file set and checksums. Processing exports the external video and
timestamp provenance; paired exports use original observations without padding.
External arrival times are not exposure timestamps. Hardware synchronization,
cross-camera calibration, and body-tracking accuracy are not established by
these capture tests.

On Quest 3 with D435 USB 3, two consecutive main-app recordings completed with
140 and 142 external 1080p frames, zero payload errors, and 152 and 155 Quest
frames respectively. Both videos in each take decoded fully and matched their
manifest SHA-256 values. These are short recording/restart tests, not a
long-duration or server-upload qualification. The final release build only
updates the recording version label after these hardware runs; the tested
camera implementation is unchanged. Release assets include exact APK and
camera-source hashes and verification details.

## Lossless streaming uploads (3.0.12)

Install `requirements-raw.txt` and update/restart the receiver before the updated
app. New uploads stream a deterministic ZIP archive through Zstd level 1. Each
independent Zstd block has at most 1 MiB of source input, a content-size declaration
and checksum. The wire format is `WZB1`, followed by big-endian uint32 compressed
block lengths and frames, terminated by a zero length. Compression and network
transfer run together without a local `upload.zip` or archive-hashing prepass.
Camera modes, raw YUYV pixels, timestamps and source manifest hashes are unchanged.
Existing fixed-length ZIP clients remain supported. The new app requires the
stream-capable receiver.

The sender hashes ZIP bytes and wire bytes as it writes. The receiver bounds both
sizes, validates block dimensions/checksums and the stream terminator, then checks
the sealed manifest hash and every expanded source size/SHA-256. Original files
are durably committed before a receipt includes both archive and transport hashes.
Quest confirms the receipt only when all expected hashes match. Temporary input
is removed on failed/interrupted uploads. Recording retains priority over upload.
Native compression uses the checksum-pinned upstream Android zstd-jni 1.5.7-6 AAR;
both upstream BSD license notices are included in the player resources.

Before sending, Quest checks `/upload_raw/receipt` with the episode ID and exact
manifest hash. A durable receipt repairs a lost HTTP response without resending
the file; a conflicting manifest or missing committed source is not acknowledged.
This recovers completed uploads; partially transferred files still retry from the
start. Receiver validation has a five-minute client response timeout. Old cached
ZIPs are removed only after a matching receipt is confirmed.

On Quest 3, matched wireless trials used the same 99,532,800 source bytes sampled
from the beginning, middle and end of a D435 recording. The existing local ZIP,
archive hash and upload path took 3.17–3.50 s (two trials). Zstd streaming took
1.77–1.84 s (three trials), sending 50,992,633 bytes; every restored sample SHA-256
matched. This is about 1.8 times faster for that sample and Wi-Fi condition, not a
universal speed guarantee. These trials used an isolated receiver and are separate
from full-app upload validation.
