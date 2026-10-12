# One XRT app: RGB + Skeleton + AprilTag

Implementation branch: `codex/unified-quest-capture`. Receiver/transport/artifact tests pass;
Unity compilation, APK installation, Quest volume-key dispatch, and physical alignment still require verification.
The previously tested companion app is a legacy capture mode, not this implementation.

The existing XRT MainScene already contains the right PCA camera, full-body provider,
and tag36h11 tracker. Warp patches that scene's scripts and uses one RGBA readback for
JPEG recording and grayscale AprilTag detection. No second camera app is involved.
The normal teleop channels and their play/pause safety gate are preserved.

## Quest control

There is no recording button on screen. Press **volume up** once to start and once again
to stop. Held-key repeats are ignored. Volume down keeps its usual behavior.
The control works while XRT is foreground; system menus can intercept hardware keys.

The small headset status label uses English only:

- `Preparing…`: connection/configuration or a start acknowledgement is pending.
- `Ready`: lab6 has configured the recorder; connection alone does not record.
- `Recording · 00:23`: server acknowledged start; elapsed time is shown.
- `Saving…`: stop acknowledged; lab6 is finishing and verifying the video.
- `Saved`: lab6 saved a complete episode and verified its MP4 frame count.
- `Recording error` / `Connection lost`: capture or acknowledgement failed.

While recording, existing canvases, skeleton visualization spheres and tag axes are
hidden by default. Their previous state is restored afterward. The physical camera
images never contain Unity UI overlays, independent of headset display visibility.

## Shared timestamps

The canonical `timestamp_ticks` is the original PCA camera timestamp, in Quest UTC
100 ns ticks. Skeleton samples retain their original `provider_query_utc` timestamps.
A bounded history interpolates position/quaternion between the samples bracketing
the camera timestamp, with a maximum 50 ms distance on each side and no extrapolation.
The bundle saves both original samples and the aligned skeleton.

The body provider API used here does not expose the skeleton sensor exposure time.
Consequently this is explicit temporal alignment of queried poses, not a claim of
hardware simultaneous exposure. PC receive time is never substituted for capture time.

RGB, aligned Skeleton and completed AprilTag results share one `frame_id` and one
timestamp. Zero detected tags is saved as an empty result; busy/error/incomplete
detection never becomes a successful tag-free frame. Missing clock brackets, camera
calibration or stalled capture fail closed. The receiver rechecks interpolation and
saved CSV timestamps. Default capture rate is 10 fps; original teleop streaming remains
independent. The source samples in each capture bundle are retained, not a continuous
90 Hz skeleton recording.

## Build and deploy

```bash
bash scripts/bootstrap.sh capture
bash scripts/01_capture/install_receiver.sh
# Unity 6000.3.11f1, Android support, and an activated Unity license are required.
export UNITY_EDITOR=/absolute/path/to/Unity
python3 scripts/01_capture/build_unified_quest.py --server-ip LAB6_IP
```

The build script extracts the pinned upstream revision into `.venv/unity-xrt-unified`,
applies the tracked patch and copies the capture/status sources. It builds the existing
`MainScene`, rather than the ZED-only scene selected by upstream's current build settings.
APK/signing keys/Editor caches are not committed. To repeat preparation, use a fresh
`--project` path; existing project directories are never overwritten.

Before deploying, stop the legacy companion and restore XRT camera permissions using
the private snapshot created by the earlier preparation command:

```bash
python3 scripts/01_capture/prepare_quest_rgb.py --adb /absolute/path/to/adb \
  --serial QUEST_SERIAL --restore-xrt-camera
adb -s QUEST_SERIAL install -r .venv/android-build/xrt-unified.apk
```

An APK signed by a different key cannot replace the installed release. Do not uninstall
the existing app and erase its preferences without preserving them and resolving the
signing/install migration explicitly. New launcher activity: `dev.warp.capture.WarpVolumeActivity`.

## lab6 recorder

```bash
bash scripts/01_capture/record_body.sh --session take_0001 --wait-seconds 300 \
  --rgb-fps 10 --tag-size-m 0.0784
```

Use the actual printed tag width for `--tag-size-m`. The shown value is the upstream
app's existing setting, not a measurement of your tag. Omit the option to keep that
app setting. `--duration` optionally imposes a server-side stop after first recorded
frame; volume up can stop earlier. A server invocation saves one episode without
overwriting; start another invocation for another take. `--keep-overlays` retains
the existing headset visualization. `--body-only` explicitly uses the legacy body receiver.

Artifacts: `body.csv`, `apriltag.csv`, `capture.jsonl` including original pose brackets,
`rgb/frame_*.jpg`, `rgb/timestamps.csv`, `rgb/video.mp4`, and `metadata.json`.

```bash
bash scripts/01_capture/validate_body.sh local_data/raw_human/take_0001 --strict-sew
```

Reference: [Meta PCA API v85](https://developers.meta.com/vr/reference/mruk/v85/class_meta_x_r_passthrough_camera_access/).
Its asynchronous direct readback avoids pairing the previous Blit texture with the latest timestamp.
