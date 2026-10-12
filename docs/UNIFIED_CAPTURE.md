# One XRT app: RGB + Skeleton + AprilTag

Implementation branch: `main`. Receiver/transport/artifact tests pass;
Unity compilation, APK installation, physical volume-up start/stop and Saved were verified.
The first 71-frame episode passed shared timestamps and detected tag ID 2, but its RGB was vertically inverted; a camera row-order fix is being rebuilt and needs a new physical clip.
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

While recording, existing canvases, the video viewport, skeleton visualization spheres and tag axes are
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
git -C third_party/capture/xrt-client lfs pull
# Unity 6000.3.11f1, Android support, and an activated Unity license are required.
export UNITY_EDITOR=/absolute/path/to/Unity
python3 scripts/01_capture/build_unified_quest.py --server-ip LAB6_IP
```

The build script extracts the pinned upstream revision into `.venv/unity-xrt-unified`,
applies the tracked patch and copies the capture/status sources. It builds the existing
`MainScene`, rather than the ZED-only scene selected by upstream's current build settings.
APK/signing keys/Editor caches are not committed. To repeat preparation, use a fresh
`--project` path; existing project directories are never overwritten.
Git LFS assets are copied from the vendor checkout only after verifying their SHA-256
and size: `git archive` otherwise leaves small pointer files in place of textures/models.
One import worker is configured for a build host sharing memory with other workloads.
The optional Unity MCP Editor package is omitted because the player does not use it.

On Linux, Meta SDK v85's simulator installer has an Editor compilation bug. After
package resolution, run `build_unified_quest.py --repair-linux-packages --project PATH`
before retrying the build. This makes the unsupported simulator installation fail
explicitly on Linux; Android runtime code is unchanged.

Before deploying, stop the legacy companion and restore XRT camera permissions using
the private snapshot created by the earlier preparation command:

```bash
python3 scripts/01_capture/prepare_quest_rgb.py --adb /absolute/path/to/adb \
  --serial QUEST_SERIAL --receiver-host LAB6_IP --restore-xrt-camera
adb -s QUEST_SERIAL install -r .venv/android-build/xrt-unified.apk
```

An APK signed by a different key cannot replace the installed release. Do not uninstall
the existing app and erase its preferences without preserving them and resolving the
signing/install migration explicitly. New launcher activity: `dev.warp.capture.WarpVolumeActivity`.
For a separately signed build, use `--application-id com.GTSafeRoboticsLab.xrteleopclient.warp`.
Its launcher label is `XRT-Client Warp`; the original release remains installed.

## lab6 recorder

```bash
bash scripts/01_capture/record_body.sh --session take_0001 --wait-seconds 300 \
  --rgb-fps 10 --tag-size-m 0.04
```

Default printed tag side length is **0.04 m (40 mm)**. lab6 sends this setting to
the app before Ready. Use `--tag-size-m` for a different measured printed tag size;
explicit `--tag-size-m 0` keeps the app setting. `--duration` optionally imposes a server-side stop after first recorded
frame; volume up can stop earlier. A server invocation saves one episode without
overwriting; start another invocation for another take. `--keep-overlays` retains
the existing headset visualization. `--body-only` explicitly uses the legacy body receiver.

Default output: `~/Warp_data/YYYY-MM-DD/1`, `2`, and so on, using the lab6 local date.
Folder reservation uses atomic mkdir and never reuses an existing take; failed/reserved
folders also keep their numbers. `--session` labels metadata and does not replace the
numeric folder. `--out-root` optionally changes the root.

Artifacts: `body.csv`, `apriltag.csv`, `capture.jsonl` including original pose brackets,
`rgb/frame_*.jpg`, `rgb/timestamps.csv`, `rgb/video.mp4`, and `metadata.json`.

```bash
bash scripts/01_capture/validate_body.sh ~/Warp_data/YYYY-MM-DD/1 --strict-sew
```

Reference: [Meta PCA API v85](https://developers.meta.com/vr/reference/mruk/v85/class_meta_x_r_passthrough_camera_access/).
Its asynchronous direct readback avoids pairing the previous Blit texture with the latest timestamp.


## Fixed timestep / direct video (30 Hz default)

The receiver writes `rgb/video.mp4` during recording without retaining JPGs.
Video frame index i, body.csv time_elapsed, apriltag.csv time_elapsed and
capture.jsonl time_elapsed all use i / recording_fps. Default is 30 Hz
(33.333333 ms); rational PTS avoids cumulative rounding drift.
`timestamp_ticks` retains the actual source camera timestamp and skeleton brackets.
`source_frame_id` links output slots to the original complete RGB/body/tag bundle.
If source frames miss a slot, the preceding complete bundle is held and marked
`repeated=true`; this applies to all three streams, including tag results.
A repeated tag is a held detection, not a newly detected tag. Incoming bundles
quantized into an already written slot are coalesced and counted. A gap above
2 seconds fails capture. Metadata reports source/repeated/coalesced counts;
30 Hz video output alone does not establish 30 unique camera frames per second.
Camera scheduling uses a camera-time phase instead of waiting from each readback.
Capture artifacts retain original timestamps; no receiver wall clock restamping.
