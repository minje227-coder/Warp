using System;
using System.Threading.Tasks;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using Meta.XR;

// Replaces the existing extractor, keeping its serialized camera/tracker references.
[DefaultExecutionOrder(1000)]
public class QuestCameraExtractor : MonoBehaviour
{
    public PassthroughCameraAccess cameraAccess;
    public QuestAprilTagTracker tracker;
    [NonSerialized] public WarpCaptureBridge capture;
    int inFlight;
    class DetectionWork {
        public byte[] gray; public int width, height;
        public double fx, fy, cx, cy; public Pose pose;
        public long frameId, timestamp;
    }
    readonly Queue<DetectionWork> detectionQueue = new Queue<DetectionWork>();
    long lastTimestamp;
    long observedTimestamp;
    double perfReportAt;
    int loops, cameraFrames, busySkips, accepted, submitSkips, callbacks;
    double gpuMs, pixelsMs, jpegMs;
    double nextCaptureTicks;
    [NonSerialized] public double captureInterval = 1.0 / 30.0;
    void Start() {
#if UNITY_ANDROID && !UNITY_EDITOR
        const string permission = "horizonos.permission.HEADSET_CAMERA";
        if (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(permission))
            UnityEngine.Android.Permission.RequestUserPermission(permission);
#endif
        if (tracker != null && cameraAccess != null)
            tracker.RayProjector = uv => cameraAccess.ViewportPointToRay(uv);
    }

    void LateUpdate() {
        if (capture != null && (capture.Armed || capture.Draining)) TryDetection();
        else detectionQueue.Clear();
        if (capture != null && capture.Draining) return;
        if (capture != null && capture.Armed) {
            loops++;
            if (cameraAccess != null && cameraAccess.IsPlaying && cameraAccess.IsUpdatedThisFrame &&
                cameraAccess.Timestamp.Ticks > observedTimestamp) {
                observedTimestamp = cameraAccess.Timestamp.Ticks; cameraFrames++;
            }
            if (inFlight >= 2) busySkips++;
            double now = Time.realtimeSinceStartupAsDouble;
            if (perfReportAt == 0) perfReportAt = now;
            if (now - perfReportAt >= 5) {
                double dt = now - perfReportAt;
                Debug.Log($"[WarpPerf] loop_fps={loops/dt:F1} camera_fps={cameraFrames/dt:F1} gpu_busy_skips={busySkips} readbacks={callbacks} accepted={accepted} submit_rejected={submitSkips} gpu_wait_ms={gpuMs/Math.Max(1,callbacks):F2} pixels_ms={pixelsMs/Math.Max(1,callbacks):F2} jpeg_ms={jpegMs/Math.Max(1,callbacks):F2}");
                loops = cameraFrames = busySkips = callbacks = accepted = submitSkips = 0;
                gpuMs = pixelsMs = jpegMs = 0; perfReportAt = now;
            }
        } else { perfReportAt = 0; }

        if (inFlight >= 2 || cameraAccess == null || !cameraAccess.IsPlaying || !cameraAccess.IsUpdatedThisFrame ||
            tracker == null || !tracker.isActiveAndEnabled) return;
        var texture = cameraAccess.GetTexture();
        long timestamp = cameraAccess.Timestamp.Ticks;
        if (texture == null || timestamp <= lastTimestamp) return;
        double intervalTicks = captureInterval * TimeSpan.TicksPerSecond;
        if (nextCaptureTicks == 0) nextCaptureTicks = timestamp;
        // Phase anchored to camera time; tolerate camera jitter at the slot boundary.
        if (timestamp + intervalTicks * 0.25 < nextCaptureTicks) return;
        int width = texture.width, height = texture.height;
        var intrinsics = cameraAccess.Intrinsics;
        if (intrinsics.FocalLength.x <= 0 || intrinsics.FocalLength.y <= 0 ||
            intrinsics.SensorResolution.x <= 0 || intrinsics.SensorResolution.y <= 0) return;
        Pose cameraPose = cameraAccess.GetCameraPose();
        if (cameraPose == default) return;
        // Meta SDK center-crop rule: intrinsics refer to the full sensor, not output dimensions.
        float scale = Mathf.Max((float)width / intrinsics.SensorResolution.x,
                                (float)height / intrinsics.SensorResolution.y);
        float cropX = (intrinsics.SensorResolution.x - width / scale) * 0.5f;
        float cropY = (intrinsics.SensorResolution.y - height / scale) * 0.5f;
        float fx = intrinsics.FocalLength.x * scale, fy = intrinsics.FocalLength.y * scale;
        float cx = (intrinsics.PrincipalPoint.x - cropX) * scale;
        float cy = (intrinsics.PrincipalPoint.y - cropY) * scale;
        string eye = cameraAccess.CameraPosition.ToString().ToLowerInvariant();
        lastTimestamp = timestamp;
        nextCaptureTicks += (Math.Floor(Math.Max(0, timestamp - nextCaptureTicks) / intervalTicks) + 1) * intervalTicks;
        inFlight++;
        // Direct readback ordered after PCA's render-thread texture update. A blocking Blit
        // would capture the previous texture and attach the wrong image timestamp.
        double requested = Time.realtimeSinceStartupAsDouble;
        AsyncGPUReadback.Request(texture, 0, TextureFormat.RGBA32, request => {
            inFlight--;
            if (request.hasError || !isActiveAndEnabled) return;
            callbacks++;
            double pixelsStart = Time.realtimeSinceStartupAsDouble;
            gpuMs += (pixelsStart-requested)*1000;
            var source = request.GetData<Color32>();
            if (source.Length < width * height) return;
            var rgba = new byte[width * height * 4];
            var gray = new byte[width * height];
            // Quest PCA readback already has upright row order on this player.
            // Preserve it for both JPEG and detection; an extra Y flip inverts the scene.
            for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) {
                var c = source[y * width + x];
                int p = y * width + x;
                rgba[p * 4] = c.r; rgba[p * 4 + 1] = c.g;
                rgba[p * 4 + 2] = c.b; rgba[p * 4 + 3] = 255;
                gray[p] = (byte)((77 * c.r + 150 * c.g + 29 * c.b + 128) >> 8);
            }
            pixelsMs += (Time.realtimeSinceStartupAsDouble-pixelsStart)*1000;
            long frameId = 0;
            if (capture != null && capture.Armed) {
                // Reserve the skeleton bracket now; delayed JPEG completion must not
                // query a later skeleton or overwrite another frame's metadata.
                if (!capture.Submit(timestamp, null, width, height, eye, fx, fy, cx, cy,
                                    cameraPose, out frameId)) { submitSkips++; return; }
                accepted++;
                long jpegFrameId = frameId;
                Task.Run(() => {
                    var timer = System.Diagnostics.Stopwatch.StartNew();
                    try {
                        byte[] jpeg = ImageConversion.EncodeArrayToJPG(rgba, GraphicsFormat.R8G8B8A8_UNorm,
                            (uint)width, (uint)height, (uint)(width * 4), 90);
                        double ms = timer.Elapsed.TotalMilliseconds;
                        UnityMainThreadDispatcher.Instance().Enqueue(() => {
                            jpegMs += ms;
                            if (capture != null) capture.CompleteJpeg(jpegFrameId, jpeg);
                        });
                    } catch (Exception error) {
                        UnityMainThreadDispatcher.Instance().Enqueue(() => {
                            Debug.LogError("[Warp] JPEG encode failed: " + error.Message);
                            if (capture != null) capture.RejectPending(jpegFrameId);
                        });
                    }
                });
            }
            Process(gray, width, height, fx, fy, cx, cy, cameraPose, frameId, timestamp);
        });
    }

    void Process(byte[] gray, int width, int height, double fx, double fy, double cx,
                 double cy, Pose pose, long frameId, long timestamp) {
        if (frameId == 0) {
            ProcessDirect(gray,width,height,fx,fy,cx,cy,pose,frameId,timestamp); return;
        }
        if (detectionQueue.Count >= 8) { capture.FailCapture("AprilTag processing cannot sustain camera rate"); return; }
        detectionQueue.Enqueue(new DetectionWork { gray=gray,width=width,height=height,
            fx=fx,fy=fy,cx=cx,cy=cy,pose=pose,frameId=frameId,timestamp=timestamp });
        TryDetection();
    }

    void TryDetection() {
        if (detectionQueue.Count == 0) return;
        var job = detectionQueue.Peek();
        if (ProcessDirect(job.gray,job.width,job.height,job.fx,job.fy,job.cx,job.cy,
                          job.pose,job.frameId,job.timestamp)) detectionQueue.Dequeue();
    }

    unsafe bool ProcessDirect(byte[] gray, int width, int height, double fx, double fy,
                              double cx, double cy, Pose pose, long frameId, long timestamp) {
        fixed (byte* ptr = gray) {
            return tracker.ProcessRawCameraFrame((IntPtr)ptr,width,height,width,
                fx,fy,cx,cy,pose,frameId,timestamp);
        }
    }
}
