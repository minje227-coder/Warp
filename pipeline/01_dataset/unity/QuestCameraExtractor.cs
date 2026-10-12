using System;
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
    bool busy;
    long lastTimestamp;
    double nextReadback;
    [NonSerialized] public double captureInterval = 0.1;
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
        if (busy || cameraAccess == null || !cameraAccess.IsPlaying || !cameraAccess.IsUpdatedThisFrame ||
            tracker == null || !tracker.isActiveAndEnabled || Time.realtimeSinceStartupAsDouble < nextReadback) return;
        var texture = cameraAccess.GetTexture();
        long timestamp = cameraAccess.Timestamp.Ticks;
        if (texture == null || timestamp <= lastTimestamp) return;
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
        lastTimestamp = timestamp; nextReadback = Time.realtimeSinceStartupAsDouble + captureInterval;
        busy = true;
        // Direct readback ordered after PCA's render-thread texture update. A blocking Blit
        // would capture the previous texture and attach the wrong image timestamp.
        AsyncGPUReadback.Request(texture, 0, TextureFormat.RGBA32, request => {
            busy = false;
            if (request.hasError || !isActiveAndEnabled) return;
            var source = request.GetData<Color32>();
            if (source.Length < width * height) return;
            var rgba = new byte[width * height * 4];
            var gray = new byte[width * height];
            // Canonical top-left image: the JPEG and native tag detector receive the same pixels.
            for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) {
                var c = source[(height - 1 - y) * width + x];
                int p = y * width + x;
                rgba[p * 4] = c.r; rgba[p * 4 + 1] = c.g;
                rgba[p * 4 + 2] = c.b; rgba[p * 4 + 3] = 255;
                gray[p] = (byte)((77 * c.r + 150 * c.g + 29 * c.b + 128) >> 8);
            }
            long frameId = 0;
            if (capture != null && capture.Armed) {
                byte[] jpeg = ImageConversion.EncodeArrayToJPG(rgba, GraphicsFormat.R8G8B8A8_UNorm,
                    (uint)width, (uint)height, (uint)(width * 4), 90);
                if (!capture.Submit(timestamp, jpeg, width, height, eye, fx, fy, cx, cy,
                                    cameraPose, out frameId)) return;
            }
            Process(gray, width, height, fx, fy, cx, cy, cameraPose, frameId, timestamp);
        });
    }

    unsafe void Process(byte[] gray, int width, int height, double fx, double fy, double cx,
                        double cy, Pose pose, long frameId, long timestamp) {
        fixed (byte* ptr = gray) {
            bool accepted = tracker.ProcessRawCameraFrame((IntPtr)ptr, width, height, width,
                fx, fy, cx, cy, pose, frameId, timestamp);
            if (!accepted && frameId != 0 && capture != null) capture.RejectPending();
        }
    }
}
