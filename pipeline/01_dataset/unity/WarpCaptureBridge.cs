using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

// Added to the existing XRT controller. No second camera or Android application.
public class WarpCaptureBridge : MonoBehaviour
{
    [Serializable] public class Sample {
        public long timestamp_ticks;
        public BodyPoseProvider.BoneData[] bones;
    }
    [Serializable] public class Bundle {
        public int version = 1;
        public string clock = "quest_utc_ticks_100ns";
        public string skeleton_time_source = "provider_query_utc";
        public string alignment = "interpolated_between_provider_samples";
        public long frame_id, timestamp_ticks;
        public int width, height;
        public string eye, tag_family = "tag36h11";
        public float tag_size_m;
        public float fx, fy, cx, cy;
        public Pose camera_pose;
        public Sample before, after;
        public BodyPoseProvider.BoneData[] bones;
        public QuestAprilTagTracker.TagResult[] tags;
    }
    public bool Armed { get; private set; }
    public WebRTCController controller;
    [Serializable] class Command {
        public string type, state;
        public float rgb_fps, tag_size_m;
        public bool clean_view;
    }
    WarpRecordingStatus status;
    bool configured, waitingForAck, cleanView;
    bool terminalState;
    double requestAt;
    readonly List<Sample> history = new List<Sample>();
    BodyPoseProvider body;
    QuestCameraExtractor camera;
    QuestAprilTagTracker tracker;
    Bundle pending;
    byte[] pendingJpeg;
    long nextFrame;

    public void SetArmed(bool armed) {
        Armed = armed; pending = null; pendingJpeg = null;
        if (!armed && status != null) status.SetClean(false);
    }

    public void HandleServerCommand(string json) {
        var command = JsonUtility.FromJson<Command>(json);
        if (command.type == "config" && camera != null && tracker != null) {
            configured = true; terminalState = false; cleanView = command.clean_view;
            camera.captureInterval = 1.0 / Mathf.Clamp(command.rgb_fps, 1, 30);
            if (command.tag_size_m > 0) tracker.tagSizeMeters = command.tag_size_m;
            status.SetState("Ready");
        } else if (command.type == "state") {
            waitingForAck = false;
            if (command.state == "recording") {
                Armed = true; status.SetClean(cleanView); status.SetState("Recording");
            } else {
                Armed = false; pending = null; pendingJpeg = null;
                status.SetState(command.state == "saving" ? "Saving…" :
                    command.state == "saved" ? "Saved" : "Recording error");
                if (command.state != "saving") status.SetClean(false);
                terminalState = command.state == "saved" || command.state == "error";
            }
        }
    }

    public void ToggleRecording() {
        if (!configured || waitingForAck) return;
        if (controller.SendWarpControl(Armed ? "WARP_REQUEST_STOP" : "WARP_REQUEST_START")) {
            waitingForAck = true; requestAt = Time.realtimeSinceStartupAsDouble;
            status.SetState(Armed ? "Saving…" : "Preparing…");
        }
    }

    void Start() {
        body = controller.bodyPoseProvider;
        camera = FindFirstObjectByType<QuestCameraExtractor>(FindObjectsInactive.Include);
        if (body == null || camera == null || camera.tracker == null) {
            Debug.LogError("[Warp] Existing body/camera/AprilTag components are required.");
            enabled = false; return;
        }
        tracker = camera.tracker;
        camera.capture = this;
        var hud = new GameObject("WarpCaptureStatus");
        status = hud.AddComponent<WarpRecordingStatus>(); status.capture = this;
        body.OnPoseUpdated += OnBody;
        tracker.OnFrameDetected += OnTags;
    }

    void OnBody(BodyPoseProvider.PoseData pose) {
        if (pose.bones == null || pose.bones.Count == 0) return;
        long timestamp = pose.timestampTicks;
        if (history.Count > 0 && timestamp <= history[history.Count - 1].timestamp_ticks) {
            // Clock adjustment / restart: never interpolate across a clock discontinuity.
            history.Clear(); pending = null;
        }
        history.Add(new Sample { timestamp_ticks = timestamp, bones = pose.bones.ToArray() });
        while (history.Count > 180) history.RemoveAt(0);
    }

    public bool Submit(long timestamp, byte[] jpeg, int width, int height, string eye,
                       float fx, float fy, float cx, float cy, Pose cameraPose,
                       out long frameId) {
        frameId = 0;
        if (!Armed || pending != null || timestamp <= 0) return false;
        Sample before = null, after = null;
        for (int i = 0; i < history.Count; i++) {
            if (history[i].timestamp_ticks <= timestamp) before = history[i];
            if (history[i].timestamp_ticks >= timestamp) { after = history[i]; break; }
        }
        // No extrapolation, stale skeleton reuse, or fabricated exposure timestamps.
        const long maxDistanceTicks = 500000; // 50 ms on either side
        if (before == null || after == null || timestamp - before.timestamp_ticks > maxDistanceTicks ||
            after.timestamp_ticks - timestamp > maxDistanceTicks || before.bones.Length != after.bones.Length)
            return false;
        var bones = new BodyPoseProvider.BoneData[before.bones.Length];
        float alpha = after.timestamp_ticks == before.timestamp_ticks ? 0 :
            (float)((double)(timestamp - before.timestamp_ticks) / (after.timestamp_ticks - before.timestamp_ticks));
        for (int i = 0; i < bones.Length; i++) {
            if (before.bones[i].id != after.bones[i].id) return false;
            bones[i] = new BodyPoseProvider.BoneData {
                id = before.bones[i].id,
                position = Vector3.Lerp(before.bones[i].position, after.bones[i].position, alpha),
                rotation = Quaternion.Slerp(before.bones[i].rotation, after.bones[i].rotation, alpha)
            };
        }
        frameId = ++nextFrame;
        pending = new Bundle { frame_id = frameId, timestamp_ticks = timestamp,
            width = width, height = height, eye = eye, fx = fx, fy = fy, cx = cx, cy = cy,
            camera_pose = cameraPose, before = before, after = after, bones = bones,
            tag_size_m = tracker.tagSizeMeters };
        pendingJpeg = jpeg;
        return true;
    }

    void OnTags(QuestAprilTagTracker.TagFrame result) {
        if (pending == null || result.frameId != pending.frame_id || result.timestampTicks != pending.timestamp_ticks) return;
        if (result.error == null && Armed) {
            pending.tags = result.tags; // Empty array is a completed detection with zero tags.
            var metadata = System.Text.Encoding.UTF8.GetBytes(JsonUtility.ToJson(pending));
            using (var stream = new MemoryStream()) using (var writer = new BinaryWriter(stream)) {
                writer.Write(metadata.Length); writer.Write(metadata); writer.Write(pendingJpeg);
                controller.SendWarpCapture(pending.frame_id, stream.ToArray());
            }
        } else Debug.LogWarning("[Warp] Capture rejected: " + result.error);
        pending = null; pendingJpeg = null;
    }

    public void RejectPending() { pending = null; pendingJpeg = null; }

    void OnDestroy() {
        if (body != null) body.OnPoseUpdated -= OnBody;
        if (tracker != null) tracker.OnFrameDetected -= OnTags;
        if (status != null) Destroy(status.gameObject);
    }

    void Update() {
        if (waitingForAck && Time.realtimeSinceStartupAsDouble - requestAt > 3) {
            waitingForAck = false; Armed = false;
            terminalState = true; pending = null; pendingJpeg = null;
            status.SetClean(false); status.SetState("Connection lost");
        }
        if (!controller.IsConnected && status != null && !terminalState) {
            bool lost = configured || Armed || waitingForAck;
            configured = false; Armed = false; waitingForAck = false;
            pending = null; pendingJpeg = null;
            status.SetClean(false); status.SetState(lost ? "Connection lost" : "Preparing…");
            terminalState = lost;
        }
    }
}
