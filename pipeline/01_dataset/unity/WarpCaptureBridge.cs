using System;
using System.Collections.Generic;
using System.IO;
using System.Collections;
using UnityEngine.Networking;
using UnityEngine;

// Added to the existing XRT controller. No second camera or Android application.
public class WarpCaptureBridge : MonoBehaviour
{
    [Serializable] public class Sample {
        public long timestamp_ticks;
        public long query_start_ticks, query_end_ticks;
        public double body_state_time_seconds, ovr_clock_seconds;
        public BodyPoseProvider.BoneData[] bones;
    }
    [Serializable] public class Bundle {
        public int version = 2;
        public string clock = "quest_utc_ticks_100ns";
        public string skeleton_time_source = "ovr_body_state_time_mapped_to_quest_utc";
        public bool tag_detection_pending = true;
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
    [Serializable] class TagMessage {
        public long frame_id, timestamp_ticks;
        public string clock = "quest_utc_ticks_100ns", tag_family = "tag36h11", error;
        public QuestAprilTagTracker.TagResult[] tags;
    }
    public bool Armed { get; private set; }
    public bool Draining { get; private set; }
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
    class PendingFrame { public Bundle bundle; public byte[] jpeg; public bool tagsReady; public double submittedAt; }
    readonly SortedDictionary<long, PendingFrame> pending = new SortedDictionary<long, PendingFrame>();
    long nextFrame;
    BinaryWriter localWriter;
    string localPath, uploadId;
    bool uploading;
    double localFps = 30;
    double lastWrittenAt;
    [Serializable] class UploadReply { public string state, upload_id, sha256, error; }

    void BeginLocal() {
        uploadId = Guid.NewGuid().ToString("N");
        string directory = Path.Combine(Application.persistentDataPath, "WarpEpisodes");
        Directory.CreateDirectory(directory);
        var backups = new List<string>(Directory.GetFiles(directory, "*.wql"));
        backups.AddRange(Directory.GetFiles(directory, "*.partial"));
        backups.Sort((a,b) => File.GetCreationTimeUtc(a).CompareTo(File.GetCreationTimeUtc(b)));
        while (backups.Count >= 10) {
            string oldest = backups.Find(path => File.Exists(path + ".receipt.json"));
            if (oldest == null) throw new IOException("10 unconfirmed local takes retained; send them before recording");
            File.Delete(oldest); File.Delete(oldest + ".receipt.json"); backups.Remove(oldest);
        }
        localPath = Path.Combine(directory, uploadId + ".partial");
        localWriter = new BinaryWriter(new FileStream(localPath, FileMode.CreateNew, FileAccess.Write, FileShare.Read));
        localWriter.Write(new byte[] {87,81,76,49}); // WQL1
        localWriter.Write(localFps);
        nextFrame = 0; pending.Clear();
        Armed = true; Draining = false; terminalState = false;
        lastWrittenAt = Time.realtimeSinceStartupAsDouble;
        status.SetClean(cleanView); status.SetState("Recording");
    }

    IEnumerator FinishLocal() {
        Armed = false; Draining = true; status.SetState("Saving…");
        double deadline = Time.realtimeSinceStartupAsDouble + 10;
        while (pending.Count > 0 && Time.realtimeSinceStartupAsDouble < deadline) {
            DrainReady(); yield return null;
        }
        if (!Draining) yield break;
        if (pending.Count > 0) { FailCapture("Local JPEG/AprilTag drain timed out; partial take retained"); yield break; }
        Draining = false;
        localWriter.Flush(); localWriter.Dispose(); localWriter = null;
        string complete = Path.ChangeExtension(localPath, ".wql");
        File.Move(localPath, complete); localPath = complete;
        status.SetClean(false);
        yield return UploadLocal();
    }

    IEnumerator UploadLocal() {
        uploading = true; status.SetState("Sending…");
        string hash;
        using (var sha = System.Security.Cryptography.SHA256.Create())
        using (var file = File.OpenRead(localPath))
            hash = BitConverter.ToString(sha.ComputeHash(file)).Replace("-", "").ToLowerInvariant();
        string url = controller.serverUrl.Substring(0, controller.serverUrl.LastIndexOf('/')) +
            "/upload?upload_id=" + uploadId;
        using (var request = new UnityWebRequest(url, "POST")) {
            request.uploadHandler = new UploadHandlerFile(localPath);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/octet-stream");
            request.SetRequestHeader("X-Warp-SHA256", hash);
            request.timeout = 600;
            yield return request.SendWebRequest();
            uploading = false;
            if (request.result == UnityWebRequest.Result.Success) {
                var reply = JsonUtility.FromJson<UploadReply>(request.downloadHandler.text);
                if (reply.state == "saved" && reply.upload_id == uploadId && reply.sha256 == hash) {
                    File.WriteAllText(localPath + ".receipt.json", request.downloadHandler.text);
                    localPath = null; terminalState = true; status.SetState("Saved"); yield break;
                }
            }
            Debug.LogError("[Warp] Upload failed; local take retained: " + request.error + " " + request.downloadHandler.text);
            terminalState = true; status.SetState("Send failed · volume up retries");
        }
    }

    double pendingTotalMs, perfAt;
    int pendingCount, rejectedBusy, rejectedBracket;

    public void SetArmed(bool armed) {
        // A signaling disconnect must not stop a local recording or its drain.
        if (localWriter != null || uploading) return;
        Armed = armed; Draining = false; pending.Clear();
        if (!armed && status != null) status.SetClean(false);
    }

    public void HandleServerCommand(string json) {
        var command = JsonUtility.FromJson<Command>(json);
        if (command.type == "config" && camera != null && tracker != null) {
            configured = true; terminalState = false; cleanView = command.clean_view;
            localFps = Mathf.Clamp(command.rgb_fps, 1, 30);
            camera.captureInterval = 1.0 / localFps;
            if (command.tag_size_m > 0) tracker.tagSizeMeters = command.tag_size_m;
            status.SetState("Ready");
        } else if (command.type == "state") {
            waitingForAck = false;
            if (command.state == "recording") {
                Armed = true; Draining = false; terminalState = false; status.SetClean(cleanView); status.SetState("Recording");
            } else {
                Armed = false; Draining = command.state == "saving";
                if (!Draining) pending.Clear();
                status.SetState(command.state == "saving" ? "Saving…" :
                    command.state == "saved" ? "Saved" : "Recording error");
                if (command.state != "saving") status.SetClean(false);
                terminalState = command.state == "saved" || command.state == "error";
            }
        }
    }

    public void ToggleRecording() {
        if (uploading || Draining) return;
        if (localPath != null && localWriter == null && File.Exists(localPath) && localPath.EndsWith(".wql")) {
            StartCoroutine(UploadLocal()); return;
        }
        if (!configured) return;
        try {
            if (Armed) StartCoroutine(FinishLocal());
            else BeginLocal();
        } catch (Exception error) { FailCapture("Local storage: " + error.Message); }
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
        // Local capture is ready without signaling; PC is only needed for upload.
        configured = true; cleanView = true;
        camera.captureInterval = 1.0 / localFps;
        status.SetState("Ready");
    }

    void OnBody(BodyPoseProvider.PoseData pose) {
        if (pose.bones == null || pose.bones.Count == 0) return;
        long timestamp = pose.timestampTicks;
        if (history.Count > 0 && timestamp == history[history.Count - 1].timestamp_ticks) return;
        if (history.Count > 0 && timestamp < history[history.Count - 1].timestamp_ticks) {
            // Clock adjustment / restart: never interpolate across a clock discontinuity.
            history.Clear(); pending.Clear();
        }
        history.Add(new Sample { timestamp_ticks = timestamp, bones = pose.bones.ToArray(),
            query_start_ticks = pose.queryStartTicks, query_end_ticks = pose.queryEndTicks,
            body_state_time_seconds = pose.bodyStateTimeSeconds, ovr_clock_seconds = pose.ovrClockSeconds });
        while (history.Count > 180) history.RemoveAt(0);
    }

    public bool Submit(long timestamp, byte[] jpeg, int width, int height, string eye,
                       float fx, float fy, float cx, float cy, Pose cameraPose,
                       out long frameId) {
        frameId = 0;
        if (!Armed || timestamp <= 0) return false;
        if (pending.Count >= 4) { rejectedBusy++; return false; }
        Sample before = null, after = null;
        for (int i = 0; i < history.Count; i++) {
            if (history[i].timestamp_ticks <= timestamp) before = history[i];
            if (history[i].timestamp_ticks >= timestamp) { after = history[i]; break; }
        }
        // No extrapolation, stale skeleton reuse, or fabricated exposure timestamps.
        const long maxDistanceTicks = 500000; // 50 ms on either side
        if (before == null || after == null || timestamp - before.timestamp_ticks > maxDistanceTicks ||
            after.timestamp_ticks - timestamp > maxDistanceTicks || before.bones.Length != after.bones.Length)
            { rejectedBracket++; return false; }
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
        var bundle = new Bundle { frame_id = frameId, timestamp_ticks = timestamp,
            width = width, height = height, eye = eye, fx = fx, fy = fy, cx = cx, cy = cy,
            camera_pose = cameraPose, before = before, after = after, bones = bones,
            tag_size_m = tracker.tagSizeMeters };
        pending.Add(frameId, new PendingFrame { bundle = bundle, jpeg = jpeg, submittedAt = Time.realtimeSinceStartupAsDouble });
        return true;
    }

    void OnTags(QuestAprilTagTracker.TagFrame result) {
        if ((!Armed && !Draining) || result.frameId == 0) return;
        if (pending.TryGetValue(result.frameId, out var frame)) {
            pendingCount++; pendingTotalMs += (Time.realtimeSinceStartupAsDouble-frame.submittedAt)*1000;
        }
        if (!pending.TryGetValue(result.frameId, out var found)) return;
        if (result.timestampTicks != found.bundle.timestamp_ticks || !string.IsNullOrEmpty(result.error)) {
            FailCapture("AprilTag timestamp/error; partial take retained"); return;
        }
        found.bundle.tags = result.tags;
        found.bundle.tag_detection_pending = false;
        found.tagsReady = true;
        DrainReady();
    }

    public void CompleteJpeg(long frameId, byte[] jpeg) {
        if ((!Armed && !Draining) || !pending.TryGetValue(frameId, out var frame)) return;
        frame.jpeg = jpeg;
        DrainReady();
    }

    void DrainReady() {
        while (pending.Count > 0 && (Armed || Draining)) {
            var iterator = pending.GetEnumerator(); iterator.MoveNext();
            var entry = iterator.Current; iterator.Dispose();
            var frame = entry.Value;
            if (frame.jpeg == null || !frame.tagsReady) return;
            var metadata = System.Text.Encoding.UTF8.GetBytes(JsonUtility.ToJson(frame.bundle));
            using (var stream = new MemoryStream()) using (var writer = new BinaryWriter(stream)) {
                writer.Write(metadata.Length); writer.Write(metadata); writer.Write(frame.jpeg);
                byte[] payload = stream.ToArray();
                try {
                    localWriter.Write(payload.Length); localWriter.Write(payload);
                    lastWrittenAt = Time.realtimeSinceStartupAsDouble;
                } catch (Exception error) { FailCapture("Local write failed: " + error.Message); return; }
            }
            pending.Remove(entry.Key);
        }
    }

    public void FailCapture(string reason) {
        Debug.LogError("[Warp] " + reason);
        if (localWriter != null) { localWriter.Dispose(); localWriter = null; }
        Armed = Draining = false; pending.Clear(); terminalState = true;
        status.SetClean(false); status.SetState("Recording error");
    }

    public void RejectPending(long frameId) { pending.Remove(frameId); DrainReady(); }

    void OnDestroy() {
        if (localWriter != null) { localWriter.Dispose(); localWriter = null; }
        if (body != null) body.OnPoseUpdated -= OnBody;
        if (tracker != null) tracker.OnFrameDetected -= OnTags;
        if (status != null) Destroy(status.gameObject);
    }

    void Update() {
        if (Armed && Time.realtimeSinceStartupAsDouble-perfAt > 5) {
            Debug.Log($"[WarpPerf] apriltag_pending_ms={pendingTotalMs/Math.Max(1,pendingCount):F2} completed={pendingCount} pending_rejected={rejectedBusy} skeleton_bracket_rejected={rejectedBracket}");
            pendingCount = rejectedBusy = rejectedBracket = 0; pendingTotalMs = 0; perfAt = Time.realtimeSinceStartupAsDouble;
        }
        if (waitingForAck && Time.realtimeSinceStartupAsDouble - requestAt > 3) {
            waitingForAck = false; Armed = false;
            terminalState = true; pending.Clear();
            status.SetClean(false); status.SetState("Connection lost");
        }
        if (Armed && Time.realtimeSinceStartupAsDouble - lastWrittenAt > 10) {
            FailCapture("Local capture stalled for 10s; partial take retained");
        }

    }
}
