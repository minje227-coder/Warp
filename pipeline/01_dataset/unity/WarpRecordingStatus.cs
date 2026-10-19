using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

// Status only: no screen controls, no raycast targets, no change to passthrough pixels.
public class WarpRecordingStatus : MonoBehaviour
{
    public WarpCaptureBridge capture;
    TMP_Text label, uploadLabel, countdownLabel, mountLabel, externalCameraLabel;
    TMP_FontAsset koreanFont;
    Canvas hud;
    string state = "Preparing…";
    double recordingStart;
    double lastPress = -1;
    double lastMountPress = -1, mountLockedUntil;
    readonly Dictionary<Canvas, bool> canvasState = new Dictionary<Canvas, bool>();
    readonly Dictionary<Renderer, bool> rendererState = new Dictionary<Renderer, bool>();
    bool clean;
    double nextCleanCheck;
    bool oldBalls, oldAxes;
    BodyPoseProvider body;
    QuestAprilTagTracker tracker;

    void Start() {
        var camera = capture.controller.vrCamera;
        if (camera == null) camera = Camera.main;
        var panel = new GameObject("WarpRecordingHUD", typeof(RectTransform), typeof(Canvas));
        hud = panel.GetComponent<Canvas>(); hud.renderMode = RenderMode.WorldSpace;
        if (camera != null) panel.transform.SetParent(camera.transform, false);
        panel.transform.localPosition = new Vector3(0.45f, -0.20f, 0.85f);
        panel.transform.localScale = Vector3.one * 0.001f;
        panel.GetComponent<RectTransform>().sizeDelta = new Vector2(360, 36);
        var text = new GameObject("Status", typeof(RectTransform), typeof(TextMeshProUGUI));
        text.transform.SetParent(panel.transform, false);
        label = text.GetComponent<TextMeshProUGUI>();
        label.fontSize = 22; label.alignment = TextAlignmentOptions.Center;
        label.raycastTarget = false;
        var rect = text.GetComponent<RectTransform>();
        rect.anchorMin = Vector2.zero; rect.anchorMax = Vector2.one;
        rect.offsetMin = rect.offsetMax = Vector2.zero;
        var upload = new GameObject("UploadQueue", typeof(RectTransform), typeof(TextMeshProUGUI));
        upload.transform.SetParent(panel.transform, false);
        uploadLabel = upload.GetComponent<TextMeshProUGUI>();
        uploadLabel.fontSize = 17; uploadLabel.alignment = TextAlignmentOptions.Center; uploadLabel.raycastTarget = false;
        var uploadRect = upload.GetComponent<RectTransform>();
        uploadRect.sizeDelta = new Vector2(360, 60); uploadRect.anchoredPosition = new Vector2(0, -54);
        var digit = new GameObject("Countdown", typeof(RectTransform), typeof(TextMeshProUGUI));
        digit.transform.SetParent(panel.transform, false);
        countdownLabel = digit.GetComponent<TextMeshProUGUI>();
        countdownLabel.fontSize = 48; countdownLabel.alignment = TextAlignmentOptions.Center;
        countdownLabel.color = Color.white; countdownLabel.raycastTarget = false;
        var digitRect = digit.GetComponent<RectTransform>();
        digitRect.sizeDelta = new Vector2(360, 72); digitRect.anchoredPosition = new Vector2(0, 54);
        var mount = new GameObject("MountMode", typeof(RectTransform), typeof(TextMeshProUGUI));
        mount.transform.SetParent(panel.transform, false);
        mountLabel = mount.GetComponent<TextMeshProUGUI>();
        mountLabel.fontSize = 20; mountLabel.alignment = TextAlignmentOptions.Center;
        mountLabel.raycastTarget = false;
        var mountRect = mount.GetComponent<RectTransform>();
        mountRect.sizeDelta = new Vector2(360, 36); mountRect.anchoredPosition = new Vector2(0, -114);
        var externalStatus = new GameObject("ExternalCameraStatus", typeof(RectTransform), typeof(TextMeshProUGUI));
        externalStatus.transform.SetParent(panel.transform, false);
        externalCameraLabel = externalStatus.GetComponent<TextMeshProUGUI>();
        externalCameraLabel.fontSize = 18; externalCameraLabel.alignment = TextAlignmentOptions.Center;
        externalCameraLabel.raycastTarget = false;
        var externalStatusRect = externalStatus.GetComponent<RectTransform>();
        externalStatusRect.sizeDelta = new Vector2(460, 60); externalStatusRect.anchoredPosition = new Vector2(0, -168);
        var font = Resources.Load<Font>("WarpKorean");
        if (font != null) { koreanFont = TMP_FontAsset.CreateFontAsset(font); externalCameraLabel.font = koreanFont; }
        SetCountdown(0);
        body = capture.controller.bodyPoseProvider;
        tracker = FindFirstObjectByType<QuestAprilTagTracker>(FindObjectsInactive.Include);
    }

    public void OnVolumeUp(string unused) {
        double now = Time.realtimeSinceStartupAsDouble;
        if (now - lastPress < 0.35) return;
        lastPress = now;
        capture.ToggleRecording();
    }

    public void OnVolumeDown(string unused) {
        double now = Time.realtimeSinceStartupAsDouble;
        if (now - lastMountPress < 0.35) return;
        lastMountPress = now;
        if (!capture.ToggleMountMode()) mountLockedUntil = now + 2;
    }

    public void SetState(string value) {
        state = value;
        if (value == "Recording") recordingStart = Time.realtimeSinceStartupAsDouble;
    }

    public void SetCountdown(int remaining) {
        if (countdownLabel == null) return;
        countdownLabel.text = remaining > 0 ? remaining.ToString() : "";
        countdownLabel.enabled = remaining > 0;
    }

    public void SetClean(bool value) {
        if (value == clean) return;
        clean = value;
        if (clean) {
            if (body != null) { oldBalls = body.enableBallRep; body.enableBallRep = false; }
            if (tracker != null) { oldAxes = tracker.drawAxesGizmo; tracker.drawAxesGizmo = false; }
        } else {
            foreach (var pair in canvasState) if (pair.Key != null) pair.Key.enabled = pair.Value;
            foreach (var pair in rendererState) if (pair.Key != null) pair.Key.enabled = pair.Value;
            canvasState.Clear(); rendererState.Clear();
            if (body != null) body.enableBallRep = oldBalls;
            if (tracker != null) tracker.drawAxesGizmo = oldAxes;
        }
    }

    void Update() {
        if (label != null) {
            int seconds = Mathf.FloorToInt((float)(Time.realtimeSinceStartupAsDouble - recordingStart));
            label.text = state == "Recording" ? $"Recording · {seconds / 60:00}:{seconds % 60:00}" : state;
            label.color = state == "Recording" ? new Color(1f, 0.25f, 0.25f) : Color.white;
        }
        if (uploadLabel != null) uploadLabel.text = capture.UploadText + (string.IsNullOrEmpty(capture.UploadDetail) ? "" : "\n" + capture.UploadDetail) + (string.IsNullOrEmpty(capture.TrackingText) ? "" : "\n" + capture.TrackingText);
        if (mountLabel != null) {
            string mode = capture.IsNeckMount ? "NECK" : "HEAD";
            string action = Time.realtimeSinceStartupAsDouble < mountLockedUntil ? "Stop to switch" :
                capture.CanChangeMountMode ? "Vol -" : "Locked";
            mountLabel.text = $"Mount: {mode} | {action}";
            mountLabel.color = capture.IsNeckMount ? new Color(0.3f, 0.9f, 1f) : Color.white;
        }
        if (externalCameraLabel != null) {
            string externalFps = capture.ExternalSourceFps >= 0 ? capture.ExternalSourceFps.ToString("F1") : "--";
            string questFps = capture.QuestSourceFps >= 0 ? capture.QuestSourceFps.ToString("F1") : "--";
            externalCameraLabel.text = capture.ExternalCameraText + " · " + externalFps + " FPS\nQuest: " + questFps + " FPS";
            externalCameraLabel.color = capture.ExternalCameraConnected ? new Color(0.3f, 0.9f, 1f) : Color.gray;
        }
        if (!clean) return;
        if (Time.realtimeSinceStartupAsDouble < nextCleanCheck) return;
        nextCleanCheck = Time.realtimeSinceStartupAsDouble + 1;
        foreach (var canvas in FindObjectsByType<Canvas>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
            if (canvas == hud) continue;
            if (!canvasState.ContainsKey(canvas)) canvasState[canvas] = canvas.enabled;
            canvas.enabled = false;
        }
        foreach (var renderer in FindObjectsByType<Renderer>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
            bool videoViewport = false;
            for (var parent = renderer.transform; parent != null; parent = parent.parent) {
                if (parent.name == "VideoStreamingViewport") { videoViewport = true; break; }
            }
            if (!videoViewport && !renderer.gameObject.name.StartsWith("Bone_", StringComparison.Ordinal)) continue;
            if (!rendererState.ContainsKey(renderer)) rendererState[renderer] = renderer.enabled;
            renderer.enabled = false;
        }
    }

    void OnDestroy() {
        SetClean(false); if (hud != null) Destroy(hud.gameObject);
        if (koreanFont != null) {
            foreach (var texture in koreanFont.atlasTextures) if (texture != null) Destroy(texture);
            if (koreanFont.material != null) Destroy(koreanFont.material);
            Destroy(koreanFont);
        }
    }
}
