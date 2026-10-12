using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

// Status only: no screen controls, no raycast targets, no change to passthrough pixels.
public class WarpRecordingStatus : MonoBehaviour
{
    public WarpCaptureBridge capture;
    TMP_Text label;
    Canvas hud;
    string state = "Preparing…";
    double recordingStart;
    double lastPress = -1;
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
        panel.transform.localPosition = new Vector3(0, -0.22f, 0.75f);
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
        body = capture.controller.bodyPoseProvider;
        tracker = FindFirstObjectByType<QuestAprilTagTracker>(FindObjectsInactive.Include);
    }

    public void OnVolumeUp(string unused) {
        double now = Time.realtimeSinceStartupAsDouble;
        if (now - lastPress < 0.35) return;
        lastPress = now;
        capture.ToggleRecording();
    }

    public void SetState(string value) {
        state = value;
        if (value == "Recording") recordingStart = Time.realtimeSinceStartupAsDouble;
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
        if (!clean) return;
        if (Time.realtimeSinceStartupAsDouble < nextCleanCheck) return;
        nextCleanCheck = Time.realtimeSinceStartupAsDouble + 1;
        foreach (var canvas in FindObjectsByType<Canvas>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
            if (canvas == hud) continue;
            if (!canvasState.ContainsKey(canvas)) canvasState[canvas] = canvas.enabled;
            canvas.enabled = false;
        }
        foreach (var renderer in FindObjectsByType<Renderer>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
            if (!renderer.gameObject.name.StartsWith("Bone_", StringComparison.Ordinal)) continue;
            if (!rendererState.ContainsKey(renderer)) rendererState[renderer] = renderer.enabled;
            renderer.enabled = false;
        }
    }

    void OnDestroy() { SetClean(false); if (hud != null) Destroy(hud.gameObject); }
}
