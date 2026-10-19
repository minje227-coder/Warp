using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Independent source recording. Server signaling must never own local capture state.
public class WarpCaptureBridge : MonoBehaviour {
    public WebRTCController controller;
    public bool Armed { get; private set; }
    public bool Draining { get; private set; }
    public bool Starting { get; private set; }
    const string MountModePreference = "warp_capture_mount_mode";
    public string MountMode { get; private set; } = "head";
    public bool IsNeckMount => MountMode == "neck";
    public bool CanChangeMountMode => !Armed && !Starting && !Draining;
    string recordingMountMode = "head";
    public string UploadText { get; private set; } = "Upload pending · 0/30";
    double nativeQuestFps=-1;
    public double QuestSourceFps => Armed?nativeQuestFps:camera!=null?camera.QuestPreviewFps:-1;
    public double ExternalSourceFps => camera!=null?camera.ExternalSourceFps:-1;
    public string ExternalCameraText => camera!=null?camera.ExternalCameraStatus:"외부 카메라: 확인 중";
    public bool ExternalCameraConnected => camera!=null&&camera.ExternalCameraConnected;
    public string TrackingText { get; private set; } = "";
    public string UploadDetail { get; private set; } = "";
    AndroidJavaObject recorder;
    WarpRecordingStatus status;
    BodyPoseProvider body;
    QuestCameraExtractor camera;
    OVRCameraRig rig;
    OVRSkeleton[] hands;
    long lastBodyTimestamp;
    double lastBodySourceTime;
    double lastBodySeen, nextPoll;
    bool hadBody;
    bool nativeStartSubmitted;
    int startGeneration;
    [Serializable] class NativeStatus { public string state,error,upload_state;public int pending,capacity;public double progress,quest_source_fps;public long free_bytes; }
    [Serializable] class Bone { public int id;public string name;public Vector3 position;public Quaternion rotation; }
    [Serializable] class Sample {
        public string kind,coordinate_frame,time_source,mount_mode,hmd_pose_role;
        public long timestamp_ticks,query_start_ticks,query_end_ticks;
        public double ovr_clock_seconds,body_state_time_seconds,unity_realtime_seconds;
        public bool valid,provider_valid,high_confidence;
        public Bone[] bones;
        public Pose head_pose,tracking_origin_pose,provider_transform;
    }
    void Awake() {
        MountMode = PlayerPrefs.GetString(MountModePreference, "head") == "neck" ? "neck" : "head";
        SyncMountMode();
    }
    public bool ToggleMountMode() {
        if (!CanChangeMountMode) return false;
        SyncMountMode();
        MountMode = IsNeckMount ? "head" : "neck";
        PlayerPrefs.SetString(MountModePreference, MountMode);
        PlayerPrefs.Save();
        UpdateProximityMode();
        Debug.Log("[Warp] Mount mode: " + MountMode);
        return true;
    }
    void UpdateProximityMode() {
#if UNITY_ANDROID && !UNITY_EDITOR
        try {
            using(var player=new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
            using(var activity=player.GetStatic<AndroidJavaObject>("currentActivity"))
                activity.Call("setNeckMount", IsNeckMount);
        } catch(Exception e) { Debug.LogError("[Warp] Mount proximity: " + e.Message); }
#endif
    }
    void SyncMountMode() {
#if UNITY_ANDROID && !UNITY_EDITOR
        try {
            using(var player=new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
            using(var activity=player.GetStatic<AndroidJavaObject>("currentActivity")) {
                string mode=activity.Call<bool>("isNeckMount") ? "neck" : "head";
                if(MountMode==mode)return;
                MountMode=mode;
                PlayerPrefs.SetString(MountModePreference, MountMode);PlayerPrefs.Save();
                Debug.Log("[Warp] Mount mode synchronized: " + MountMode);
            }
        } catch(Exception e) { Debug.LogError("[Warp] Mount sync: " + e.Message); }
#endif
    }
    void SetCaptureBusy(bool value) {
#if UNITY_ANDROID && !UNITY_EDITOR
        try {
            using(var player=new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
            using(var activity=player.GetStatic<AndroidJavaObject>("currentActivity"))
                activity.Call("setCaptureBusy", value);
        } catch(Exception e) { Debug.LogError("[Warp] Capture proximity lock: " + e.Message); }
#endif
    }
    void Start() {
        body=controller.bodyPoseProvider;camera=FindFirstObjectByType<QuestCameraExtractor>(FindObjectsInactive.Include);
        rig=FindFirstObjectByType<OVRCameraRig>();
        var list=new List<OVRSkeleton>();
        foreach(var s in FindObjectsByType<OVRSkeleton>(FindObjectsInactive.Include,FindObjectsSortMode.None)) {
            var t=s.GetSkeletonType();
            if(t==OVRSkeleton.SkeletonType.HandLeft||t==OVRSkeleton.SkeletonType.HandRight||
               t==OVRSkeleton.SkeletonType.XRHandLeft||t==OVRSkeleton.SkeletonType.XRHandRight)list.Add(s);
        }
        hands=list.ToArray();
        var hud=new GameObject("WarpCaptureStatus");status=hud.AddComponent<WarpRecordingStatus>();status.capture=this;
        if(body==null||camera==null){status.SetState("Recorder components missing");enabled=false;return;}
        camera.capture=this;body.OnPoseUpdated+=OnBody;
#if UNITY_ANDROID && !UNITY_EDITOR
        using(var player=new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
        using(var activity=player.GetStatic<AndroidJavaObject>("currentActivity"))
            recorder=new AndroidJavaObject("dev.warp.capture.WarpRawRecorder",activity,controller.serverUrl);
#endif
        OVRManager.HMDUnmounted+=OnUnmounted;
        if(OVRManager.display!=null)OVRManager.display.RecenteredPose+=OnRecenter;
        status.SetState("Ready");
        Debug.Log("[Warp] Raw recorder v3: 1280x1280, hardware H264, offline AprilTag, queue 30");
    }
    public void SetArmed(bool value) { /* Teleop disconnects do not alter local recording. */ }
    public void HandleServerCommand(string json) { /* Legacy streamed capture config is intentionally isolated. */ }
    public void ToggleRecording() {
        if(Starting||Draining)return;
        if(Armed){recorder?.Call("stop");Armed=false;Draining=true;status.SetState("Saving…");return;}
        StartCoroutine(Begin());
    }
    IEnumerator Begin() {
        int generation=++startGeneration;
        nativeStartSubmitted=false;Starting=true;SetCaptureBusy(true);SyncMountMode();recordingMountMode=MountMode;status.SetState("Starting…");
        camera.ReleasePreview();
        // PCA must release its Camera2 device before native capture owns it.
        double deadline=Time.realtimeSinceStartupAsDouble+2;
        while(!camera.PreviewReleased&&Time.realtimeSinceStartupAsDouble<deadline)yield return null;
        yield return new WaitForSecondsRealtime(0.2f);
        if(generation!=startGeneration||!Starting)yield break;
        // Keep the preview released and show only digits before native capture starts.
        for(int remaining=3;remaining>0;remaining--){
            status.SetCountdown(remaining);
            yield return new WaitForSecondsRealtime(1f);
            if(generation!=startGeneration||!Starting)yield break;
        }
        status.SetCountdown(0);
        if(!camera.PreviewReleased){
            nativeStartSubmitted=false;Starting=false;SetCaptureBusy(false);
            status.SetState("Camera preview did not release");camera.ResumePreview();yield break;
        }
        lastBodyTimestamp=0;lastBodySourceTime=0;hadBody=false;
        bool ok=false;
        try { if(recorder!=null){recorder.Call("setServerUrl",controller.serverUrl);nativeStartSubmitted=true;ok=recorder.Call<bool>("start",recordingMountMode);} }
        catch(Exception e){Debug.LogError("[Warp] Native start: "+e.Message);}
        if(!ok){nativeStartSubmitted=false;Starting=false;SetCaptureBusy(false);status.SetState("Recording error");camera.ResumePreview();}
    }
    Pose PoseOf(Transform t) {return t==null?new Pose(Vector3.zero,Quaternion.identity):new Pose(t.position,t.rotation);}
    void OnBody(BodyPoseProvider.PoseData p) {
        if(!Armed||p.bones==null||p.bodyStateTimeSeconds<=0||p.bodyStateTimeSeconds==lastBodySourceTime)return;
        if(lastBodySourceTime>0&&p.bodyStateTimeSeconds<lastBodySourceTime)recorder.Call("event","body_source_time_regressed");
        lastBodySourceTime=p.bodyStateTimeSeconds;
        if(lastBodyTimestamp>0&&p.timestampTicks<lastBodyTimestamp)recorder.Call("event","body_clock_regressed");
        lastBodyTimestamp=p.timestampTicks;lastBodySeen=Time.realtimeSinceStartupAsDouble;hadBody=true;
        var bones=new Bone[p.bones.Count];
        for(int i=0;i<bones.Length;i++){var b=p.bones[i];bones[i]=new Bone{id=(int)b.id,name=b.id.ToString(),position=b.position,rotation=b.rotation};}
        Write(new Sample{kind="body",coordinate_frame="meta_source_native_skeleton_untransformed",time_source="ovr_body_target_time_mapped_to_utc",
            timestamp_ticks=p.timestampTicks,query_start_ticks=p.queryStartTicks,query_end_ticks=p.queryEndTicks,
            ovr_clock_seconds=p.ovrClockSeconds,body_state_time_seconds=p.bodyStateTimeSeconds,
            unity_realtime_seconds=Time.realtimeSinceStartupAsDouble,valid=true,bones=bones,provider_transform=PoseOf(body.transform),
            head_pose=PoseOf(controller.vrCamera!=null?controller.vrCamera.transform:null),tracking_origin_pose=PoseOf(rig!=null?rig.trackingSpace:null)});
    }
    void LateUpdate() {
        if(recorder==null)return;
        if(Time.realtimeSinceStartupAsDouble>=nextPoll){
            nextPoll=Time.realtimeSinceStartupAsDouble+0.2;
            SyncMountMode();
            var s=JsonUtility.FromJson<NativeStatus>(recorder.Call<string>("status"));
            nativeQuestFps=s.quest_source_fps;
            UploadText=$"Upload pending · {s.pending}/30";
            UploadDetail=s.upload_state=="Sending"?$"Sending · {s.progress*100:F0}%":s.upload_state=="Idle"?"":s.upload_state;
            if(s.state=="Recording"&&!Armed){Starting=false;Draining=false;Armed=true;status.SetClean(true);status.SetState("Recording");}
            // Native Ready belongs to the previous take until Begin submits start.
            // Resuming PCA while Begin is releasing it races Camera2 ownership.
            else if(s.state=="Ready"&&(Armed||Draining||(Starting&&nativeStartSubmitted))){nativeStartSubmitted=false;Armed=Draining=Starting=false;SetCaptureBusy(false);TrackingText="";status.SetClean(false);status.SetState("Ready");camera.ResumePreview();}
            else if(s.state=="Recording error"&&(!Starting||nativeStartSubmitted)){nativeStartSubmitted=false;Armed=Draining=Starting=false;SetCaptureBusy(false);status.SetClean(false);status.SetState(s.error);camera.ResumePreview();}
        }
        if(!Armed)return;
        var head=new Sample{kind="head_and_tracking_status",coordinate_frame="unity_world_meters_xyzw",time_source="query_utc",
            timestamp_ticks=DateTime.UtcNow.Ticks,ovr_clock_seconds=OVRPlugin.GetTimeInSeconds(),unity_realtime_seconds=Time.realtimeSinceStartupAsDouble,
            valid=hadBody&&Time.realtimeSinceStartupAsDouble-lastBodySeen<0.1,
            head_pose=PoseOf(controller.vrCamera!=null?controller.vrCamera.transform:null),tracking_origin_pose=PoseOf(rig!=null?rig.trackingSpace:null)};
        Write(head);
        int validHands=0;
        foreach(var hand in hands){
            bool providerValid=hand.isActiveAndEnabled&&hand.IsDataValid;
            var bones=new List<Bone>();
            bool nonzero=false;
            if(hand.Bones!=null)foreach(var b in hand.Bones)if(b.Transform!=null){
                Vector3 position=b.Transform.position; nonzero |= position.sqrMagnitude>1e-12f;
                bones.Add(new Bone{id=(int)b.Id,name=hand.GetSkeletonType()+"_"+(int)b.Id,position=position,rotation=b.Transform.rotation});
            }
            // SDK validity can remain true with uninitialized all-zero hands.
            bool valid=providerValid&&bones.Count>3&&nonzero;
            if(valid)validHands++;
            Write(new Sample{kind=hand.GetSkeletonType().ToString(),coordinate_frame="unity_world_meters_xyzw",time_source="query_utc_not_sensor_exposure",
                timestamp_ticks=DateTime.UtcNow.Ticks,ovr_clock_seconds=OVRPlugin.GetTimeInSeconds(),valid=valid,provider_valid=providerValid,high_confidence=hand.IsDataHighConfidence,bones=bones.ToArray()});
        }
        TrackingText=!head.valid?"Body tracking unavailable":validHands<2?"Hand tracking unavailable":"";
    }
    void Write(Sample s){
        s.mount_mode=recordingMountMode;
        // head_pose is the raw XR device pose retained for schema compatibility,
        // not an anatomical head measurement when the device is neck-mounted.
        s.hmd_pose_role=recordingMountMode=="neck"?"neck_mounted_device":"head_mounted_device";
        recorder.Call("appendTracking",JsonUtility.ToJson(s));
    }
    void StopWithEvent(string value){
        if(recorder==null||(!Armed&&!Starting))return;
        ++startGeneration;
        status.SetCountdown(0);
        if(Starting&&!nativeStartSubmitted){Starting=false;SetCaptureBusy(false);status.SetState("Ready");camera.ResumePreview();return;}
        recorder.Call("event",value);recorder.Call("stop");Armed=Starting=false;Draining=true;
    }
    void OnUnmounted(){
        if(!IsNeckMount){StopWithEvent("headset_unmounted");return;}
        if(Armed)recorder.Call("event","headset_unmounted_neck_mode");
    }
    void OnRecenter(){if(Armed)recorder.Call("event","tracking_origin_recentered");}
    void OnApplicationPause(bool paused){if(paused)StopWithEvent("application_paused");}
    void OnDestroy(){StopWithEvent("component_destroyed");if(body!=null)body.OnPoseUpdated-=OnBody;OVRManager.HMDUnmounted-=OnUnmounted;if(OVRManager.display!=null)OVRManager.display.RecenteredPose-=OnRecenter;recorder?.Dispose();}
}
