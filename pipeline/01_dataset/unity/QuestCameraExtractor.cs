using System;
using Meta.XR;
using UnityEngine;
using UnityEngine.UI;

// PCA is used only for an uncropped idle preview. Native Camera2 owns recording.
[DefaultExecutionOrder(1000)]
public class QuestCameraExtractor : MonoBehaviour {
    public PassthroughCameraAccess cameraAccess;
    public QuestAprilTagTracker tracker;
    [NonSerialized] public WarpCaptureBridge capture;
    [NonSerialized] public double captureInterval=1.0/30;
    Canvas canvas, externalCanvas;
    RawImage externalImage;
    Texture2D externalTexture;
    AndroidJavaObject externalPreview;
    bool applicationPaused;
    double nextExternalPoll, lastQuestFrameSeen;
    long lastQuestPreviewTicks, questFpsWindowStart;
    int questFpsWindowCount;
    public double QuestPreviewFps { get; private set; } = -1;
    public double ExternalSourceFps { get; private set; } = -1;
    [Serializable] class UsbPreviewStatus { public bool connected, permitted, released;public string error;public double source_fps; }
    public bool ExternalCameraConnected { get; private set; }
    public string ExternalCameraStatus { get; private set; } = "외부 카메라: 연결 안 됨";
    public bool PreviewReleased {
        get {
            if(cameraAccess!=null&&cameraAccess.IsPlaying)return false;
            return externalPreview==null||externalPreview.Call<bool>("isReleased");
        }
    }
    RawImage image;
    bool preview=true;
    void Start(){
#if UNITY_ANDROID && !UNITY_EDITOR
        const string permission="horizonos.permission.HEADSET_CAMERA";
        if(!UnityEngine.Android.Permission.HasUserAuthorizedPermission(permission))UnityEngine.Android.Permission.RequestUserPermission(permission);
#endif
        if(tracker!=null)tracker.enabled=false;
        if(cameraAccess!=null)cameraAccess.RequestedResolution=new Vector2Int(1280,1280);
        var panel=new GameObject("WarpCameraPreview",typeof(RectTransform),typeof(Canvas));
        canvas=panel.GetComponent<Canvas>();canvas.renderMode=RenderMode.WorldSpace;
        var eye=Camera.main;if(eye!=null)panel.transform.SetParent(eye.transform,false);
        panel.transform.localPosition=new Vector3(-0.35f,-0.36f,0.85f);panel.transform.localScale=Vector3.one*0.001f;
        panel.GetComponent<RectTransform>().sizeDelta=new Vector2(250,250);
        var view=new GameObject("FullCameraFrame",typeof(RectTransform),typeof(RawImage));view.transform.SetParent(panel.transform,false);
        image=view.GetComponent<RawImage>();image.raycastTarget=false;
        var r=view.GetComponent<RectTransform>();r.anchorMin=Vector2.zero;r.anchorMax=Vector2.one;r.offsetMin=r.offsetMax=Vector2.zero;
        var externalPanel=new GameObject("WarpExternalCameraPreview",typeof(RectTransform),typeof(Canvas));
        externalCanvas=externalPanel.GetComponent<Canvas>();externalCanvas.renderMode=RenderMode.WorldSpace;externalCanvas.enabled=false;
        if(eye!=null)externalPanel.transform.SetParent(eye.transform,false);
        externalPanel.transform.localPosition=new Vector3(-0.35f,-0.08f,0.85f);externalPanel.transform.localScale=Vector3.one*0.001f;
        externalPanel.GetComponent<RectTransform>().sizeDelta=new Vector2(250,140.625f);
        var externalView=new GameObject("ExternalCameraFrame",typeof(RectTransform),typeof(RawImage));externalView.transform.SetParent(externalPanel.transform,false);
        externalImage=externalView.GetComponent<RawImage>();externalImage.raycastTarget=false;
        var externalRect=externalView.GetComponent<RectTransform>();externalRect.anchorMin=Vector2.zero;externalRect.anchorMax=Vector2.one;externalRect.offsetMin=externalRect.offsetMax=Vector2.zero;
        externalTexture=new Texture2D(2,2,TextureFormat.RGB24,false);
#if UNITY_ANDROID && !UNITY_EDITOR
        using(var player=new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
        using(var activity=player.GetStatic<AndroidJavaObject>("currentActivity"))
            externalPreview=new AndroidJavaObject("dev.warp.capture.WarpUsbPreview",activity);
#endif
    }
    public void ReleasePreview(){
        QuestPreviewFps=-1;lastQuestPreviewTicks=questFpsWindowStart=0;questFpsWindowCount=0;
        preview=false;if(canvas!=null)canvas.enabled=false;if(externalCanvas!=null)externalCanvas.enabled=false;
        externalPreview?.Call("stop");if(cameraAccess!=null)cameraAccess.enabled=false;
    }
    public void ResumePreview(){preview=true;if(cameraAccess!=null&&!cameraAccess.enabled)cameraAccess.enabled=true;}
    void PollExternalPreview(){
        if(externalPreview==null||Time.realtimeSinceStartupAsDouble<nextExternalPoll)return;
        nextExternalPoll=Time.realtimeSinceStartupAsDouble+1.0/15.0;
        try {
            var usb=JsonUtility.FromJson<UsbPreviewStatus>(externalPreview.Call<string>("status"));
            ExternalCameraConnected=usb.connected;
            ExternalSourceFps=usb.connected?usb.source_fps:-1;
            ExternalCameraStatus=usb.connected?"외부 카메라: 연결됨":"외부 카메라: 연결 안 됨";
            if(usb.connected&&!usb.permitted)ExternalCameraStatus+=" · 접근 권한 필요";
            bool recordingPreview=!preview&&capture!=null&&capture.Armed;
            if((!preview&&!recordingPreview)||applicationPaused||!usb.connected||!usb.permitted){externalPreview.Call("stop");externalCanvas.enabled=false;return;}
            // Recording owns the USB fd. Display its bounded snapshot cache;
            // never open a second USB connection while capturing raw frames.
            if(preview)externalPreview.Call("start");
            byte[] jpeg=externalPreview.Call<byte[]>("frame");
            if(jpeg!=null&&jpeg.Length>0&&externalTexture.LoadImage(jpeg,false)){
                externalImage.texture=externalTexture;
                externalCanvas.GetComponent<RectTransform>().sizeDelta=new Vector2(250,250f*externalTexture.height/externalTexture.width);
                externalCanvas.enabled=true;
            }else externalCanvas.enabled=false;
        }catch(Exception){
            ExternalSourceFps=-1;ExternalCameraStatus="외부 카메라: 확인 불가";externalCanvas.enabled=false;
        }
    }
    void LateUpdate(){
        PollExternalPreview();
        if(!preview||cameraAccess==null||!cameraAccess.IsPlaying){QuestPreviewFps=-1;return;}
        long ticks=cameraAccess.Timestamp.Ticks;
        if(cameraAccess.IsUpdatedThisFrame&&ticks>0&&ticks!=lastQuestPreviewTicks){
            lastQuestFrameSeen=Time.realtimeSinceStartupAsDouble;
            if(questFpsWindowStart==0||ticks<lastQuestPreviewTicks){questFpsWindowStart=ticks;questFpsWindowCount=0;QuestPreviewFps=-1;}
            else {
                questFpsWindowCount++;
                if(ticks-questFpsWindowStart>=TimeSpan.TicksPerSecond/2){
                    QuestPreviewFps=questFpsWindowCount*(double)TimeSpan.TicksPerSecond/(ticks-questFpsWindowStart);
                    questFpsWindowStart=ticks;questFpsWindowCount=0;
                }
            }
            lastQuestPreviewTicks=ticks;
        }
        if(lastQuestPreviewTicks>0&&Time.realtimeSinceStartupAsDouble-lastQuestFrameSeen>1)QuestPreviewFps=0;
        if(image!=null){image.texture=cameraAccess.GetTexture();canvas.enabled=image.texture!=null;}
    }
    void OnApplicationPause(bool paused){
        applicationPaused=paused;
        if(paused){externalPreview?.Call("stop");if(externalCanvas!=null)externalCanvas.enabled=false;}
        else nextExternalPoll=0;
    }
    void OnDisable(){externalPreview?.Call("stop");if(externalCanvas!=null)externalCanvas.enabled=false;}
    void OnDestroy(){
        externalPreview?.Call("stop");externalPreview?.Dispose();
        if(externalTexture!=null)Destroy(externalTexture);
        if(externalCanvas!=null)Destroy(externalCanvas.gameObject);
        if(canvas!=null)Destroy(canvas.gameObject);
    }
}
