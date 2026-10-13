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
    Canvas canvas;
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
        panel.transform.localPosition=new Vector3(-0.35f,-0.08f,0.85f);panel.transform.localScale=Vector3.one*0.001f;
        panel.GetComponent<RectTransform>().sizeDelta=new Vector2(250,250);
        var view=new GameObject("FullCameraFrame",typeof(RectTransform),typeof(RawImage));view.transform.SetParent(panel.transform,false);
        image=view.GetComponent<RawImage>();image.raycastTarget=false;
        var r=view.GetComponent<RectTransform>();r.anchorMin=Vector2.zero;r.anchorMax=Vector2.one;r.offsetMin=r.offsetMax=Vector2.zero;
    }
    public void ReleasePreview(){preview=false;if(canvas!=null)canvas.enabled=false;if(cameraAccess!=null)cameraAccess.enabled=false;}
    public void ResumePreview(){preview=true;if(cameraAccess!=null&&!cameraAccess.enabled)cameraAccess.enabled=true;}
    void LateUpdate(){
        if(!preview||cameraAccess==null||!cameraAccess.IsPlaying)return;
        if(image!=null){image.texture=cameraAccess.GetTexture();canvas.enabled=image.texture!=null;}
    }
    void OnDestroy(){if(canvas!=null)Destroy(canvas.gameObject);}
}
