package dev.warp.capture;

import android.app.Activity;
import android.hardware.camera2.*;
import android.hardware.camera2.params.*;
import android.media.*;
import android.os.*;
import android.util.Range;
import android.util.Size;
import android.view.Surface;
import org.json.*;
import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.security.MessageDigest;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.zip.*;

/** Camera2 -> encoder surface. Tracking and upload never gate camera admission. */
public final class WarpRawRecorder {
    public static final int CAPACITY = 30;
    private final Activity activity;
    private final File root;
    private final Handler cameraHandler, codecHandler, trackingHandler;
    private final ExecutorService sealing = Executors.newSingleThreadExecutor();
    private volatile Episode active;
    private volatile String state = "Ready", error = "", uploadState = "Idle", serverUrl;
    private volatile double progress;
    private volatile boolean alive = true;
    private volatile String uploadingId = "";
    private final AtomicInteger trackingQueued = new AtomicInteger();
    private static final long MIN_FREE = 512L * 1024 * 1024;
    private static final long UNIX_TICKS = 621355968000000000L;
    private static class Episode {
        File dir; String id, mountMode; CameraDevice camera; CameraCaptureSession session;
        MediaCodec codec; MediaMuxer muxer; Surface surface;
        BufferedWriter frames, encoded, tracking, events;
        int track = -1, sensorFrames, encodedFrames; volatile long trackingSamples;
        volatile double measuredQuestFps=-1;
        volatile long lastFrameElapsedNs;
        long fpsWindowStartNs;
        int fpsWindowCount;
        boolean muxing, repeatingStarted, wasRecording, finalizeStarted;
        WarpUsbCamera external; volatile boolean externalClosed=true;
        volatile boolean stopping, cameraClosed, codecClosed, trackingClosed;
        volatile String failure = ""; volatile boolean finalized;
        long created = System.currentTimeMillis(); JSONObject calibration;
        Episode(File d) { dir=d; id=d.getName(); }
    }
    public WarpRawRecorder(Activity a, String url) {
        activity=a; root=new File(a.getExternalFilesDir(null),"WarpRawEpisodes"); root.mkdirs();
        serverUrl=url;
        cameraHandler=handler("WarpCamera"); codecHandler=handler("WarpCodec"); trackingHandler=handler("WarpTracking");
        // A crashed partial take is retained, never silently promoted or deleted.
        for (File d: episodes()) if (!new File(d,"manifest.json").exists()) {
            try { writeAtomic(new File(d,"interrupted.json"),new JSONObject().put("reason","process_restart_before_seal").toString()); }
            catch(Exception ignored) { }
        }
        Thread upload=new Thread(this::uploadLoop,"WarpUploader"); upload.setDaemon(true); upload.start();
    }
    private static Handler handler(String name) { HandlerThread t=new HandlerThread(name,android.os.Process.THREAD_PRIORITY_BACKGROUND);t.start();return new Handler(t.getLooper()); }
    public void setServerUrl(String value) { serverUrl=value; }
    public String status() {
        Episode snapshot=active;
        double questFps=snapshot==null?-1:snapshot.lastFrameElapsedNs>0&&SystemClock.elapsedRealtimeNanos()-snapshot.lastFrameElapsedNs>1000000000L?0:snapshot.measuredQuestFps;
        try { return new JSONObject().put("quest_source_fps",questFps).put("state",state).put("error",error).put("pending",pending())
            .put("capacity",CAPACITY).put("upload_state",uploadState).put("progress",progress)
            .put("free_bytes",root.getUsableSpace()).toString(); } catch(Exception e) {return "{}";}
    }
    public String capabilities() {
        try {
            CameraManager m=(CameraManager)activity.getSystemService(Activity.CAMERA_SERVICE);
            JSONArray cameras=new JSONArray();
            for(String id:m.getCameraIdList()) {
                CameraCharacteristics c=m.getCameraCharacteristics(id); Byte source=vendor(c,"com.meta.extra_metadata.camera_source");
                if(source==null || source!=0) continue;
                Byte eye=vendor(c,"com.meta.extra_metadata.position");
                // Standard calibrated left/right streams; virtual stitched streams are excluded.
                float[] k=c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION); if(k==null) continue;
                JSONArray sizes=new JSONArray(); StreamConfigurationMap map=c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                Size[] supported=map.getOutputSizes(MediaCodec.class); if(supported!=null) for(Size s:supported) sizes.put(s.getWidth()+"x"+s.getHeight());
                cameras.put(new JSONObject().put("id",id).put("eye",eye).put("sizes",sizes));
            }
            return new JSONObject().put("cameras",cameras).toString();
        }catch(Exception e){return "{\"error\":\"camera query failed\"}";}
    }
    private static Byte vendor(CameraCharacteristics c,String name) { for(CameraCharacteristics.Key<?> k:c.getKeys()) if(k.getName().equals(name)) { Object value=c.get(k); if(value instanceof byte[]) { byte[] a=(byte[])value;return a.length==0?null:a[0]; } if(value instanceof Number)return ((Number)value).byteValue(); }return null; }
    private void setCaptureBusy(boolean value) {
        // Camera smoke probes use a plain Activity without the Unity plugin.
        try { activity.getClass().getMethod("setCaptureBusy", boolean.class).invoke(activity, value); }
        catch(NoSuchMethodException ignored) { }
        catch(ReflectiveOperationException ex) { android.util.Log.e("WarpMount", "Native capture lock failed", ex); }
    }
    private void recordingFinished(Episode e) {
        if(!e.wasRecording) { setCaptureBusy(false); return; }
        try { activity.getClass().getMethod("onRecordingFinished").invoke(activity); }
        catch(NoSuchMethodException ignored) { }
        catch(ReflectiveOperationException ex) { android.util.Log.e("WarpMount", "Native recording-end reset failed", ex); }
    }
    public synchronized boolean start(String mountMode) {
        if(active!=null || state.equals("Saving"))return false;
        error="";
        try {
            if(!"head".equals(mountMode) && !"neck".equals(mountMode))
                throw new IllegalArgumentException("Invalid mount mode");
            ensureCapacity();
            if(root.getUsableSpace()<MIN_FREE)throw new IOException("Storage low");
            Episode e=new Episode(new File(root,UUID.randomUUID().toString().replace("-","")));e.mountMode=mountMode;e.dir.mkdir();
            e.frames=writer(e.dir,"camera_frames.jsonl");e.encoded=writer(e.dir,"encoded_frames.jsonl");
            e.tracking=writer(e.dir,"tracking.jsonl");e.events=writer(e.dir,"events.jsonl");
            setCaptureBusy(true);active=e;state="Starting";
            event("start"); cameraHandler.post(()->open(e)); return true;
        }catch(Exception ex){error=ex.getMessage();state="Recording error";return false;}
    }
    private void ensureCapacity() throws Exception {
        List<File> all=episodes(); all.sort(Comparator.comparingLong(this::episodeCreated));
        while(all.size()>=CAPACITY || root.getUsableSpace()<MIN_FREE) {
            File victim=null;for(File d:all)if(new File(d,"receipt.json").exists()&&!d.getName().equals(uploadingId)){victim=d;break;}
            if(victim==null)throw new IOException(all.size()>=CAPACITY?"Queue full: 30 unconfirmed episodes retained":"Storage low: unconfirmed recordings protected");
            deleteTree(victim);all.remove(victim);
        }
    }
    private void open(Episode e) {
        try {
            if(e.stopping){e.cameraClosed=true;finishEncoder(e);return;}
            e.external=WarpUsbCamera.openIfAttached(activity,e.dir);
            if(e.external!=null){
                e.externalClosed=false;
                e.external.start(usbError->{
                    synchronized(WarpRawRecorder.this) {
                        if(!usbError.isEmpty()){
                            if(e.failure.isEmpty())e.failure=usbError;
                            if(!e.stopping)fail(e,new IOException(usbError));
                        }
                        e.externalClosed=true;
                        maybeSeal(e);
                    }
                });
                if(e.stopping)e.external.stop();
            }
            CameraManager m=(CameraManager)activity.getSystemService(Activity.CAMERA_SERVICE);
            String chosen=null; CameraCharacteristics c=null;
            for(String id:m.getCameraIdList()) {
                CameraCharacteristics candidate=m.getCameraCharacteristics(id);
                Byte source=vendor(candidate,"com.meta.extra_metadata.camera_source"),eye=vendor(candidate,"com.meta.extra_metadata.position");
                if(source!=null&&source==0&&eye!=null&&eye==1&&candidate.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)!=null){chosen=id;c=candidate;break;}
            }
            if(chosen==null)throw new IOException("Calibrated right passthrough camera unavailable");
            StreamConfigurationMap map=c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            boolean supported=false;for(Size s:map.getOutputSizes(MediaCodec.class))if(s.getWidth()==1280&&s.getHeight()==1280)supported=true;
            if(!supported)throw new IOException("1280x1280 encoder surface not supported; no silent crop fallback");
            e.calibration=new JSONObject().put("camera_id",chosen).put("eye","right").put("width",1280).put("height",1280)
                .put("timestamp_source",c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE))
                .put("intrinsics",array(c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)))
                .put("lens_pose_translation",array(c.get(CameraCharacteristics.LENS_POSE_TRANSLATION)))
                .put("lens_pose_rotation",array(c.get(CameraCharacteristics.LENS_POSE_ROTATION)))
                .put("distortion",array(c.get(CameraCharacteristics.LENS_DISTORTION)))
                .put("pose_reference",c.get(CameraCharacteristics.LENS_POSE_REFERENCE))
                .put("active_array",String.valueOf(c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)))
                .put("sensor_orientation",c.get(CameraCharacteristics.SENSOR_ORIENTATION));
            writeAtomic(new File(e.dir,"calibration.json"),e.calibration.toString());
            e.codec=MediaCodec.createEncoderByType("video/avc");
            MediaFormat format=MediaFormat.createVideoFormat("video/avc",1280,1280);
            format.setInteger(MediaFormat.KEY_COLOR_FORMAT,MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface);
            format.setInteger(MediaFormat.KEY_BIT_RATE,20000000);format.setInteger(MediaFormat.KEY_FRAME_RATE,30);
            format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL,1);
            e.muxer=new MediaMuxer(new File(e.dir,"camera.mp4").getPath(),MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);
            e.codec.setCallback(new MediaCodec.Callback(){
                public void onInputBufferAvailable(MediaCodec codec,int index){}
                public void onOutputFormatChanged(MediaCodec codec,MediaFormat output){try{e.track=e.muxer.addTrack(output);e.muxer.start();e.muxing=true;}catch(Exception ex){fail(e,ex);}}
                public void onError(MediaCodec codec,MediaCodec.CodecException ex){fail(e,ex);closeCodec(e);}
                public void onOutputBufferAvailable(MediaCodec codec,int index,MediaCodec.BufferInfo info){
                    boolean eos=(info.flags&MediaCodec.BUFFER_FLAG_END_OF_STREAM)!=0;
                    try {
                        ByteBuffer b=codec.getOutputBuffer(index);
                        if(info.size>0&&(info.flags&MediaCodec.BUFFER_FLAG_CODEC_CONFIG)==0) {
                            if(!e.muxing)throw new IOException("Encoder sample before muxer format");
                            e.muxer.writeSampleData(e.track,b,info);
                            line(e.encoded,new JSONObject().put("index",e.encodedFrames++).put("pts_us",info.presentationTimeUs).put("flags",info.flags).put("bytes",info.size));
                        }
                        codec.releaseOutputBuffer(index,false);
                    }catch(Exception ex){fail(e,ex);eos=true;}
                    if(eos)closeCodec(e);
                }
            },codecHandler);
            e.codec.configure(format,null,null,MediaCodec.CONFIGURE_FLAG_ENCODE);e.surface=e.codec.createInputSurface();e.codec.start();
            m.openCamera(chosen,new CameraDevice.StateCallback(){
                public void onOpened(CameraDevice camera){
                    e.camera=camera;
                    if(e.stopping){camera.close();return;}
                    try {
                        OutputConfiguration out=new OutputConfiguration(e.surface);
                        if(Build.VERSION.SDK_INT>=33)out.setTimestampBase(OutputConfiguration.TIMESTAMP_BASE_SENSOR);
                        camera.createCaptureSession(new SessionConfiguration(SessionConfiguration.SESSION_REGULAR,
                            Collections.singletonList(out),r->cameraHandler.post(r),new CameraCaptureSession.StateCallback(){
                            public void onConfigured(CameraCaptureSession session){
                                e.session=session;if(e.stopping){session.close();camera.close();return;}
                                try {
                                    CaptureRequest.Builder b=camera.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);b.addTarget(e.surface);
                                    b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,new Range<>(30,30));
                                    b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                                    session.setRepeatingRequest(b.build(),new CameraCaptureSession.CaptureCallback(){
                                        public void onCaptureCompleted(CameraCaptureSession session,CaptureRequest request,TotalCaptureResult result){
                                            try {
                                                long elapsed=SystemClock.elapsedRealtimeNanos(),up=System.nanoTime(),utc=System.currentTimeMillis();
                                                JSONObject f=new JSONObject().put("frame_number",result.getFrameNumber()).put("sensor_timestamp_ns",result.get(CaptureResult.SENSOR_TIMESTAMP))
                                                    .put("callback_elapsed_ns",elapsed).put("callback_uptime_ns",up).put("callback_unix_ms",utc)
                                                    .put("crop_region",String.valueOf(result.get(CaptureResult.SCALER_CROP_REGION)))
                                                    .put("exposure_ns",result.get(CaptureResult.SENSOR_EXPOSURE_TIME));
                                                line(e.frames,f);e.sensorFrames++;
                                                e.lastFrameElapsedNs=elapsed;
                                                Long stamp=result.get(CaptureResult.SENSOR_TIMESTAMP);
                                                if(stamp!=null){
                                                    if(e.fpsWindowStartNs==0||stamp<=e.fpsWindowStartNs){e.fpsWindowStartNs=stamp;e.fpsWindowCount=0;e.measuredQuestFps=-1;}
                                                    else {
                                                        e.fpsWindowCount++;
                                                        if(stamp-e.fpsWindowStartNs>=500000000L){
                                                            e.measuredQuestFps=e.fpsWindowCount*1e9/(stamp-e.fpsWindowStartNs);
                                                            e.fpsWindowStartNs=stamp;e.fpsWindowCount=0;
                                                        }
                                                    }
                                                }
                                                if(root.getUsableSpace()<MIN_FREE)fail(e,new IOException("Storage low during capture"));
                                            }catch(Exception ex){fail(e,ex);}
                                        }
                                        public void onCaptureFailed(CameraCaptureSession s,CaptureRequest r,CaptureFailure f){event("camera_capture_failed:"+f.getFrameNumber());}
                                        public void onCaptureSequenceCompleted(CameraCaptureSession s,int id,long lastFrame){
                                            if(e.stopping){e.repeatingStarted=false;s.close();if(e.camera!=null)e.camera.close();}
                                        }
                                        public void onCaptureSequenceAborted(CameraCaptureSession s,int id){
                                            if(e.stopping){e.repeatingStarted=false;if(e.failure.isEmpty())e.failure="Capture sequence aborted before metadata drain";s.close();if(e.camera!=null)e.camera.close();}
                                        }
                                    },cameraHandler);e.repeatingStarted=true;e.wasRecording=true;state="Recording";
                                }catch(Exception ex){fail(e,ex);}
                            }
                            public void onConfigureFailed(CameraCaptureSession session){fail(e,new IOException("Camera encoder configuration failed"));}
                        }));
                    }catch(Exception ex){fail(e,ex);}
                }
                public void onDisconnected(CameraDevice camera){fail(e,new IOException("Camera disconnected"));camera.close();}
                public void onError(CameraDevice camera,int code){fail(e,new IOException("Camera error "+code));camera.close();}
                public void onClosed(CameraDevice camera){e.cameraClosed=true;if(e.stopping)finishEncoder(e);maybeSeal(e);}
            },cameraHandler);
        }catch(Exception ex){fail(e,ex);}
    }
    public void appendTracking(String json) {
        Episode e=active;if(e==null||e.stopping)return;
        if(trackingQueued.incrementAndGet()>256){trackingQueued.decrementAndGet();fail(e,new IOException("Tracking writer backlog"));return;}
        trackingHandler.post(()->{try{e.tracking.write(json);e.tracking.newLine();e.trackingSamples++;}catch(Exception ex){fail(e,ex);}finally{trackingQueued.decrementAndGet();}});
    }
    public void event(String value) {
        Episode e=active;if(e==null||e.stopping)return;
        trackingHandler.post(()->{try{line(e.events,new JSONObject().put("event",value).put("unix_ticks",System.currentTimeMillis()*10000+UNIX_TICKS).put("elapsed_ns",SystemClock.elapsedRealtimeNanos()));}catch(Exception ex){fail(e,ex);}});
    }
    public synchronized void stop() {
        Episode e=active;if(e==null||e.stopping)return;
        event("stop");e.stopping=true;state="Saving";
        if(e.external!=null)e.external.stop();
        trackingHandler.post(()->{try{e.tracking.close();e.events.close();}catch(Exception ex){e.failure=ex.toString();}e.trackingClosed=true;maybeSeal(e);});
        cameraHandler.post(()->{
            try{
                if(e.session!=null&&e.repeatingStarted){e.repeatingStarted=false;e.session.stopRepeating();}
                else if(e.camera!=null)e.camera.close();
                else {e.cameraClosed=true;finishEncoder(e);}
            } catch(Exception ex){if(e.failure.isEmpty())e.failure=ex.toString();if(e.camera!=null)e.camera.close();else{e.cameraClosed=true;finishEncoder(e);}}
            // Keep in-flight capture results until sequence completion. Closing or
            // aborting immediately can lose metadata for the last encoded frame.
            cameraHandler.postDelayed(()->{if(!e.finalized){e.failure="Stop timed out; partial take retained";if(e.camera!=null)e.camera.close();e.cameraClosed=true;codecHandler.post(()->closeCodec(e));}},10000);
        });
    }
    private void finishEncoder(Episode e){
        codecHandler.post(()->{try{if(e.codec!=null&&!e.codecClosed)e.codec.signalEndOfInputStream();else closeCodec(e);}catch(Exception ex){e.failure=ex.toString();closeCodec(e);}});
    }
    private void fail(Episode e,Exception ex){if(e.failure.isEmpty())e.failure=ex.toString();if(active==e){error=e.failure;stop();}}
    private synchronized void closeCodec(Episode e) {
        if(e.codecClosed)return;
        try{if(e.codec!=null){e.codec.stop();e.codec.release();}if(e.muxer!=null){if(e.muxing)e.muxer.stop();e.muxer.release();}if(e.surface!=null)e.surface.release();e.encoded.close();}
        catch(Exception ex){e.failure=ex.toString();}
        e.codecClosed=true;maybeSeal(e);
    }
    private synchronized void maybeSeal(Episode e) {
        if(!e.stopping||!e.cameraClosed||!e.codecClosed||!e.trackingClosed||!e.externalClosed||e.finalizeStarted)return;
        e.finalizeStarted=true;
        try{e.frames.close();}catch(Exception ex){e.failure=ex.toString();}
        e.finalized=true;if(active==e){active=null;recordingFinished(e);}
        if(e.failure.isEmpty()&&(e.sensorFrames==0||e.encodedFrames==0||e.trackingSamples==0))e.failure="Missing required camera or tracking samples";
        if(!e.failure.isEmpty()) {
            state="Recording error";error=e.failure;
            try{writeAtomic(new File(e.dir,"interrupted.json"),new JSONObject().put("reason",e.failure).toString());}catch(Exception ignored){}
            return;
        }
        state="Ready";
        sealing.execute(()->{try{
            JSONObject files=new JSONObject();for(File f:e.dir.listFiles())if(f.isFile())files.put(f.getName(),new JSONObject().put("bytes",f.length()).put("sha256",sha(f)));
            JSONObject manifest=new JSONObject().put("schema_version",3).put("episode_id",e.id).put("created_unix_ms",e.created)
                .put("width",1280).put("height",1280).put("requested_fps",30).put("codec","video/avc").put("bitrate",20000000)
                .put("sensor_frames",e.sensorFrames).put("encoded_frames",e.encodedFrames).put("tracking_samples",e.trackingSamples)
                .put("hardware_synchronized",false).put("clock_alignment","unverified_offline_required")
                .put("external_camera",new JSONObject().put("enabled",e.external!=null).put("schema_version",1).put("clock_alignment","same_host_arrival_timestamps_exposure_alignment_unverified"))
                .put("camera_pose_source","camera_extrinsics_plus_timestamped_hmd_device_pose_requires_calibration")
                .put("mount_mode",e.mountMode)
                .put("hmd_pose_role",e.mountMode.equals("neck")?"neck_mounted_device":"head_mounted_device")
                .put("mount_calibration","unverified")
                .put("body_pose_validation",e.mountMode.equals("neck")?"unverified_neck_mount":"meta_runtime_accuracy_unverified")
                .put("proximity_policy",e.mountMode.equals("neck")?"periodic_close_override_until_head_or_20min_idle":"normal_sensor")
                .put("app_version","3.0.12-raw").put("files",files);
            writeAtomic(new File(e.dir,"manifest.json"),manifest.toString());
        }catch(Exception ex){try{writeAtomic(new File(e.dir,"interrupted.json"),new JSONObject().put("reason",ex.toString()).toString());}catch(Exception ignored){}}});
    }
    private long episodeCreated(File directory){
        try(InputStream in=new FileInputStream(new File(directory,"manifest.json"))){return new JSONObject(read(in)).getLong("created_unix_ms");}
        catch(Exception e){return Long.MAX_VALUE;}
    }
    private List<File> episodes(){File[] all=root.listFiles(File::isDirectory);return all==null?new ArrayList<>():new ArrayList<>(Arrays.asList(all));}
    private int pending(){int n=0;for(File d:episodes())if(new File(d,"manifest.json").exists()&&!new File(d,"receipt.json").exists())n++;return n;}
    private void uploadLoop(){int delay=2;while(alive){try{
        List<File> q=episodes();q.sort(Comparator.comparingLong(this::episodeCreated));File next=null;
        for(File d:q)if(new File(d,"manifest.json").exists()&&!new File(d,"receipt.json").exists()){next=d;break;}
        if(next==null){uploadState="Idle";progress=0;Thread.sleep(1000);continue;}
        uploadingId=next.getName();uploadState="Checking receiver";progress=0;
        File manifestFile=new File(next,"manifest.json");String manifestDigest=sha(manifestFile);
        String endpoint=serverUrl.replaceAll("/offer/?$","")+"/upload_raw";
        HttpURLConnection probe=(HttpURLConnection)new URL(endpoint+"/receipt?upload_id="+next.getName()+"&manifest_sha256="+manifestDigest).openConnection();
        try {
            probe.setConnectTimeout(10000);probe.setReadTimeout(15000);
            int code=probe.getResponseCode();
            if(code==200){String reply=read(probe.getInputStream());checkReceipt(reply,next.getName(),manifestDigest,null,null);confirmUpload(next,reply);delay=2;continue;}
            if(code!=404)throw new IOException("Receipt lookup HTTP "+code);
        }finally{probe.disconnect();}
        JSONObject manifest;try(InputStream in=new FileInputStream(manifestFile)){manifest=new JSONObject(read(in));}
        List<String> names=new ArrayList<>();names.add("manifest.json");long total=manifestFile.length();
        JSONObject entries=manifest.getJSONObject("files");Iterator<String> keys=entries.keys();
        while(keys.hasNext()){String name=keys.next();long bytes=entries.getJSONObject(name).getLong("bytes");
            if(new File(next,name).length()!=bytes)throw new IOException("Source size differs from manifest: "+name);
            names.add(name);total=Math.addExact(total,bytes);}
        final long sourceBytes=total;long started=SystemClock.elapsedRealtime();uploadState="Compressing + sending";
        HttpURLConnection c=(HttpURLConnection)new URL(endpoint+"?upload_id="+next.getName()).openConnection();
        try {
            c.setRequestMethod("POST");c.setDoOutput(true);c.setConnectTimeout(10000);c.setReadTimeout(300000);c.setChunkedStreamingMode(262144);
            c.setRequestProperty("Content-Type","application/octet-stream");c.setRequestProperty("X-Warp-Stream","zip-zstd-block-v1");
            c.setRequestProperty("X-Warp-Source-Bytes",Long.toString(sourceBytes));c.setRequestProperty("X-Warp-Manifest-SHA256",manifestDigest);
            WarpZstdOutputStream compressed=new WarpZstdOutputStream(c.getOutputStream());
            WarpUploadArchive.Result result=WarpUploadArchive.write(compressed,next,names,0,(raw,wire)->{progress=(double)raw/sourceBytes;yieldToCapture();});
            if(result.sourceBytes!=sourceBytes)throw new IOException("Source size changed during upload");
            uploadState="Verifying on receiver";
            if(c.getResponseCode()!=200)throw new IOException("Receiver HTTP "+c.getResponseCode()+": "+read(c.getErrorStream()));
            String reply=read(c.getInputStream());checkReceipt(reply,next.getName(),manifestDigest,result.sha256,compressed.sha256);
            confirmUpload(next,reply);delay=2;
            android.util.Log.i("WarpRaw","Upload complete id="+next.getName()+" source_bytes="+result.sourceBytes+" wire_bytes="+compressed.wireBytes+" elapsed_ms="+(SystemClock.elapsedRealtime()-started)+" compression=zstd-1");
        }finally{c.disconnect();}
    }catch(Exception ex){android.util.Log.w("WarpRaw","Upload retry: "+ex.getMessage());uploadState="Retrying";uploadingId="";progress=0;try{Thread.sleep(delay*1000L);}catch(InterruptedException ignored){}delay=Math.min(60,delay*2);}}}
    private void checkReceipt(String reply,String id,String manifestDigest,String archiveDigest,String wireDigest)throws Exception{
        JSONObject r=new JSONObject(reply);
        if(!r.getString("upload_id").equals(id)||!r.getString("state").equals("saved")||!r.getString("manifest_sha256").equals(manifestDigest)
            ||(archiveDigest!=null&&!r.getString("sha256").equals(archiveDigest))
            ||(wireDigest!=null&&!r.getJSONObject("transport").getString("wire_sha256").equals(wireDigest)))throw new IOException("Receipt mismatch");
    }
    private void confirmUpload(File directory,String reply)throws Exception{
        writeAtomic(new File(directory,"receipt.json"),reply);
        new File(directory,"upload.zip").delete();new File(directory,"upload.zip.partial").delete();uploadingId="";progress=1;
    }
    private void yieldToCapture() throws InterruptedException{if(active!=null)Thread.sleep(10);}
    private String sha(File f)throws Exception{MessageDigest md=MessageDigest.getInstance("SHA-256");try(InputStream in=new FileInputStream(f)){byte[] b=new byte[65536];int n;while((n=in.read(b))!=-1){md.update(b,0,n);yieldToCapture();}}StringBuilder s=new StringBuilder();for(byte b:md.digest())s.append(String.format("%02x",b&255));return s.toString();}
    private static BufferedWriter writer(File d,String n)throws IOException{return new BufferedWriter(new OutputStreamWriter(new FileOutputStream(new File(d,n)),"UTF-8"),65536);}
    private static void line(BufferedWriter w,JSONObject j)throws IOException{w.write(j.toString());w.newLine();}
    private static JSONArray array(float[] a)throws JSONException{JSONArray j=new JSONArray();if(a!=null)for(float x:a)j.put(x);return j;}
    private static String read(InputStream in)throws IOException{if(in==null)return "";ByteArrayOutputStream b=new ByteArrayOutputStream();byte[] x=new byte[4096];int n;while((n=in.read(x))!=-1){if(b.size()>1048576)throw new IOException("Reply too large");b.write(x,0,n);}return b.toString("UTF-8");}
    private static void writeAtomic(File f,String content)throws IOException{android.util.AtomicFile a=new android.util.AtomicFile(f);FileOutputStream o=a.startWrite();try{o.write(content.getBytes("UTF-8"));a.finishWrite(o);}catch(IOException e){a.failWrite(o);throw e;}}
    private static void deleteTree(File d)throws IOException{File[] fs=d.listFiles();if(fs!=null)for(File f:fs){if(f.isDirectory())deleteTree(f);else if(!f.delete())throw new IOException("Cannot delete confirmed backup");}if(!d.delete())throw new IOException("Cannot remove confirmed backup");}
}
