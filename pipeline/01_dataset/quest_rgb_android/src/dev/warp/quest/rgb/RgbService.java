package dev.warp.quest.rgb;

import android.app.*;
import android.content.*;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.camera2.*;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.*;
import android.util.Size;
import org.json.*;
import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

/** Camera2 passthrough RGB only. Never captures the rendered VR view or drives a robot. */
public final class RgbService extends Service {
    private final AtomicReference<Frame> latest = new AtomicReference<>();
    private volatile boolean running = false;
    private volatile String error = "starting";
    private String token, allowedClient, eye;
    private int port, fps;
    private long sequence, lastEncodedNs, skipped;
    private volatile long lastRequestNs;
    private HandlerThread cameraThread;
    private Handler handler;
    private CameraDevice camera;
    private CameraCaptureSession session;
    private ImageReader reader;
    private ServerSocket server;
    private JSONObject calibration = new JSONObject();

    private static final class Frame {
        final byte[] jpeg; final long sequence, sensorNs, encodedNs; final int width, height;
        Frame(byte[] jpeg, long sequence, long sensorNs, int w, int h) {
            this.jpeg=jpeg; this.sequence=sequence; this.sensorNs=sensorNs;
            this.width=w; this.height=h; this.encodedNs=SystemClock.elapsedRealtimeNanos();
        }
    }
    @Override public IBinder onBind(Intent intent) { return null; }
    @Override public int onStartCommand(Intent intent, int flags, int id) {
        if (running) return START_NOT_STICKY;
        token=intent == null ? null : intent.getStringExtra("token");
        allowedClient=intent == null ? null : intent.getStringExtra("client");
        eye=intent == null ? "right" : intent.getStringExtra("eye");
        if (eye == null) eye="right";
        port=intent == null ? 18091 : intent.getIntExtra("port",18091);
        fps=intent == null ? 10 : intent.getIntExtra("fps",10);
        if (token == null || token.length()<32 || allowedClient == null || fps<1 || fps>30
                || !(eye.equals("left") || eye.equals("right"))) { stopSelf(); return START_NOT_STICKY; }
        NotificationManager notifications=getSystemService(NotificationManager.class);
        notifications.createNotificationChannel(new NotificationChannel("rgb","Warp RGB recording",NotificationManager.IMPORTANCE_LOW));
        startForeground(1,new Notification.Builder(this,"rgb").setSmallIcon(android.R.drawable.ic_menu_camera)
            .setContentTitle("Warp Quest RGB capture").setContentText("Physical "+eye+" passthrough camera; local lab receiver only").build());
        running=true; lastRequestNs=SystemClock.elapsedRealtimeNanos();
        cameraThread=new HandlerThread("warp-camera"); cameraThread.start(); handler=new Handler(cameraThread.getLooper());
        new Thread(this::serve,"warp-rgb-http").start();
        handler.post(this::openCamera);
        handler.postDelayed(new Runnable() {
            public void run() {
                if (!running) return;
                if (SystemClock.elapsedRealtimeNanos()-lastRequestNs>120_000_000_000L) { stopSelf(); return; }
                handler.postDelayed(this,5000);
            }
        },5000);
        return START_NOT_STICKY;
    }

    @SuppressWarnings("unchecked")
    private Object tag(CameraCharacteristics chars, String name) {
        for (CameraCharacteristics.Key<?> key : chars.getKeys())
            if (key.getName().equals(name)) return chars.get(key);
        return null;
    }
    private Integer vendorInt(Object value) {
        if(value instanceof Number) return ((Number)value).intValue();
        if(value instanceof byte[] && ((byte[])value).length==1) return ((byte[])value)[0]&255;
        if(value instanceof int[] && ((int[])value).length==1) return ((int[])value)[0];
        return null;
    }
    private boolean valueIs(Object value, int wanted) {
        Integer number=vendorInt(value); return number!=null && number==wanted;
    }
    private JSONArray array(float[] values) throws JSONException {
        JSONArray result=new JSONArray(); if (values!=null) for(float value:values) result.put((double)value); return result;
    }
    private void openCamera() {
        try {
            CameraManager manager=getSystemService(CameraManager.class);
            String selected=null; CameraCharacteristics chars=null;
            JSONArray inventory=new JSONArray();
            for (String candidate : manager.getCameraIdList()) {
                CameraCharacteristics c=manager.getCameraCharacteristics(candidate);
                Object source=tag(c,"com.meta.extra_metadata.camera_source"), position=tag(c,"com.meta.extra_metadata.position");
                inventory.put(new JSONObject().put("camera_id",candidate).put("source",vendorInt(source)).put("position",vendorInt(position)));
                if (valueIs(source,0) && valueIs(position,eye.equals("left")?0:1)) { selected=candidate; chars=c; }
            }
            calibration.put("inventory",inventory);
            if (selected==null) throw new IllegalStateException("No vendor-identified "+eye+" passthrough RGB camera");
            StreamConfigurationMap map=chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size chosen=null; JSONArray sizes=new JSONArray();
            for(Size size:map.getOutputSizes(ImageFormat.YUV_420_888)) {
                sizes.put(size.getWidth()+"x"+size.getHeight());
                if(size.getWidth()==1280 && size.getHeight()==960) chosen=size;
            }
            if(chosen==null) throw new IllegalStateException("Tested 1280x960 YUV420 output unavailable; supported="+sizes);
            calibration.put("source","quest_passthrough_camera2").put("eye",eye).put("camera_id",selected)
                .put("width",chosen.getWidth()).put("height",chosen.getHeight()).put("supported_yuv_sizes",sizes)
                .put("timestamp_source",chars.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE))
                .put("sensor_orientation",chars.get(CameraCharacteristics.SENSOR_ORIENTATION))
                .put("intrinsics_raw_sensor",array(chars.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)))
                .put("lens_distortion",array(chars.get(CameraCharacteristics.LENS_DISTORTION)))
                .put("lens_pose_rotation",array(chars.get(CameraCharacteristics.LENS_POSE_ROTATION)))
                .put("lens_pose_translation",array(chars.get(CameraCharacteristics.LENS_POSE_TRANSLATION)))
                .put("intrinsics_note","Raw Camera2 calibration; not claimed as calibrated output pixels or Quest tracking-space extrinsics")
                .put("target_jpeg_fps",fps).put("jpeg_quality",90);
            Rect active=chars.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
            if(active!=null) calibration.put("sensor_active_array",new JSONArray(new int[]{active.left,active.top,active.right,active.bottom}));
            reader=ImageReader.newInstance(chosen.getWidth(),chosen.getHeight(),ImageFormat.YUV_420_888,3);
            reader.setOnImageAvailableListener(this::onImage,handler);
            manager.openCamera(selected,new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice device) { camera=device; configure(); }
                @Override public void onDisconnected(CameraDevice device) { error="camera disconnected"; device.close(); latest.set(null); }
                @Override public void onError(CameraDevice device,int code) { error="Camera2 error "+code; device.close(); latest.set(null); }
            },handler);
        } catch(Exception e) { error=e.getClass().getSimpleName()+": "+e.getMessage(); android.util.Log.e("WarpRGB",error,e); }
    }
    private void configure() {
        try {
            camera.createCaptureSession(Collections.singletonList(reader.getSurface()),new CameraCaptureSession.StateCallback() {
                @Override public void onConfigured(CameraCaptureSession captureSession) {
                    session=captureSession;
                    try {
                        CaptureRequest.Builder request=camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                        request.addTarget(reader.getSurface()); request.set(CaptureRequest.CONTROL_MODE,CaptureRequest.CONTROL_MODE_AUTO);
                        session.setRepeatingRequest(request.build(),null,handler); error=null;
                    } catch(Exception e) { error=e.toString(); }
                }
                @Override public void onConfigureFailed(CameraCaptureSession ignored) { error="YUV capture session configuration failed"; }
            },handler);
        } catch(Exception e) { error=e.toString(); }
    }
    private void onImage(ImageReader source) {
        Image image=null;
        try {
            image=source.acquireLatestImage(); if(image==null) return;
            long now=SystemClock.elapsedRealtimeNanos();
            if(now-lastEncodedNs<1_000_000_000L/fps) { skipped++; return; }
            lastEncodedNs=now; int w=image.getWidth(),h=image.getHeight();
            byte[] nv21=new byte[w*h*3/2]; Image.Plane[] planes=image.getPlanes();
            for(int y=0;y<h;y++) for(int x=0;x<w;x++)
                nv21[y*w+x]=planes[0].getBuffer().get(y*planes[0].getRowStride()+x*planes[0].getPixelStride());
            for(int y=0;y<h/2;y++) for(int x=0;x<w/2;x++) {
                int out=w*h+y*w+x*2;
                nv21[out]=planes[2].getBuffer().get(y*planes[2].getRowStride()+x*planes[2].getPixelStride());
                nv21[out+1]=planes[1].getBuffer().get(y*planes[1].getRowStride()+x*planes[1].getPixelStride());
            }
            ByteArrayOutputStream jpeg=new ByteArrayOutputStream();
            if(!new YuvImage(nv21,ImageFormat.NV21,w,h,null).compressToJpeg(new Rect(0,0,w,h),90,jpeg)) throw new IOException("JPEG encode failed");
            latest.set(new Frame(jpeg.toByteArray(),++sequence,image.getTimestamp(),w,h));
        } catch(Exception e) { error=e.toString(); latest.set(null); }
        finally { if(image!=null) image.close(); }
    }
    private void serve() {
        try {
            server=new ServerSocket(); server.setReuseAddress(true); server.bind(new InetSocketAddress(port));
            while(running) {
                try(Socket client=server.accept()) { client.setSoTimeout(3000); respond(client); }
                catch(Exception e) { if(running) android.util.Log.w("WarpRGB","HTTP: "+e.toString()); }
            }
        } catch(Exception e) { if(running) error="HTTP server: "+e.toString(); }
    }
    private void respond(Socket client) throws Exception {
        BufferedReader input=new BufferedReader(new InputStreamReader(client.getInputStream(),StandardCharsets.US_ASCII));
        String request=input.readLine(); if(request==null || request.length()>512) return;
        String supplied=null,line; int bytes=0;
        while((line=input.readLine())!=null && !line.isEmpty()) {
            bytes+=line.length(); if(bytes>8192) return;
            if(line.toLowerCase(Locale.ROOT).startsWith("x-warp-token:")) supplied=line.substring(13).trim();
        }
        OutputStream output=client.getOutputStream();
        if(!client.getInetAddress().getHostAddress().equals(allowedClient) || !token.equals(supplied)) {
            send(output,403,"application/json","{}".getBytes(StandardCharsets.UTF_8),""); return;
        }
        lastRequestNs=SystemClock.elapsedRealtimeNanos(); Frame frame=latest.get();
        if(request.startsWith("GET /frame ")) {
            if(frame==null || SystemClock.elapsedRealtimeNanos()-frame.encodedNs>2_000_000_000L) {
                send(output,503,"application/json",status().toString().getBytes(StandardCharsets.UTF_8),""); return;
            }
            String extra="X-Frame-Sequence: "+frame.sequence+"\r\nX-Sensor-Timestamp-Ns: "+frame.sensorNs
                +"\r\nX-Quest-Elapsed-Ns: "+SystemClock.elapsedRealtimeNanos()+"\r\nX-Encoded-Elapsed-Ns: "+frame.encodedNs
                +"\r\nX-Width: "+frame.width+"\r\nX-Height: "+frame.height+"\r\n";
            send(output,200,"image/jpeg",frame.jpeg,extra);
        } else if(request.startsWith("GET /status ")) send(output,200,"application/json",status().toString().getBytes(StandardCharsets.UTF_8),"");
        else if(request.startsWith("GET /calibration ")) send(output,200,"application/json",calibration.toString().getBytes(StandardCharsets.UTF_8),"");
        else send(output,404,"application/json","{}".getBytes(StandardCharsets.UTF_8),"");
    }
    private JSONObject status() throws JSONException {
        Frame frame=latest.get();
        return new JSONObject().put("source","quest_passthrough_camera2").put("eye",eye).put("error",error==null?JSONObject.NULL:error)
            .put("encoded_frames",sequence).put("decimated_frames",skipped).put("quest_elapsed_ns",SystemClock.elapsedRealtimeNanos())
            .put("latest_sensor_timestamp_ns",frame==null?JSONObject.NULL:frame.sensorNs)
            .put("latest_age_s",frame==null?JSONObject.NULL:(SystemClock.elapsedRealtimeNanos()-frame.encodedNs)/1e9);
    }
    private void send(OutputStream output,int code,String type,byte[] body,String extra) throws IOException {
        String header="HTTP/1.1 "+code+" "+(code==200?"OK":"Error")+"\r\nContent-Type: "+type+"\r\nContent-Length: "+body.length
            +"\r\nConnection: close\r\nCache-Control: no-store\r\n"+extra+"\r\n";
        output.write(header.getBytes(StandardCharsets.US_ASCII)); output.write(body); output.flush();
    }
    @Override public void onDestroy() {
        running=false;
        try { if(server!=null) server.close(); } catch(IOException ignored) {}
        if(session!=null) session.close(); if(camera!=null) camera.close(); if(reader!=null) reader.close();
        if(cameraThread!=null) cameraThread.quitSafely(); latest.set(null); stopForeground(STOP_FOREGROUND_REMOVE); super.onDestroy();
    }
}
