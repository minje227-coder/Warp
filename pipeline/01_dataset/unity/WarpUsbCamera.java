package dev.warp.capture;

import android.app.Activity;
import android.hardware.usb.*;
import android.os.SystemClock;
import android.util.Base64;
import android.util.Log;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import java.io.*;
import org.json.*;

/** One external UVC MJPEG stream, retained with raw packet-arrival timestamps. */
public final class WarpUsbCamera {
    private static native String capture(int fd,int endpoint,int packet,String path,int durationMs);
    private static native String captureBulk(int fd,int endpoint,int payload,int frameBytes,String path,int durationMs);
    private static native void resetStop();
    private static native void requestStop();
    private static native void resetPreviewStop();
    private static native void requestPreviewStop();
    private static native byte[] latestPreviewJpeg();
    private static native double measuredFps();
    private static volatile boolean nativeLoaded;
    private static volatile WarpUsbCamera recordingCamera;
    public static byte[] recordingPreviewFrame(){WarpUsbCamera camera=recordingCamera;return camera==null?null:camera.previewFrame();}
    public static double sourceFps(){return nativeLoaded?measuredFps():-1;}
    public interface Completion {void done(String error);}
    private final UsbDeviceConnection connection;
    private final UsbInterface base,active;
    private final UsbEndpoint endpoint;
    private final File directory;
    private final JSONObject metadata;
    private final int packet;
    private final boolean yuyv;
    private final int imageWidth,imageHeight,bulkPayload;
    private final String rawName;
    private volatile boolean stopRequested,finished;
    private volatile String error="";

    /** No connected UVC camera preserves the existing Quest-only recording flow. */
    public static WarpUsbCamera openIfAttached(Activity activity,File directory) throws Exception {
        UsbManager manager=(UsbManager)activity.getSystemService(Activity.USB_SERVICE);
        UsbDevice selected=null;
        for(UsbDevice d:manager.getDeviceList().values()) {
            boolean video=false;
            for(int i=0;i<d.getInterfaceCount();i++)if(d.getInterface(i).getInterfaceClass()==14)video=true;
            if(!video){if(d.getVendorId()==7994&&d.getProductId()==4098)throw new IOException("Switch the USB camera to PC Camera mode");continue;}
            if(selected!=null)throw new IOException("Multiple USB cameras attached; connect one camera");
            selected=d;
        }
        if(selected==null)return null;
        if(!manager.hasPermission(selected)){activity.runOnUiThread(()->requestAttachedAccess(activity));throw new IOException("Allow USB camera access, then start a new recording");}
        System.loadLibrary("warp_usb_iso");nativeLoaded=true;
        return new WarpUsbCamera(manager,selected,directory);
    }
    /** Request only when a UVC camera is present; the next capture uses the grant. */
    public static void requestAttachedAccess(Activity activity) {
        UsbManager manager=(UsbManager)activity.getSystemService(Activity.USB_SERVICE);
        for(UsbDevice device:manager.getDeviceList().values()) {
            boolean video=false;
            for(int i=0;i<device.getInterfaceCount();i++)if(device.getInterface(i).getInterfaceClass()==14)video=true;
            if(!video||manager.hasPermission(device))continue;
            java.util.ArrayList<String> missing=new java.util.ArrayList<>();
            for(String permission:new String[]{"android.permission.CAMERA","horizonos.permission.USB_CAMERA"})
                if(activity.checkSelfPermission(permission)!=android.content.pm.PackageManager.PERMISSION_GRANTED)missing.add(permission);
            if(!missing.isEmpty()) {activity.requestPermissions(missing.toArray(new String[0]),47120);return;}
            android.content.Intent intent=new android.content.Intent(activity.getPackageName()+".USB_PERMISSION").setPackage(activity.getPackageName());
            manager.requestPermission(device,android.app.PendingIntent.getBroadcast(activity,47121,intent,android.app.PendingIntent.FLAG_IMMUTABLE));
            return;
        }
    }
    private static int u16(byte[] b,int o){return (b[o]&255)|((b[o+1]&255)<<8);}
    private static long u32(byte[] b,int o){return (u16(b,o)&65535L)|((u16(b,o+2)&65535L)<<16);}
    private static void put32(byte[] b,int o,long v){for(int i=0;i<4;i++)b[o+i]=(byte)(v>>(i*8));}
    private static JSONObject anchor() throws JSONException {
        long before=SystemClock.elapsedRealtimeNanos(),utc=System.currentTimeMillis(),after=SystemClock.elapsedRealtimeNanos();
        return new JSONObject().put("elapsed_before_ns",before).put("unix_ms",utc).put("elapsed_after_ns",after);
    }
    private static void save(File file,JSONObject value) throws Exception {
        android.util.AtomicFile atomic=new android.util.AtomicFile(file);FileOutputStream out=atomic.startWrite();
        try{out.write(value.toString().getBytes("UTF-8"));atomic.finishWrite(out);}catch(Exception e){atomic.failWrite(out);throw e;}
    }
    /** Select only a YUY2 RGB streaming interface; depth/IR interfaces are never opened. */
    static final class RgbMode {
        int iface=-1,format,frame,width,height;long interval;
    }
    static RgbMode d435RgbMode(byte[] raw) throws IOException {
        RgbMode best=null;int cls=0,sub=0,iface=-1,format=-1,frameKind=5;
        for(int o=0;o+2<=raw.length;){
            int n=raw[o]&255;if(n<2||o+n>raw.length)throw new IOException("Malformed USB camera descriptor");
            int type=raw[o+1]&255;
            if(type==4&&n>=9){iface=raw[o+2]&255;cls=raw[o+5]&255;sub=raw[o+6]&255;format=-1;}
            if(cls==14&&sub==2&&type==0x24&&n>=4){
                int kind=raw[o+2]&255;
                if((kind==4||kind==16)&&n>=27){
                    byte[] guid={'Y','U','Y','2',0,0,0x10,0,(byte)0x80,0,0,(byte)0xaa,0,0x38,(byte)0x9b,0x71};
                    boolean match=(raw[o+21]&255)==16;
                    for(int j=0;j<16;j++)match&=raw[o+5+j]==guid[j];
                    format=match?raw[o+3]&255:-1;frameKind=kind==16?17:5;
                }else if(kind>=4&&kind!=5&&kind!=17)format=-1;
                if(kind==frameKind&&n>=26&&format>0){
                    int w=u16(raw,o+5),h=u16(raw,o+7),count=raw[o+(kind==17?21:25)]&255;
                    if(w<=0||h<=0||(w&1)!=0||w>8192||h>8192||(long)w*h*2>8*1024*1024){o+=n;continue;}
                    long interval=0;
                    if(count>0){
                        if(n<26+count*4)throw new IOException("Truncated UVC frame intervals");
                        for(int j=0;j<count;j++){long v=u32(raw,o+26+j*4);if(v>=333333&&v<=333334&&(interval==0||v<interval))interval=v;}
                    }else if(n>=38){
                        long lo=u32(raw,o+26),hi=u32(raw,o+30),step=u32(raw,o+34);
                        long wanted=Math.max(lo,333333);if(step>0)wanted=lo+((wanted-lo+step-1)/step)*step;
                        if(lo>0&&wanted<=hi&&wanted<=333334)interval=wanted;
                    }
                    if(interval>0&&(best==null||w*h>best.width*best.height||(w*h==best.width*best.height&&interval<best.interval))){
                        best=new RgbMode();best.iface=iface;best.format=format;best.frame=raw[o+3]&255;best.width=w;best.height=h;best.interval=interval;
                    }
                }
            }
            o+=n;
        }
        if(best==null)throw new IOException("D435 has no supported YUY2 RGB mode at 30fps");
        return best;
    }
    private WarpUsbCamera(UsbManager manager,UsbDevice device,File dir)throws Exception {
        directory=dir;UsbDeviceConnection c=manager.openDevice(device);
        if(c==null)throw new IOException("USB camera open failed");
        UsbInterface b=null,a=null;UsbEndpoint ep=null;
        try {
            byte[] raw=c.getRawDescriptors();int cls=0,sub=0,currentFormat=-1,format=-1,frame=-1,width=0,height=0;long interval=0,clockHz=0;
            yuyv=device.getVendorId()==0x8086&&(device.getProductId()==0x0b07||device.getProductId()==0x0b3a);
            int rgbInterface=-1;
            if(yuyv){RgbMode mode=d435RgbMode(raw);rgbInterface=mode.iface;format=mode.format;frame=mode.frame;width=mode.width;height=mode.height;interval=mode.interval;}
            for(int o=0;o+2<=raw.length;){
                int n=raw[o]&255;if(n<2||o+n>raw.length)throw new IOException("Malformed USB camera descriptor");
                int type=raw[o+1]&255;
                if(type==4&&n>=9){cls=raw[o+5]&255;sub=raw[o+6]&255;}
                if(type==0x24&&n>=4){int kind=raw[o+2]&255;
                    if(cls==14&&sub==1&&kind==1&&n>=11)clockHz=u32(raw,o+7);
                    if(cls==14&&sub==2&&kind==6)currentFormat=raw[o+3]&255;
                    if(!yuyv&&cls==14&&sub==2&&kind==7&&n>=26&&currentFormat>0&&frame<0){format=currentFormat;frame=raw[o+3]&255;width=u16(raw,o+5);height=u16(raw,o+7);interval=u32(raw,o+21);}
                }
                o+=n;
            }
            if(frame<0||width<=0||height<=0||interval<=0)throw new IOException("USB camera has no supported MJPEG mode");
            for(int i=0;i<device.getInterfaceCount();i++){
                UsbInterface it=device.getInterface(i);if(it.getInterfaceClass()!=14||it.getInterfaceSubclass()!=2)continue;
                if(yuyv&&it.getId()!=rgbInterface)continue;
                if(it.getAlternateSetting()==0)b=it;
                for(int j=0;j<it.getEndpointCount();j++){UsbEndpoint e=it.getEndpoint(j);if(e.getDirection()==128&&e.getType()==(yuyv?2:1)){a=it;ep=e;}}
            }
            if(b==null||a==null||ep==null)throw new IOException("USB camera requires unsupported transport");
            if(!c.claimInterface(b,true))throw new IOException("USB camera streaming interface unavailable");
            // Bulk UVC has no alternate setting that stops the device. Reset a
            // previous stream's endpoint before probing the next capture.
            if(yuyv && c.controlTransfer(0x02,1,0,ep.getAddress(),null,0,2000)!=0)
                throw new IOException("USB camera bulk endpoint reset failed");
            byte[] len=new byte[2];int got=c.controlTransfer(0xa1,0x85,0x100,b.getId(),len,2,1000);int count=got==2?u16(len,0):26;
            boolean swapped=false;
            if(count!=26&&count!=34&&count!=48){int reverse=((len[0]&255)<<8)|(len[1]&255);if(got==2&&(reverse==26||reverse==34||reverse==48)){count=reverse;swapped=true;}else throw new IOException("Unsupported USB probe length");}
            byte[] probe=new byte[count];probe[0]=1;probe[2]=(byte)format;probe[3]=(byte)frame;put32(probe,4,interval);
            if(c.controlTransfer(0x21,1,0x100,b.getId(),probe,count,2000)!=count)throw new IOException("USB camera mode request failed");
            if(c.controlTransfer(0xa1,0x81,0x100,b.getId(),probe,count,2000)!=count)throw new IOException("USB camera mode query failed");
            if(yuyv&&((probe[2]&255)!=format||(probe[3]&255)!=frame||u32(probe,4)!=interval))throw new IOException("D435 changed the requested RGB mode");
            if(c.controlTransfer(0x21,1,0x200,b.getId(),probe,count,2000)!=count)throw new IOException("USB camera mode commit failed");
            int rawPacket=ep.getMaxPacketSize();packet=(rawPacket&2047)*(1+((rawPacket>>11)&3));
            long payload=u32(probe,22);
            if(yuyv&&(payload<12||payload>8*1024*1024+65536))throw new IOException("Unsupported D435 bulk payload size: "+payload);
            bulkPayload=yuyv?(int)payload:0;imageWidth=width;imageHeight=height;rawName=yuyv?"external_camera.yuyv":"external_camera.mjpeg";
            metadata=new JSONObject().put("schema_version",1).put("state","recording").put("vendor_id",device.getVendorId())
                .put("product_id",device.getProductId()).put("device_name",device.getDeviceName()).put("product_name",device.getProductName())
                .put("width",width).put("height",height).put("codec",yuyv?"yuyv422":"mjpeg").put("transport",yuyv?"bulk":"isochronous")
                .put("raw_file",rawName).put("stream_interface",b.getId()).put("bulk_payload_bytes",bulkPayload).put("requested_interval_100ns",interval)
                .put("negotiated_interval_100ns",u32(probe,4)).put("requested_format_index",format).put("requested_frame_index",frame)
                .put("negotiated_format_index",probe[2]&255).put("negotiated_frame_index",probe[3]&255)
                .put("get_len_big_endian_quirk",swapped).put("uvc_descriptor_clock_hz",clockHz)
                .put("usb_descriptors_base64",Base64.encodeToString(raw,Base64.NO_WRAP))
                .put("probe_base64",Base64.encodeToString(probe,Base64.NO_WRAP)).put("endpoint_packet_bytes",packet)
                .put("start_clock_anchor",anchor()).put("hardware_synchronized",false)
                .put("clock_alignment","same_host_arrival_timestamps_exposure_alignment_unverified")
                .put("time_source","usb_payload_arrival_not_exposure").put("dataset_ready",false);
            if(dir!=null)save(new File(dir,"external_camera.json"),metadata);
            if(!c.setInterface(a))throw new IOException("USB camera stream activation failed");
            connection=c;base=b;active=a;endpoint=ep;
            Log.i("WarpUsbCamera","Opened "+device.getProductName()+": "+width+"x"+height+" @ "+(1e7/interval)+" "+(yuyv?"YUY2/Bulk":"MJPEG/ISO")+" interface="+b.getId()+" payload="+bulkPayload);
        }catch(Exception|Error ex){if(b!=null){try{c.setInterface(b);c.releaseInterface(b);}catch(Exception ignored){}}c.close();throw ex;}
    }
    public void start(Completion completion) {
        resetStop();resetPreviewStop();recordingCamera=this;
        new Thread(()->{
            try {
                JSONObject result=new JSONObject(runCapture(new File(directory,rawName).getAbsolutePath()));
                metadata.put("capture_result",result).put("stop_clock_anchor",anchor());
                if(!result.optBoolean("ok")||result.optInt("frames")==0)throw new IOException("External camera capture failed: "+result);
                if(!stopRequested)throw new IOException("External camera ended before recording stop");
                // The native index filename follows the raw video filename.
                File nativeIndex=new File(directory,rawName+".jsonl"),index=new File(directory,"external_camera_frames.jsonl");
                if(!nativeIndex.renameTo(index))throw new IOException("External camera index finalize failed");
                metadata.put("state","complete");
            }catch(Exception ex){error=ex.toString();try{metadata.put("state","error").put("error",error);}catch(JSONException ignored){}}
            finally {
                if(recordingCamera==this)recordingCamera=null;
                String releaseError=releaseConnection();
                if(!releaseError.isEmpty()){error=releaseError;try{metadata.put("state","error").put("error",error);}catch(JSONException ignored){}}
                try{save(new File(directory,"external_camera.json"),metadata);}catch(Exception ex){error=ex.toString();}
                finished=true;Log.i("WarpUsbCamera","Finished: "+metadata);completion.done(error);
            }
        },"WarpUsbVideo").start();
    }
    /** A null capture path selects memory-only preview; no episode files are made. */
    public void startPreview(Completion completion) {
        if(directory!=null)throw new IllegalStateException("Preview must use a separate idle camera");
        resetPreviewStop();
        new Thread(()->{
            String failure="";
            try {
                JSONObject result=new JSONObject(runCapture(null));
                if(!stopRequested)failure="External preview ended: "+result;
            }catch(Exception|LinkageError ex){failure=ex.toString();}
            finally {String releaseError=releaseConnection();if(failure.isEmpty())failure=releaseError;finished=true;completion.done(failure);}
        },"WarpUsbPreviewCapture").start();
    }
    private String runCapture(String path){return yuyv?captureBulk(connection.getFileDescriptor(),endpoint.getAddress(),bulkPayload,imageWidth*imageHeight*2,path,Integer.MAX_VALUE):capture(connection.getFileDescriptor(),endpoint.getAddress(),packet,path,Integer.MAX_VALUE);}
    public byte[] previewFrame(){
        byte[] frame=latestPreviewJpeg();if(frame==null||!yuyv)return frame;
        if(frame.length!=imageWidth*imageHeight*2)return null;
        ByteArrayOutputStream jpeg=new ByteArrayOutputStream();
        if(!new YuvImage(frame,ImageFormat.YUY2,imageWidth,imageHeight,null).compressToJpeg(new Rect(0,0,imageWidth,imageHeight),80,jpeg))return null;
        return jpeg.toByteArray();
    }
    public void stopPreview(){stopRequested=true;requestPreviewStop();}
    private String releaseConnection(){
        String failure="";
        try {
            // Match UVC bulk stop: after the native worker reaps all URBs,
            // CLEAR_FEATURE(ENDPOINT_HALT) tells the device to stop streaming.
            // SET_INTERFACE(0) only stops isochronous streaming.
            if(yuyv){
                if(connection.controlTransfer(0x02,1,0,endpoint.getAddress(),null,0,2000)!=0)
                    failure="USB camera bulk stream stop failed";
            }else if(!connection.setInterface(base))failure="USB camera stream deactivation failed";
        }catch(Exception ex){failure=ex.toString();}
        finally {
            try{connection.releaseInterface(base);}catch(Exception ignored){}
            connection.close();
        }
        return failure;
    }
    public void closeUnstartedPreview(){
        String failure=releaseConnection();
        if(!failure.isEmpty())Log.w("WarpUsbCamera",failure);
    }
    public void stop(){stopRequested=true;requestStop();}
    public boolean isFinished(){return finished;}
}
