package dev.warp.capture;

import android.app.Activity;
import android.hardware.usb.*;
import org.json.JSONObject;

/** Idle-only USB owner. Recording waits until this owner's connection is closed. */
public final class WarpUsbPreview {
    private final Activity activity;
    private WarpUsbCamera camera;
    private boolean opening, wanted;
    private String error = "";
    private long lastReportMs;

    public WarpUsbPreview(Activity activity) { this.activity = activity; }

    public synchronized String status() {
        try {
            UsbManager manager = (UsbManager)activity.getSystemService(Activity.USB_SERVICE);
            boolean connected = false, permitted = false;
            for (UsbDevice device : manager.getDeviceList().values()) {
                for (int i = 0; i < device.getInterfaceCount(); i++) {
                    if (device.getInterface(i).getInterfaceClass() == 14) {
                        connected = true;
                        permitted |= manager.hasPermission(device);
                        break;
                    }
                }
            }
            for (String permission : new String[]{"android.permission.CAMERA", "horizonos.permission.USB_CAMERA"})
                permitted &= activity.checkSelfPermission(permission) == android.content.pm.PackageManager.PERMISSION_GRANTED;
            double fps=connected?WarpUsbCamera.sourceFps():-1;
            long now=android.os.SystemClock.elapsedRealtime();
            if(camera!=null&&fps>0&&now-lastReportMs>=5000){
                lastReportMs=now;android.util.Log.i("WarpUsbPreview","Live external RGB arrival FPS="+fps);
            }
            return new JSONObject().put("connected", connected).put("permitted", permitted)
                .put("source_fps", fps).put("released", !opening && camera == null).put("error", error).toString();
        } catch (Exception e) {
            return "{\"connected\":false,\"permitted\":false,\"error\":\"USB query failed\"}";
        }
    }

    public synchronized void start() {
        wanted = true;
        if (opening || camera != null) return;
        opening = true;
        new Thread(() -> {
            WarpUsbCamera opened = null;
            try {
                opened = WarpUsbCamera.openIfAttached(activity, null);
                synchronized (WarpUsbPreview.this) {
                    if (!wanted) {
                        if (opened != null) opened.closeUnstartedPreview();
                        opening = false;
                        return;
                    }
                    if (opened != null) {
                        final WarpUsbCamera stream = opened;
                        camera = stream;
                        stream.startPreview(failure -> {
                            synchronized (WarpUsbPreview.this) {
                                error = failure;
                                if (camera == stream) camera = null;
                            }
                        });
                    }
                    error = "";
                    opening = false;
                }
            } catch (Exception | LinkageError e) {
                synchronized (WarpUsbPreview.this) {
                    if (opened != null && camera == opened) {
                        opened.closeUnstartedPreview();
                        camera = null;
                    }
                    error = e.toString();
                    opening = false;
                }
            }
        }, "WarpUsbPreviewOpen").start();
    }

    public synchronized void stop() {
        wanted = false;
        if (camera != null) camera.stopPreview();
    }
    public synchronized boolean isReleased() { return !opening && camera == null; }
    public synchronized byte[] frame() { return camera == null ? WarpUsbCamera.recordingPreviewFrame() : camera.previewFrame(); }
}
