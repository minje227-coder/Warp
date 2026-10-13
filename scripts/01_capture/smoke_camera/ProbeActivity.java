package dev.warp.captureprobe;
import android.app.Activity;
import android.os.*;
import android.util.Log;
import dev.warp.capture.WarpRawRecorder;

/** Camera-only smoke probe; deliberately does not fabricate body tracking. */
public class ProbeActivity extends Activity {
    private WarpRawRecorder recorder;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private int polls;
    @Override public void onCreate(Bundle saved) {
        super.onCreate(saved);
        getWindow().addFlags(128);
        recorder = new WarpRawRecorder(this, "http://127.0.0.1:1");
        try {
            android.hardware.camera2.CameraManager manager = (android.hardware.camera2.CameraManager)getSystemService(CAMERA_SERVICE);
            for (String id : manager.getCameraIdList()) {
                android.hardware.camera2.CameraCharacteristics c = manager.getCameraCharacteristics(id);
                for (android.hardware.camera2.CameraCharacteristics.Key<?> key : c.getKeys()) {
                    if (key.getName().startsWith("com.meta")) {
                        Object value = c.get(key);
                        Log.i("WarpProbe", "camera=" + id + " key=" + key.getName() + " value=" + value + " class=" + (value == null ? "null" : value.getClass().getName()));
                    }
                }
            }
        } catch (Exception ex) { Log.e("WarpProbe", "query", ex); }
        Log.i("WarpProbe", "capabilities=" + recorder.capabilities());
        handler.postDelayed(() -> Log.i("WarpProbe", "start=" + recorder.start()), 1500);
        handler.postDelayed(new Runnable() {
            public void run() {
                Log.i("WarpProbe", "status=" + recorder.status());
                polls++;
                if (polls == 12) recorder.stop();
                if (polls < 28) handler.postDelayed(this, 1000);
                else { Log.i("WarpProbe", "DONE"); finish(); }
            }
        }, 1000);
    }
    @Override public void onDestroy() {
        if (recorder != null) recorder.stop();
        super.onDestroy();
    }
}
