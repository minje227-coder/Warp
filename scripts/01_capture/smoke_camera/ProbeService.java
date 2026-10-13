package dev.warp.captureprobe;
import android.app.*;
import android.os.*;
import android.content.Intent;
import java.io.File;
import dev.warp.capture.WarpRawRecorder;

/** Headless foreground-camera probe; delegates only the two context APIs used. */
public class ProbeService extends Service {
    WarpRawRecorder recorder;
    Handler h = new Handler(Looper.getMainLooper());
    class ContextActivity extends Activity {
        @Override public Object getSystemService(String name) { return ProbeService.this.getSystemService(name); }
        @Override public File getExternalFilesDir(String type) { return ProbeService.this.getExternalFilesDir(type); }
    }
    public void onCreate() {
        super.onCreate();
        NotificationManager n = getSystemService(NotificationManager.class);
        n.createNotificationChannel(new NotificationChannel("probe", "Camera smoke test", NotificationManager.IMPORTANCE_LOW));
        startForeground(1, new Notification.Builder(this,"probe").setSmallIcon(android.R.drawable.ic_menu_camera).setContentTitle("Warp camera probe").build());
        recorder = new WarpRawRecorder(new ContextActivity(), "http://127.0.0.1:1");
        android.util.Log.i("WarpProbe", "service start="+recorder.start());
        h.postDelayed(() -> recorder.stop(), 12000);
        h.postDelayed(() -> {android.util.Log.i("WarpProbe","service final="+recorder.status()); stopSelf();}, 28000);
    }
    public IBinder onBind(Intent i) { return null; }
    public void onDestroy() { recorder.stop(); super.onDestroy(); }
}
