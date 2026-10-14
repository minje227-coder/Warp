package dev.warp.capture;

import android.view.KeyEvent;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import com.unity3d.player.UnityPlayer;
import com.unity3d.player.UnityPlayerGameActivity;

/** Volume up toggles capture; volume down selects the headset mount mode. */
public class WarpVolumeActivity extends UnityPlayerGameActivity {
    private boolean volumeUpHeld = false;
    private boolean volumeDownHeld = false;
    private boolean neckMount, proximityOverrideActive, captureBusy;
    private long neckIdleSinceMs;
    private static final long NECK_IDLE_TIMEOUT_MS = 20L * 60L * 1000L;
    private final Handler proximityHandler = new Handler(Looper.getMainLooper());
    // A short override lease limits the effect after an unexpected process exit.
    private static final int PROXIMITY_LEASE_MS = 6000;
    private final Runnable proximityPulse = new Runnable() {
        @Override public void run() { refreshMountProximity(); }
    };
    private final Runnable neckIdleCheck = new Runnable() {
        @Override public void run() { checkNeckIdle(); }
    };
    @Override protected void onCreate(Bundle savedState) {
        super.onCreate(savedState);
        neckMount = getSharedPreferences("WarpMountMode", MODE_PRIVATE).getBoolean("neck", false);
        neckIdleSinceMs = SystemClock.elapsedRealtime();
    }
    @Override protected void onStart() {
        super.onStart();
        if (neckMount) { refreshMountProximity(); scheduleNeckIdleCheck(); }
    }
    public synchronized boolean isNeckMount() { return neckMount; }
    /** A repeated mode sync must not restart the idle period. */
    public synchronized void setNeckMount(boolean value) {
        if (neckMount == value || captureBusy) return;
        neckMount = value;
        getSharedPreferences("WarpMountMode", MODE_PRIVATE).edit().putBoolean("neck", value).apply();
        if (neckMount) {
            neckIdleSinceMs = SystemClock.elapsedRealtime();
            refreshMountProximity();
        } else releaseMountProximity();
        scheduleNeckIdleCheck();
    }
    /** Lock before countdown; native sealing unlocks even when Unity is paused. */
    public synchronized void setCaptureBusy(boolean value) {
        if (captureBusy == value) return;
        captureBusy = value;
        scheduleNeckIdleCheck();
    }
    /** Only a completed recording resets the reference time; canceled countdowns do not. */
    public synchronized void onRecordingFinished() {
        if (!captureBusy) return;
        if (neckMount) neckIdleSinceMs = SystemClock.elapsedRealtime();
        setCaptureBusy(false);
    }
    private synchronized void scheduleNeckIdleCheck() {
        proximityHandler.removeCallbacks(neckIdleCheck);
        if (!neckMount || captureBusy || isFinishing() || isDestroyed()) return;
        long remaining = NECK_IDLE_TIMEOUT_MS - (SystemClock.elapsedRealtime() - neckIdleSinceMs);
        proximityHandler.postDelayed(neckIdleCheck, Math.max(0L, remaining));
    }
    private synchronized void checkNeckIdle() {
        if (!neckMount || captureBusy || isFinishing() || isDestroyed()) return;
        if (SystemClock.elapsedRealtime() - neckIdleSinceMs < NECK_IDLE_TIMEOUT_MS) {
            scheduleNeckIdleCheck(); return;
        }
        setNeckMount(false);
        Log.i("WarpMount", "NECK idle timeout: HEAD after 20 minutes without recording");
    }
    private synchronized void refreshMountProximity() {
        proximityHandler.removeCallbacks(proximityPulse);
        if (!neckMount || isFinishing() || isDestroyed()) return;
        try {
            Intent intent = new Intent("com.oculus.vrpowermanager.prox_close");
            intent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
            intent.putExtra("duration", PROXIMITY_LEASE_MS);
            sendBroadcast(intent);
            if (!proximityOverrideActive) Log.i("WarpMount", "NECK proximity override requested");
            proximityOverrideActive = true;
        } catch (Exception e) { Log.e("WarpMount", "Proximity override failed", e); }
        proximityHandler.postDelayed(proximityPulse, 2000);
    }
    public synchronized void releaseMountProximity() {
        if (captureBusy) return;
        proximityHandler.removeCallbacks(neckIdleCheck);
        proximityHandler.removeCallbacks(proximityPulse);
        if (!proximityOverrideActive) return;
        try {
            Intent intent = new Intent("com.oculus.vrpowermanager.automation_disable");
            intent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
            sendBroadcast(intent);
            proximityOverrideActive = false;
            Log.i("WarpMount", "Proximity override released");
        } catch (Exception e) { Log.e("WarpMount", "Proximity restore failed", e); }
    }
    @Override protected void onStop() {
        // Quest may stop this Activity temporarily while VR focus changes.
        // Keep the selected NECK override across that transition.
        if (isFinishing()) releaseMountProximity();
        super.onStop();
    }
    @Override protected void onDestroy() {
        releaseMountProximity(); super.onDestroy();
    }
    @Override public boolean dispatchKeyEvent(KeyEvent event) {
        int key = event.getKeyCode();
        if (key != KeyEvent.KEYCODE_VOLUME_UP && key != KeyEvent.KEYCODE_VOLUME_DOWN)
            return super.dispatchKeyEvent(event);
        boolean up = key == KeyEvent.KEYCODE_VOLUME_UP;
        if (event.getAction() == KeyEvent.ACTION_DOWN) {
            boolean held = up ? volumeUpHeld : volumeDownHeld;
            if (up) volumeUpHeld = true; else volumeDownHeld = true;
            if (!held && event.getRepeatCount() == 0)
                UnityPlayer.UnitySendMessage("WarpCaptureStatus", up ? "OnVolumeUp" : "OnVolumeDown", "");
        } else if (event.getAction() == KeyEvent.ACTION_UP) {
            if (up) volumeUpHeld = false; else volumeDownHeld = false;
        }
        return true;
    }
    @Override public void onWindowFocusChanged(boolean focused) {
        if (!focused) { volumeUpHeld = false; volumeDownHeld = false; }
        super.onWindowFocusChanged(focused);
    }
}
