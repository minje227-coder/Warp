package dev.warp.capture;

import android.view.KeyEvent;
import com.unity3d.player.UnityPlayer;
import com.unity3d.player.UnityPlayerGameActivity;

/** In-app volume-up capture toggle. Volume-down retains its normal behavior. */
public class WarpVolumeActivity extends UnityPlayerGameActivity {
    private boolean volumeUpHeld = false;
    @Override public boolean dispatchKeyEvent(KeyEvent event) {
        if (event.getKeyCode() != KeyEvent.KEYCODE_VOLUME_UP) return super.dispatchKeyEvent(event);
        if (event.getAction() == KeyEvent.ACTION_DOWN) {
            if (!volumeUpHeld && event.getRepeatCount() == 0) {
                volumeUpHeld = true;
                UnityPlayer.UnitySendMessage("WarpCaptureStatus", "OnVolumeUp", "");
            }
        } else if (event.getAction() == KeyEvent.ACTION_UP) {
            volumeUpHeld = false;
        }
        return true;
    }
    @Override public void onWindowFocusChanged(boolean focused) {
        if (!focused) volumeUpHeld = false;
        super.onWindowFocusChanged(focused);
    }
}
