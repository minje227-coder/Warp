package dev.warp.quest.rgb;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.widget.TextView;

public final class MainActivity extends Activity {
    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        TextView view = new TextView(this);
        view.setText("Warp RGB captures the physical Quest passthrough camera.\nStarting camera service…");
        view.setTextSize(22); setContentView(view);
        if (checkSelfPermission("android.permission.CAMERA") != PackageManager.PERMISSION_GRANTED
                || checkSelfPermission("horizonos.permission.HEADSET_CAMERA") != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{"android.permission.CAMERA", "horizonos.permission.HEADSET_CAMERA"}, 1);
        } else startCapture();
    }
    @Override public void onRequestPermissionsResult(int code, String[] names, int[] grants) {
        super.onRequestPermissionsResult(code, names, grants);
        if (code == 1) {
            for (int result : grants) if (result != PackageManager.PERMISSION_GRANTED) return;
            startCapture();
        }
    }
    private void startCapture() {
        Intent service = new Intent(this, RgbService.class);
        android.content.SharedPreferences prefs=getSharedPreferences("capture",MODE_PRIVATE);
        for(String name:new String[]{"token","client","eye"}) {
            String value=getIntent().getStringExtra(name);
            if(value!=null) prefs.edit().putString(name,value).apply();
            else value=prefs.getString(name,name.equals("eye")?"right":null);
            if(value!=null) service.putExtra(name,value);
        }
        service.putExtra("port",getIntent().getIntExtra("port",18091));
        service.putExtra("fps",getIntent().getIntExtra("fps",10));
        startForegroundService(service);
        new Handler().postDelayed(this::finish, 1000);
    }
}
