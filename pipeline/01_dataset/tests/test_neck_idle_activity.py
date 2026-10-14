"""Execute the production Activity against a deterministic Android clock/handler.

This validates timer and sensor broadcasts, not Quest firmware or wearer capture.
Set WARP_JAVA_HOME to the Unity Android OpenJDK if javac is not on PATH.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

STUBS = {
'android/view/KeyEvent.java': '''package android.view; public class KeyEvent {
public static final int KEYCODE_VOLUME_UP=24,KEYCODE_VOLUME_DOWN=25,ACTION_DOWN=0,ACTION_UP=1;
public int getKeyCode(){return 0;} public int getAction(){return 0;} public int getRepeatCount(){return 0;}}''',
'android/content/Intent.java': '''package android.content; public class Intent {
public static final int FLAG_INCLUDE_STOPPED_PACKAGES=32; public String action; public int duration;
public Intent(String a){action=a;} public Intent addFlags(int f){return this;}
public Intent putExtra(String k,int v){duration=v;return this;}}''',
'android/content/SharedPreferences.java': '''package android.content; public class SharedPreferences {
public static boolean value; public boolean getBoolean(String k,boolean d){return value;}
public Editor edit(){return new Editor();} public static class Editor {
public Editor putBoolean(String k,boolean v){value=v;return this;} public void apply(){}}}''',
'android/os/Bundle.java': 'package android.os; public class Bundle {}',
'android/os/Looper.java': 'package android.os; public class Looper {public static Looper getMainLooper(){return new Looper();}}',
'android/os/SystemClock.java': 'package android.os; public class SystemClock {public static long now; public static long elapsedRealtime(){return now;}}',
'android/os/Handler.java': '''package android.os; import java.util.*; public class Handler {
public static Handler latest; static class Task {Runnable run; long at; Task(Runnable r,long t){run=r;at=t;}}
public List<Task> tasks=new ArrayList<>(); public Handler(Looper l){latest=this;}
public void removeCallbacks(Runnable r){tasks.removeIf(x->x.run==r);}
public boolean postDelayed(Runnable r,long d){tasks.add(new Task(r,SystemClock.now+d));return true;}
public void advance(long delta){long target=SystemClock.now+delta; while(true){
Task next=tasks.stream().filter(t->t.at<=target).min(Comparator.comparingLong(t->t.at)).orElse(null);
if(next==null)break;tasks.remove(next);SystemClock.now=next.at;next.run.run();}SystemClock.now=target;}}''',
'android/util/Log.java': '''package android.util; public class Log {
public static int i(String t,String s){return 0;} public static int e(String t,String s,Throwable e){throw new AssertionError(e);}}''',
'com/unity3d/player/UnityPlayer.java': '''package com.unity3d.player; public class UnityPlayer {
public static void UnitySendMessage(String o,String m,String v){}}''',
'com/unity3d/player/UnityPlayerGameActivity.java': '''package com.unity3d.player;
import java.util.*; import android.os.*; import android.content.*; import android.view.*;
public class UnityPlayerGameActivity {
public static final int MODE_PRIVATE=0; public static List<Intent> messages=new ArrayList<>();
public boolean finishing,destroyed;
protected void onCreate(Bundle b){} protected void onStart(){} protected void onStop(){}
protected void onDestroy(){destroyed=true;}
public boolean isFinishing(){return finishing;} public boolean isDestroyed(){return destroyed;}
public SharedPreferences getSharedPreferences(String k,int m){return new SharedPreferences();}
public void runOnUiThread(Runnable r){r.run();} public void sendBroadcast(Intent i){messages.add(i);}
public boolean dispatchKeyEvent(KeyEvent e){return false;} public void onWindowFocusChanged(boolean f){}}''',
'NeckIdleSmoke.java': '''import com.unity3d.player.*; import dev.warp.capture.*;
import android.os.*; import android.content.*;
public class NeckIdleSmoke {
static final long TIMEOUT=20*60*1000L;
static class Activity extends WarpVolumeActivity {
void create(){onCreate(null);onStart();} void stop(){onStop();} void start(){onStart();} void destroy(){onDestroy();}}
static void check(boolean b,String m){if(!b)throw new AssertionError(m);}
static long releases(){return UnityPlayerGameActivity.messages.stream().filter(i->i.action.endsWith("automation_disable")).count();}
static String last(){return UnityPlayerGameActivity.messages.get(UnityPlayerGameActivity.messages.size()-1).action;}
static Activity fresh(){SharedPreferences.value=false;UnityPlayerGameActivity.messages.clear();SystemClock.now=500000;
Activity a=new Activity();a.create();return a;}
static void waitMs(long ms){Handler.latest.advance(ms);}
public static void main(String[] args){
Activity a=fresh();waitMs(TIMEOUT*2);check(UnityPlayerGameActivity.messages.isEmpty(),"HEAD touched sensor");
a.setNeckMount(true);check(last().endsWith("prox_close"),"NECK did not override");
waitMs(TIMEOUT-1);check(a.isNeckMount()&&releases()==0,"timeout too early");
waitMs(1);check(!a.isNeckMount()&&releases()==1&&!SharedPreferences.value,"20-minute boundary/persistence");
check(Handler.latest.tasks.isEmpty(),"expired timer/pulse still scheduled");
// Prior HEAD time and repeated sync must neither count nor restart NECK idle.
a=fresh();waitMs(TIMEOUT*3);a.setNeckMount(true);waitMs(TIMEOUT/2);a.setNeckMount(true);
waitMs(TIMEOUT/2);check(!a.isNeckMount(),"repeated sync extended idle period");
// Lock just before expiry; more than 20 minutes of recording must never release.
a=fresh();a.setNeckMount(true);waitMs(TIMEOUT-1);a.setCaptureBusy(true);
a.setNeckMount(false);a.releaseMountProximity();a.stop();
waitMs(TIMEOUT*3);check(a.isNeckMount()&&releases()==0,"recording enabled sensor or changed mode");
check(last().endsWith("prox_close"),"recording stopped proximity pulses");
a.onRecordingFinished();waitMs(TIMEOUT-1);check(a.isNeckMount(),"end reset did not grant full idle period");
a.onRecordingFinished();waitMs(1);check(!a.isNeckMount(),"repeated unlock reset timer");
// Cancel/reject before camera recording resumes the original idle window.
a=fresh();a.setNeckMount(true);waitMs(TIMEOUT-1);a.setCaptureBusy(true);waitMs(5000);
a.setCaptureBusy(false);waitMs(0);check(!a.isNeckMount(),"canceled countdown reset idle reference");
// Another recording restarts the full window even after substantial idle time.
a=fresh();a.setNeckMount(true);waitMs(TIMEOUT/2);a.setCaptureBusy(true);waitMs(10000);
a.onRecordingFinished();waitMs(TIMEOUT/2);a.setCaptureBusy(true);waitMs(TIMEOUT*2);
a.onRecordingFinished();waitMs(TIMEOUT-1);check(a.isNeckMount(),"second recording reset missing");
waitMs(1);check(!a.isNeckMount(),"second idle never expired");
// Temporary Activity stops retain pulses but cannot extend idle.
a=fresh();a.setNeckMount(true);waitMs(TIMEOUT/2);a.stop();a.start();
waitMs(TIMEOUT/2);check(!a.isNeckMount()&&releases()==1,"temporary stop extended/canceled idle");
// Manual HEAD cancels both callbacks; re-entry gets a full window.
a=fresh();a.setNeckMount(true);waitMs(1000);a.setNeckMount(false);
check(Handler.latest.tasks.isEmpty(),"HEAD left callbacks");waitMs(TIMEOUT*2);
a.setNeckMount(true);waitMs(TIMEOUT-1);check(a.isNeckMount(),"reentry counted HEAD time");waitMs(1);
check(!a.isNeckMount(),"reentry did not expire");
// Idle lifecycle termination releases; persisted NECK starts at Activity entry.
a=fresh();a.setNeckMount(true);a.finishing=true;a.stop();
check(releases()==1&&Handler.latest.tasks.isEmpty(),"finish did not release");a.destroy();
Activity b=new Activity();b.create();waitMs(TIMEOUT-1);check(b.isNeckMount(),"persisted entry timer");
waitMs(1);check(!b.isNeckMount(),"persisted entry did not expire");b.destroy();
System.out.println("PASS: boundary, long recording lock, recording-end reset, repeat sync, focus loss, persistence, manual HEAD");
}}
''',
}

class NeckIdleActivityTests(unittest.TestCase):
    def test_timer_and_proximity_lifecycle(self):
        java_home = os.environ.get('WARP_JAVA_HOME')
        javac = str(Path(java_home) / 'bin/javac') if java_home else shutil.which('javac')
        java = str(Path(java_home) / 'bin/java') if java_home else shutil.which('java')
        if not javac or not java:
            self.skipTest('Java runtime unavailable; set WARP_JAVA_HOME')
        production = Path(__file__).resolve().parents[1] / 'unity/WarpVolumeActivity.java'
        with tempfile.TemporaryDirectory(prefix='warp-neck-idle-test-') as tmp:
            root = Path(tmp)
            for rel, source in STUBS.items():
                dest = root / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_text(source)
            dest = root / 'dev/warp/capture/WarpVolumeActivity.java'
            dest.parent.mkdir(parents=True)
            shutil.copy2(production, dest)
            subprocess.run([javac, '-d', str(root / 'classes'), *map(str, root.rglob('*.java'))], check=True, capture_output=True, text=True, timeout=30)
            result = subprocess.run([java, '-cp', str(root / 'classes'), 'NeckIdleSmoke'], check=True, capture_output=True, text=True, timeout=30)
            self.assertIn('PASS:', result.stdout)
            print(result.stdout.strip())

if __name__ == '__main__':
    unittest.main()
