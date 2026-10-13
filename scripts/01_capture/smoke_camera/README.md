# Native camera smoke probe

This separate debug APK exercises the production `WarpRawRecorder.java` Camera2
to MediaCodec surface route, without Unity or fabricated tracking samples. It
captures approximately 10 seconds after startup. Because there are no tracking
samples, the recorder intentionally retains the take as interrupted; the MP4 and
sensor/encoder logs can still be inspected. This does not verify skeleton or hand
tracking, the production UI, or camera behavior under the Unity rendering load.

On lab6, run `bash build_probe.sh`, install `build/probe.apk`, grant
`android.permission.CAMERA` and `horizonos.permission.HEADSET_CAMERA`, then launch
`dev.warp.captureprobe/.ProbeActivity`. Logs use the `WarpProbe` tag. Pull the
newest directory under
`/sdcard/Android/data/dev.warp.captureprobe/files/WarpRawEpisodes/`, then run
`python analyze_probe.py EPISODE_DIRECTORY` in an environment with PyAV and NumPy.
Pillow is optional for a representative PNG.

If Quest's panel launcher is blocked by a system dialog, the same recorder can be
tested headlessly using
`adb shell am start-foreground-service -n dev.warp.captureprobe/.ProbeService`.
This declares Android's camera foreground-service type and delegates only
`getSystemService` and `getExternalFilesDir` to the service context. It tests the
native capture code, without proving the Activity or Unity lifecycle.

For an unattended device, Meta documents temporary proximity/sleep overrides:
`adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable` and
`adb shell am broadcast -a com.oculus.vrpowermanager.prox_close`. Restore normal
behavior after testing with `automation_enable` and `prox_auto` broadcasts.
See https://developers.meta.com/vr/documentation/unity/meta-xr-operator/quest/ .

Always stop the probe before starting the production APK. Neither the build
script nor the probe modifies production app data or generates fake body samples.
Do not include `build/` outputs in source control.
