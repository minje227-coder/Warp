#!/usr/bin/env bash
set -euo pipefail
SDK=${SDK:-/home/lab6/.cache/warp-unity/6000.3.11f1/editor/Editor/Data/PlaybackEngines/AndroidPlayer/SDK}
JDK=${JDK:-/home/lab6/.cache/warp-unity/6000.3.11f1/AndroidPlayer/OpenJDK}
TOOLS="$SDK/build-tools/36.0.0"
HERE=$(cd "$(dirname "$0")" && pwd)
OUT="$HERE/build"
mkdir -p "$OUT/classes" "$OUT/dex" "$OUT/src/dev/warp/capture"
cp "${RECORDER_SOURCE:-/home/lab6/Warp/pipeline/01_dataset/unity/WarpRawRecorder.java}" "$OUT/src/dev/warp/capture/"
"$JDK/bin/javac" -source 8 -target 8 -cp "$SDK/platforms/android-34/android.jar" -d "$OUT/classes" "$OUT/src/dev/warp/capture/WarpRawRecorder.java" "$HERE/ProbeActivity.java" "$HERE/ProbeService.java"
find "$OUT/classes" -name '*.class' > "$OUT/classes.list"
JAVA_HOME="$JDK" PATH="$JDK/bin:$PATH" "$TOOLS/d8" --lib "$SDK/platforms/android-34/android.jar" --output "$OUT/dex" $(cat "$OUT/classes.list")
"$TOOLS/aapt2" link -I "$SDK/platforms/android-34/android.jar" --manifest "$HERE/AndroidManifest.xml" -o "$OUT/probe-unsigned.apk"
(cd "$OUT/dex" && zip -q "$OUT/probe-unsigned.apk" classes.dex)
"$TOOLS/zipalign" -f 4 "$OUT/probe-unsigned.apk" "$OUT/probe-aligned.apk"
JAVA_HOME="$JDK" PATH="$JDK/bin:$PATH" "$TOOLS/apksigner" sign --ks /home/lab6/.android/debug.keystore --ks-pass pass:android --key-pass pass:android --out "$OUT/probe.apk" "$OUT/probe-aligned.apk"
echo "$OUT/probe.apk"
