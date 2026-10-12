#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SDK="${WARP_ANDROID_SDK:-$HOME/.cache/warp-android}"
TOOLS="$SDK/build-tools/34.0.0"
ANDROID_JAR="$SDK/platforms/android-34/android.jar"
SRC="$ROOT/pipeline/01_dataset/quest_rgb_android"
BUILD="$ROOT/.venv/android-build"
for tool in javac java keytool; do command -v "$tool" >/dev/null || { echo "Missing $tool" >&2; exit 3; }; done
[[ -f "$ANDROID_JAR" && -x "$TOOLS/aapt2" && -x "$TOOLS/d8" ]] || { echo "Android API 34 + build-tools 34.0.0 required at $SDK (or WARP_ANDROID_SDK)" >&2; exit 3; }
mkdir -p "$BUILD/classes" "$BUILD/dex"
javac -source 8 -target 8 -classpath "$ANDROID_JAR" -d "$BUILD/classes" $(find "$SRC/src" -name '*.java')
"$TOOLS/d8" --lib "$ANDROID_JAR" --min-api 29 --output "$BUILD/dex" $(find "$BUILD/classes" -name '*.class')
"$TOOLS/aapt2" link -I "$ANDROID_JAR" --manifest "$SRC/AndroidManifest.xml" -o "$BUILD/unsigned.apk"
(cd "$BUILD/dex" && zip -q -u "$BUILD/unsigned.apk" classes.dex)
"$TOOLS/zipalign" -f 4 "$BUILD/unsigned.apk" "$BUILD/aligned.apk"
if [[ ! -f "$BUILD/debug.keystore" ]]; then
  keytool -genkeypair -keystore "$BUILD/debug.keystore" -storepass android -keypass android -alias androiddebugkey -keyalg RSA -keysize 2048 -validity 10000 -dname 'CN=Android Debug,O=Android,C=US' >/dev/null 2>&1
fi
"$TOOLS/apksigner" sign --ks "$BUILD/debug.keystore" --ks-pass pass:android --key-pass pass:android --out "$BUILD/warp-quest-rgb.apk" "$BUILD/aligned.apk"
"$TOOLS/apksigner" verify "$BUILD/warp-quest-rgb.apk"
echo "$BUILD/warp-quest-rgb.apk"
