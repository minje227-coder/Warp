# Unity APK build on lab6

XRT-Client Warp의 빌드 작업은 **lab6의 main**에서 한다. Mac은 Quest USB 연결을 전달하는 용도로만 사용한다.
최초 작성: 2026-10-12. APK 빌드/설치, 실제 볼륨 업 start/stop, Saved, 71 shared frames와 tag ID 2 검출을 확인했다. 첫 영상의 상하 반전을 발견해 행 순서 수정 APK를 재빌드 중이다. 수정 후 영상 방향을 재확인한다.

## 이미 설치된 환경: 다시 다운로드하지 말 것

| 항목 | lab6 경로 / 버전 |
|---|---|
| 소스 | /home/lab6/Warp, main |
| 기존 빌드 캐시 | /home/lab6/Warp-unified-build/.venv/unity-xrt-unified |
| Unity Editor | /home/lab6/.cache/warp-unity/6000.3.11f1/editor/Editor/Unity |
| Unity CLI | /home/lab6/.cache/warp-unity/bin/unity, 1.0.0-beta.13 |
| Android support | /home/lab6/.cache/warp-unity/6000.3.11f1/AndroidPlayer |
| Java / NDK | OpenJDK 17.0.9+9 / Android NDK r27c |
| SDK | platforms 34/35/36, build-tools 36.0.0, platform-tools 36 |
| Gradle / Android plugin | Unity bundled 8.13 / 8.10.0 |
| APK output | /home/lab6/Warp-unified-build/.venv/android-build/xrt-unified.apk |
| Quest | serial 2G97C5ZH60035R |
| app | com.GTSafeRoboticsLab.xrteleopclient.warp |
| activity | dev.warp.capture.WarpVolumeActivity |

Editor/AndroidPlayer/SDK/Library/Bee 캐시는 유지한다. 매번 새 프로젝트/캐시로 시작하면 이미지 import, shader와 IL2CPP 전체 빌드를 반복한다.
오래된 APK가 남아 있을 수 있으므로 파일 존재만으로 이번 빌드 성공을 판단하지 않는다.

## 재빌드 전 확인

```bash
cd /home/lab6/Warp
git status --short
git pull --ff-only origin main
free -h
/home/lab6/.cache/warp-unity/bin/unity auth status --non-interactive --json
/home/lab6/.cache/warp-unity/6000.3.11f1/AndroidPlayer/OpenJDK/bin/java -version
```

lab6은 RAM 16 GiB이고 다른 프로세스도 사용한다. 먼저 available 메모리를 확인한다.
2026-10-12에는 swap 2 GiB가 이미 가득 차 있었고, 네이티브 컴파일 중 OOM으로 Unity가 종료되었다.
**BEE_BUILD_THREADS=2**를 빌드 프로세스에 설정한다. available 메모리가 부족하면 2로 더 줄인다.
`-job-worker-count 2`만으로 Bee의 C++ 동시 컴파일 수가 제한되지는 않는다.
다른 사람의 Python/학습 프로세스를 종료하지 않는다.

Unity CLI의 로그인과 Personal 라이선스는 이 lab6 계정에서 활성화했다.
토큰이 만료되면 CLI auth login 후 출력된 브라우저 URL에서 승인한다.
로그/ps 전체 명령행에는 access token이 들어갈 수 있으므로 공개하거나 Git에 올리지 않는다.
Mac 로그인 상태가 lab6 로그인 상태를 대신하지 않는다.

## 현재 준비된 프로젝트로 빌드

prepared Assets가 현재 main과 일치할 때 사용하는 명령이다. 변경된 C#/Java/patch가 있다면 다음 절차로 갱신한 뒤 빌드한다.

```bash
env BEE_BUILD_THREADS=2 \
  WARP_APK_OUTPUT=/home/lab6/Warp-unified-build/.venv/android-build/xrt-unified.apk \
  WARP_APPLICATION_ID=com.GTSafeRoboticsLab.xrteleopclient.warp \
  /home/lab6/.cache/warp-unity/bin/unity build \
  /home/lab6/Warp-unified-build/.venv/unity-xrt-unified \
  --target Android \
  --editor-path /home/lab6/.cache/warp-unity/6000.3.11f1/editor/Editor/Unity \
  --execute-method WarpBuild.Android \
  --args "-job-worker-count 2" \
  --log-file /home/lab6/Warp-unified-build/.venv/unity-build-next.log \
  --no-tail --non-interactive --json \
  > /home/lab6/Warp-unified-build/.venv/unity-build-cli-next.log 2>&1
```

동시에 두 Unity 프로세스로 같은 prepared 프로젝트를 열지 않는다.
Unity 성공 exit code, BuildReport Succeeded, APK 변경 시각, APK 서명/패키지, ADB install Success를 각각 확인한다.
빌드 시작 전 APK SHA-256을 기록하면 오래된 APK를 새 결과로 착각하지 않는다.

## main 소스가 바뀐 경우: 캐시를 보존하면서 갱신

1. 해당 프로젝트의 Unity 빌드가 끝난 상태에서 수행한다.
2. main에서 pinned submodule을 준비하고 Git LFS 실제 파일을 받는다.
3. 새 staging 경로에 prepare-only로 소스를 생성한다.
4. Library/Logs/Temp/Obj/UserSettings와 이미 생성된 meta 파일은 제외하고 prepared 프로젝트에 동기화한다.
5. Linux SDK package workaround를 확인한 뒤 위 빌드를 실행한다.

```bash
cd /home/lab6/Warp
bash scripts/bootstrap.sh capture
export PATH=/home/lab6/.cache/warp-unity/tools/git-lfs-3.8.0:$PATH
git -C third_party/capture/xrt-client lfs install --local
git -C third_party/capture/xrt-client lfs pull

# 기존 디렉터리를 덮어쓰지 않도록 매번 비어 있는 staging 이름 사용
STAGING=/home/lab6/Warp/.venv/unity-staging-$(date +%Y%m%d-%H%M%S)
python3 scripts/01_capture/build_unified_quest.py \
  --prepare-only --project "$STAGING" --server-ip 192.168.0.32
rsync -a --delete \
  --exclude=Library --exclude=Logs --exclude=Temp --exclude=Obj \
  --exclude=UserSettings --exclude=.git --exclude='*.meta' \
  "$STAGING/" /home/lab6/Warp-unified-build/.venv/unity-xrt-unified/
python3 scripts/01_capture/build_unified_quest.py --repair-linux-packages \
  --project /home/lab6/Warp-unified-build/.venv/unity-xrt-unified
```

기존 project를 새 prepare 대상으로 주면 안전상 실패한다. staging/SDK/Library 캐시는 Git에 올리지 않는다.
서버 IP가 바뀌면 staging 생성 시 실제 LAN IP를 입력한다.

## 이번에 발생한 문제와 해결

| 증상 | 원인 | 해결 / 다음 확인 |
|---|---|---|
| PNG/모델 파일을 읽지 못함 | git archive가 Git LFS pointer를 그대로 복사함 | git lfs pull 후 prepare. helper가 각 파일의 size/SHA-256을 검증하고 실제 binary를 복사함. 이번에 196개, 약 166 MB 복구 |
| 선택적 MCP Editor의 NuGet 타입 compile error | 앱과 무관한 Editor package에 외부 DLL 설치 필요 | prepare 단계에서 com.ivanmurzak.unity.mcp만 제거 |
| Meta SDK Installer.cs의 downloadedInstallerPath 미정의 | v85 simulator installer에 Linux 분기가 없음 | --repair-linux-packages 적용. Linux simulator 설치를 명시적으로 실패시키고 Android runtime은 그대로 유지 |
| Unity native crash / import worker 문제 | 메모리 부족 및 import worker 설정 | desired/standby import worker 모두 1. standby=0이나 ForceToDesiredWorkerCount 호출은 사용하지 않음 |
| Can not sign / provide passwords | upstream의 private release keystore 설정 상속 | 별도 .warp package에 Unity local debug key 사용. WarpBuild에서 useCustomKeystore=false |
| Could not determine SDK root | cmdline-tools/latest가 SDK 바깥으로 향하는 symlink | SDK/cmdline-tools/latest 안에 실제 디렉터리 복사. sdkmanager가 실제 부모 경로에서 SDK root를 탐색함 |
| SSH 응답이 느려지고 빌드 중단 | 네이티브 컴파일에서 메모리 고갈, kernel OOM killer가 Unity 종료 | BEE_BUILD_THREADS=2, 필요하면 2. free -h와 kernel log 확인. 기존 Bee cache 재사용 |
| AprilTag detection not completed | native TagResult struct의 Serializable 속성 누락으로 Unity JSON에서 tags 필드가 빠짐 | [Serializable] 추가. WarpBuild가 Unity JsonUtility로 빈 결과와 실제 결과의 round trip을 검사한 뒤 APK 빌드 |
| 실제 볼륨 버튼 명령이 녹화 요청으로 처리되지 않음 | Unity Send(byte[])는 binary message, 기존 Python은 text만 비교 | receiver가 정확한 START/STOP binary command도 처리. 실제 WebRTC binary control 테스트 통과 |
| lab6 git push가 403 | 현재 Git CLI credential은 다른 계정 oxeunn | 그 계정의 전역 설정을 바꾸지 않음. 소유자 minje227-coder의 연결된 GitHub 도구로 main에 게시 |

오류 로그 확인:

```bash
journalctl -k --since "30 minutes ago" --no-pager | \
  grep -E 'Out of memory|Killed process|oom-kill'
# 실행 중인 Unity 명령행 전체 출력은 피할 것: 인증 토큰 노출 가능
ps -eo pid,comm,rss --sort=-rss | head
```

## 다운로드/설치 기록

lab6 Editor archive: Unity-6000.3.11f1.tar.xz, 4,458,039,720 bytes,
MD5(base64) Eb19GJTSaJMe2ExISQJMOA==.
Mac Editor pkg도 다운로드 완료했으나 현재 빌드에는 사용하지 않는다.

Android support archive: UnitySetup-Android-Support-for-Editor-6000.3.11f1.pkg,
2,093,675,367 bytes, MD5(base64) L33Lg+gzOHGj7TMgpC8ZSQ==.
공식 파일의 AndroidPlayer payload를 추출해 lab6에 설치했고, Linux용 JDK/NDK/SDK를 별도로 배치했다.
Linux용으로 추측한 Android support tar.xz URL은 404였다.
캐시의 다운로드 manifest에는 실제 공식 URL/해시가 있다: unity-linux-dependencies.json.
다운로드 중 같은 캐시에 install --dry-run --resume을 실행하지 않는다. 당시 CLI가 실제 다운로드를 수행해 파일을 손상시켰다.
해시 검증을 마친 파일을 재사용하며, 해시 불일치 파일을 그대로 설치하지 않는다.

## Quest 설치: lab6에서 실행

Mac USB ADB server를 SSH reverse forward로 lab6 localhost:5038에 연결한 경우:

```bash
ADB=/home/lab6/.cache/warp-unity/6000.3.11f1/AndroidPlayer/SDK/platform-tools/adb
"$ADB" -H 127.0.0.1 -P 5038 devices -l
"$ADB" -H 127.0.0.1 -P 5038 -s 2G97C5ZH60035R install -r \
  /home/lab6/Warp-unified-build/.venv/android-build/xrt-unified.apk
```

USB가 lab6에 직접 연결돼 있으면 -H/-P를 생략한다. 새 컴퓨터의 USB 디버깅은 headset 승인 필요.
원래 release와 .warp app은 서명이 다르므로 원래 앱을 삭제해서 해결하지 않는다.
이전 RGB companion은 종료하고 camera permission snapshot을 복원한다. 카메라를 두 앱에서 동시에 사용하지 않는다.
.warp 앱의 HEADSET_CAMERA와 BODY_TRACKING 권한을 확인한 뒤 activity를 실행한다.

## 실제 완료 판정

```bash
cd /home/lab6/Warp
bash scripts/01_capture/record_body.sh --session take_0001 --wait-seconds 600
# Quest: volume up으로 시작, 다시 volume up으로 종료
bash scripts/01_capture/validate_body.sh ~/Warp_data/YYYY-MM-DD/1 --strict-sew
```

하나의 앱에서 RGB + Skeleton + AprilTag를 동일 frame_id/camera timestamp로 저장한다.
Ready → Recording + 경과시간 → Saving → Saved, 실제 JPEG/MP4 decode와 shared timestamp 검사를 확인한다.
Skeleton은 원본 provider-query sample 사이의 보간이며 센서 하드웨어의 동시 노출을 주장하지 않는다.
실제 AprilTag 검출/metric pose 확인은 인쇄된 tag와 실제 크기가 필요하다.
Software 테스트 성공, APK 빌드 성공, 설치 성공, headset 실물 녹화 성공을 구분해서 기록한다.

태그 기본값은 한 변 **0.04 m (40 × 40 mm)**이다. lab6 receiver가 Ready 전에 앱에 전송하고 metric pose 계산에 사용한다. 다른 인쇄 크기는 --tag-size-m으로 지정한다.

추가 메모리 제한: WarpBuild는 IL2CPP conversion에도 --jobs=2를 전달하고, Android module의 Gradle JVM heap을 2048 MB로 설정한다. Bee 동시 작업 제한만으로 IL2CPP conversion의 내부 병렬성이 제한되지는 않는다. 중단된 빌드의 Gradle/VBCSCompiler 프로세스가 남아 있는지도 PID와 빌드 경로로 확인한다. 다른 작업의 프로세스는 종료하지 않는다.

저장 경로: lab6의 ~/Warp_data/YYYY-MM-DD/N. N은 1부터 시작하며 atomic mkdir로 중복을 피한다. --session은 metadata label이고, 같은 label을 반복해도 기존 take를 덮어쓰지 않는다. 날짜는 lab6 로컬 날짜이며 서버 실행 시 폴더를 예약한다. 기존 첫 영상은 이 경로로 이동했고, 원본 상하 반전 이슈를 metadata에 기록했다. 이후 upright clip으로 실물 확인한다.


30 Hz update: receiver default --rgb-fps 30; direct CFR MP4 without JPGs.
Camera extractor schedules by camera timestamp phase. Shared output time_elapsed
is frame_index/30 and source camera timestamps remain unchanged. Missing slots
hold the previous complete bundle with repeated=true for all modalities.
Unique capture FPS must be measured from source_frames, not output frames.
