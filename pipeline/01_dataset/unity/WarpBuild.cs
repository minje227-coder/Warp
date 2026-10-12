#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class WarpBuild
{
    public static void Android() {
        // Exercise Unity serialization itself; Python synthetic JSON cannot catch
        // a missing [Serializable] on the native detector result type.
        var probe = new WarpCaptureBridge.Bundle { tags = new QuestAprilTagTracker.TagResult[0] };
        if (!JsonUtility.ToJson(probe).Contains("\"tags\":[]"))
            throw new Exception("Completed empty AprilTag results must serialize as tags:[]");
        probe.tags = new[] { new QuestAprilTagTracker.TagResult { id = 17,
            position = new Vector3(1, 2, 3), rotation = Quaternion.identity } };
        var roundTrip = JsonUtility.FromJson<WarpCaptureBridge.Bundle>(JsonUtility.ToJson(probe));
        if (roundTrip.tags == null || roundTrip.tags.Length != 1 || roundTrip.tags[0].id != 17)
            throw new Exception("AprilTag results failed Unity JSON round trip");
        string output = Environment.GetEnvironmentVariable("WARP_APK_OUTPUT");
        if (string.IsNullOrEmpty(output)) throw new Exception("WARP_APK_OUTPUT must be set");
        string applicationId = Environment.GetEnvironmentVariable("WARP_APPLICATION_ID") ?? "com.GTSafeRoboticsLab.xrteleopclient";
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android, applicationId);
        if (applicationId != "com.GTSafeRoboticsLab.xrteleopclient") {
            PlayerSettings.productName = "XRT-Client Warp";
            // The upstream release key is private. Sign our separate package with
            // Unity's local debug key without changing the original installation.
            PlayerSettings.Android.useCustomKeystore = false;
        }
        var java = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpVolumeActivity.java") as PluginImporter;
        if (java == null) throw new Exception("Volume-up Java source plugin was not imported");
        java.SetCompatibleWithAnyPlatform(false);
        java.SetCompatibleWithEditor(false);
        java.SetCompatibleWithPlatform(BuildTarget.Android, true);
        java.SaveAndReimport();
        PlayerSettings.SetAdditionalIl2CppArgs("--jobs=2");
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
        foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies()) {
            if (assembly.GetName().Name != "UnityEditor.Android.Extensions") continue;
            foreach (var type in assembly.GetTypes()) {
                var heap = type.GetProperty("maxJvmHeapSize", System.Reflection.BindingFlags.Static |
                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
                if (heap == null || !heap.CanWrite) continue;
                heap.SetValue(null, Convert.ChangeType(2048, heap.PropertyType));
                Debug.Log("[Warp] Gradle heap limit: 2048 MB");
            }
        }
        // MainScene is the existing PCA + full body + AprilTag scene. ZED scenes have no PCA.
        var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
            scenes = new[] { "Assets/Scenes/MainScene.unity" }, locationPathName = output,
            target = BuildTarget.Android, options = BuildOptions.None
        });
        if (report.summary.result != BuildResult.Succeeded) throw new Exception("Android APK build failed");
        Debug.Log("[Warp] Unified APK: " + Path.GetFullPath(output));
    }
}
#endif
