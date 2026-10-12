#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class WarpBuild
{
    public static void Android() {
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
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
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
