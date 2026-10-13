#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Android;
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
        PlayerSettings.insecureHttpOption = InsecureHttpOption.AlwaysAllowed;
        PlayerSettings.bundleVersion = "3.0.0-raw";
        PlayerSettings.Android.bundleVersionCode = 30000;
        var recorderPlugin = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpRawRecorder.java") as PluginImporter;
        if (recorderPlugin == null) throw new Exception("Native raw recorder not imported");
        recorderPlugin.SetCompatibleWithAnyPlatform(false); recorderPlugin.SetCompatibleWithEditor(false);
        recorderPlugin.SetCompatibleWithPlatform(BuildTarget.Android, true); recorderPlugin.SaveAndReimport();
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
// Unity generates a network-security XML which overrides usesCleartextTraffic.
// The capture receiver is an explicitly configured LAN HTTP endpoint.
public class WarpLanNetworkSecurity : IPostGenerateGradleAndroidProject {
    public int callbackOrder => 10000;
    public void OnPostGenerateGradleAndroidProject(string path) {
        string config=Path.Combine(path,"src/main/res/xml/warp_lan_security.xml");
        Directory.CreateDirectory(Path.GetDirectoryName(config));
        File.WriteAllText(config,"<?xml version=\"1.0\" encoding=\"utf-8\"?><network-security-config><base-config cleartextTrafficPermitted=\"true\" /></network-security-config>");
        string manifest=Path.Combine(path,"src/main/AndroidManifest.xml");
        string text=File.ReadAllText(manifest);
        text=System.Text.RegularExpressions.Regex.Replace(text,"android:networkSecurityConfig=\"[^\"]*\"", "android:networkSecurityConfig=\"@xml/warp_lan_security\"");
        File.WriteAllText(manifest,text);
        Debug.Log("[Warp] LAN HTTP network security config applied");
    }
}
#endif
