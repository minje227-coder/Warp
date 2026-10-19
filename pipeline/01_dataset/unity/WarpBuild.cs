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
        PlayerSettings.bundleVersion = "3.0.12-raw";
        PlayerSettings.Android.bundleVersionCode = 30012;
        var uploadPlugin = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpUploadArchive.java") as PluginImporter;
        if (uploadPlugin == null) throw new Exception("Upload stream plugin missing");
        uploadPlugin.SetCompatibleWithAnyPlatform(false); uploadPlugin.SetCompatibleWithEditor(false);
        uploadPlugin.SetCompatibleWithPlatform(BuildTarget.Android, true); uploadPlugin.SaveAndReimport();
        foreach (string pluginPath in new[] { "Assets/WarpAndroid/WarpZstdOutputStream.java", "Assets/Plugins/Android/zstd-jni-1.5.7-6.aar" }) {
            var plugin = AssetImporter.GetAtPath(pluginPath) as PluginImporter;
            if (plugin == null) throw new Exception("Zstd upload plugin missing: " + pluginPath);
            plugin.SetCompatibleWithAnyPlatform(false); plugin.SetCompatibleWithEditor(false);
            plugin.SetCompatibleWithPlatform(BuildTarget.Android, true); plugin.SaveAndReimport();
        }
        var recorderPlugin = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpRawRecorder.java") as PluginImporter;
        if (recorderPlugin == null) throw new Exception("Native raw recorder not imported");
        recorderPlugin.SetCompatibleWithAnyPlatform(false); recorderPlugin.SetCompatibleWithEditor(false);
        recorderPlugin.SetCompatibleWithPlatform(BuildTarget.Android, true); recorderPlugin.SaveAndReimport();
        var usbJava = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpUsbCamera.java") as PluginImporter;
        if (usbJava == null) throw new Exception("USB camera Java plugin missing");
        usbJava.SetCompatibleWithAnyPlatform(false); usbJava.SetCompatibleWithEditor(false);
        usbJava.SetCompatibleWithPlatform(BuildTarget.Android, true); usbJava.SaveAndReimport();
        var usbPreview = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpUsbPreview.java") as PluginImporter;
        if (usbPreview == null) throw new Exception("USB preview Java plugin missing");
        usbPreview.SetCompatibleWithAnyPlatform(false); usbPreview.SetCompatibleWithEditor(false);
        usbPreview.SetCompatibleWithPlatform(BuildTarget.Android, true); usbPreview.SaveAndReimport();
        Directory.CreateDirectory("Assets/Resources");
        File.Copy("Assets/Bhaptics/SDK2/Fonts/SpoqaHanSansNeo_TTF_subset/SpoqaHanSansNeo-Regular.ttf", "Assets/Resources/WarpKorean.ttf", true);
        AssetDatabase.ImportAsset("Assets/Resources/WarpKorean.ttf", ImportAssetOptions.ForceUpdate);
        string koreanLicense = "Assets/Bhaptics/SDK2/Fonts/SpoqaHanSansNeo_TTF_subset/LICENSE";
        if (File.Exists(koreanLicense)) {
            File.Copy(koreanLicense, "Assets/Resources/WarpKorean-LICENSE.txt", true);
            AssetDatabase.ImportAsset("Assets/Resources/WarpKorean-LICENSE.txt", ImportAssetOptions.ForceUpdate);
        }
        var usbNative = AssetImporter.GetAtPath("Assets/Plugins/Android/arm64-v8a/libwarp_usb_iso.so") as PluginImporter;
        if (usbNative == null) throw new Exception("USB camera native plugin missing");
        usbNative.SetCompatibleWithAnyPlatform(false); usbNative.SetCompatibleWithEditor(false);
        usbNative.SetCompatibleWithPlatform(BuildTarget.Android, true); usbNative.SetPlatformData(BuildTarget.Android, "CPU", "ARM64"); usbNative.SaveAndReimport();
        var java = AssetImporter.GetAtPath("Assets/WarpAndroid/WarpVolumeActivity.java") as PluginImporter;
        if (java == null) throw new Exception("Volume-up Java source plugin was not imported");
        java.SetCompatibleWithAnyPlatform(false);
        java.SetCompatibleWithEditor(false);
        java.SetCompatibleWithPlatform(BuildTarget.Android, true);
        java.SaveAndReimport();
        PlayerSettings.SetAdditionalIl2CppArgs("--jobs=1");
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
        foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies()) {
            if (assembly.GetName().Name != "UnityEditor.Android.Extensions") continue;
            foreach (var type in assembly.GetTypes()) {
                var heap = type.GetProperty("maxJvmHeapSize", System.Reflection.BindingFlags.Static |
                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
                if (heap == null || !heap.CanWrite) continue;
                heap.SetValue(null, Convert.ChangeType(1024, heap.PropertyType));
                Debug.Log("[Warp] Gradle heap limit: 1024 MB");
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
        foreach (string permission in new[] { "android.permission.CAMERA", "horizonos.permission.USB_CAMERA" })
            if (!text.Contains("android:name=\"" + permission + "\""))
                text = text.Replace("<application", "<uses-permission android:name=\"" + permission + "\" />\n<application");
        if (!text.Contains("android.hardware.usb.host"))
            text = text.Replace("<application", "<uses-feature android:name=\"android.hardware.usb.host\" android:required=\"false\" />\n<application");
        File.WriteAllText(manifest,text);
        Debug.Log("[Warp] LAN HTTP network security config applied");
    }
}
#endif
