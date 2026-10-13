#!/usr/bin/env python3
"""Build a patched copy of the pinned XRT app; never mutate the vendor checkout."""
import argparse
import hashlib
import ipaddress
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PIN = 'a82247aaf1100cbdd045034fe34c7d56c106412c'

def hydrate_assets(destination, vendor):
    """git archive emits LFS pointers; copy only verified hydrated asset bytes."""
    count = 0
    for path in (destination / 'Assets').rglob('*'):
        if not path.is_file() or path.stat().st_size > 512:
            continue
        pointer = path.read_bytes()
        if not pointer.startswith(b'version https://git-lfs.github.com/spec/v1\n'):
            continue
        oid = re.search(rb'^oid sha256:([0-9a-f]{64})$', pointer, re.MULTILINE)
        size = re.search(rb'^size ([0-9]+)$', pointer, re.MULTILINE)
        if not oid or not size:
            raise RuntimeError(f'Invalid LFS pointer: {path}')
        original = vendor / path.relative_to(destination)
        if not original.is_file() or original.stat().st_size != int(size[1]):
            raise RuntimeError(f'Run git lfs pull in {vendor}: missing asset {original}')
        data = original.read_bytes()
        if hashlib.sha256(data).hexdigest() != oid[1].decode():
            raise RuntimeError(f'LFS asset checksum mismatch: {original}')
        path.write_bytes(data)
        count += 1
    return count

def configure_import_workers(destination):
    path = destination / 'UserSettings/EditorUserSettings.asset'
    path.parent.mkdir(exist_ok=True)
    text = path.read_text() if path.exists() else (
        '%YAML 1.1\n%TAG !u! tag:unity3d.com,2011:\n'
        '--- !u!162 &1\nEditorUserSettings:\n  m_ObjectHideFlags: 0\n')
    for key in ('m_DesiredImportWorkerCount', 'm_StandbyImportWorkerCount'):
        pattern = rf'^  {key}:.*$'
        replacement = f'  {key}: 1'
        text = re.sub(pattern, replacement, text, flags=re.MULTILINE) if re.search(pattern, text, re.MULTILINE) else text + replacement + '\n'
    path.write_text(text)

def prepare(destination, server_ip=None):
    vendor = ROOT / 'third_party/capture/xrt-client'
    if subprocess.check_output(['git', '-C', str(vendor), 'rev-parse', 'HEAD'], text=True).strip() != PIN:
        raise RuntimeError('XRT vendor revision differs from the reviewed patch base')
    destination.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryFile() as archive:
        subprocess.run(['git', '-C', str(vendor), 'archive', PIN], stdout=archive, check=True)
        archive.seek(0)
        with tarfile.open(fileobj=archive) as source:
            source.extractall(destination)
    hydrate_assets(destination, vendor)
    configure_import_workers(destination)
    source = ROOT / 'pipeline/01_dataset/unity'
    manifest = destination / 'Packages/manifest.json'
    packages = json.loads(manifest.read_text())
    # This optional Editor agent package requires out-of-band NuGet DLL setup.
    # No app script uses it; it is not needed to compile the capture player.
    packages['dependencies'].pop('com.ivanmurzak.unity.mcp', None)
    manifest.write_text(json.dumps(packages, indent=2) + '\n')
    subprocess.run(['git', 'apply', '--unsafe-paths', '--directory=' + str(destination),
                    str(source / 'patches/xrt-unified.patch')], cwd=ROOT, check=True)
    for name in ('QuestCameraExtractor.cs', 'WarpCaptureBridge.cs', 'WarpRecordingStatus.cs'):
        shutil.copy2(source / name, destination / 'Assets/Scripts' / name)
    shutil.copy2(source / 'WarpBuild.cs', destination / 'Assets/Editor/WarpBuild.cs')
    # Unity imports Java source into unityLibrary, whose classpath includes UnityPlayer.
    java = destination / 'Assets/WarpAndroid'
    java.mkdir()
    shutil.copy2(source / 'WarpVolumeActivity.java', java / 'WarpVolumeActivity.java')
    shutil.copy2(source / 'WarpRawRecorder.java', java / 'WarpRawRecorder.java')
    scene = destination / 'Assets/Scenes/MainScene.unity'
    text = scene.read_text()
    if text.count('RequestedResolution: {x: 1280, y: 960}') != 1:
        raise RuntimeError('Expected one reviewed PCA resolution setting')
    scene.write_text(text.replace('RequestedResolution: {x: 1280, y: 960}', 'RequestedResolution: {x: 1280, y: 1280}'))
    manifest = destination / 'Assets/Plugins/Android/AndroidManifest.xml'
    manifest.write_text(manifest.read_text().replace('com.unity3d.player.UnityPlayerGameActivity', 'dev.warp.capture.WarpVolumeActivity'))
    text = manifest.read_text()
    if 'android:usesCleartextTraffic=' not in text:
        text = text.replace('<application ', '<application android:usesCleartextTraffic="true" ', 1)
    if 'android:networkSecurityConfig=' in text:
        text = re.sub(r'android:networkSecurityConfig="[^"]*"', 'android:networkSecurityConfig="@xml/warp_lan_security"', text)
    else:
        text = text.replace('<application ', '<application android:networkSecurityConfig="@xml/warp_lan_security" ', 1)
    manifest.write_text(text)
    if server_ip:
        ipaddress.IPv4Address(server_ip)
        controller = destination / 'Assets/Scripts/WebRTCController.cs'
        controller.write_text(controller.read_text().replace('http://localhost:8080/offer', f'http://{server_ip}:8080/offer'))
        scene = destination / 'Assets/Scenes/MainScene.unity'
        text = scene.read_text().replace('autoStartConnection: 0','autoStartConnection: 1')
        text = re.sub(r'^  serverUrl:.*$', f'  serverUrl: http://{server_ip}:8080/offer', text, flags=re.MULTILINE)
        scene.write_text(text)
    return destination

def repair_linux_packages(project):
    """SDK v85's simulator installer lacks a Linux variable/unsupported branch."""
    files = list((project / 'Library/PackageCache').glob('com.meta.xr.sdk.core@*/Editor/MetaXRSimulator/Installer.cs'))
    if len(files) != 1:
        raise RuntimeError('Expected one resolved Meta SDK core package')
    path = files[0]
    text = path.read_text()
    marker = 'Warp: Meta XR Simulator installer is unavailable on Linux'
    if marker in text: return
    before = '#endif\n            if (!await DownloadInstaller(downloadedInstallerPath'
    after = ('#else\n            string downloadedInstallerPath = null;\n'
             '            onError?.Invoke("' + marker + '");\n'
             '            return false;\n' + before)
    if text.count(before) != 1:
        raise RuntimeError('Meta SDK installer differs from reviewed v85 workaround')
    path.write_text(text.replace(before, after))

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path, default=ROOT / '.venv/unity-xrt-unified')
    p.add_argument('--editor', type=Path, default=os.environ.get('UNITY_EDITOR'))
    p.add_argument('--prepare-only', action='store_true')
    p.add_argument('--server-ip', help='Preset the lab PC IPv4 address and connect automatically')
    p.add_argument('--application-id', default='com.GTSafeRoboticsLab.xrteleopclient')
    p.add_argument('--repair-linux-packages', action='store_true', help='Apply v85 Editor-only fix after package resolution')
    p.add_argument('--output', type=Path, default=ROOT / '.venv/android-build/xrt-unified.apk')
    args = p.parse_args()
    if args.repair_linux_packages:
        repair_linux_packages(args.project.resolve())
        return 0
    project = prepare(args.project.resolve(),args.server_ip)
    print('Prepared unified XRT project:', project, flush=True)
    if args.prepare_only: return 0
    if not args.editor or not args.editor.is_file():
        p.exit(3, 'Unity 6000.3.11f1 Android Editor with an activated license is required. Set UNITY_EDITOR.\n')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, WARP_APK_OUTPUT=str(output), WARP_APPLICATION_ID=args.application_id)
    return subprocess.run([str(args.editor), '-batchmode', '-nographics', '-quit',
        '-projectPath', str(project), '-buildTarget', 'Android', '-executeMethod',
        'WarpBuild.Android', '-logFile', str(project / 'build.log')], env=env).returncode

if __name__ == '__main__': raise SystemExit(main())
