#!/usr/bin/env python3
"""Build a patched copy of the pinned XRT app; never mutate the vendor checkout."""
import argparse
import ipaddress
import os
import re
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PIN = 'a82247aaf1100cbdd045034fe34c7d56c106412c'

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
    source = ROOT / 'pipeline/01_dataset/unity'
    subprocess.run(['git', 'apply', '--unsafe-paths', '--directory=' + str(destination),
                    str(source / 'patches/xrt-unified.patch')], cwd=ROOT, check=True)
    for name in ('QuestCameraExtractor.cs', 'WarpCaptureBridge.cs', 'WarpRecordingStatus.cs'):
        shutil.copy2(source / name, destination / 'Assets/Scripts' / name)
    shutil.copy2(source / 'WarpBuild.cs', destination / 'Assets/Editor/WarpBuild.cs')
    # Unity imports Java source into unityLibrary, whose classpath includes UnityPlayer.
    java = destination / 'Assets/WarpAndroid'
    java.mkdir()
    shutil.copy2(source / 'WarpVolumeActivity.java', java / 'WarpVolumeActivity.java')
    manifest = destination / 'Assets/Plugins/Android/AndroidManifest.xml'
    manifest.write_text(manifest.read_text().replace('com.unity3d.player.UnityPlayerGameActivity', 'dev.warp.capture.WarpVolumeActivity'))
    if server_ip:
        ipaddress.IPv4Address(server_ip)
        controller = destination / 'Assets/Scripts/WebRTCController.cs'
        controller.write_text(controller.read_text().replace('http://localhost:8080/offer', f'http://{server_ip}:8080/offer'))
        scene = destination / 'Assets/Scenes/MainScene.unity'
        text = scene.read_text().replace('autoStartConnection: 0','autoStartConnection: 1')
        text = re.sub(r'^  serverUrl:.*$', f'  serverUrl: http://{server_ip}:8080/offer', text, flags=re.MULTILINE)
        scene.write_text(text)
    return destination

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path, default=ROOT / '.venv/unity-xrt-unified')
    p.add_argument('--editor', type=Path, default=os.environ.get('UNITY_EDITOR'))
    p.add_argument('--prepare-only', action='store_true')
    p.add_argument('--server-ip', help='Preset the lab PC IPv4 address and connect automatically')
    p.add_argument('--application-id', default='com.GTSafeRoboticsLab.xrteleopclient')
    p.add_argument('--output', type=Path, default=ROOT / '.venv/android-build/xrt-unified.apk')
    args = p.parse_args()
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
