#!/usr/bin/env python3
"""Configure the separate physical Camera2 sender over an already-authorized ADB connection."""
import argparse
import ipaddress
import json
from pathlib import Path
import re
import secrets
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--adb", default=shutil.which("adb"))
p.add_argument("--serial", required=True)
p.add_argument("--receiver-host", required=True, help="lab receiver LAN IP permitted to read frames")
p.add_argument("--apk", type=Path)
p.add_argument("--eye", choices=("left", "right"), default="right")
p.add_argument("--config", type=Path, default=root / "configs/quest_rgb.json")
p.add_argument("--restore-xrt-camera", action="store_true", help="Stop the RGB helper and restore XRT camera permissions saved by this setup")
args = p.parse_args()
if not args.adb:
    p.error("adb not found; pass --adb /path/to/adb")
ipaddress.ip_address(args.receiver_host)
adb = [args.adb, "-s", args.serial]
def run(*command):
    return subprocess.check_output(adb + list(command), text=True).strip()
if run("get-state") != "device":
    raise SystemExit("Quest USB debugging is not authorized")
model = run("shell", "getprop", "ro.product.model")
if "Quest 3" not in model:
    raise SystemExit("This sender is only qualified for Quest 3/3S")
match = re.search(r"\binet (\d+\.\d+\.\d+\.\d+)/", run("shell", "ip", "-4", "addr", "show", "wlan0"))
if not match:
    raise SystemExit("Quest has no Wi-Fi IPv4 address")
quest_ip = match.group(1)
token = secrets.token_hex(32)
previous = json.loads(args.config.read_text()) if args.config.exists() else {}
xrt_package = "com.GTSafeRoboticsLab.xrteleopclient"
camera_permissions = ("android.permission.CAMERA", "horizonos.permission.HEADSET_CAMERA")
snapshot = previous.get("xrt_camera_permissions")
if args.restore_xrt_camera:
    if not snapshot or previous.get("device_serial") != args.serial:
        raise SystemExit("No matching saved XRT camera permission state")
    run("shell", "am", "force-stop", "dev.warp.quest.rgb")
    for permission, state in snapshot.items():
        if not state["user_fixed"]:
            run("shell", "pm", "clear-permission-flags", xrt_package, permission, "user-fixed")
        run("shell", "pm", "grant" if state["granted"] else "revoke", xrt_package, permission)
    print("Restored saved XRT camera permissions; RGB companion stopped")
    raise SystemExit(0)
if not snapshot or previous.get("device_serial") != args.serial:
    package_info = run("shell", "dumpsys", "package", xrt_package)
    snapshot = {}
    for permission in camera_permissions:
        match = re.search(re.escape(permission) + r": granted=(true|false), flags=\[([^]]*)\]", package_info)
        if match:
            snapshot[permission] = {"granted": match[1] == "true", "user_fixed": "USER_FIXED" in match[2]}
if not snapshot:
    raise SystemExit("Cannot save XRT camera permission state before avoiding camera contention")
config = {"url": f"http://{quest_ip}:18091", "token": token, "eye": args.eye,
          "device_serial": args.serial, "xrt_camera_permissions": snapshot}
# Persist the rollback state before touching either app's camera permissions.
args.config.parent.mkdir(parents=True, exist_ok=True)
args.config.write_text(json.dumps(config, indent=2) + "\n")
args.config.chmod(0o600)
if args.apk:
    print(run("install", "-r", str(args.apk)))
run("shell", "pm", "grant", "dev.warp.quest.rgb", "android.permission.CAMERA")
run("shell", "pm", "grant", "dev.warp.quest.rgb", "horizonos.permission.HEADSET_CAMERA")
# Quest evicts a background camera client when XRT reopens its AprilTag camera.
# Body/hand permissions are separate. Save camera grants and deny only this unused
# XRT camera path; never bypass camera ownership, and provide an explicit restore.
for permission in snapshot:
    run("shell", "pm", "revoke", xrt_package, permission)
    run("shell", "pm", "set-permission-flags", xrt_package, permission, "user-fixed")
# Only this capture helper is restarted; preserve the XRT client's state and recorded data.
run("shell", "am", "force-stop", "dev.warp.quest.rgb")
run("shell", "am", "start", "-n", "dev.warp.quest.rgb/.MainActivity", "--es", "token", token,
    "--es", "client", args.receiver_host, "--es", "eye", args.eye)
print(f"Configured {model} physical {args.eye} RGB camera; receiver {args.receiver_host}")
print(f"Private config: {args.config} (copy to the receiver's configs/quest_rgb.json)")
print("XRT AprilTag camera disabled to avoid Quest camera eviction; body/hand permissions preserved.")
run("shell", "am", "start", "-n", xrt_package + "/com.unity3d.player.UnityPlayerGameActivity")
