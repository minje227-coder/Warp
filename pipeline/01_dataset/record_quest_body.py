#!/usr/bin/env python3
"""Minimal Meta Quest upper-body recording via pinned XRT_devices XRDevice.

Works after the upstream `XRT_devices` and its XR extras have been installed.
Does NOT record egocentric RGB, camera timestamps, box pose, or WARP actions.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys
import time

from validate_csv import inspect_csv

DEFAULT_ROOT = Path(__file__).resolve().parents[2] / "local_data" / "raw_human"
SESSION_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Record XRT-Client human skeleton to an episode CSV")
    p.add_argument("--session", help="Episode ID, e.g. rack_0001 (no overwrite)")
    p.add_argument("--out-root", type=Path, default=DEFAULT_ROOT)
    p.add_argument("--host", default="0.0.0.0", help="PC interface to bind (not Quest IP)")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--wait-seconds", type=float, default=90, help="Timeout until first valid pose")
    p.add_argument("--duration", type=float, default=0,
                   help="Seconds AFTER first pose; 0 records until Ctrl+C")
    p.add_argument("--min-frames", type=int, default=30)
    p.add_argument("--check", action="store_true", help="Probe installed XRT_devices, do not record")
    args = p.parse_args(argv)
    if args.session and not SESSION_RE.fullmatch(args.session):
        p.error("--session must be 1-64 ASCII letters/digits/_/- beginning with alphanumeric")
    if not 0 < args.port < 65536:
        p.error("--port must be 1-65535")
    if args.wait_seconds <= 0 or args.duration < 0 or args.min_frames < 1:
        p.error("wait-seconds > 0, duration >= 0, min-frames >= 1 required")
    return args


def run_capture(args: argparse.Namespace) -> int:
    try:
        import xrt_devices
        from xrt_devices import XRDevice
        from xrt_devices.recording.writer import CSVRecorder
    except (ModuleNotFoundError, ImportError) as exc:
        print(f"ERROR: XRT_devices is not installed correctly: {exc}", file=sys.stderr)
        print("Run: bash scripts/bootstrap.sh capture", file=sys.stderr)
        print("Then: bash scripts/01_capture/install_receiver.sh", file=sys.stderr)
        return 3

    print(f"XRT_devices: {getattr(xrt_devices, '__version__', 'version unknown')}")
    if args.check:
        print("[OK] XRT import check only. Quest connection and CSV output NOT tested.")
        return 0

    session = args.session or datetime.now(timezone.utc).strftime("take_%Y%m%dT%H%M%SZ")
    episode = args.out_root.expanduser().resolve() / session
    try:
        episode.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"ERROR: Refusing to overwrite existing episode: {episode}", file=sys.stderr)
        return 2

    body_csv = episode / "body.csv"
    metadata_path = episode / "metadata.json"
    meta = {
        "source": "Meta Quest XRT-Client → XRT_devices CSVRecorder",
        "status": "starting",
        "episode": session,
        "start_utc": datetime.now(timezone.utc).isoformat(),
        "host": args.host,
        "port": args.port,
        "streams_captured": ["body_pose_bones"],
        "streams_not_captured": ["rgb_left", "rgb_right", "box_pose", "robot_actions"],
        "time_reference": "PC monotonic packet receive elapsed time; NOT Quest camera exposure time",
        "observed_unique_pose_sequences": 0,
        "recorded_skeleton_frames": 0,
        "recorder_dropped_frames": None,
    }
    last_seq = None
    received = 0
    dropped = None
    error = None
    interrupted = False
    first_frame_at = None
    server_start_at = time.monotonic()
    last_report = server_start_at
    print(f"Episode: {episode}", flush=True)
    print(f"Listening on {args.host}:{args.port} (Quest app must use PC LAN IP + port {args.port})", flush=True)
    print("Wait for Quest connection; after recording starts Ctrl+C stops safely.", flush=True)
    try:
        with CSVRecorder(body_csv) as recorder:
            try:
                with XRDevice(host=args.host, port=args.port, recorder=recorder) as device:
                    while True:
                        now = time.monotonic()
                        frame = device.get_frame()
                        if frame is not None and frame.sequence != last_seq:
                            last_seq = frame.sequence
                            received += 1
                            if first_frame_at is None:
                                first_frame_at = now
                                print("First tracked pose received; capture started.", flush=True)
                        if first_frame_at is None and now - server_start_at > args.wait_seconds:
                            error = "No valid pose within wait timeout (check Quest app/IP/network)"
                            break
                        if first_frame_at is not None and args.duration > 0 and now - first_frame_at >= args.duration:
                            break
                        if now - last_report >= 5:
                            print(f"Latest unique frames={received} connected={device.is_connected} "
                                  f"writer_dropped={recorder.dropped_frames}", flush=True)
                            last_report = now
                        time.sleep(0.02)
            except KeyboardInterrupt:
                interrupted = True
                print("Ctrl+C received; flushing CSV...", flush=True)
            finally:
                dropped = recorder.dropped_frames
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"

    summary = inspect_csv(body_csv, min_frames=args.min_frames, strict_sew=False)
    meta.update({
        "status": "complete" if (not error and summary["ok"] and received >= args.min_frames) else "failed",
        "end_utc": datetime.now(timezone.utc).isoformat(),
        "observed_unique_pose_sequences": received,
        "recorded_skeleton_frames": summary["frame_count"],
        "recorder_dropped_frames": dropped,
        "interrupted_by_user": interrupted,
        "validation_messages": summary["messages"],
        "error": error,
    })
    metadata_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Recorded skeleton frames: {summary['frame_count']}; observed sequences: {received}; "
          f"writer dropped: {dropped}")
    print(f"Metadata: {metadata_path}")
    print("Next: python pipeline/01_dataset/validate_csv.py " + str(episode) + " --strict-sew")
    if meta["status"] != "complete":
        print("[FAIL] Capture incomplete: " + (error or "; ".join(summary["messages"])), file=sys.stderr)
        return 2
    print("[OK] Raw human skeleton CSV recorded. NOT a validated WARP/gripper training dataset.")
    return 0


def main(argv: list[str] | None = None) -> int:
    return run_capture(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
