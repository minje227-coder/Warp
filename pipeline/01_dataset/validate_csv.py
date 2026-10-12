#!/usr/bin/env python3
"""Inspect XRT_devices CSVRecorder output before using it as training data.

This verifies recorded **bones** (human motion), not robot retargeting, vision,
or calibrated world coordinates. Uses Python standard library only.
"""
from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median
from typing import Any

# XRT_devices/OpenXR FullBodyBoneId values, pinned in docs/UPSTREAMS.md.
SEW_BONES = {
    "spine_middle": 3,
    "left_upper_arm": 10,
    "left_lower_arm": 11,
    "left_wrist": 19,
    "right_upper_arm": 15,
    "right_lower_arm": 16,
    "right_wrist": 45,
}
REQUIRED_COLUMNS = (
    "time_elapsed", "data_type", "id", "pos_x", "pos_y", "pos_z",
    "rot_x", "rot_y", "rot_z", "rot_w",
)


def inspect_csv(path: Path, *, min_frames: int = 1, min_sew_coverage: float = 0.9,
                strict_sew: bool = False, max_gap: float = 0.25) -> dict[str, Any]:
    """Return a diagnostic summary; ok=False means this take should not be trusted."""
    result: dict[str, Any] = {
        "ok": False, "path": str(path), "frame_count": 0,
        "bone_rows": 0, "tag_rows": 0, "action_rows": 0,
        "invalid_rows": 0, "duration_s": 0.0, "sew_coverage": {},
        "large_gaps": 0, "median_interval_s": None, "messages": [],
    }
    if not path.is_file():
        result["messages"].append("CSV file does not exist")
        return result
    times: dict[float, set[int]] = defaultdict(set)
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames or any(k not in reader.fieldnames for k in REQUIRED_COLUMNS):
                result["messages"].append("Missing XRT CSVRecorder columns")
                return result
            for row in reader:
                try:
                    ts = float(row["time_elapsed"])
                    kind = row["data_type"]
                    if not (ts >= 0 and ts < float("inf")):
                        raise ValueError("invalid timestamp")
                    if kind == "bone":
                        bone_id = int(row["id"])
                        values = [float(row[k]) for k in REQUIRED_COLUMNS[3:]]
                        if any(v != v or abs(v) == float("inf") for v in values):
                            raise ValueError("nonfinite skeleton value")
                        times[ts].add(bone_id)
                        result["bone_rows"] += 1
                    elif kind == "apriltag":
                        result["tag_rows"] += 1
                    elif kind == "action":
                        result["action_rows"] += 1
                except (TypeError, ValueError, KeyError):
                    result["invalid_rows"] += 1
    except (OSError, UnicodeError, csv.Error) as exc:
        result["messages"].append(f"Cannot read CSV: {exc}")
        return result
    result["frame_count"] = len(times)
    ordered = sorted(times)
    if len(ordered) >= 2:
        diffs = [b - a for a, b in zip(ordered[:-1], ordered[1:])]
        result["duration_s"] = ordered[-1] - ordered[0]
        result["median_interval_s"] = round(median(diffs), 4)
        result["large_gaps"] = sum(d > max_gap for d in diffs)
    if times:
        result["sew_coverage"] = {
            name: round(sum(bone_id in b for b in times.values()) / len(times), 3)
            for name, bone_id in SEW_BONES.items()
        }
    if result["frame_count"] < min_frames:
        result["messages"].append(f"Only {result['frame_count']} skeleton frames (< {min_frames})")
    if result["invalid_rows"]:
        result["messages"].append(f"{result['invalid_rows']} invalid bone rows")
    low = [name for name, rate in result["sew_coverage"].items() if rate < min_sew_coverage]
    if low:
        result["messages"].append("Low required SEW landmark coverage: " + ", ".join(low))
    if result["large_gaps"]:
        result["messages"].append(f"{result['large_gaps']} receive-time gaps > {max_gap}s")
    # Large gaps are warnings, not proof of incorrect capture; lower-body gaps do
    # not gate SEW unless --strict-sew was requested.
    result["ok"] = (result["frame_count"] >= min_frames
                    and result["invalid_rows"] == 0
                    and (not strict_sew or not low))
    return result


def main() -> int:
    p = argparse.ArgumentParser(description="Validate XRT_devices raw Quest skeleton CSV")
    p.add_argument("csv", type=Path, help="body.csv or episode directory")
    p.add_argument("--min-frames", type=int, default=30)
    p.add_argument("--strict-sew", action="store_true", help="Require bilateral arm/wrist landmarks")
    p.add_argument("--min-sew-coverage", type=float, default=0.9)
    p.add_argument("--max-gap", type=float, default=0.25)
    args = p.parse_args()
    if args.min_frames < 1 or not 0 <= args.min_sew_coverage <= 1 or args.max_gap <= 0:
        p.error("min-frames >=1, coverage in [0,1], max-gap >0 are required")
    path = args.csv / "body.csv" if args.csv.is_dir() else args.csv
    r = inspect_csv(path, min_frames=args.min_frames,
                    min_sew_coverage=args.min_sew_coverage,
                    strict_sew=args.strict_sew, max_gap=args.max_gap)
    print(f"[{ 'PASS' if r['ok'] else 'FAIL' }] {path}")
    print(f"Skeleton frames: {r['frame_count']} | Bone rows: {r['bone_rows']} | "
          f"Duration: {r['duration_s']:.2f}s | >gap: {r['large_gaps']}")
    if r["sew_coverage"]:
        print("SEW landmark coverage: " + ", ".join(
            f"{n}={v:.1%}" for n, v in r["sew_coverage"].items()))
    for message in r["messages"]:
        print("WARNING: " + message)
    print("NOTE: PASS checks file format/coverage only; no hardware accuracy, RGB synchronization, or grasp success.")
    return 0 if r["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
