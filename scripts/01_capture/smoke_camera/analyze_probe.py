"""Fully decode a camera-only smoke take and compare sensor and codec clocks."""
import json
import sys
from pathlib import Path
import av
import numpy as np

episode = Path(sys.argv[1])
sensor = [json.loads(s) for s in (episode / "camera_frames.jsonl").read_text().splitlines()]
encoded = [json.loads(s) for s in (episode / "encoded_frames.jsonl").read_text().splitlines()]
container = av.open(str(episode / "camera.mp4"))
stream = container.streams.video[0]
frames = list(container.decode(stream))
pts = np.array([float(f.pts * f.time_base) for f in frames])
sensor_ts = np.array([r["sensor_timestamp_ns"] / 1e9 for r in sensor])
def timing(values):
    gaps = np.diff(values)
    return {"count": len(values), "mean_fps": float((len(values)-1)/(values[-1]-values[0])) if len(values)>1 else None,
            "gap_max_ms": float(gaps.max()*1000) if len(gaps) else None,
            "gap_p99_ms": float(np.quantile(gaps, .99)*1000) if len(gaps) else None,
            "nonpositive_gaps": int((gaps<=0).sum())}
report = {"episode": str(episode), "width": stream.width, "height": stream.height,
          "codec": stream.codec_context.name, "decoded": timing(pts), "sensor": timing(sensor_ts),
          "encoded_samples": len(encoded), "crop_regions": sorted({r.get("crop_region", "") for r in sensor}),
          "calibration": json.loads((episode / "calibration.json").read_text()),
          "tracking_samples": len((episode / "tracking.jsonl").read_text().splitlines())}
if len(sensor_ts) == len(encoded):
    delta = sensor_ts - np.array([r["pts_us"]/1e6 for r in encoded])
    report["sensor_minus_codec_ms"] = {"min": float(delta.min()*1000), "max": float(delta.max()*1000), "std": float(delta.std()*1000)}
if frames:
    try:
        frames[len(frames)//2].to_image().save(episode / "probe-frame.png")
    except ImportError:
        pass
print(json.dumps(report, indent=2))
(episode / "probe-quality.json").write_text(json.dumps(report, indent=2))
