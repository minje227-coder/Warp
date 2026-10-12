"""Receive physical Quest Camera2 JPEGs; retain original sensor and receive clocks."""
from __future__ import annotations

import csv
import io
import json
from pathlib import Path
import threading
import time
import urllib.request


def read_config(path: Path) -> dict:
    try:
        config = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"Quest RGB configuration missing/invalid at {path}. Run prepare_quest_rgb.py first.") from exc
    if not config.get("url", "").startswith("http://") or len(config.get("token", "")) < 32:
        raise RuntimeError("Quest RGB config needs a local HTTP URL and receiver token")
    return config


class QuestRgbRecorder:
    def __init__(self, folder: Path, config: dict):
        self.folder, self.config = folder, config
        self._stop = threading.Event()
        self._thread = None
        self._active = False
        self.origin_ns = None
        self.error = None
        self.frames = 0
        self.latest_received_ns = None
        self.calibration = None
        self.last_sequence = None
        self.source_sequence_gaps = 0
        self._csv_file = None
        self._writer = None

    def _get(self, path: str):
        request = urllib.request.Request(self.config["url"].rstrip("/") + path,
                                         headers={"X-Warp-Token": self.config["token"]})
        # A frame request deliberately bypasses environment proxies on the local LAN.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        return opener.open(request, timeout=3)

    def check(self) -> dict:
        with self._get("/status") as response:
            status = json.load(response)
        with self._get("/calibration") as response:
            self.calibration = json.load(response)
        if status.get("source") != "quest_passthrough_camera2" or status.get("error"):
            raise RuntimeError(f"Quest RGB camera not ready: {status.get('error', 'wrong source')}")
        if status.get("latest_age_s") is None or status["latest_age_s"] > 2:
            raise RuntimeError("Quest RGB camera has no fresh physical frame")
        if self.calibration.get("source") != "quest_passthrough_camera2" or self.calibration.get("eye") not in ("left", "right"):
            raise RuntimeError("Quest RGB calibration/source provenance missing")
        if status.get("eye") != self.config.get("eye") or self.calibration["eye"] != self.config.get("eye"):
            raise RuntimeError("Quest RGB eye differs from the configured source")
        return status

    def start(self):
        self.check()
        self.folder.mkdir(parents=True, exist_ok=False)
        (self.folder / "calibration.json").write_text(json.dumps(self.calibration, indent=2) + "\n")
        self._csv_file = (self.folder / "timestamps.csv").open("w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(("file", "sequence", "sensor_timestamp_ns", "quest_elapsed_at_send_ns",
                               "encoded_elapsed_ns", "pc_request_monotonic_ns", "pc_receive_monotonic_ns",
                               "clock_offset_estimate_ns", "clock_uncertainty_ns", "capture_pc_estimate_ns",
                               "body_origin_pc_monotonic_ns", "capture_body_elapsed_s", "width", "height"))
        self._thread = threading.Thread(target=self._run, name="quest-rgb", daemon=True)
        self._thread.start()
        return self

    def activate(self, body_origin_ns: int):
        self.origin_ns = body_origin_ns
        self._active = True

    def _run(self):
        try:
            while not self._stop.is_set():
                before = time.monotonic_ns()
                with self._get("/frame") as response:
                    jpeg = response.read(16 * 1024 * 1024 + 1)
                    headers = response.headers
                    received = time.monotonic_ns()
                if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9") or len(jpeg) > 16 * 1024 * 1024:
                    raise RuntimeError("Invalid/oversized Quest RGB JPEG")
                seq = int(headers["X-Frame-Sequence"])
                sensor_ns = int(headers["X-Sensor-Timestamp-Ns"])
                quest_send_ns = int(headers["X-Quest-Elapsed-Ns"])
                encoded_ns = int(headers["X-Encoded-Elapsed-Ns"])
                if not 0 <= quest_send_ns - encoded_ns <= 2_000_000_000:
                    raise RuntimeError("Stale/future Quest RGB frame; timestamp preserved")
                self.latest_received_ns = received
                if self.last_sequence is not None and seq < self.last_sequence:
                    raise RuntimeError("Quest RGB source restarted during episode")
                if seq != self.last_sequence:
                    if self._active:
                        if self.last_sequence is not None:
                            self.source_sequence_gaps += max(0, seq - self.last_sequence - 1)
                        name = f"frame_{self.frames:06d}.jpg"
                        (self.folder / name).write_bytes(jpeg)
                        offset = (before + received) // 2 - quest_send_ns
                        uncertainty = (received - before) // 2
                        realtime = self.calibration.get("timestamp_source") == 1
                        capture_ns = sensor_ns + offset if realtime else None
                        relative = (capture_ns - self.origin_ns) / 1e9 if realtime else None
                        self._writer.writerow((name, seq, sensor_ns, quest_send_ns, encoded_ns, before,
                                               received, offset, uncertainty, capture_ns, self.origin_ns,
                                               relative, headers["X-Width"], headers["X-Height"]))
                        self.frames += 1
                    self.last_sequence = seq
                self._stop.wait(max(0, 0.04 - (time.monotonic_ns() - before) / 1e9))
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=4)
            if self._thread.is_alive():
                self.error = "RGB worker did not stop"
                return
        if self._csv_file:
            self._csv_file.close()

    def make_video(self):
        """Encode a convenience MP4 with camera interval PTS; sidecar clocks are authoritative."""
        import av
        from fractions import Fraction
        with (self.folder / "timestamps.csv").open() as timestamps:
            rows = list(csv.DictReader(timestamps))
        if not rows:
            raise RuntimeError("No RGB frames to encode")
        path = self.folder / "video.mp4"
        with av.open(str(path), "w") as output:
            stream = output.add_stream("libx264", rate=10)
            stream.width = int(rows[0]["width"])
            stream.height = int(rows[0]["height"])
            stream.pix_fmt = "yuv420p"
            stream.options = {"crf": "20", "preset": "veryfast"}
            time_base = Fraction(1, 1_000_000)
            stream.time_base = time_base
            stream.codec_context.time_base = time_base
            first = int(rows[0]["sensor_timestamp_ns"])
            previous_pts = -1
            for row in rows:
                with av.open(io.BytesIO((self.folder / row["file"]).read_bytes()), "r", format="mjpeg") as image:
                    frame = next(image.decode(video=0))
                pts = (int(row["sensor_timestamp_ns"]) - first) // 1000
                if pts <= previous_pts:
                    raise RuntimeError("RGB capture timestamps are not strictly increasing")
                previous_pts = pts
                frame.pts = pts
                frame.time_base = time_base
                for packet in stream.encode(frame):
                    output.mux(packet)
            for packet in stream.encode():
                output.mux(packet)
        with av.open(str(path)) as video:
            decoded = sum(1 for _ in video.decode(video=0))
        if decoded != len(rows):
            raise RuntimeError(f"MP4 verification mismatch: {decoded} != {len(rows)}")
        return {"file": "rgb/video.mp4", "decoded_frames": decoded}
