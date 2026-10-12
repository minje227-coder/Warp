"""Offline tests for the capture wrapper; NOT a Quest hardware or WebRTC test."""
from __future__ import annotations

import csv
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
from validate_csv import REQUIRED_COLUMNS, SEW_BONES, inspect_csv  # noqa: E402

spec = importlib.util.spec_from_file_location("record_quest_body", STAGE / "record_quest_body.py")
assert spec and spec.loader
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def make_csv(path: Path, frames: int, *, missing_left_wrist: bool = False) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(REQUIRED_COLUMNS)
        for n in range(frames):
            for body_id in SEW_BONES.values():
                if missing_left_wrist and body_id == 19:
                    continue
                writer.writerow((round(n * 0.02, 3), "bone", body_id,
                                 0.1, 0.2, 0.3, 0, 0, 0, 1))


class ValidatorTest(unittest.TestCase):
    def test_valid_skeleton_csv(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "body.csv"
            make_csv(path, 12)
            r = inspect_csv(path, min_frames=10, strict_sew=True)
            self.assertTrue(r["ok"], r["messages"])
            self.assertEqual(r["frame_count"], 12)
            self.assertEqual(r["sew_coverage"]["right_wrist"], 1.0)

    def test_missing_wrist_detected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "body.csv"
            make_csv(path, 12, missing_left_wrist=True)
            r = inspect_csv(path, min_frames=10, strict_sew=True)
            self.assertFalse(r["ok"])
            self.assertIn("left_wrist", " ".join(r["messages"]))

    def test_empty_csv_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "body.csv"
            path.write_text("time_elapsed,data_type\n", encoding="utf-8")
            self.assertFalse(inspect_csv(path)["ok"])


class RecorderMockTest(unittest.TestCase):
    def test_recording_lifecycle_without_hardware(self):
        """Fake frames validate our CLI/writer interface, NOT vendor implementation."""
        fake_xrt = types.ModuleType("xrt_devices")
        fake_xrt.__version__ = "fake-test-only"
        recording = types.ModuleType("xrt_devices.recording")
        writer_module = types.ModuleType("xrt_devices.recording.writer")

        class FakeCSVRecorder:
            def __init__(self, path):
                self.path = Path(path)
                self.dropped_frames = 0

            def __enter__(self):
                self.file = self.path.open("x", newline="")
                self.writer = csv.writer(self.file)
                self.writer.writerow(REQUIRED_COLUMNS)
                return self

            def __exit__(self, *args):
                self.file.close()

        class FakeXRDevice:
            def __init__(self, *, host, port, recorder):
                self.seq = 0
                self.recorder = recorder
                self.is_connected = True

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return None

            def get_frame(self):
                self.seq += 1
                for body_id in SEW_BONES.values():
                    self.recorder.writer.writerow((self.seq * 0.02, "bone", body_id,
                                                  0.1, 0.2, 0.3, 0, 0, 0, 1))
                return types.SimpleNamespace(sequence=self.seq)

        fake_xrt.XRDevice = FakeXRDevice
        writer_module.CSVRecorder = FakeCSVRecorder
        with tempfile.TemporaryDirectory() as temp:
            with patch.dict(sys.modules, {
                "xrt_devices": fake_xrt,
                "xrt_devices.recording": recording,
                "xrt_devices.recording.writer": writer_module,
            }):
                args = capture.parse_args(["--out-root", temp, "--session", "mock_take",
                                           "--wait-seconds", "1", "--duration", "0.2",
                                           "--min-frames", "4"])
                with patch("sys.stdout", new_callable=io.StringIO):
                    rc = capture.run_capture(args)
            self.assertEqual(rc, 0)
            episode = Path(temp) / "mock_take"
            meta = json.loads((episode / "metadata.json").read_text())
            self.assertEqual(meta["status"], "complete")
            self.assertGreaterEqual(meta["recorded_skeleton_frames"], 4)
            self.assertTrue(inspect_csv(episode / "body.csv", min_frames=4, strict_sew=True)["ok"])


if __name__ == "__main__":
    unittest.main()
