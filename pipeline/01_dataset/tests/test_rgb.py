"""Transport/clock tests with an explicitly fake JPEG; no camera accuracy claims."""
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_rgb import QuestRgbRecorder
import record_quest_body as capture


class RgbTransportTest(unittest.TestCase):
    def exercise(self, timestamp_source):
        class Handler(BaseHTTPRequestHandler):
            counter = 0
            def log_message(self, *args):
                pass
            def do_GET(self):
                self.send_response(200)
                if self.path == '/status':
                    payload = json.dumps({'source':'quest_passthrough_camera2','eye':'right','error':None,'latest_age_s':0}).encode()
                elif self.path == '/calibration':
                    payload = json.dumps({'source':'quest_passthrough_camera2','eye':'right','timestamp_source':timestamp_source}).encode()
                else:
                    Handler.counter += 1
                    now = time.monotonic_ns()
                    for name, value in {'X-Frame-Sequence':Handler.counter,'X-Sensor-Timestamp-Ns':now-10_000_000,
                                        'X-Quest-Elapsed-Ns':now,'X-Encoded-Elapsed-Ns':now,
                                        'X-Width':1280,'X-Height':960}.items():
                        self.send_header(name,str(value))
                    payload = b'\xff\xd8fake-transport-test-only\xff\xd9'
                self.send_header('Content-Length',str(len(payload))); self.end_headers(); self.wfile.write(payload)
        server = ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread = threading.Thread(target=server.serve_forever,daemon=True); thread.start()
        try:
            with tempfile.TemporaryDirectory() as temp:
                rgb = QuestRgbRecorder(Path(temp)/'rgb',{'url':f'http://127.0.0.1:{server.server_port}','token':'a'*64,'eye':'right'})
                rgb.start()
                self.assertEqual(rgb.frames,0)  # no recorded RGB until skeleton capture starts
                rgb.activate(time.monotonic_ns())
                deadline = time.monotonic()+3
                while rgb.frames < 3 and time.monotonic() < deadline: time.sleep(.02)
                rgb.close(); self.assertIsNone(rgb.error); self.assertGreaterEqual(rgb.frames,3)
                with (rgb.folder/'timestamps.csv').open() as timestamps:
                    rows = list(csv.DictReader(timestamps))
                self.assertEqual(len(rows),rgb.frames)
                for row in rows:
                    self.assertGreater(int(row['sensor_timestamp_ns']),0)
                    self.assertTrue((rgb.folder/row['file']).is_file())
                    if timestamp_source == 1:
                        self.assertEqual(int(row['capture_pc_estimate_ns']), int(row['sensor_timestamp_ns'])+int(row['clock_offset_estimate_ns']))
                    else:
                        self.assertEqual(row['capture_pc_estimate_ns'],'')
                        self.assertEqual(row['capture_body_elapsed_s'],'')
        finally:
            server.shutdown(); server.server_close(); thread.join(timeout=2)

    def test_sensor_and_receive_clocks_retained(self):
        self.exercise(1)

    def test_unknown_camera_clock_has_no_invented_alignment(self):
        self.exercise(0)

    def test_default_missing_rgb_fails_before_creating_episode(self):
        fake = types.ModuleType('xrt_devices'); fake.XRDevice = object
        writer = types.ModuleType('xrt_devices.recording.writer'); writer.CSVRecorder = object
        with tempfile.TemporaryDirectory() as temp, patch.dict(sys.modules,{'xrt_devices':fake,'xrt_devices.recording.writer':writer}):
            args = capture.parse_args(['--out-root',temp,'--session','missing_rgb','--rgb-config',str(Path(temp)/'missing.json')])
            self.assertFalse(args.body_only)
            self.assertEqual(capture.run_capture(args),3)
            self.assertFalse((Path(temp)/'missing_rgb').exists())
