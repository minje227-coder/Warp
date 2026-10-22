"""Read-only camera transport and pose math, independent of dashboard UI."""

import json, math, threading, time

import numpy as np

AGENT_IDLE_STOP_S = 30.0

def quat_to_R(q):
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("quaternion norm 0")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def pose_to_T(pose):
    """[x, y, z, qx, qy, qz, qw] → 4×4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(pose[3:7])
    T[:3, 3] = pose[:3]
    return T

class Agent(object):
    """Read-only subprocess stream; retain the latest timestamped observation."""

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.sample = None
        self.urdf = None
        self.urdf_sha = None
        self.logs = []
        self.polled = 0.0
        self.started_at = None
        self.rx_times = []

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def _log(self, text):
        with self.lock:
            self.logs = (self.logs + ['%s %s' % (time.strftime('%H:%M:%S'), text)])[-20:]

    def _read(self, proc):
        for raw in iter(proc.stdout.readline, b''):
            line = raw.decode('utf-8', 'replace').strip()
            if not line.startswith('{'):
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            self.feed(msg)
        self._log('수집 프로그램 끝남 (code %s)' % proc.wait())

    def feed(self, msg, now=None):
        now = time.time() if now is None else now
        if 'urdf' in msg:
            with self.lock:
                self.urdf, self.urdf_sha = (msg['urdf'], msg.get('urdf_sha'))
            return
        if 'log' in msg:
            self._log(msg['log'])
            return
        msg['received'] = now
        with self.lock:
            self.sample = msg
            self.rx_times = [t for t in self.rx_times if now - t < 2.0] + [now]

    def _idle(self, proc):
        while proc.poll() is None:
            time.sleep(2.0)
            if time.time() - self.polled > AGENT_IDLE_STOP_S:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
                time.sleep(2.0)
                if proc.poll() is None:
                    proc.terminate()
                return

    def snapshot(self):
        with self.lock:
            rate = len(self.rx_times) / 2.0 if self.rx_times else 0.0
            return (self.sample, self.urdf, self.urdf_sha, list(self.logs), rate)
