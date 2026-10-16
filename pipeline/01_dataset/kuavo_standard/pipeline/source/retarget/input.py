"""Raw-v3 tracking input with exact provenance and bounded interpolation."""
from bisect import bisect_left
import hashlib
import json
from pathlib import Path
import numpy as np
from .geometry import UNITY_TO_ROS, chest_frame, vector, yaw_rotation

UNIX_TICKS = 621355968000000000
REQUIRED = {'body': (1, 3, 10, 11, 15, 16, 19, 45), 'XRHandLeft': (1, 3, 5, 7, 10), 'XRHandRight': (1, 3, 5, 7, 10)}
FRAMES = {'body': 'meta_source_native_skeleton_untransformed', 'XRHandLeft': 'unity_world_meters_xyzw', 'XRHandRight': 'unity_world_meters_xyzw'}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read_episode(path):
    path = Path(path).resolve()
    manifest = json.loads((path / 'manifest.json').read_text())
    if manifest.get('schema_version') != 3:
        raise ValueError('Expected raw-v3 episode')
    for name, spec in manifest['files'].items():
        if Path(name).name != name or not name:
            raise ValueError('Unsafe source filename')
        source = path / name
        if source.stat().st_size != spec['bytes'] or sha256(source) != spec['sha256']:
            raise ValueError('Raw checksum mismatch: ' + name)
    samples = {kind: [] for kind in REQUIRED}
    last = {}
    with (path / 'tracking.jsonl').open() as stream:
        for index, line in enumerate(stream):
            sample = json.loads(line)
            kind = sample.get('kind')
            if kind not in samples:
                continue
            ticks = int(sample['timestamp_ticks'])
            if kind in last and ticks <= last[kind]:
                raise ValueError('Duplicate/regressed tracking timestamp: ' + kind)
            last[kind] = ticks
            try:
                bones = sample['bones']
                ids = [int(b['id']) for b in bones]
                if len(set(ids)) != len(ids):
                    raise ValueError('duplicate_bone_id')
                points = {int(b['id']): vector(b['position']) for b in bones}
                valid = bool(sample.get('valid')) and all(i in points for i in REQUIRED[kind]) and sample.get('coordinate_frame') == FRAMES[kind]
                valid = valid and max((np.linalg.norm(v) for v in points.values()), default=0.) > 1e-6
            except (KeyError, TypeError, ValueError):
                points, valid = {}, False
            samples[kind].append({'ticks': ticks, 'utc_ns': (ticks - UNIX_TICKS) * 100, 'line': index + 1,
                'points': points, 'valid': valid, 'frame': sample.get('coordinate_frame'), 'time_source': sample.get('time_source'),
                'origin': sample.get('tracking_origin_pose'), 'provider': sample.get('provider_transform')})
    events = [json.loads(line) for line in (path / 'events.jsonl').open() if line.strip()]
    discontinuities = [int(e['unix_ticks']) for e in events if 'unix_ticks' in e and any(k in e.get('event', '') for k in ('recenter', 'pause', 'regressed', 'tracking_lost'))]
    return manifest, samples, discontinuities


def interpolate(samples, ticks, maximum_gap_s, discontinuities=()):
    times = [s['ticks'] for s in samples]
    i = bisect_left(times, ticks)
    if i < len(samples) and times[i] == ticks:
        lo = hi = i
    elif i == 0 or i == len(samples):
        raise ValueError('outside_tracking')
    else:
        lo, hi = i - 1, i
    a, b = samples[lo], samples[hi]
    if not a['valid'] or not b['valid']:
        raise ValueError('invalid_tracking')
    if (b['ticks'] - a['ticks']) * 1e-7 > maximum_gap_s:
        raise ValueError('tracking_gap')
    if a['frame'] != b['frame'] or a['origin'] != b['origin'] or a['provider'] != b['provider'] or any(a['ticks'] < d <= b['ticks'] for d in discontinuities):
        raise ValueError('tracking_discontinuity')
    w = 0. if lo == hi else (ticks - a['ticks']) / (b['ticks'] - a['ticks'])
    points = {j: (1 - w) * a['points'][j] + w * b['points'][j] for j in a['points'].keys() & b['points'].keys()}
    return points, {'source_lines': [a['line'], b['line']], 'source_timestamp_ticks': [a['ticks'], b['ticks']],
        'interpolation_weight': w, 'coordinate_frame': a['frame'], 'time_source': a['time_source']}


class WorldAlignment:
    """One constant world transform; never re-center TCP targets every frame."""
    def __init__(self, body_points, config):
        body = {i: UNITY_TO_ROS @ v for i, v in body_points.items()}
        _, frame = chest_frame(body)
        yaw = np.arctan2(frame[1, 0], frame[0, 0])
        self.rotation = yaw_rotation(config['initial_base_pose'][2] - yaw)
        root = body[1].copy()
        root[2] = 0.
        self.translation = np.array(config['initial_base_pose'][:2] + [config['height_offset_m']]) - self.rotation @ root
        self.matrix = self.rotation @ UNITY_TO_ROS
        self.status = 'first_body_yaw_and_pelvis_xy_provisional'

    def points(self, value):
        return {i: self.matrix @ v + self.translation for i, v in value.items()}

    def metadata(self):
        return {'status': self.status, 'position_basis': self.matrix.tolist(), 'translation_m': self.translation.tolist(),
            'native_body_basis_verified': False, 'quest_to_robot_extrinsics_verified': False,
            'note': 'Position-based shared-basis assumption gated by per-frame XR/body wrist agreement; no native quaternion reinterpretation.'}
