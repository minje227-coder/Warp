"""Read-only, timestamp-backed preview selection. Never creates an executable cut."""
import hashlib
import json
import math
import subprocess
from functools import lru_cache
from pathlib import Path

import numpy as np


def signature(entry):
    if not entry.get('video_manifest') or not entry.get('result'):
        raise ValueError('영상과 실행 궤적의 시각 매핑이 없습니다.')
    paths = [Path(entry['video']), Path(entry['video_manifest']),
             Path(entry['result'])/'results/execution-trajectory-100hz.npz',
             Path(entry['result'])/'results/video-mapping.jsonl']
    return tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


@lru_cache(maxsize=8)
def _load(sig):
    video, manifest, trajectory, mapping = [Path(p[0]) for p in sig]
    meta = json.loads(manifest.read_text())
    bound_trajectory = meta.get('trajectory') or str(Path(meta['result'])/'results/execution-trajectory-100hz.npz')
    if Path(meta['video']).resolve() != video.resolve() or Path(bound_trajectory).resolve() != trajectory.resolve():
        raise ValueError('영상 manifest의 궤적·영상 경로가 다릅니다.')
    if sha(video) != meta['video_sha256'] or sha(trajectory) != meta['trajectory_sha256']:
        raise ValueError('영상 또는 궤적 해시가 manifest와 다릅니다.')
    probe = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                            '-show_frames', '-show_entries', 'frame=best_effort_timestamp_time',
                            '-show_entries', 'format=duration', '-of', 'json', str(video)],
                           check=True, capture_output=True, text=True, timeout=20)
    info = json.loads(probe.stdout)
    pts = np.array([float(f['best_effort_timestamp_time']) for f in info['frames']])
    rows = [json.loads(line) for line in mapping.read_text().splitlines() if line.strip()]
    duration = float(info['format']['duration'])
    with np.load(trajectory, allow_pickle=False) as z:
        t, source, position, names = [z[k].copy() for k in ('execution_s', 'source_s', 'position', 'names')]
    if 'preview_interval_execution_s' in meta:
        begin, end = map(float, meta['preview_interval_execution_s'])
        if abs(duration-(end-begin)) > .002:
            raise ValueError('선언된 영상 구간과 실제 영상 길이가 다릅니다.')
    elif meta.get('original_render_pts_preserved') and Path(meta['result']).resolve() == trajectory.parents[1].resolve():
        if abs(float(meta['duration_s'])-duration) > .002 or not rows:
            raise ValueError('렌더 영상의 PTS 보존 정보를 확인하지 못했습니다.')
        begin = float(rows[0]['execution_s'])
        # An MP4's last frame lasts beyond its PTS. Clamp that display tail to the
        # real trajectory endpoint; never extrapolate a robot target past it.
        end = min(begin+duration, float(t[-1]))
    else:
        raise ValueError('영상의 실행 시각 기준점이 없습니다.')
    if (len(pts) < 2 or len(rows) < len(pts) or len(pts) != meta['frames']
            or not np.all(np.diff(pts) > 0) or abs(pts[0]) > .001
            or not math.isfinite(duration) or duration <= 0 or duration-pts[-1] > .1
            or not np.isfinite(t).all() or not np.isfinite(source).all() or not np.isfinite(position).all()
            or not np.all(np.diff(t) > 0) or np.any(np.diff(source) < 0)
            or position.shape != (len(t), 21) or len(source) != len(t) or len(names) != 21
            or begin < t[0]-.000001 or end > t[-1]+.000001):
        raise ValueError('영상·실행 궤적의 시각 범위를 검증하지 못했습니다.')
    execution = np.array([float(r['execution_s']) for r in rows[:len(pts)]])
    phases = np.array([float(r['source_s']) for r in rows[:len(pts)]])
    # This preview declares an execution-time cut. Prove PTS parity frame by frame.
    if ([r['display_index'] for r in rows[:len(pts)]] != list(range(len(pts)))
            or not np.isfinite(execution).all() or not np.isfinite(phases).all()
            or not np.allclose(execution, begin+pts, atol=.000002, rtol=0)
            or not np.allclose(phases, np.interp(execution, t, source), atol=.000002, rtol=0)
            or pts[-1] >= duration):
        raise ValueError('영상 프레임과 실행 궤적 매핑이 일치하지 않습니다.')
    # The final point is the exclusive video boundary, not a fabricated video frame.
    points = [dict(video_s=float(v), execution_s=float(e), source_s=float(s))
              for v, e, s in zip(pts, execution, phases)]
    points.append(dict(video_s=duration, execution_s=end, source_s=float(np.interp(end, t, source))))
    fingerprint = hashlib.sha256(json.dumps(sig).encode()).hexdigest()
    return dict(points=points, fingerprint=fingerprint, t=t, source=source, position=position,
                names=names.tolist(), trajectory_sha256=meta['trajectory_sha256'], video_sha256=meta['video_sha256'])


def load(entry):
    return _load(signature(entry))


def public(data):
    return dict(available=True, points=data['points'], fingerprint=data['fingerprint'],
                trajectory_sha256=data['trajectory_sha256'], video_sha256=data['video_sha256'])


def interval(data, start, end):
    if (isinstance(start, bool) or isinstance(end, bool) or not isinstance(start, (int, float))
            or not isinstance(end, (int, float)) or not all(math.isfinite(v) for v in (start, end))):
        raise ValueError('시작·끝 시각은 유한한 숫자여야 합니다.')
    points = data['points']
    if not 0 <= start < end <= points[-1]['video_s'] + .000001:
        raise ValueError('선택 구간이 영상 범위를 벗어났습니다.')
    # Snap to real frame timestamps, keeping the exclusive ending boundary available.
    a = min(points[:-1], key=lambda p: abs(p['video_s']-start))
    b = min(points[1:], key=lambda p: abs(p['video_s']-end))
    if b['video_s'] <= a['video_s'] or b['execution_s'] <= a['execution_s']:
        raise ValueError('최소 한 프레임 이상 선택하세요.')
    return dict(start=dict(a), end=dict(b), duration_s=b['execution_s']-a['execution_s'],
                validation_status='pending', executable=False)


def initial(data, execution_s):
    x = [float(np.interp(execution_s, data['t'], data['position'][:, j])) for j in range(21)]
    return dict(source_s=float(np.interp(execution_s, data['t'], data['source'])),
                execution_s=execution_s, base_pose=x[:3], q=x[3:], joint_names=data['names'][3:])
