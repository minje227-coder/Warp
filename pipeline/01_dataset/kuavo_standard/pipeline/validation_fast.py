"""Process-isolated validation with exact-key environment interpolation reuse."""
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
import json
import multiprocessing
import os
from pathlib import Path
import sys
import types
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from path_tools import Reference


class WorldCache:
    def __init__(self, samples):
        self.samples = samples
        self.times = np.array([r['elapsed_s'] for r in samples])
        self.interpolations = 0

    @lru_cache(maxsize=16)
    def segment(self, lo, hi):
        result = []
        for a, b in zip(self.samples[lo]['world'], self.samples[hi]['world']):
            if a['id'] != b['id']:
                raise ValueError('World object order changed')
            rotations = Rotation.from_quat([a['quaternion_xyzw'], b['quaternion_xyzw']])
            result.append((a, np.array(a['position']), np.array(b['position']), Slerp([0., 1.], rotations)))
        return result

    @lru_cache(maxsize=2)
    def at(self, t):
        hi = min(int(np.searchsorted(self.times, t)), len(self.times)-1)
        lo = max(0, hi-1)
        u = 0. if hi == lo else float(np.clip((t-self.times[lo])/(self.times[hi]-self.times[lo]), 0, 1))
        self.interpolations += 1
        return [(meta, (1-u)*a+u*b, slerp(u)) for meta, a, b, slerp in self.segment(lo, hi)]

    def world(self, t):
        result = []
        for meta, p, r in self.at(float(t)):
            z = dict(meta)
            z.update(position=p.tolist(), quaternion_xyzw=r.as_quat().tolist())
            result.append(z)
        return result


def install_environment_cache(model):
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform, RotationMatrix
    from retarget.geometry import yaw_rotation
    model.validation_world = WorldCache(model.environment_samples)
    model._validation_environment_key = None
    model._validation_environment_context = None

    def update(self, base, elapsed_s=None):
        t = self.environment_time if elapsed_s is None else float(elapsed_s)
        b = np.asarray(base)
        key = (float(t), b.dtype.str, b.tobytes())
        if self._validation_environment_context is self.scene_context and self._validation_environment_key == key:
            return
        rb = yaw_rotation(base[2]); origin = np.r_[base[:2], 0.]
        for meta, position, rotation in self.validation_world.at(float(t)):
            if meta['id'] == 'ground':
                continue
            pose = RigidTransform(RotationMatrix(rb.T @ rotation.as_matrix()), rb.T @ (position-origin))
            self.scene.ChangeShape(self.scene_context, self.plant.get_source_id(),
                self.environment_geometries[meta['id']], Box(*meta['dimensions']), pose)
        self._validation_environment_context = self.scene_context
        self._validation_environment_key = key

    model.update_environment = types.MethodType(update, model)
    return model


_STATE = None


def make_model(root, cache):
    root = Path(root)
    sys.path.insert(0, str(root/'source'))
    from retarget.model import KuavoModel
    cfg = json.loads((root/'data/source-config.json').read_text())['ik']
    cfg['environment_world_json'] = str(root/'data/world.json')
    model = KuavoModel('/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf', cache, cfg)
    return install_environment_cache(model)


def initialize(root):
    global _STATE
    root = Path(root)
    _STATE = (Reference(root), make_model(root, root/'validation-workers'/str(os.getpid())))


def validate_chunk(chunk):
    ref, model = _STATE
    result = []
    for k, s in chunk:
        x = np.array(s['x']); t = s['source_s']; base, q = x[:3], x[3:]
        hi = min(int(np.searchsorted(ref.ts, t)), len(ref.ts)-1); lo = max(0, hi-1)
        u = 0 if lo == hi else float(np.clip((t-ref.ts[lo])/(ref.ts[hi]-ref.ts[lo]), 0, 1))
        reference = (1-u)*ref.x[lo]+u*ref.x[hi]
        goals = ref.goals(t); allowance = ref.allowance(t)
        model.cfg['tcp_position_tolerance_m'] = allowance
        model.set_environment(t)
        check = model.validate(q, base, goals)
        candidate = model.poses(q, base)
        # validate() already installed this q and environment.
        pairs = model.query().ComputeSignedDistancePairwiseClosestPoints(.02)
        distance = min((float(p.distance) for p in pairs), default=.02)
        baseline_check = model.validate(reference[3:], reference[:3], goals)
        baseline = model.poses(reference[3:], reference[:3])
        tcp_delta = max(np.linalg.norm(p[0]-r[0]) for p, r in zip(candidate, baseline))
        pose_ok = all(e['position_m'] <= allowance+1e-5 for e in check['tcp_errors']) and check['mutual_tcp_orientation_error_rad'] <= np.deg2rad(10)+1e-5
        baseline_ok = all(e['position_m'] <= allowance+1e-5 for e in baseline_check['tcp_errors']) and baseline_check['mutual_tcp_orientation_error_rad'] <= np.deg2rad(10)+1e-5
        audit = dict(index=k, execution_s=s['execution_s'], source_s=t, position_allowance_m=allowance,
            box_surface_distance_m=float(np.interp(t, ref.ts, ref.distances)), minimum_distance_m=distance,
            check=check, reference_path_check=baseline_check)
        moveit = dict(index=k, elapsed_s=t, execution_s=s['execution_s'], kind='accepted', q=q.tolist(), base=base.tolist(),
            world=model.validation_world.world(t), tcp_world_m=[p.tolist() for p, r in candidate])
        result.append((audit, moveit, float(tcp_delta), abs(x-reference).tolist(), bool(pose_ok), bool(baseline_ok)))
    return result


def worker_count(value=None):
    n = int(os.environ.get('KUAVO_VALIDATION_WORKERS', '4') if value is None else value)
    if not 1 <= n <= 4:
        raise ValueError('KUAVO_VALIDATION_WORKERS must be between 1 and 4')
    available = len(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else (os.cpu_count() or 1)
    return min(n, available)


def validate_samples(root, samples, workers=None):
    n = worker_count(workers)
    indexed = list(enumerate(samples))
    chunks = [indexed[j:j+256] for j in range(0, len(indexed), 256)]
    if n == 1:
        initialize(str(root))
        for c in chunks:
            yield from validate_chunk(c)
        return
    # Each spawned process owns its Drake scene/context and cache.
    with ProcessPoolExecutor(max_workers=n, mp_context=multiprocessing.get_context('spawn'),
                             initializer=initialize, initargs=(str(root),)) as pool:
        for result in pool.map(validate_chunk, chunks):
            yield from result
