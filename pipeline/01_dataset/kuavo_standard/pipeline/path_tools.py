"""The exact geometric path shared by smoothing, TOPPRA and verification.

Source timestamps and IK rows are immutable references. A separate B-spline
artifact stores the optimized curve; never reconstruct it from output waypoints.
"""
from pathlib import Path
import json
import numpy as np
from scipy.interpolate import BSpline, make_interp_spline
from scipy.spatial.transform import Rotation, Slerp
from scipy.special import betainc
from toppra.interpolator import AbstractGeometricPath

PATH_KIND = 'constrained_bspline_v1'


class SplinePath(AbstractGeometricPath):
    def __init__(self, knots, coefficients, degree=3):
        self.spline = BSpline(np.asarray(knots), np.asarray(coefficients), int(degree), extrapolate=False)
        if not np.isfinite(self.spline.t).all() or not np.isfinite(self.spline.c).all():
            raise ValueError('Nonfinite path')

    def __call__(self, s, order=0):
        return self.spline(s, nu=order)

    @property
    def dof(self):
        return self.spline.c.shape[1]

    @property
    def path_interval(self):
        b = self.spline
        return np.array([b.t[b.k], b.t[-b.k-1]])

    def fields(self):
        b = self.spline
        return dict(path_kind=PATH_KIND, path_knots=b.t, path_coefficients=b.c, path_degree=b.k)


def load_parameterized_path(z):
    if 'path_kind' in z:
        if str(z['path_kind']) != PATH_KIND:
            raise ValueError('Unknown geometric path format')
        return SplinePath(z['path_knots'], z['path_coefficients'], z['path_degree'])
    # Historical trajectories retain precisely their historical geometry.
    from toppra import SplineInterpolator
    return SplineInterpolator(z['source_s'], z['knots'], bc_type='natural')


class Reference:
    def __init__(self, root):
        self.root = Path(root)
        self.rows = [r for r in map(json.loads, (self.root/'data/source-trajectory.jsonl').read_text().splitlines()) if r['valid']]
        self.ts = np.array([r['elapsed_s'] for r in self.rows])
        self.x = np.array([np.r_[r['base_pose'], r['q']] for r in self.rows])
        if len(self.ts) < 3 or not np.all(np.diff(self.ts) > 0):
            raise ValueError('Need at least three strictly ordered IK states')
        self.profile = json.loads((self.root/'PROFILE.json').read_text())
        self.distances = np.array([r['box_surface_distance_m'] for r in self.rows])
        self.initial = make_interp_spline(self.ts, self.x, k=3, bc_type='natural')

    def allowance(self, t):
        if not self.profile.get('adaptive'):
            return self.profile.get('tol', .01)
        d = float(np.interp(t, self.ts, self.distances))
        return float(.01 + .01*betainc(3, 3, np.clip((d-.1)/.1, 0, 1)))

    def goals(self, t):
        hi = min(int(np.searchsorted(self.ts, t)), len(self.ts)-1)
        lo = max(0, hi-1)
        u = 0. if hi == lo else float(np.clip((t-self.ts[lo])/(self.ts[hi]-self.ts[lo]), 0, 1))
        goals = []
        for a, b in zip(self.rows[lo]['tcp_targets'], self.rows[hi]['tcp_targets']):
            g = {}
            for key, value in a.items():
                if key not in b:
                    continue
                if key in ('rotation', 'raw_rotation'):
                    g[key] = Slerp([0., 1.], Rotation.from_matrix([value, b[key]]))(u).as_matrix()
                elif isinstance(value, list):
                    g[key] = (1-u)*np.asarray(value) + u*np.asarray(b[key])
                else:
                    g[key] = value
            for key in ('body_down_direction', 'body_forward_direction'):
                g[key] /= np.linalg.norm(g[key])
            goals.append(g)
        return goals


def audit(path, reference, model, times, cache=None, statistics=None):
    """Independent FK/distance acceptance; no solver status substitutes for it."""
    failures = []
    for t in np.unique(times):
        x = path(float(t))
        # A source phase fixes goals/world/allowance within this refinement run.
        # Reuse only bit-identical coordinates; never use a geometric epsilon.
        key = float(t)
        saved = cache.get(key) if cache is not None else None
        if saved is not None and np.array_equal(saved[0], x):
            check = saved[1]
            if statistics is not None:statistics['reused'] += 1
        else:
            model.cfg['tcp_position_tolerance_m'] = reference.allowance(t)
            model.set_environment(t)
            check = model.validate(x[3:], x[:3], reference.goals(t))
            if cache is not None:cache[key] = (x.copy(), check)
            if statistics is not None:statistics['computed'] += 1
        if not check['valid']:
            failures.append(dict(source_s=float(t), check=check))
    return failures
