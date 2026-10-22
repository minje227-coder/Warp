import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from scipy.interpolate import CubicSpline, make_interp_spline

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pipeline'))
from path_tools import SplinePath, Reference, load_parameterized_path


class ExactPathTests(unittest.TestCase):
    def setUp(self):
        self.ts = np.array([.2, .35, .9, 1.4, 2.])
        self.x = np.random.default_rng(7).normal(size=(5, 21))*.1
        self.b = make_interp_spline(self.ts, self.x, bc_type='natural')
        self.path = SplinePath(self.b.t, self.b.c, self.b.k)
        self.grid = np.linspace(self.ts[0], self.ts[-1], 501)

    def test_initial_geometry_matches_previous_natural_cubic(self):
        legacy = CubicSpline(self.ts, self.x, bc_type='natural')
        for derivative in [0, 1, 2]:
            np.testing.assert_allclose(self.path(self.grid, derivative), legacy(self.grid, derivative), atol=1e-11)

    def test_serialized_curve_preserves_derivatives_not_just_waypoints(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d)/'curve.npz'
            np.savez(f, **self.path.fields(), source_s=self.ts, knots=self.x)
            with np.load(f) as z:
                restored = load_parameterized_path(z)
            for derivative in [0, 1, 2]:
                np.testing.assert_array_equal(restored(self.grid, derivative), self.path(self.grid, derivative))

    def test_legacy_artifacts_keep_legacy_curve(self):
        restored = load_parameterized_path(dict(source_s=self.ts, knots=self.x))
        np.testing.assert_allclose(restored(self.grid), self.path(self.grid), atol=1e-13)

    def test_unknown_path_format_is_rejected(self):
        with self.assertRaises(ValueError):
            load_parameterized_path(dict(path_kind='unknown'))

    def test_nonfinite_control_point_is_rejected(self):
        c = self.b.c.copy();c[2, 3] = np.nan
        with self.assertRaises(ValueError):
            SplinePath(self.b.t, c)

    def test_joint_control_bounds_cover_entire_curve(self):
        for joint in range(21):
            values = self.path(self.grid)[:, joint]
            self.assertGreaterEqual(values.min(), self.b.c[:, joint].min()-1e-14)
            self.assertLessEqual(values.max(), self.b.c[:, joint].max()+1e-14)

    def test_curve_has_continuous_velocity_and_acceleration(self):
        for t in self.ts[1:-1]:
            for order in [0, 1, 2]:
                np.testing.assert_allclose(self.path(t-1e-10, order), self.path(t+1e-10, order), atol=1e-7)

    def test_naive_cubic_can_violate_joint_range_between_valid_knots(self):
        b = make_interp_spline([0., 1., 2., 3.], [[0.], [1.], [1.], [0.]], bc_type='natural')
        p = SplinePath(b.t, b.c)
        self.assertGreater(p(np.linspace(0, 3, 301)).max(), 1.)


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('standard_under_test', ROOT/'standard.py')
        self.standard = importlib.util.module_from_spec(spec);spec.loader.exec_module(self.standard)

    def report(self, **changes):
        v = dict(source_knot_invalid=0, sampled_pose_failures=0, sampled_state_invalid=0,
                 clearance_below10mm_minus1um_states=0, max_sampled_velocity_ratio=.5, max_sampled_acceleration_ratio=.9,
                 summary=dict(duration_s=2.))
        v.update(changes)
        return dict(status='verified', validation=v, moveit=dict(accepted_invalid=0, interframe_invalid=0, checked_states=100))

    def check_rejected(self, report):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d);(p/'REPORT.json').write_text(json.dumps(report));(p/'artifact-sha256.json').write_text('{}')
            with self.assertRaises(AssertionError):
                self.standard.verify(p)

    def test_joint_guard_failure_cannot_pass_pose_only_verification(self):
        self.check_rejected(self.report(sampled_state_invalid=9))

    def test_rejected_status_cannot_be_verified(self):
        r = self.report();r['status']='rejected_by_dense_validation'
        self.check_rejected(r)

    def test_velocity_failure_cannot_be_verified(self):
        self.check_rejected(self.report(max_sampled_velocity_ratio=1.01))


if __name__ == '__main__':
    unittest.main()
