"""The solver rescaling must preserve physical feasibility and AutoDiff."""
from pathlib import Path
import sys
import unittest
import numpy as np
from pydrake.autodiffutils import InitializeAutoDiff, ExtractGradient, ExtractValue
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import PrismaticJoint, SpatialInertia, UnitInertia
from pydrake.multibody.inverse_kinematics import PointToPointDistanceConstraint

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pipeline'))
from refine import normalized_squared_distance


class DistanceNormalizationTests(unittest.TestCase):
    def setUp(self):
        self.plant = MultibodyPlant(0.)
        body = self.plant.AddRigidBody('point', SpatialInertia(1., np.zeros(3), UnitInertia(1., 1., 1.)))
        self.plant.AddJoint(PrismaticJoint('x', self.plant.world_frame(), body.body_frame(), [1., 0., 0.]))
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()
        self.body = body

    def constraint(self, radius):
        return PointToPointDistanceConstraint(self.plant, self.body.body_frame(), np.zeros(3),
            self.plant.world_frame(), np.zeros(3), 0., radius, self.context)

    def test_same_physical_boundary_at_near_and_far_allowances(self):
        for radius in [.0098, .0196]:
            original = self.constraint(radius)
            owner, normalized = normalized_squared_distance(original)
            np.testing.assert_array_equal(normalized.upper_bound(), [1.])
            for factor in [-1.1, -1.000001, -.999999, 0., .999999, 1.000001, 1.1]:
                q = np.array([factor*radius])
                self.assertEqual(original.CheckSatisfied(q, 0.), normalized.CheckSatisfied(q, 0.))
                self.assertEqual(normalized.CheckSatisfied(q, 0.), abs(factor)<=1)

    def test_native_autodiff_is_preserved(self):
        radius = .0098
        original = self.constraint(radius)
        owner, normalized = normalized_squared_distance(original)
        q = InitializeAutoDiff(np.array([.007]))
        expected = original.Eval(q)/(radius**2)
        actual = normalized.Eval(q)
        np.testing.assert_allclose(ExtractValue(actual), ExtractValue(expected), atol=1e-15)
        np.testing.assert_allclose(ExtractGradient(actual), ExtractGradient(expected), atol=1e-12)
        epsilon = 1e-7
        numerical = (normalized.Eval([.007+epsilon])-normalized.Eval([.007-epsilon]))/(2*epsilon)
        np.testing.assert_allclose(ExtractGradient(actual).ravel(), numerical, rtol=1e-9)

    def test_zero_squared_radius_is_rejected(self):
        with self.assertRaises(ValueError):
            normalized_squared_distance(self.constraint(0.))


if __name__ == '__main__':
    unittest.main()
