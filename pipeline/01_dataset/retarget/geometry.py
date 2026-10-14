"""Explicit position-based frames; never interpret native OVR quaternions as Unity."""
import numpy as np
from scipy.spatial.transform import Rotation

# Unity: right/up/forward (left handed); robot: forward/left/up (right handed).
UNITY_TO_ROS = np.array([[0., 0., 1.], [-1., 0., 0.], [0., 1., 0.]])


def vector(value):
    result = np.array([value[k] for k in 'xyz'], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('nonfinite_position')
    return result


def unit(value, epsilon=1e-6):
    norm = np.linalg.norm(value)
    if norm < epsilon:
        raise ValueError('degenerate_frame')
    return value / norm


def yaw_rotation(yaw):
    return Rotation.from_euler('z', yaw).as_matrix()


def wrap(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


def rotation_error(a, b):
    return float(Rotation.from_matrix(a.T @ b).magnitude())


def check_rotation(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(matrix), 1., atol=1e-6):
        raise ValueError('Expected a proper orthonormal 3x3 rotation')
    return matrix


def chest_frame(points):
    center = (points[10] + points[15]) / 2
    lateral = unit(points[10] - points[15])
    up_hint = unit(center - points[3])
    forward = unit(np.cross(lateral, up_hint))
    up = unit(np.cross(forward, lateral))
    return center, np.column_stack((forward, lateral, up))


def pinch_frame(points, correction):
    """RoboWheel-inspired axes; fixed correction maps robot -Z to index direction.

    The opening axis is derived from MCPs, not a nearly closed fingertip chord.
    Degenerate frames are invalid; no invented orientation or last-pose padding.
    """
    forward = unit(points[10] - points[7])
    normal = unit(np.cross(forward, points[7] - points[3]))
    opening = unit(np.cross(normal, forward))
    frame = np.column_stack((opening, normal, forward))
    return (points[5] + points[10]) / 2, check_rotation(frame @ check_rotation(correction)), float(np.linalg.norm(points[5] - points[10]))


class LazyBase:
    """WARP-style critically damped SE(2) deadband, with explicit rate bounds."""
    def __init__(self, config, initial=None):
        self.cfg = config
        self.pose = np.zeros(3) if initial is None else np.array(initial, dtype=float)
        self.velocity = np.zeros(3)

    def update(self, desired, dt):
        if not 0 < dt <= self.cfg['max_gap_s']:
            raise ValueError('base_time_discontinuity')
        # Small integration steps keep the second-order filter stable at low FPS.
        count = max(1, int(np.ceil(dt / .005)))
        h = dt / count
        omega = 2 * np.pi * self.cfg['frequency_hz']
        for _ in range(count):
            error = np.asarray(desired) - self.pose
            distance = np.linalg.norm(error[:2])
            error[:2] *= max(0., distance - self.cfg['xy_deadband_m']) / max(distance, 1e-12)
            error[2] = np.sign(wrap(error[2])) * max(0., abs(wrap(error[2])) - self.cfg['yaw_deadband_rad'])
            accel = omega**2 * error - 2 * self.cfg['damping_ratio'] * omega * self.velocity
            limits = np.array([self.cfg['accel_m_s2']] * 2 + [self.cfg['yaw_accel_rad_s2']])
            accel = np.clip(accel, -limits, limits)
            n = np.linalg.norm(accel[:2])
            accel[:2] *= min(1., self.cfg['accel_m_s2'] / max(n, 1e-12))
            self.velocity += h * accel
            speed = np.linalg.norm(self.velocity[:2])
            self.velocity[:2] *= min(1., self.cfg['speed_m_s'] / max(speed, 1e-12))
            self.velocity[2] = np.clip(self.velocity[2], -self.cfg['yaw_speed_rad_s'], self.cfg['yaw_speed_rad_s'])
            self.pose += h * self.velocity
            self.pose[2] = wrap(self.pose[2])
        return self.pose.copy()


class TorsoReference:
    """Schmitt-trigger target: remove tiny sway while retaining bend/height intent."""
    def __init__(self, position, rotation, config):
        self.position = np.array(position, dtype=float)
        self.rotation = np.array(rotation, dtype=float)
        self.cfg = config
        self.active = False

    def update(self, position, rotation):
        d = np.linalg.norm(position - self.position)
        angle = rotation_error(self.rotation, rotation)
        if self.active and d < self.cfg['position_exit_m'] and angle < self.cfg['angle_exit_rad']:
            self.active = False
        if d > self.cfg['position_enter_m'] or angle > self.cfg['angle_enter_rad']:
            self.active = True
        if self.active:
            self.position = np.array(position, dtype=float)
            self.rotation = np.array(rotation, dtype=float)
        return self.position.copy(), self.rotation.copy(), self.active
