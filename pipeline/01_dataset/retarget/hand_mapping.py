"""Experimental hand targets for review; no changes to the production IK default."""
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import check_rotation, pinch_frame, unit


def tcp_frame(points, correction, method):
    if method == 'mcp_filtered':
        method = 'mcp_pinch'
    if method == 'legacy_pinch':
        return pinch_frame(points, correction)
    if method == 'index_palm':
        return ((points[5]+points[10])/2, check_rotation(index_palm_frame(points) @ check_rotation(correction)), float(np.linalg.norm(points[5]-points[10])))
    if method == 'palm_calibrated':
        return ((points[5]+points[10])/2, check_rotation(palm_frame(points) @ check_rotation(correction)), float(np.linalg.norm(points[5]-points[10])))
    if method != 'mcp_pinch':
        raise ValueError('unknown_hand_mapping')
    return ((points[5]+points[10])/2,
        check_rotation(mcp_pinch_frame(points) @ check_rotation(correction)),
        float(np.linalg.norm(points[5]-points[10])))


def palm_frame(points):
    """Stable proper frame from wrist/index MCP/ring MCP, without fingertip axes.

    Inspired by RoboWheel's palm-plane construction. The bisector/closing axes
    here differ from its Algorithm 1 and require a fixed tool calibration.
    """
    forward = unit((points[7] + points[17]) / 2 - points[1])
    closing = points[17] - points[7]
    closing = unit(closing - forward * np.dot(closing, forward))
    normal = unit(np.cross(forward, closing))
    return check_rotation(np.column_stack((closing, normal, forward)))


def mcp_pinch_frame(points):
    """HumanEgo Eq.6, expressed in the existing hand-frame convention.

    HumanEgo defines [closing, forward, normal]; a constant -90deg X
    basis rotation expresses the same frame as [closing, -normal, forward].
    The configured hand-to-tool rotation subsequently maps robot axes.
    No fingertip participates in orientation; thumb opposition still can.
    """
    closing = unit(points[7] - points[3])
    forward = (points[7] + points[3]) / 2 - points[1]
    forward = unit(forward - closing * np.dot(forward, closing))
    normal = unit(np.cross(closing, forward))
    return check_rotation(np.column_stack((closing, -normal, forward)))


def calibrate_hand(samples, correction, frame_method='palm_reference'):
    """Fit one constant wrist-to-pinch point and palm-to-tool rotation.

    This estimates a demonstration reference, not robot contact calibration.
    Original fingertip-based orientation supplies the provisional tool axes.
    """
    if not samples:
        raise ValueError('no_hand_calibration_samples')
    offsets, rotations = [], []
    for points in samples:
        palm = mcp_pinch_frame(points) if frame_method == 'mcp_pinch' else palm_frame(points)
        center, rotation, _ = pinch_frame(points, correction)
        offsets.append(palm.T @ (center - points[1]))
        rotations.append(check_rotation(correction) if frame_method == 'mcp_pinch' else palm.T @ rotation)
    offset = np.median(offsets, axis=0)
    rotation = Rotation.from_matrix(rotations).mean().as_matrix()
    return {'wrist_to_tcp_palm_m': offset.tolist(), 'palm_to_tcp_rotation': rotation.tolist(),
        'samples': len(samples), 'offset_residual_p95_m': float(np.percentile(np.linalg.norm(np.array(offsets) - offset, axis=1), 95)),
        'frame_method': frame_method, 'calibration_verified': False,
        'method': 'reference_window_median_position; MCP axes + constant tool transform' if frame_method == 'mcp_pinch' else 'reference_window_median_position_and_SO3_mean; no contact/robot calibration'}


def hand_candidates(points, calibration, correction):
    palm = mcp_pinch_frame(points) if calibration.get('frame_method') == 'mcp_pinch' else palm_frame(points)
    center, old_rotation, width = pinch_frame(points, correction)
    stable = check_rotation(palm @ np.array(calibration['palm_to_tcp_rotation']))
    fixed = points[1] + palm @ np.array(calibration['wrist_to_tcp_palm_m'])
    # Parallel jaws have an unoriented closing axis. Near closure, a fingertip
    # chord is ill-conditioned, so report no angle rather than invent a vector.
    chord_angle = None if width < .01 else float(np.degrees(np.arccos(np.clip(abs(np.dot(stable[:, 0], (points[10] - points[5]) / width)), 0., 1.))))
    return {'pinch_palm': {'position': center, 'rotation': stable},
        'palm_fixed': {'position': fixed, 'rotation': stable},
        'original_pinch': {'position': center, 'rotation': old_rotation},
        'palm_rotation': palm, 'pinch_width_m': width, 'closing_axis_tip_chord_deg': chord_angle}


def calibrate_palm_axes(samples, correction):
    """Constant palm-to-tool transform from a reference window; no IK fitting."""
    relative=[palm_frame(p).T @ mcp_pinch_frame(p) @ check_rotation(correction) for p in samples]
    if len(relative)<10:
        raise ValueError('insufficient_palm_axis_reference')
    fitted=Rotation.from_matrix(relative).mean().as_matrix()
    residual=Rotation.from_matrix(np.asarray(relative) @ fitted.T).magnitude()
    return fitted, {'samples':len(relative),'residual_p95_rad':float(np.percentile(residual,95)),
        'method':'initial_1s_SO3_mean_palm_to_MCP_tool; no IK fitting', 'robot_contact_calibration_verified':False}


def index_palm_frame(points):
    """Index proximal phalanx approach plus palm plane; no thumb/fingertip axes.

    Robot -Z follows index MCP-to-PIP; projected index-to-ring MCP supplies
    parallel-jaw roll. This is an explicit geometric mapping, not contact fitting.
    """
    forward=unit(points[8]-points[7])
    lateral=points[17]-points[7]
    closing=unit(lateral-forward*np.dot(lateral,forward))
    normal=unit(np.cross(forward,closing))
    return check_rotation(np.column_stack((closing,normal,forward)))


class HandOrientationFilter:
    """Causal SO(3) filter before IK, with an explicit cap on orientation lag."""
    def __init__(self,tau_s=.04,max_lag_rad=.05235987756):
        self.tau=tau_s; self.max_lag=max_lag_rad; self.rotation=None

    def update(self,raw,dt=None):
        raw=check_rotation(raw)
        if self.rotation is None or dt is None:
            self.rotation=raw.copy();return self.rotation.copy()
        delta=Rotation.from_matrix(self.rotation.T @ raw).as_rotvec()
        alpha=1-np.exp(-dt/self.tau)
        result=self.rotation @ Rotation.from_rotvec(delta*alpha).as_matrix()
        error=Rotation.from_matrix(result.T @ raw).as_rotvec();angle=np.linalg.norm(error)
        if angle>self.max_lag:
            result=result @ Rotation.from_rotvec(error*(1-self.max_lag/angle)).as_matrix()
        self.rotation=check_rotation(result)
        return self.rotation.copy()
