import copy
import json
import os
from pathlib import Path
import numpy as np
import pytest
from retarget.geometry import LazyBase, TorsoReference, UNITY_TO_ROS, pinch_frame, rotation_error
from retarget.input import interpolate, WorldAlignment
from retarget.convert import validate_config

ROOT = Path(__file__).resolve().parents[3]
CONFIG = json.loads((ROOT / 'configs/kuavo_retarget.json').read_text())


def sample(ticks, valid=True):
    return {'ticks': ticks, 'line': ticks, 'valid': valid, 'points': {1: np.array([ticks, 0., 0.])},
        'frame': 'unity', 'origin': {}, 'provider': {}, 'time_source': 'source'}


def test_bounded_interpolation_and_exact_provenance():
    points, origin = interpolate([sample(100), sample(200)], 150, .05)
    assert points[1][0] == 150
    assert origin['source_timestamp_ticks'] == [100, 200]
    assert origin['interpolation_weight'] == .5
    with pytest.raises(ValueError, match='outside_tracking'):
        interpolate([sample(100), sample(200)], 99, .05)


@pytest.mark.parametrize('cause', ['invalid', 'gap', 'origin', 'recenter'])
def test_bad_tracking_not_padded(cause):
    a, b = sample(0), sample(1_000_000 if cause == 'gap' else 100)
    if cause == 'invalid':
        b['valid'] = False
    if cause == 'origin':
        b['origin'] = {'changed': True}
    with pytest.raises(ValueError):
        interpolate([a, b], 50, .05, [75] if cause == 'recenter' else [])


def test_closed_pinch_frame_remains_proper():
    p = {3: np.array([-.05, 0., 0.]), 7: np.zeros(3), 10: np.array([0., 0., .1]), 5: np.array([0., 0., .1])}
    target, rotation, width = pinch_frame(p, CONFIG['hand_to_tcp_rotation']['left'])
    assert width == 0
    assert np.allclose(rotation.T @ rotation, np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1)
    assert np.allclose(-rotation[:, 2], [0, 0, 1])
    assert np.allclose(target, [0, 0, .1])
    p[7] = p[10].copy()
    with pytest.raises(ValueError, match='degenerate'):
        pinch_frame(p, CONFIG['hand_to_tcp_rotation']['left'])


def test_base_lazy_deadband_rate_and_yaw_wrap():
    cfg = CONFIG['base']
    base = LazyBase(cfg)
    for _ in range(100):
        assert np.allclose(base.update([.02, .01, .05], .02), 0)
    for _ in range(100):
        before = base.pose.copy()
        result = base.update([2., 1., 1.], .02)
        assert np.linalg.norm((result - before)[:2]) <= cfg['speed_m_s'] * .02 + 1e-6
        assert abs(base.velocity[2]) <= cfg['yaw_speed_rad_s'] + 1e-9
    assert np.linalg.norm(base.pose[:2]) > .1
    with pytest.raises(ValueError):
        base.update([0, 0, 0], 1.)


def test_torso_jitter_rejected_real_height_change_kept():
    reference = TorsoReference(np.array([0., 0., 1.3]), np.eye(3), CONFIG['torso_reference'])
    p, r, active = reference.update(np.array([0., 0., 1.304]), np.eye(3))
    assert not active and p[2] == 1.3
    p, _, active = reference.update(np.array([0., 0., 1.1]), np.eye(3))
    assert active and p[2] == 1.1


def test_config_rejects_reflection_and_over_hardware_speed():
    cfg = copy.deepcopy(CONFIG)
    cfg['hand_to_tcp_rotation']['left'] = UNITY_TO_ROS.tolist()
    with pytest.raises(ValueError):
        validate_config(cfg)
    cfg = copy.deepcopy(CONFIG)
    cfg['ik']['velocity_scale'] = 1.01
    with pytest.raises(ValueError):
        validate_config(cfg)


def test_fixed_world_does_not_remove_human_translation():
    body = {1: np.array([0., 1., 0.]), 3: np.array([0., 1.1, 0.]), 10: np.array([-.2, 1.4, 0.]), 15: np.array([.2, 1.4, 0.])}
    align = WorldAlignment(body, CONFIG['world'])
    a = align.points(body)
    moved = {i: p + np.array([0., 0., .5]) for i, p in body.items()}
    b = align.points(moved)
    assert np.linalg.norm(b[1] - a[1]) == pytest.approx(.5)
    assert np.allclose(b[10] - a[10], b[1] - a[1])


@pytest.fixture(scope='module')
def robot(tmp_path_factory):
    from retarget.model import KuavoModel
    path = os.environ.get('KUAVO_RETARGET_TEST_URDF')
    if not path:
        pytest.skip('Set KUAVO_RETARGET_TEST_URDF for real URDF integration checks')
    return KuavoModel(path, tmp_path_factory.mktemp('ik'), CONFIG['ik'])


def safe_q(robot):
    # A synthetic pose, selected independently of recorded demonstration targets.
    q = np.array(CONFIG['ik']['initial_q'])
    goals = [{'position': p, 'rotation': r} for p, r in robot.poses(q, np.zeros(3))]
    return q, goals


def test_real_urdf_hard_pose_and_collision_ik(robot):
    q, goals = safe_q(robot)
    chest, rotation = robot.shoulder_pose(q, np.zeros(3))
    reference = {'position': chest, 'rotation': rotation, 'elbows': [np.array([.3, .3, 1.]), np.array([.3, -.3, 1.])]}
    solution, check = robot.solve(goals, np.zeros(3), reference, q)
    assert check['valid']
    assert solution is not None
    assert robot.plant.num_positions() == 18
    assert robot.metadata['head_joint_values'] == {'zhead_1_joint': 0., 'zhead_2_joint': 0.}
    assert np.allclose(robot.speed[:4], 25.33)
    assert check['collision_violations'] == 0


def test_impossible_pose_and_continuity_fail_closed(robot):
    q, goals = safe_q(robot)
    chest, rotation = robot.shoulder_pose(q, np.zeros(3))
    ref = {'position': chest, 'rotation': rotation, 'elbows': [np.array([.3, .3, 1.]), np.array([.3, -.3, 1.])]}
    goals[0]['position'] = np.array([10., 10., 10.])
    solution, check = robot.solve(goals, np.zeros(3), ref, q, previous=q, velocity=np.zeros(18), dt=.03, velocity_dt=.03)
    assert solution is None and not check['valid']


def test_final_check_rejects_mechanical_limit_violation(robot):
    q, goals = safe_q(robot)
    q[0] = robot.upper[0] + .1
    assert not robot.validate(q, np.zeros(3), goals)['valid']


def test_independent_mujoco_drake_fk_agree(robot, tmp_path):
    import mujoco
    from retarget.preview import render_model
    from retarget.model import JOINT_NAMES, TCP_NAMES
    q, _ = safe_q(robot)
    model = render_model(robot.metadata['source_urdf'], tmp_path)
    data = mujoco.MjData(model)
    for name, angle in zip(JOINT_NAMES, q):
        data.qpos[model.joint(name).qposadr[0]] = angle
    mujoco.mj_forward(model, data)
    for name, (position, rotation) in zip(TCP_NAMES, robot.poses(q, np.zeros(3))):
        body = model.body(name).id
        # MuJoCo's saved XML rounds frame poses; demand sub-micrometer parity.
        assert np.linalg.norm(data.xpos[body] - position) < 1e-6
        assert rotation_error(data.xmat[body].reshape(3, 3), rotation) < 1e-6


def test_final_check_rejects_excessive_acceleration(robot):
    q, _ = safe_q(robot)
    previous = q.copy()
    q[0] += .001
    goals = [{'position': p, 'rotation': r} for p, r in robot.poses(q, np.zeros(3))]
    check = robot.validate(q, np.zeros(3), goals, previous, np.zeros(18), .01, .01)
    assert not check['valid']
    assert abs(check['acceleration_rad_s2'][0]) > robot.accel[0]
