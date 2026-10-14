import copy
import json
import os
from pathlib import Path
import numpy as np
import pytest
from retarget.geometry import LazyBase, PlacementFilter, TorsoReference, UNITY_TO_ROS, adaptive_torso_target, chest_frame, pinch_frame, rotation_error
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
    return KuavoModel(path, tmp_path_factory.mktemp('ik'), dict(CONFIG['ik'], bounded_torso=False))


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


def test_adaptive_centroid_placement_preserves_world_goals():
    body = {1: np.array([0., 0., .8]), 3: np.array([0., 0., 1.]), 10: np.array([0., .25, 1.3]), 15: np.array([0., -.25, 1.3]),
        11: np.array([.2, .25, 1.1]), 16: np.array([.2, -.25, 1.1]), 19: np.array([.4, .25, 1.1]), 45: np.array([.4, -.25, 1.1])}
    geometry = {side: {'shoulder_offset': [0., y, 0.], 'upper_length_m': np.sqrt(.08), 'lower_length_m': .2, 'wrist_to_tcp': [0., 0., 0.]} for side, y in (('left', .25), ('right', -.25))}
    goals = [{'position': body[i].copy() + [0.1, 0., 0.], 'rotation': np.eye(3)} for i in (19, 45)]
    original = [g['position'].copy() for g in goals]
    center, _ = chest_frame(body)
    placement, _, offset = adaptive_torso_target(body, goals, geometry)
    assert np.allclose(offset, [0.1, 0., 0.])
    assert np.allclose(placement, center + [0.1, 0., 0.])
    assert all(np.array_equal(g['position'], p) for g, p in zip(goals, original))


def test_placement_filter_remains_continuous_during_arm_failure():
    f = PlacementFilter(np.zeros(3), np.eye(3), CONFIG['placement'])
    last = f.position.copy()
    for _ in range(60):
        p, rotation = f.update(np.array([.2, 0., -.1]), np.eye(3), .033)
        assert np.linalg.norm(p - last) <= CONFIG['placement']['speed_m_s'] * .033 + 1e-8
        last = p.copy()
    assert p[0] > .1
    assert np.allclose(rotation, np.eye(3))


def test_torso_orientation_cannot_be_sacrificed_to_centroid(robot):
    q, _ = safe_q(robot)
    position, rotation = robot.shoulder_pose(q, np.zeros(3))
    body, check = robot.place_torso(position + [1., 0., -.2], rotation, np.zeros(3), q)
    assert body is not None and check['valid']
    assert check['represented_orientation_error_rad'] <= CONFIG['ik']['torso_orientation_tolerance_rad'] + 1e-5
    assert check['position_residual_m'] > .1


def test_arm_ik_cannot_change_prescribed_body(robot):
    q, goals = safe_q(robot)
    chest, rotation = robot.shoulder_pose(q, np.zeros(3))
    reference = {'position': chest + [0., 0., -.2], 'rotation': rotation, 'elbows': [np.array([.3, .3, 1.]), np.array([.3, -.3, 1.])]}
    solution, check = robot.solve(goals, np.zeros(3), reference, q, fixed_torso=q[:4])
    assert solution is not None and check['valid']
    assert np.max(np.abs(solution[:4] - q[:4])) < 1e-7


def test_geometry_horizon_preserves_fixed_torso_and_checks_future(robot):
    q, goals = safe_q(robot)
    chest, rotation = robot.shoulder_pose(q, np.zeros(3))
    ref = {'position': chest, 'rotation': rotation, 'elbows': [np.array([.3,.3,1.]),np.array([.3,-.3,1.])]}
    targets = [dict(goals=copy.deepcopy(goals), base=np.zeros(3), reference=ref, ticks=i*333333, fixed_torso=q[:4]) for i in range(3)]
    solution, check = robot.solve_horizon(targets,q,previous=q,velocity=np.zeros(18),dt=1/30,velocity_dt=1/30,time_scale=2.)
    assert solution is not None and check['horizon_all_frames_verified']
    assert all(np.max(np.abs(np.asarray(row)[:4]-q[:4]))<1e-7 for row in check['horizon_plan_q'])
    targets[-1]['goals'][0]['position'] = np.array([10.,10.,10.])
    solution, check = robot.solve_horizon(targets,q)
    assert solution is None and not check['valid']


def test_binary_preview_opens_linkage_without_moving_tcp(robot, tmp_path):
    import mujoco
    from retarget.preview import render_model
    from retarget.gripper_signal import linkage_angles
    from retarget.model import JOINT_NAMES, TCP_NAMES
    q, _ = safe_q(robot)
    model = render_model(robot.metadata['source_urdf'], tmp_path, binary_gripper=True)
    data = mujoco.MjData(model)
    for name, angle in zip(JOINT_NAMES,q):
        data.qpos[model.joint(name).qposadr[0]] = angle
    mujoco.mj_forward(model,data)
    positions = [data.xpos[model.body(n).id].copy() for n in TCP_NAMES]
    for side in ('left','right'):
        for name,angle in linkage_angles(side,True).items():
            joint=model.joint(name)
            assert joint.range[0]<=angle<=joint.range[1]
            data.qpos[joint.qposadr[0]]=angle
    mujoco.mj_forward(model,data)
    assert all(np.allclose(p,data.xpos[model.body(n).id]) for p,n in zip(positions,TCP_NAMES))


def test_reach_certificate_includes_tcp_tolerances(robot):
    q, goals=safe_q(robot)
    target=dict(goals=copy.deepcopy(goals),base=np.zeros(3),fixed_torso=q[:4])
    assert robot.precheck_horizon([target],q) is None
    robot.set_q(q)
    shoulder=robot.plant.CalcRelativeTransform(robot.context,robot.plant.world_frame(),robot.plant.GetFrameByName('zarm_l1_link')).translation()
    offset=np.asarray(robot.arm_geometry['left']['wrist_to_tcp'])
    allowance=robot.cfg['tcp_position_tolerance_m']+2*np.linalg.norm(offset)*np.sin(robot.cfg['tcp_orientation_tolerance_rad']/2)
    target['goals'][0]['rotation']=np.eye(3)
    target['goals'][0]['position']=shoulder+np.array([robot.reach_radius['left']+.5*allowance,0.,0.])+robot.tcp_rotations['left'].T@offset
    assert robot.precheck_horizon([target],q) is None
    target['goals'][0]['position']+=np.array([.1,0.,0.])
    assert robot.precheck_horizon([target],q)['reason']=='certified_outside_arm_reach'


def test_bounded_torso_can_adjust_and_rejects_excess(robot):
    q,goals=safe_q(robot)
    base=np.zeros(3)
    chest,rotation=robot.shoulder_pose(q,base)
    reference={'position':chest,'rotation':rotation,'elbows':[np.array([.3,.3,1.]),np.array([.3,-.3,1.])]}
    old=robot.cfg.get('bounded_torso',False)
    robot.cfg['bounded_torso']=True
    try:
        anchor=q[:4].copy(); anchor[3]+=.01
        assert robot.torso_adjust_valid(q,anchor,base,reference)
        bad=q.copy(); bad[3]+=.1
        assert not robot.torso_adjust_valid(bad,anchor,base,reference)
        targets=[dict(goals=copy.deepcopy(goals),base=base,reference=reference,ticks=i*333333,fixed_torso=anchor) for i in range(3)]
        solution,check=robot.solve_horizon(targets,q,previous=q,velocity=np.zeros(18),dt=1/30,velocity_dt=1/30)
        assert solution is not None and check['horizon_all_frames_verified']
        assert all(robot.torso_adjust_valid(np.array(row),anchor,base,reference) for row in check['horizon_plan_q'])
    finally:
        robot.cfg['bounded_torso']=old


def test_braking_rejects_moving_into_limit_but_allows_leaving():
    from retarget.model import KuavoModel
    from pydrake.autodiffutils import InitializeAutoDiff
    assert np.min(KuavoModel.braking_slack(.99,1.,-1.,1.,8.,1/30))<0
    assert np.min(KuavoModel.braking_slack(.99,-.2,-1.,1.,8.,1/30))>0
    x=InitializeAutoDiff(np.array([.99,1.])).reshape(-1)
    s=KuavoModel.braking_slack(x[0],x[1],-1.,1.,8.,1/30)
    assert len(s[0].derivatives())==2


def test_braking_horizon_compiles_and_verifies(robot):
    q,goals=safe_q(robot);base=np.zeros(3);p,r=robot.shoulder_pose(q,base)
    ref={'position':p,'rotation':r,'elbows':[np.array([.3,.3,1.]),np.array([.3,-.3,1.])]}
    old=robot.cfg.get('joint_braking',False);robot.cfg['joint_braking']=True
    try:
        targets=[dict(goals=copy.deepcopy(goals),base=base,reference=ref,ticks=i*333333,fixed_torso=q[:4]) for i in range(3)]
        solution,check=robot.solve_horizon(targets,q,previous=q,velocity=np.zeros(18),dt=1/30,velocity_dt=1/30)
        assert solution is not None and check['horizon_all_frames_verified']
    finally:robot.cfg['joint_braking']=old
