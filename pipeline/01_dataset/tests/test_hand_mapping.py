import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from retarget.geometry import rotation_error
from retarget.hand_mapping import calibrate_hand, hand_candidates, mcp_pinch_frame, palm_frame, tcp_frame


def hand():
    return {1: np.zeros(3), 3: np.array([-.03, 0., .025]),
        5: np.array([-.02, 0., .11]), 7: np.array([-.015, 0., .07]),
        10: np.array([-.015, 0., .15]), 17: np.array([.025, 0., .06])}


def test_finger_closure_cannot_change_palm_orientation_or_fixed_tcp():
    p = hand();correction = np.diag([-1., 1., -1.])
    calibration = calibrate_hand([p], correction)
    before = hand_candidates(p, calibration, correction)
    bent = copy.deepcopy(p)
    bent[10] += [.01, -.035, -.03];bent[5] = bent[10] + [.006, 0., 0.]
    after = hand_candidates(bent, calibration, correction)
    assert np.allclose(before['palm_fixed']['position'], after['palm_fixed']['position'])
    assert rotation_error(before['pinch_palm']['rotation'], after['pinch_palm']['rotation']) < 1e-8
    assert rotation_error(before['original_pinch']['rotation'], after['original_pinch']['rotation']) > .1
    assert not np.allclose(before['pinch_palm']['position'], after['pinch_palm']['position'])
    assert after['pinch_width_m'] == pytest.approx(.006)


def test_rigid_hand_motion_transfers_without_refitting_reference():
    p = hand();c = np.diag([-1., 1., -1.]);cal = calibrate_hand([p], c)
    before = hand_candidates(p, cal, c)
    r = Rotation.from_euler('xyz', [.3, -.6, .2]).as_matrix();t = np.array([.2, -.5, .8])
    after = hand_candidates({j: r @ v + t for j, v in p.items()}, cal, c)
    for mode in ('pinch_palm', 'palm_fixed', 'original_pinch'):
        assert np.allclose(after[mode]['position'], r @ before[mode]['position'] + t)
        assert np.allclose(after[mode]['rotation'], r @ before[mode]['rotation'])
    assert after['pinch_width_m'] == pytest.approx(before['pinch_width_m'])


def test_degenerate_palm_rejected_and_left_right_frames_proper():
    p = hand()
    for reflection in (np.eye(3), np.diag([-1., 1., 1.])):
        r = palm_frame({j: reflection @ v for j, v in p.items()})
        assert np.linalg.det(r) == pytest.approx(1.)
        assert np.allclose(r.T @ r, np.eye(3))
    p[17] = p[7].copy()
    with pytest.raises(ValueError, match='degenerate'):
        palm_frame(p)


def test_mcp_pinch_axes_and_closed_fingertips_are_independent():
    p = hand();r = mcp_pinch_frame(p)
    assert np.allclose(r[:, 0], (p[7]-p[3])/np.linalg.norm(p[7]-p[3]))
    cal = calibrate_hand([p], np.diag([-1., 1., -1.]), 'mcp_pinch')
    p[5] = p[10].copy()
    after = hand_candidates(p, cal, np.diag([-1., 1., -1.]))
    assert np.allclose(r @ np.diag([-1., 1., -1.]), after['pinch_palm']['rotation'])
    assert after['closing_axis_tip_chord_deg'] is None
    assert np.linalg.det(r) == pytest.approx(1.)


def test_selected_tcp_mapping_survives_degenerate_old_index_axis():
    p=hand();p[10]=p[7].copy();p[5]=p[10].copy()
    center,rotation,width=tcp_frame(p,np.diag([-1.,1.,-1.]),'mcp_pinch')
    assert width==0 and np.allclose(center,p[7])
    assert np.linalg.det(rotation)==pytest.approx(1.)
    with pytest.raises(ValueError,match='degenerate'):
        tcp_frame(p,np.eye(3),'legacy_pinch')


def test_calibrated_palm_rejects_thumb_opposition_as_wrist_rotation():
    from retarget.hand_mapping import calibrate_palm_axes
    p=hand();c=np.diag([-1.,1.,-1.])
    fitted,meta=calibrate_palm_axes([p]*10,c)
    before=tcp_frame(p,fitted,'palm_calibrated')
    moved=copy.deepcopy(p);moved[3]+=[-.025,.02,-.01];moved[5]=moved[10]+[.003,0,0]
    after=tcp_frame(moved,fitted,'palm_calibrated')
    assert rotation_error(before[1],after[1])<1e-8
    assert rotation_error(before[1],mcp_pinch_frame(p) @ c)<1e-8
    assert after[2]==pytest.approx(.003)
    rr=Rotation.from_euler('xyz',[.3,-.4,.1]).as_matrix()
    rigid=tcp_frame({i:rr @ v+np.array([.1,.2,.3]) for i,v in p.items()},fitted,'palm_calibrated')
    assert np.allclose(rigid[1],rr @ before[1])


def test_index_palm_direction_uses_proximal_index_and_palm_not_thumb():
    from retarget.hand_mapping import index_palm_frame
    p=hand();p[8]=np.array([-.015,0.,.11])
    c=np.diag([-1.,1.,-1.]);before=tcp_frame(p,c,'index_palm')
    changed=copy.deepcopy(p);changed[3]+=[-.03,.04,-.02];changed[5]=changed[10]+[.003,0,0]
    after=tcp_frame(changed,c,'index_palm')
    assert rotation_error(before[1],after[1])<1e-8
    assert np.allclose(-before[1][:,2],(p[8]-p[7])/np.linalg.norm(p[8]-p[7]))
    rr=Rotation.from_euler('xyz',[.3,-.4,.1]).as_matrix()
    moved=tcp_frame({i:rr @ v+np.array([.2,.1,-.3]) for i,v in p.items()},c,'index_palm')
    assert np.allclose(moved[1],rr @ before[1])
    changed[8]=changed[7].copy()
    with pytest.raises(ValueError,match='degenerate'):
        index_palm_frame(changed)


def test_orientation_filter_is_equivariant_and_caps_lag():
    from retarget.hand_mapping import HandOrientationFilter
    a=HandOrientationFilter(.04,.05235987756);b=HandOrientationFilter(.04,.05235987756)
    world=Rotation.from_euler('xyz',[.2,-.4,.8]).as_matrix()
    for i in range(20):
        raw=Rotation.from_euler('xyz',[.1*i,(-1)**i*.08,.03*i]).as_matrix()
        filtered=a.update(raw,None if i==0 else 1/30)
        transformed=b.update(world @ raw,None if i==0 else 1/30)
        assert rotation_error(filtered,raw)<=.05235987756+1e-8
        assert np.allclose(transformed,world @ filtered)
    a.rotation=None
    raw=Rotation.from_euler('x',1.2).as_matrix()
    assert np.allclose(a.update(raw),raw)
