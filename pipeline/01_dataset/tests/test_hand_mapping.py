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
