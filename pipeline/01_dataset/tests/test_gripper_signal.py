import numpy as np
from retarget.gripper_signal import binary_signals, linkage_angles


def rows(widths):
    return [{'index':i,'elapsed_s':i*.033,'hands':{s:{'valid':w is not None,
        'reason':'lost','candidates':{'pinch_width_m':w}} for s in ('left','right')}} for i,w in enumerate(widths)]


def test_binary_outputs_and_hysteresis_survive_tracking_loss():
    data=rows([.10]*30+[.02]*30+[.06]*8+[None]+[.06]*8+[.10]*30)
    signals,_=binary_signals(data)
    for side in ('left','right'):
        assert signals[29]['hands'][side]['state']=='open'
        assert signals[59]['hands'][side]['state']=='closed'
        assert signals[67]['hands'][side]['state']=='closed'
        assert not signals[68]['hands'][side]['valid']
        assert not signals[69]['hands'][side]['valid']
        assert signals[-1]['hands'][side]['state']=='open'
        assert all(r['hands'][side]['open_target'] in (0,1,None) for r in signals if 'open_target' in r['hands'][side])


def test_linkage_is_mirrored_and_fixed_joints_not_commanded():
    q=linkage_angles('left',True)
    assert len(q)==6
    assert q['l_f_bar_1_joint']==-q['l_b_bar_1_joint']
    assert all(abs(a)<=.698 for a in q.values())
    assert not any('bar_2' in n or 'finger_joint' in n for n in q)
    assert np.allclose(list(linkage_angles('left',False).values()),0)
