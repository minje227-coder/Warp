import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pipeline'))
from validation_fast import WorldCache, worker_count
from path_tools import audit


def legacy_world(rows, t):
    times = np.array([r['elapsed_s'] for r in rows])
    hi = min(int(np.searchsorted(times,t)),len(times)-1);lo=max(0,hi-1)
    u=0. if hi==lo else float(np.clip((t-times[lo])/(times[hi]-times[lo]),0,1));out=[]
    for a,b in zip(rows[lo]['world'],rows[hi]['world']):
        z=dict(a);z['position']=((1-u)*np.array(a['position'])+u*np.array(b['position'])).tolist()
        z['quaternion_xyzw']=Slerp([0,1],Rotation.from_quat([a['quaternion_xyzw'],b['quaternion_xyzw']]))(u).as_quat().tolist()
        out.append(z)
    return out


class WorldTests(unittest.TestCase):
    def setUp(self):
        self.rows=[]
        for t in [.1,.4,1.]:
            self.rows.append(dict(elapsed_s=t,world=[dict(id='moving_box',position=[t,t*t,1.],
                quaternion_xyzw=Rotation.from_euler('zy',[t*.5,t*.2]).as_quat().tolist(),dimensions=[.2+t,.3,.4],held=t>.5)]))

    def test_cache_matches_old_world_interpolation_exactly(self):
        c=WorldCache(self.rows)
        for t in [-1.,.1,.2,.4,.40000001,.9,1.,2.]:
            self.assertEqual(c.world(t),legacy_world(self.rows,t))

    def test_same_phase_reuses_interpolation_but_changed_phase_does_not(self):
        c=WorldCache(self.rows)
        c.world(.2);c.world(.2)
        self.assertEqual(c.interpolations,1)
        c.world(np.nextafter(.2,1.))
        self.assertEqual(c.interpolations,2)

    def test_dimension_and_metadata_follow_original_lower_keyframe(self):
        c=WorldCache(self.rows)
        self.assertEqual(c.world(.5)[0]['dimensions'],self.rows[1]['world'][0]['dimensions'])
        self.assertEqual(c.world(.5)[0]['held'],self.rows[1]['world'][0]['held'])

    def test_cache_cannot_hide_changed_object_order(self):
        rows=copy.deepcopy(self.rows);rows[1]['world'][0]['id']='other_box'
        with self.assertRaises(ValueError):WorldCache(rows).world(.2)

    def test_invalid_worker_count_rejected(self):
        for n in [0,5,-1]:
            with self.assertRaises(ValueError):worker_count(n)

    def test_worker_count_respects_cpu_affinity(self):
        with patch('validation_fast.os.sched_getaffinity',return_value={0,1}):
            self.assertEqual(worker_count(4),2)


class AuditCacheTests(unittest.TestCase):
    def run_audit(self, values, cache, stats):
        class Model:
            cfg={}
            def set_environment(self,t):pass
            def validate(self,q,base,goals):
                return dict(valid=bool(q[0]>=0))
        class Reference:
            def allowance(self,t):return .01
            def goals(self,t):return []
        return audit(lambda t:np.array([0.,0.,0.,values[t]]),Reference(),Model(),list(values),cache,stats)

    def test_unchanged_states_reused_and_changed_state_rechecked(self):
        cache={};stats=dict(computed=0,reused=0)
        self.assertEqual(len(self.run_audit({0.:-1.,1.:1.},cache,stats)),1)
        self.assertEqual(len(self.run_audit({0.:1.,1.:1.},cache,stats)),0)
        self.assertEqual(stats,dict(computed=3,reused=1))

    def test_cached_failure_remains_a_failure(self):
        cache={};stats=dict(computed=0,reused=0)
        self.run_audit({0.:-1.},cache,stats)
        self.assertEqual(len(self.run_audit({0.:-1.},cache,stats)),1)
        self.assertEqual(stats,dict(computed=1,reused=1))

    def test_one_ulp_change_never_reuses_a_result(self):
        cache={};stats=dict(computed=0,reused=0)
        self.run_audit({0.:1.},cache,stats)
        self.run_audit({0.:np.nextafter(1.,2.)},cache,stats)
        self.assertEqual(stats,dict(computed=2,reused=0))


if __name__=='__main__':unittest.main()
