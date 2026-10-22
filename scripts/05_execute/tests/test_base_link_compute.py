import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if not (ROOT/'kuavo_execution').exists(): ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('base_compute_test_backend',ROOT/'kuavo_execution/__init__.py',submodule_search_locations=[str(ROOT/'kuavo_execution')])
import sys
package=importlib.util.module_from_spec(spec);sys.modules[spec.name]=package;spec.loader.exec_module(package)
from importlib import import_module
compute=import_module(spec.name+'.compute')
calibration=import_module(spec.name+'.calibration')
catalog=import_module(spec.name+'.catalog')

class FreshObservationTests(unittest.TestCase):
    def sample(self):
        return dict(received=time.time(),image_age_s=0.01,tf_ready=True,base_camera_pose=[0,0,1,0,0,0,1],tags=[{'id':1}])
    def test_transformed_boxes_keep_relative_geometry_and_raw_input(self):
        boxes=[dict(position=[1,2,3],quaternion_xyzw=[0,0,0,1],dimensions=[1,2,3])]
        T=np.eye(4);T[:3,3]=[.1,.2,.3]
        out=catalog.transform_boxes(boxes,T)
        np.testing.assert_allclose(out[0]['position'],[1.1,2.2,3.3]);self.assertEqual(boxes[0]['position'],[1,2,3])
        self.assertEqual(out[0]['dimensions'],boxes[0]['dimensions'])

    def test_quaternion_conversion_preserves_rotations_near_half_turn(self):
        from importlib import import_module
        pose_to_T=import_module(spec.name+'.head_stream').pose_to_T
        for q in ([1,0,0,0],[0,1,0,0],[0,0,1,0],[.2,.3,.4,.5]):
            M=pose_to_T([0,0,0]+list(q))[:3,:3]
            converted=catalog.rotation_quaternion(M)
            np.testing.assert_allclose(pose_to_T([0,0,0]+converted)[:3,:3],M,atol=1e-12)

class QueueTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        root=Path(self.tmp.name);self.catalog=root/'catalog.json';self.jobs=root/'compute'
        prepared=root/'prepared.json';prepared.write_text('{}')
        self.catalog.write_text(json.dumps(dict(episodes=[dict(id=k,prepared=str(prepared)) for k in ('a','b')])))
        for name,value in [('CATALOG',self.catalog),('JOBS',self.jobs)]:
            p=patch.object(compute,name,value);p.start();self.addCleanup(p.stop)
    def test_rejection_does_not_modify_queue(self):
        with patch.object(compute,'capture_calibration',side_effect=ValueError('no tags')):
            with self.assertRaises(ValueError):compute.enqueue(['a'])
        self.assertFalse((self.jobs/'jobs.json').exists())
    def test_batch_is_atomic_when_second_episode_unobservable(self):
        with patch.object(compute,'capture_calibration',side_effect=[dict(captured_at=1),ValueError('no tags')]):
            with self.assertRaises(ValueError):compute.enqueue(['a','b'])
        self.assertFalse((self.jobs/'jobs.json').exists())
    def test_latest_slot_replaced_and_calibration_required_each_time(self):
        with patch.object(compute,'capture_calibration',return_value=dict(captured_at=123)) as capture:
            compute.enqueue(['a']);first=compute.jobs()['a']['id']
            compute.update('a',status='done');compute.enqueue(['a'])
            rows=compute.jobs();self.assertEqual(len(rows),1);self.assertNotEqual(first,rows['a']['id'])
            self.assertEqual(capture.call_count,2);self.assertEqual(rows['a']['coordinate_frame'],'fixed_rack_height')
    def test_worker_rechecks_before_creating_input_or_starting_solver(self):
        with patch.object(compute,'PROTECTED',()),patch.object(compute,'update'),patch.object(compute,'capture_calibration',side_effect=ValueError('no fresh tags')),patch.object(compute,'prepare') as prepare,patch.object(compute,'run_checked') as run:
            with self.assertRaises(ValueError):compute.worker('a','a'*32)
            prepare.assert_not_called();run.assert_not_called()

if __name__=='__main__':unittest.main()
