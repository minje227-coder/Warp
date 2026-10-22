import importlib.util,unittest
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
spec=importlib.util.spec_from_file_location('worker',Path(__file__).parents[1]/'kuavo_execution/standard_rack_worker.py');w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)
class RwhPoseTests(unittest.TestCase):
 def request(self):
  T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',.4).as_matrix();T[:3,3]=[.2,.3,.1]
  known={};tags=[]
  for k,p in enumerate([[0,0,1],[.7,0,1],[.35,0,1.5]]):
   M=np.eye(4);M[:3,3]=p;known[str(k)]=M.tolist();tags.append(dict(id=k,T_camera_tag=(T@M).tolist()))
  return dict(detector='apt_apriltag_ros',base_camera_pose=[0,0,0,0,0,0,1],rack_tags=known,tags=tags),T
 def test_recovers_pose_from_rwh_centers(self):
  r,T=self.request();q=w.solve(r);np.testing.assert_allclose(q['T_camera_recording'],T,atol=1e-12);self.assertLess(q['rms_m'],1e-12)
 def test_single_tag_pose_available_for_projection_only(self):
  r,T=self.request();r['tags']=r['tags'][1:2];r['allow_single_tag_projection']=True
  q=w.solve(r);np.testing.assert_allclose(q['T_camera_recording'],T,atol=1e-12)
  self.assertTrue(q['projection_only']);self.assertIsNone(q['rms_m']);self.assertEqual(q['tag_ids'],[1])
 def test_multiple_tags_keep_existing_fit_with_projection_allowed(self):
  r,T=self.request();r['allow_single_tag_projection']=True;q=w.solve(r)
  self.assertEqual(q['method'],'rwh_planar_tag_positions');self.assertFalse(q.get('projection_only',False))
 def test_rejects_legacy_detector(self):
  r,_=self.request();r['detector']='up_opencv_subpixel'
  with self.assertRaises(ValueError):w.solve(r)
 def test_rejects_single_tag_and_bad_fit(self):
  r,_=self.request();r['tags']=r['tags'][:1]
  with self.assertRaises(ValueError):w.solve(r)
  r,_=self.request();r['tags'][0]['T_camera_tag'][0][3]+=1
  with self.assertRaises(ValueError):w.solve(r)
if __name__=='__main__':unittest.main()
