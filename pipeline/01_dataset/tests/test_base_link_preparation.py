import importlib.util
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

p=Path(__file__).resolve().parents[3]/'scripts/02_convert/prepare_base_link.py'
if not p.exists(): p=Path(__file__).with_name('prepare_base_link.py')
s=importlib.util.spec_from_file_location('prepare_base_link',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)

class TransformTest(unittest.TestCase):
 def test_relative_geometry_and_local_axes(self):
  T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.03,.12,.4]).as_matrix();T[:3,3]=[.1,-.2,.17]
  r={'tcp_targets':[{'position':[.4,.2,1.],'rotation':np.eye(3).tolist(),'body_down_direction':[0,0,-1],'camera_mount_side_axis_tcp':[1,0,0]}], 'base_pose':[.1,.2,.3], 'elapsed_s':3., 'q':[1,2]}
  w={'samples':[{'world':[{'id':'box','position':[.5,.2,1.],'quaternion_xyzw':[0,0,0,1]},{'id':'ground','position':[0,0,-.05]}]}]}
  a=m.transform_row(r,T);b=m.transform_world(w,T)
  delta=np.array(b['samples'][0]['world'][0]['position'])-a['tcp_targets'][0]['position']
  np.testing.assert_allclose(delta,T[:3,:3]@np.array([.1,0,0]),atol=1e-12)
  self.assertEqual(a['tcp_targets'][0]['camera_mount_side_axis_tcp'],[1,0,0])
  self.assertEqual(a['elapsed_s'],3.);self.assertEqual(a['q'],r['q'])
  self.assertEqual(b['samples'][0]['world'][1]['position'],[0,0,-.05])
  self.assertEqual(r['tcp_targets'][0]['position'],[.4,.2,1.])
 def test_inverse_restores_goals(self):
  T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.1,.2,.3]).as_matrix();T[:3,3]=[1,2,3]
  r={'torso_placement_target':{'position':[.1,.2,1.2],'rotation':np.eye(3).tolist()},'human_points':{'body':{'1':[.2,.3,.4]}},'tcp_targets':[]}
  b=m.transform_row(m.transform_row(r,T),np.linalg.inv(T))
  np.testing.assert_allclose(b['torso_placement_target']['position'],r['torso_placement_target']['position'],atol=1e-12)
  np.testing.assert_allclose(b['human_points']['body']['1'],r['human_points']['body']['1'],atol=1e-12)
 def test_rejects_reflection_nan_and_scale(self):
  for val in [-1,2,float('nan')]:
   T=np.eye(4);T[0,0]=val
   with self.assertRaises(ValueError):m.transform_matrix(T)

if __name__=='__main__':unittest.main()
