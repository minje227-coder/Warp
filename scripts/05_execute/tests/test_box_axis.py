import importlib.util
from pathlib import Path
import copy,unittest
import numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).with_name('prepare_base_link.py')
if not p.exists():p=Path(__file__).resolve().parents[2]/'02_convert/prepare_base_link.py'
spec=importlib.util.spec_from_file_location('box_axis_prepare',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class BoxAxisTests(unittest.TestCase):
 def pose(self,t=0.,angle=.4):
  R=Rotation.from_euler('y',angle).as_matrix()
  # Canonical tag +Y is box +Z, tag +Z is the outward box face normal.
  tag_R=R@np.array([[1,0,0],[0,0,-1],[0,1,0]])
  return dict(t=t,center=np.array([0.,0.,1.]),rotation=R,down=-tag_R[:,1],half=np.array([.2,.1,.15]),tag_ids=[110],method='single_tag')
 def row(self,t):
  return dict(valid=True,elapsed_s=t,tcp_targets=[dict(position=[0,0,1],rotation=np.eye(3).tolist())],human_points={'body':{'1':[1,2,3]}},base_pose=[0,0,0],q=list(range(18)))
 def test_shift_is_parallel_to_tilted_tag_face_and_exact_length(self):
  rows=[self.row(.5)];old=copy.deepcopy(rows);p=self.pose()
  self.assertEqual(m.lower_along_box_tags(rows,{'box_110_A13':[p]},.07),0)
  offset=np.array(rows[0]['tcp_targets'][0]['position'])-old[0]['tcp_targets'][0]['position']
  np.testing.assert_allclose(offset,.07*p['down'],atol=1e-12)
  self.assertAlmostEqual(np.linalg.norm(offset),.07)
  self.assertGreater(abs(offset[0]),.02)
  for k in ('human_points','base_pose','q'):self.assertEqual(rows[0][k],old[0][k])
  self.assertEqual(rows[0]['tcp_targets'][0]['rotation'],old[0]['tcp_targets'][0]['rotation'])
 def test_last_pose_hold_never_uses_future(self):
  rows=[self.row(.5),self.row(1.5)]
  m.lower_along_box_tags(rows,{'b':[self.pose(0,.1),self.pose(1,.5)]},.07)
  self.assertEqual([r['tcp_targets'][0]['tcp_lowering']['observation_source_s'] for r in rows],[0,1])
 def test_missing_prefix_refused_without_silent_vertical_fallback(self):
  with self.assertRaisesRegex(ValueError,'아래축 없음'):
   m.lower_along_box_tags([self.row(.5)],{'b':[self.pose(1)]},.07)
 def test_hysteresis_prevents_association_flapping(self):
  a=self.pose();b=self.pose();b['center']=a['center']+np.array([.01,0,0]);b['down']=np.array([0.,0.,-1.])
  rows=[self.row(.5),self.row(.6)]
  self.assertEqual(m.lower_along_box_tags(rows,{'a':[a],'b':[b]},.07),0)
  self.assertEqual([r['tcp_targets'][0]['tcp_lowering']['box_id'] for r in rows],['a','a'])

if __name__=='__main__':unittest.main()
