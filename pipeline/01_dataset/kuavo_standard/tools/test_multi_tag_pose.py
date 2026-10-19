import unittest
from itertools import combinations
import cv2
import numpy as np
from multi_tag_pose import estimate_tag_bundle, square

class BundlePoseTests(unittest.TestCase):
 def setUp(self):
  self.K=np.array([[866.,0,638.],[0,866.,633.],[0,0,1.]])
  self.Rwc=np.array([[1.,0,0],[0,0,1],[0,-1,0]])
  self.twc=np.array([.2,-.1,1.3])
  self.R=cv2.Rodrigues(np.array([.12,-.08,.04]))[0]@np.diag([1.,-1.,-1.])
  self.t=np.array([.02,-.15,1.4])
  self.rack={0:np.array([-.38,0,.05]),1:np.array([.38,0,.05]),2:np.array([-.38,-.61,.05]),3:np.array([.38,-.61,.05]),4:np.array([0.,0,.05]),5:np.array([0.,-.61,.05])}
  self.box={100:np.array([-.0975,.076,0]),101:np.array([.0975,.076,0])}
 def detections(self,centers,size,noise=0,seed=0):
  rng=np.random.default_rng(seed);result={}
  for tid,center in centers.items():
   uv=cv2.projectPoints(square(size)+center,cv2.Rodrigues(self.R)[0],self.t,self.K,None)[0].reshape(4,2)
   uv+=rng.normal(0,noise,uv.shape)
   result[tid]={'corners':uv[[3,2,1,0]].tolist()}
  return result
 def solve(self,ds,centers,size):
  return estimate_tag_bundle(ds,centers,size,self.K,self.Rwc,self.twc)
 def assert_pose(self,q,tolerance=1e-7):
  self.assertIsNotNone(q)
  np.testing.assert_allclose(q['position'],self.Rwc@self.t+self.twc,atol=tolerance)
  np.testing.assert_allclose(q['rotation'],self.Rwc@self.R,atol=tolerance)
 def test_every_rack_subset_and_box_joint(self):
  for n in range(1,7):
   for ids in combinations(self.rack,n):
    with self.subTest(ids=ids):
     centers={tid:self.rack[tid] for tid in ids};q=self.solve(self.detections(centers,.04),self.rack,.04);self.assert_pose(q)
     self.assertEqual(q['method'],'single_tag' if n==1 else 'joint_multi_tag')
     self.assertEqual(q['tag_ids'],list(ids));self.assertEqual(q['corner_count'],4*n)
  q=self.solve(self.detections(self.box,.06),self.box,.06);self.assert_pose(q)
  self.assertEqual(q['tag_ids'],[100,101]);self.assertEqual(q['corner_count'],8)
 def test_three_tags_include_all_twelve_corners(self):
  ds=self.detections({k:self.rack[k] for k in [0,4,1]},.04,noise=.15)
  q=self.solve(ds,self.rack,.04);self.assertEqual(q['method'],'joint_multi_tag')
  self.assertEqual(q['tag_ids'],[0,1,4]);self.assertEqual(q['corner_count'],12)
 def test_corrupted_tag_excluded_as_a_whole(self):
  ds=self.detections(self.rack,.04,noise=.1)
  ds[4]['corners']=(np.array(ds[4]['corners'])+[25.,-30.]).tolist()
  q=self.solve(ds,self.rack,.04)
  self.assertEqual(q['method'],'joint_multi_tag');self.assertEqual(q['rejected_tag_ids'],[4])
  self.assertEqual(q['tag_ids'],[0,1,2,3,5]);self.assertEqual(q['corner_count'],20)
  self.assert_pose(q,tolerance=.005)
 def test_one_corrupted_corner_rejects_whole_tag(self):
  ds=self.detections(self.rack,.04,noise=.05);ds[5]['corners'][2][0]+=35
  q=self.solve(ds,self.rack,.04)
  self.assertEqual(q['method'],'joint_multi_tag');self.assertEqual(q['rejected_tag_ids'],[5])
  self.assertEqual(q['corner_count'],20)
 def test_inconsistent_two_tag_layout_is_explicit_fallback(self):
  ds=self.detections(self.box,.06);ds[101]['corners']=(np.array(ds[101]['corners'])+[35.,25.]).tolist()
  q=self.solve(ds,self.box,.06)
  self.assertEqual(q['method'],'single_tag_fallback');self.assertEqual(q['corner_count'],4)
  self.assertEqual(q['observed_tag_ids'],[100,101]);self.assertIn('joint_failure_reason',q)
 def test_no_known_tags_or_nonfinite_input(self):
  self.assertIsNone(self.solve({},self.rack,.04))
  self.assertIsNone(self.solve({100:{'corners':[[0,0]]*4}},self.rack,.04))
  ds=self.detections({0:self.rack[0]},.04);ds[0]['corners'][0][0]=float('nan')
  self.assertIsNone(self.solve(ds,self.rack,.04))
 def test_single_tag_matches_existing_ippe(self):
  for centers,size in [(self.rack,.04),(self.box,.06)]:
   ds=self.detections(centers,size,noise=.15)
   for tid in centers:
    uv=np.array(ds[tid]['corners'])[[3,2,1,0]]
    _,rvs,tvs,_=cv2.solvePnPGeneric(square(size),uv,self.K,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates=[]
    for rv,tv in zip(rvs,tvs):
     R=cv2.Rodrigues(rv)[0];t=tv.ravel();rw=self.Rwc@R
     if t[2]<=0 or R[:,2]@(-t)<=0 or rw[2,1]<.65:continue
     rms=float(np.sqrt(np.mean(np.sum((cv2.projectPoints(square(size),rv,tv,self.K,None)[0].reshape(4,2)-uv)**2,axis=1))))
     candidates.append(dict(rotation=rw,position=self.Rwc@t+self.twc-rw@centers[tid],rms=rms))
    old=min(candidates,key=lambda q:q['rms']);new=self.solve({tid:ds[tid]},centers,size)
    np.testing.assert_array_equal(old['position'],new['position']);np.testing.assert_array_equal(old['rotation'],new['rotation'])
 def test_noisy_joint_reduces_position_error(self):
  joint=[];single=[]
  for seed in range(50):
   ds=self.detections(self.rack,.04,noise=.2,seed=seed)
   q=self.solve(ds,self.rack,.04);self.assertEqual(q['method'],'joint_multi_tag')
   joint.append(np.linalg.norm(q['position']-(self.Rwc@self.t+self.twc)))
   singles=[self.solve({tid:ds[tid]},self.rack,.04) for tid in self.rack]
   best=min(singles,key=lambda q:q['rms']);single.append(np.linalg.norm(best['position']-(self.Rwc@self.t+self.twc)))
  self.assertLess(np.median(joint),np.median(single))


class CalibratedBoxTests(unittest.TestCase):
 def setUp(self):
  from pathlib import Path
  import json
  self.base=BundlePoseTests();self.base.setUp()
  root=Path(__file__).resolve().parent.parent
  self.config=json.loads((root/"object_overlay_config.json").read_text())
  self.cal=json.loads((root/"box_tag_calibration.json").read_text())
  self.centers={int(k):np.array(v) for k,v in self.config["box"]["tag_offsets_in_front_face_axes_m"].items()}
  self.angles={int(k):v for k,v in self.config["box"]["tag_inplane_rotations_rad"].items()}
 def detections(self):
  result={}
  for tid,center in self.centers.items():
   relative=cv2.Rodrigues(np.array([0.,0.,self.angles[tid]]))[0]
   points=square(.06)@relative.T+center
   uv=cv2.projectPoints(points,cv2.Rodrigues(self.base.R)[0],self.base.t,self.base.K,None)[0].reshape(4,2)
   result[tid]={"corners":uv[[3,2,1,0]].tolist()}
  return result
 def solve(self,ds):
  b=self.base
  return estimate_tag_bundle(ds,self.centers,.06,b.K,b.Rwc,b.twc,tag_inplane_rotations_rad=self.angles)
 def test_rotated_right_tag_joint_recovers_box_frame(self):
  q=self.solve(self.detections());self.base.assert_pose(q)
  self.assertEqual(q["method"],"joint_multi_tag");self.assertEqual(q["corner_count"],8)
 def test_rotated_right_tag_alone_recovers_box_frame(self):
  ds=self.detections();q=self.solve({101:ds[101]});self.base.assert_pose(q)
  self.assertEqual(q["method"],"single_tag");self.assertEqual(q["tag_ids"],[101])
 def test_left_tag_anchor_is_preserved(self):
  ds=self.detections();q=self.solve({100:ds[100]});self.base.assert_pose(q)
  np.testing.assert_array_equal(self.centers[100],[-.0975,.076,0.])
 def test_configuration_matches_approved_fixed_candidate(self):
  np.testing.assert_allclose(self.centers[101]-self.centers[100],self.cal["relative_translation_101_in_tag100_axes_m"],atol=1e-16)
  self.assertEqual(self.angles[101],self.cal["relative_inplane_rotation_101_rad"])
  self.assertEqual(self.cal["selected_review_candidate"],"early_to_late")
  self.assertFalse(self.cal["time_dependent_switching"])
  self.assertEqual(self.config["box"]["size_m"],[.265,.185,.222])

if __name__=='__main__':unittest.main(verbosity=2)
