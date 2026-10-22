import importlib.util
from pathlib import Path
import unittest
import tempfile,json,hashlib,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parents[1]
spec=importlib.util.spec_from_file_location('grasp_policy',ROOT/'kuavo_execution/grasp_policy.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
spec=importlib.util.spec_from_file_location('grasp_replay',ROOT/'kuavo_replay.py');worker=importlib.util.module_from_spec(spec);spec.loader.exec_module(worker)
spec=importlib.util.spec_from_file_location('grasp_initial',ROOT/'kuavo_execution/first_state.py');initial=importlib.util.module_from_spec(spec);spec.loader.exec_module(initial)
BOX=dict(id='box_A',position=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.],dimensions=[.265,.185,.222])
class Tests(unittest.TestCase):
 def close(self,latch):
  for t in np.arange(0,.31,.02):r=latch.update(float(t),.3,[.14,0,.09],[BOX])
  self.assertTrue(r['closed']);return .30
 def test_depth_not_strict_xy_inside(self):
  s=g.Latch();self.close(s)
 def test_above_rim_does_not_close(self):
  s=g.Latch()
  for t in np.arange(0,1,.02):r=s.update(float(t),.1,[0,0,.13],[BOX])
  self.assertFalse(r['closed']);self.assertEqual(r['open_fraction'],1.)
 def test_far_away_at_same_depth_does_not_close(self):
  s=g.Latch()
  for t in np.arange(0,1,.02):r=s.update(float(t),.1,[1,0,.09],[BOX])
  self.assertFalse(r['closed'])
 def test_aperture_chatter_never_releases(self):
  s=g.Latch();self.close(s)
  for i,t in enumerate(np.arange(.32,3,.02)):r=s.update(float(t),[.39,.41,.69,.71,.95][i%5],[.14,0,.09],[BOX])
  self.assertTrue(r['closed'])
 def test_rigid_translation_and_rotation_do_not_release(self):
  s=g.Latch();self.close(s)
  for t in np.arange(.32,2,.02):
   q=[0,0,math.sin(t/2),math.cos(t/2)];R=g.quaternion_matrix(q);p=np.array([t/2,-t/3,t/4]);b=dict(BOX,position=p.tolist(),quaternion_xyzw=q);r=s.update(float(t),.9,(R@np.array([.14,0,.09])+p).tolist(),[b])
  self.assertTrue(r['closed'])
 def test_closed_fingers_withdrawing_do_not_release(self):
  s=g.Latch();self.close(s)
  for t in np.arange(.32,2,.02):r=s.update(float(t),.3,[.14,0,.09+.1*(t-.30)],[BOX])
  self.assertTrue(r['closed'])
 def test_open_and_withdraw_release(self):
  s=g.Latch();self.close(s)
  events=[]
  for t in np.arange(.32,2,.02):
   r=s.update(float(t),.85,[.14,0,.09+.1*(t-.30)],[BOX]);events.append(r['reason'])
  self.assertFalse(r['closed']);self.assertIn('release_open_and_relative_withdrawal',events)
 def test_single_tracking_jump_is_not_persistent_withdrawal(self):
  s=g.Latch();self.close(s)
  # A one-frame jump then a static offset must not satisfy persistent motion.
  for t in np.arange(.32,1,.02):r=s.update(float(t),.9,[.14,0,.15],[BOX])
  # Windowed velocity can retain a jump for .2 s, therefore confirmation must
  # use consecutive per-frame movement in addition to a windowed estimate.
  self.assertTrue(r['closed'])
 def test_missing_geometry_holds_attached_box(self):
  s=g.Latch();self.close(s)
  for t in np.arange(.32,2,.02):r=s.update(float(t),1.,[0,0,.4],[])
  self.assertTrue(r['closed']);self.assertEqual(r['reason'],'hold_missing_geometry')
 def test_box_association_is_latched(self):
  s=g.Latch();self.close(s)
  b=dict(BOX,id='box_B',position=[.1,0,0])
  for t in np.arange(.32,1,.02):r=s.update(float(t),.65,[.14,0,.09],[b,BOX])
  self.assertEqual(r['box_id'],'box_A')
 def test_short_pulse_cannot_release(self):
  s=g.Latch();self.close(s)
  for t in np.arange(.32,.34,.02):s.update(float(t),.95,[.14,0,.09+.8*(t-.30)],[BOX])
  r=s.update(.34,.3,[.14,0,.16],[BOX]);self.assertTrue(r['closed'])
 def test_release_cannot_immediately_reclose_in_overlap(self):
  s=g.Latch();self.close(s)
  for t in np.arange(.32,.8,.02):r=s.update(float(t),.6,[.14,0,.09+.03*(t-.30)],[BOX])
  self.assertFalse(r['closed'])
  for t in np.arange(.8,1.5,.02):r=s.update(float(t),.6,[.14,0,.105],[BOX])
  self.assertFalse(r['closed'])
 def test_reinserted_hand_can_close_again(self):
  s=g.Latch();self.close(s)
  for t in np.arange(.32,1.1,.02):r=s.update(float(t),.6,[.14,0,.09+.1*(t-.30)],[BOX])
  self.assertFalse(r['closed'])
  for t in np.arange(1.1,1.5,.02):r=s.update(float(t),.6,[.14,0,.09],[BOX])
  self.assertTrue(r['closed'])



class Contracts(unittest.TestCase):
 def test_known_quaternion_rotation(self):
  R=g.quaternion_matrix([0,0,math.sqrt(.5),math.sqrt(.5)])
  np.testing.assert_allclose(R,[[0,-1,0],[1,0,0],[0,0,1]],atol=1e-12)
 def test_frozen_helper_and_standard_metadata_match(self):
  frozen=REPO/'pipeline/01_dataset/kuavo_standard'
  self.assertEqual((ROOT/'kuavo_execution/grasp_policy.py').read_bytes(),(frozen/'pipeline/source/retarget/box_grasp.py').read_bytes())
  policy=json.loads((frozen/'STANDARD.json').read_text())['workflow']['gripper']
  for key,value in g.POLICY.items():self.assertEqual(policy[key],value,key)
 def fixture(self,root):
  provider=root/'provider';(provider/'data').mkdir(parents=True);(root/'data').mkdir();(root/'results').mkdir()
  T=np.eye(4);T[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];T[:3,3]=[2,3,4]
  (root/'POLICY.json').write_text(json.dumps(dict(source_candidate=str(provider))))
  (root/'data/source-config.json').write_text(json.dumps(dict(coordinate_preparation=dict(T_base_recording=T.tolist()))))
  rows=[];scenes=[]
  for i,t in enumerate([0.,.1,.2]):
   hp={}
   for kind,z in [('XRHandLeft',.09),('XRHandRight',.15)]:
    hp[kind]={str(k):(T[:3,:3]@np.array([.14,y,z])+T[:3,3]).tolist() for k,y in [(5,-.02),(10,.02)]}
   rows.append(dict(valid=True,elapsed_s=t,external_index=i,human_points=hp,grippers=[dict(open_fraction=.3)]*2))
   scenes.append(dict(external_index=i,world=[dict(BOX,id='box_fixture_A13')]))
  (root/'data/source-trajectory.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
  (provider/'data/frame-scenes.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in scenes))
  times=np.arange(0,.201,.01);names=['base_x','base_y','base_yaw']+worker.NAMES;zeros=np.zeros((len(times),21))
  np.savez(root/'results/execution-trajectory-100hz.npz',execution_s=times,source_s=times,position=zeros,velocity=zeros,acceleration=zeros,names=names)
  (root/'REPORT.json').write_text(json.dumps(dict(status='verified',validation=dict(summary=dict(joint_peaks=[dict(name=n,velocity_limit=1.,acceleration_limit=1.) for n in names])))))
  return root
 def test_coordinate_transform_and_discrete_seek(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=self.fixture(Path(tmp));tl=g.load(root)
   self.assertEqual(g.sample(tl,0.),[1.,1.]);self.assertEqual(g.sample(tl,.1),[0.,1.])
   self.assertEqual(g.at_execution(root,.15),[0.,1.])
   self.assertEqual(g.sample(tl,0.),[1.,1.])
   np.testing.assert_array_equal(g.sample_many(tl,[0.,.099,.1,.15]),[[1,1],[1,1],[0,1],[0,1]])
 def test_package_first_state_and_preview_share_states_without_changing_body(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=self.fixture(Path(tmp));paths=[root/'results/execution-trajectory-100hz.npz',root/'data/source-trajectory.jsonl',root/'REPORT.json'];before=[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]
   stats=tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in paths)
   payload=worker.build_package(('fixture',stats,(0.,.2),False,g.signature(root)))
   replay=worker.ReplayPath(payload)
   for t in [0.,.099,.1,.105,.15,.2]:
    fractions=g.at_execution(root,t)
    self.assertEqual(replay.sample(t)[3],[100*(1-x) for x in fractions])
    detail=dict(initial=dict(q=[0.]*18,joint_names=initial.NAMES,base_pose=[0,0,0],source_s=t,grippers=[dict(open_fraction=x) for x in fractions],gripper_policy=g.POLICY))
    self.assertEqual(initial.target(detail)['claw'],replay.sample(t)[3])
    for values in replay.sample(t)[:3]:self.assertEqual(values,[0.]*21)
   self.assertEqual(before,[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths])
   old_signature=g.signature(root)
   (root/'data/source-config.json').write_text('{}')
   with self.assertRaisesRegex(ValueError,'입력이 바뀌었습니다'):
    worker.build_package(('fixture',stats,(0.,.2),False,old_signature))

if __name__=='__main__':unittest.main()
