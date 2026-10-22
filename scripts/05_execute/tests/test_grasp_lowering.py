import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
PREPARER=ROOT/'prepare_base_link.py'
COMPUTE=ROOT/'compute.py'
if not PREPARER.exists():
 PREPARER=ROOT.parents[1]/'02_convert/prepare_base_link.py'
 COMPUTE=ROOT.parent/'kuavo_execution/compute.py'
spec=importlib.util.spec_from_file_location('lowering_prepare', PREPARER)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
spec=importlib.util.spec_from_file_location('lowering_compute', COMPUTE)
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)

class LoweringTests(unittest.TestCase):
 def test_box_axis_after_calibration_and_originals_untouched(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);episode=root/'take';episode.mkdir()
   T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.2,.3,.7]).as_matrix();T[:3,3]=[.3,.1,.2]
   row=dict(valid=True,tcp_targets=[dict(position=[.2,.3,1.],rotation=np.eye(3).tolist(),camera_mount_side_axis_tcp=[1,0,0]),dict(position=[.4,.3,1.],raw_position=[.4,.3,1.],rotation=np.eye(3).tolist())],base_pose=[0,0,0],elapsed_s=1.,grippers=[dict(open_fraction=.4)],human_points={'body':{'1':[1,2,3]}},chest_target=dict(position=[1,2,3]))
   world={'samples':[dict(world=[dict(id='box',position=[.4,.3,1.],quaternion_xyzw=[0,0,0,1]),dict(id='ground',position=[0,0,-.05])])]}
   (root/'reference.jsonl').write_text(json.dumps(row)+'\n');(root/'world.json').write_text(json.dumps(world));(root/'config.json').write_text(json.dumps(dict(ik={})))
   (root/'prepared.json').write_text(json.dumps(dict(episode=str(episode),reference='reference.jsonl',world='world.json',reference_config='config.json')))
   (root/'cal.json').write_text(json.dumps(dict(episode_id='take',T_base_recording=T.tolist())))
   direction=np.array([.6,0.,-.8]);track=dict(t=0.,center=np.array([0.,0.,1.]),rotation=np.eye(3),half=np.ones(3),down=direction,tag_ids=[110],method="test")
   outputs=[]
   for value in (0,7):
    target=root/str(value)
    with patch.object(m,'box_tag_tracks',return_value=({'box':[track]},{})):m.prepare(root/'prepared.json',root/'cal.json',target,value)
    outputs.append(json.loads((target/'reference.jsonl').read_text()))
   a,b=outputs
   for left,right in zip(a['tcp_targets'],b['tcp_targets']):
    np.testing.assert_allclose(np.array(right['position'])-left['position'],direction*.07,atol=1e-12)
    np.testing.assert_allclose(right['raw_position'],left.get('raw_position',left['position']))
    self.assertEqual(left['rotation'],right['rotation'])
    np.testing.assert_allclose(right['target_offset_world_m'],direction*.07)
   for key in ('base_pose','elapsed_s','grippers','human_points','chest_target'):self.assertEqual(a[key],b[key])
   self.assertEqual((root/'0/world.json').read_text(),(root/'7/world.json').read_text())
   self.assertEqual(json.loads((root/'reference.jsonl').read_text()),row)
   config=json.loads((root/'7/reference-config.json').read_text())
   self.assertEqual(config['coordinate_preparation']['tcp_lowering_cm'],7)
   self.assertNotIn('tcp_target_offset_world_m',config)
   self.assertEqual(config['coordinate_preparation']['tcp_lowering_frame'],'box_tag')
 def test_worker_passes_saved_cm_to_preparer(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);cat=root/'catalog.json';cat.write_text(json.dumps(dict(episodes=[dict(id='a')])))
   (root/'jobs/job').mkdir(parents=True)
   (root/'jobs/jobs.json').write_text(json.dumps(dict(a=dict(id='job',tcp_lowering_cm=7.))))
   class StopAfterCommand(Exception):pass
   with patch.object(c,'CATALOG',cat),patch.object(c,'JOBS',root/'jobs'),patch.object(c,'PROTECTED',()),patch.object(c,'capture_calibration',return_value=dict(captured_at=123)),patch.object(c,'prepare',return_value=(root/'source.json',{})),patch.object(c,'run_checked',side_effect=StopAfterCommand) as run:
    with self.assertRaises(StopAfterCommand):c.worker('a','job')
    command=run.call_args.args[0]
    self.assertEqual(command[command.index('--tcp-lowering-cm')+1],'7.0')
 def test_validation_before_observation_or_queue_mutation(self):
  for value in (-1,31,True,None,'7',float('nan'),float('inf')):
   with self.subTest(value=value):
    with self.assertRaises(ValueError):m.grasp_lowering_cm(value)
    with patch.object(c,'capture_calibration') as capture:
     with self.assertRaises(ValueError):c.enqueue(['a'],value)
     capture.assert_not_called()
 def test_queue_snapshots_explicit_value_and_default(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);prepared=root/'p.json';prepared.write_text('{}');cat=root/'catalog.json'
   cat.write_text(json.dumps(dict(episodes=[dict(id='a',prepared=str(prepared))])))
   with patch.object(c,'CATALOG',cat),patch.object(c,'JOBS',root/'jobs'),patch.object(c,'capture_calibration',return_value=dict(captured_at=123)):
    c.enqueue(['a']);self.assertEqual(c.jobs()['a']['tcp_lowering_cm'],3)
    c.update('a',status='done');c.enqueue(['a'],2.5);self.assertEqual(c.jobs()['a']['tcp_lowering_cm'],2.5)
    c.enqueue(['a'],9);self.assertEqual(c.jobs()['a']['tcp_lowering_cm'],2.5)
    c.update('a',status='done');c.enqueue(['a'],0);self.assertEqual(c.jobs()['a']['tcp_lowering_cm'],0)

if __name__=='__main__':unittest.main()
