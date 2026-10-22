import importlib.util,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
ROOT=Path('/home/lab6/Warp/scripts/05_execute')
spec=importlib.util.spec_from_file_location('height_calibration',ROOT/'kuavo_execution/calibration.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
class FixedHeightTests(unittest.TestCase):
 def test_no_robot_access_preserves_xy_rotation_and_records_residual(self):
  tags={}
  for tag,height in c.FIXED_TAG_HEIGHTS_M.items():
   M=np.eye(4);M[:3,3]=[int(tag)*.1,.2,height-.17];tags[tag]=M.tolist()
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'prepared.json';p.write_text('{}')
   episodes=SimpleNamespace(entry=lambda key:dict(input_prepared=str(p)))
   with patch.object(c,'rack_reference',return_value=dict(rack_tags=tags)):
    a=c.capture(episodes,'take');b=c.capture(episodes,'take')
   T=np.array(a['T_base_recording']);np.testing.assert_allclose(T[:3,:3],np.eye(3));np.testing.assert_allclose(T[:3,3],[0,0,.17])
   self.assertFalse(a['robot_observation_used']);self.assertLess(max(map(abs,a['height_residual_m'].values())),1e-12)
   self.assertEqual(a['T_base_recording'],b['T_base_recording'])
 def test_missing_tags_refuses(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'prepared.json';p.write_text('{}')
   with patch.object(c,'rack_reference',return_value=dict(rack_tags={})):
    with self.assertRaises(ValueError):c.capture(SimpleNamespace(entry=lambda k:dict(prepared=str(p))),'take')
if __name__=='__main__':unittest.main()
