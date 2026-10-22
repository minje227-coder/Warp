import importlib.util,sys,unittest
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
root=Path('/home/lab6/Warp/scripts/05_execute/kuavo_execution')
spec=importlib.util.spec_from_file_location('shared_head_test_backend',root/'__init__.py',submodule_search_locations=[str(root)])
pkg=importlib.util.module_from_spec(spec);sys.modules[spec.name]=pkg;spec.loader.exec_module(pkg)
c=import_module(spec.name+'.catalog')
class SharedHeadTests(unittest.TestCase):
 def test_stale_tag_does_not_freeze_shared_display_or_enable_alignment(self):
  old=dict(received=1.,image_age_s=0.,frame_id='old-tag',tf_ready=True)
  frame=dict(jpeg='shared',seq=10,t=99.9,age=.01)
  link=SimpleNamespace(frame=lambda key:dict(frame))
  with patch.object(c,'common',SimpleNamespace(LINK=link)):
   stale=c.overlay({},old,now=100.)
   self.assertFalse(stale['ok']);self.assertIsNone(stale['robot_error']);self.assertEqual(stale['boxes'],[])
   a=c.head_display(now=100.);self.assertTrue(a['ok']);self.assertEqual(a['url'],'/kuavo/cam/head')
   frame.update(seq=11,t=100.1)
   b=c.head_display(now=100.2);self.assertTrue(b['ok']);self.assertNotEqual(a['frame_id'],b['frame_id'])
 def test_missing_or_stale_shared_camera_remains_marked_unavailable(self):
  with patch.object(c,'common',SimpleNamespace(LINK=None)):self.assertFalse(c.head_display(now=100.)['ok'])
  link=SimpleNamespace(frame=lambda key:dict(jpeg='old',seq=1,t=1.,age=0.))
  with patch.object(c,'common',SimpleNamespace(LINK=link)):self.assertFalse(c.head_display(now=100.)['ok'])
if __name__=='__main__':unittest.main()
