import importlib.util,sys,time,unittest
from importlib import import_module
from pathlib import Path
import numpy as np
root=Path('/home/lab6/Warp/scripts/05_execute/kuavo_execution')
spec=importlib.util.spec_from_file_location('single_projection_integration',root/'__init__.py',submodule_search_locations=[str(root)])
pkg=importlib.util.module_from_spec(spec);sys.modules[spec.name]=pkg;spec.loader.exec_module(pkg)
c=import_module(spec.name+'.catalog')
class ProjectionTests(unittest.TestCase):
 def test_one_tag_projects_boxes_without_enabling_alignment(self):
  M=np.eye(4);seen=M.copy();seen[2,3]=3
  detail=dict(rack_tags={'4':M.tolist()},rack_fit_settings={},boxes=[dict(id='box_100',position=[0,0,0],quaternion_xyzw=[0,0,0,1],dimensions=[.3,.2,.2])])
  sample=dict(received=time.time(),image_age_s=0,tf_ready=True,base_camera_pose=[0,0,0,0,0,0,1],K=[300,0,320,0,300,240,0,0,1],tags=[dict(id=4,T_camera_tag=seen.tolist())],detector='apt_apriltag_ros')
  out=c.overlay(detail,sample)
  self.assertTrue(out.get('projection_ok'),out);self.assertFalse(out['ok']);self.assertIsNone(out['robot_error']);self.assertEqual(out['rack_ids'],[4]);self.assertEqual(len(out['boxes'][0]['edges']),12)
if __name__=='__main__':unittest.main()
