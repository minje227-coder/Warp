import importlib.util,unittest
from pathlib import Path
ROOT=Path('/home/lab6/Warp/scripts/05_execute')
def load(name,p):
 s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
w=load('snap_replay',ROOT/'kuavo_replay.py');f=load('snap_first',ROOT/'kuavo_execution/first_state.py')
class Snap30Tests(unittest.TestCase):
 def test_threshold_runs_after_interpolation_without_changing_body(self):
  p=w.ReplayPath.__new__(w.ReplayPath);p.t=[0.,1.];p.p=[[0.]*21,[1.]*21];p.v=p.a=[[0.]*21,[0.]*21];p.claw=[[60.,0.],[80.,100.]];p.gripper_close_fraction=.30
  a=p.sample(.5);self.assertEqual(a[3],[100.,50.]);self.assertEqual(a[0],[.5]*21)
  self.assertAlmostEqual(p.sample(.49)[3][0],69.8)
  self.assertEqual(p.sample(.9)[3],[100.,100.])
 def test_legacy_package_has_no_implicit_threshold(self):
  p=w.ReplayPath.__new__(w.ReplayPath);p.t=[0.,1.];p.p=p.v=p.a=[[0.]*21,[0.]*21];p.claw=[[80.,80.],[80.,80.]];p.gripper_close_fraction=None
  self.assertEqual(p.sample(.5)[3],[80.,80.])
if __name__=='__main__':unittest.main()
