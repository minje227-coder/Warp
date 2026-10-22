import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

p=Path(__file__).with_name('history.py')
if not p.exists():p=Path(__file__).resolve().parents[1]/'kuavo_execution/history.py'
spec=importlib.util.spec_from_file_location('dashboard_history',p)
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)

class DashboardRunsTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
  self.root=Path(self.temp.name);h.datadir=h.DataDirectory(str(self.root))
 def write(self,path,meta):
  p=self.root/path;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(meta))
 def test_runs_share_history_identity_duration_and_status(self):
  self.write('retargeting/runs/a/manifest.json',dict(source_episode='take_a',status='failed',started_at=5,duration_s=14.1,collection=dict(bytes=4000000)))
  run=h.dashboard_runs()[0]
  self.assertEqual(run['run_id'],h.listing()['recordings'][0]['id'])
  self.assertEqual(run['kind'],'retargeting_run');self.assertEqual(run['duration'],14.1)
  self.assertEqual(run['status'],'failed');self.assertEqual(run['size_mb'],4.)
 def test_legacy_recordings_not_duplicated_and_trash_is_excluded(self):
  self.write('recordings/legacy/meta.json',dict(source_episode='take',status='completed'))
  self.write('.trash/replay-recordings/x/recording/manifest.json',dict(source_episode='take',status='completed'))
  self.assertEqual(h.dashboard_runs(),[])
 def test_missing_bag_bytes_does_not_imply_zero_sized_recording(self):
  self.write('retargeting/runs/a/manifest.json',dict(source_episode='take',collection=dict(bytes=0)))
  self.assertIsNone(h.dashboard_runs()[0]['size_mb'])

if __name__=='__main__':unittest.main()
