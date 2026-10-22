from pathlib import Path
from contextlib import contextmanager
import importlib.util,json,tempfile,unittest
from unittest.mock import patch
module=Path(__file__).with_name('episode_delete.py')
if not module.exists():module=Path(__file__).resolve().parents[1]/'kuavo_execution/episode_delete.py'
spec=importlib.util.spec_from_file_location('episode_delete',module)
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
KEY='13-42-32_'+'a'*32
OTHER='13-43-00_'+'b'*32

class FakeCompute:
 ACTIVE=('queued','running')
 def __init__(self,p):
  self.DATA=p/'data/retargeting';self.JOBS=self.DATA/'compute';self.JOBS.mkdir(parents=True)
  self.CATALOG=self.DATA/'catalog.json';self.STANDARD=p/'Warp-review/standard';(self.STANDARD/'results').mkdir(parents=True)
 @contextmanager
 def locked(self,name='queue'):yield
 def read(self,p):return json.loads(Path(p).read_text())
 def write(self,p,value):
  p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value))
 def jobs(self):return self.read(self.JOBS/'jobs.json')

class DeletionTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.c=FakeCompute(self.root)
  self.raw=self.root/'Warp_data/2026-10-19'/KEY;self.raw.mkdir(parents=True);(self.raw/'camera.mp4').write_bytes(b'raw')
  self.provider=self.root/'Warp-review/provider'/KEY;self.provider.mkdir(parents=True)
  self.c.write(self.provider/'prepared.json',dict(episode=str(self.raw)))
  self.result=self.c.STANDARD/'results/test';self.result.mkdir();self.c.write(self.result/'POLICY.json',dict(take=KEY))
  (self.result/'video.mp4').write_bytes(b'video')
  self.other=self.c.STANDARD/'results/other';self.other.mkdir();(self.other/'keep').write_text('keep')
  self.c.write(self.c.CATALOG,dict(episodes=[dict(id=KEY,input_prepared=str(self.provider/'prepared.json'),result=str(self.result)),dict(id=OTHER,result=str(self.other))]))
  self.c.write(self.c.JOBS/'jobs.json',{})
  self.patch=patch.object(d,'RAW_ROOT',self.root/'Warp_data');self.patch.start();d.TICKETS.clear()
 def tearDown(self):self.patch.stop();self.tmp.cleanup()
 def test_preview_and_cancel_never_delete(self):
  plan=d.preview(self.c,KEY);self.assertGreater(plan['bytes'],0);self.assertTrue(self.raw.exists())
  with self.assertRaises(ValueError):d.delete(self.c,KEY,plan['ticket'],False)
  self.assertTrue(self.result.exists())
 def test_confirm_deletes_only_owned_data_without_following_symlinks(self):
  shared=self.root/'shared';shared.mkdir();(shared/'keep').write_text('keep');(self.provider/'link').symlink_to(shared,target_is_directory=True)
  plan=d.preview(self.c,KEY);result=d.delete(self.c,KEY,plan['ticket'],True)
  self.assertTrue(result['ok']);self.assertFalse(self.raw.exists());self.assertFalse(self.result.exists());self.assertFalse(self.provider.exists())
  self.assertTrue((self.other/'keep').exists());self.assertTrue((shared/'keep').exists())
  self.assertEqual([e['id'] for e in self.c.read(self.c.CATALOG)['episodes']],[OTHER])
 def test_changed_plan_requires_new_confirmation(self):
  plan=d.preview(self.c,KEY);(self.raw/'camera.mp4').write_bytes(b'changed')
  with self.assertRaisesRegex(ValueError,'변경'):d.delete(self.c,KEY,plan['ticket'],True)
  self.assertTrue(self.raw.exists())
 def test_active_computation_and_current_standard_block_deletion(self):
  self.c.write(self.c.JOBS/'jobs.json',{KEY:dict(status='running')})
  with self.assertRaisesRegex(ValueError,'계산 중'):d.preview(self.c,KEY)
  self.c.write(self.c.JOBS/'jobs.json',{});self.c.write(self.c.STANDARD/'CURRENT.json',dict(take=KEY))
  with self.assertRaisesRegex(ValueError,'채택'):d.preview(self.c,KEY)
 def test_shared_source_blocked(self):
  catalog=self.c.read(self.c.CATALOG);catalog['episodes'][1]['input_prepared']=str(self.provider/'prepared.json');self.c.write(self.c.CATALOG,catalog)
  with self.assertRaisesRegex(ValueError,'다른 episode'):d.preview(self.c,KEY)
 def test_expired_ticket_and_path_traversal_blocked(self):
  p=d.preview(self.c,KEY);d.TICKETS[p['ticket']]['expires']=0
  with self.assertRaisesRegex(ValueError,'만료'):d.delete(self.c,KEY,p['ticket'],True)
  with self.assertRaises(ValueError):d.preview(self.c,'../../anything')
 def test_metadata_failure_restores_quarantined_files(self):
  plan=d.preview(self.c,KEY);write=self.c.write;failed=[]
  def fail_once(p,x):
   if Path(p)==self.c.CATALOG and not failed:failed.append(True);raise OSError('fixture failure')
   return write(p,x)
  with patch.object(self.c,'write',side_effect=fail_once):
   with self.assertRaises(OSError):d.delete(self.c,KEY,plan['ticket'],True)
  self.assertTrue(self.raw.exists());self.assertTrue(self.result.exists())
  self.assertEqual(len(self.c.read(self.c.CATALOG)['episodes']),2)

if __name__=='__main__':unittest.main()
