import importlib.util,json,tempfile,unittest
from pathlib import Path
p=Path(__file__).with_name('history.py')
if not p.exists():p=Path(__file__).resolve().parents[1]/'kuavo_execution/history.py'
spec=importlib.util.spec_from_file_location('delete_history',p);h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
class DeleteTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);h.datadir=h.DataDirectory(str(self.root))
 def recording(self,status='failed',legacy=False):
  folder=self.root/('recordings/one' if legacy else 'retargeting/runs/one');folder.mkdir(parents=True)
  (folder/('meta.json' if legacy else 'manifest.json')).write_text(json.dumps(dict(source_episode='take',status=status)))
  (folder/'clip.mp4').write_bytes(b'fixture');return folder,h.listing()['recordings'][0]['id']
 def test_completed_folder_is_recoverable_and_sources_preserved(self):
  folder,key=self.recording();source=self.root/'original';source.write_bytes(b'keep');preview=h.deletion_preview(key);r=h.delete(key,preview['revision'])
  self.assertTrue(r['recoverable']);self.assertFalse(folder.exists());self.assertEqual(h.listing()['recordings'],[]);self.assertEqual(source.read_bytes(),b'keep')
  saved=list((self.root/'.trash/replay-recordings').glob('*/recording'));self.assertEqual(len(saved),1);self.assertEqual((saved[0]/'clip.mp4').read_bytes(),b'fixture')
 def test_all_active_states_are_rejected(self):
  folder,key=self.recording()
  for status in h.ACTIVE:
   (folder/'manifest.json').write_text(json.dumps(dict(source_episode='take',status=status)))
   with self.assertRaises(ValueError):h.deletion_preview(key)
   with self.assertRaises(ValueError):h.delete(key,'bad')
  self.assertTrue(folder.exists())
 def test_changed_inventory_invalidates_confirmation(self):
  folder,key=self.recording();preview=h.deletion_preview(key);(folder/'clip.mp4').write_bytes(b'changed fixture with different size')
  with self.assertRaises(ValueError):h.delete(key,preview['revision'])
  self.assertTrue(folder.exists())
 def test_external_symlink_target_is_never_moved_or_counted(self):
  folder,key=self.recording();external=self.root/'source.bag';external.write_bytes(b'preserved');(folder/'external.bag').symlink_to(external)
  preview=h.deletion_preview(key);h.delete(key,preview['revision']);self.assertEqual(external.read_bytes(),b'preserved')
 def test_legacy_recording_uses_same_deletion(self):
  folder,key=self.recording(legacy=True);preview=h.deletion_preview(key);h.delete(key,preview['revision']);self.assertFalse(folder.exists())
 def test_trash_cannot_escape_storage(self):
  folder,key=self.recording();preview=h.deletion_preview(key)
  with tempfile.TemporaryDirectory() as outside:
   (self.root/'.trash').symlink_to(outside)
   with self.assertRaises(ValueError):h.delete(key,preview['revision'])
  self.assertTrue(folder.exists())
 def test_invalid_key_cannot_target_other_folders(self):
  with self.assertRaises(ValueError):h.deletion_preview('../../outside')
 def test_fifo_keeps_exactly_three_most_recent_deletions(self):
  expired=[]
  for i in range(5):
   folder,key=self.recording();p=folder/'manifest.json';meta=json.loads(p.read_text());meta['name']='run'+str(i);p.write_text(json.dumps(meta))
   preview=h.deletion_preview(key);r=h.delete(key,preview['revision']);expired.append(r['expired_count'])
  trash=self.root/'.trash/replay-recordings'
  receipts=[json.loads(p.read_text()) for p in trash.glob('*/RESTORE.json')]
  self.assertEqual(sorted(r['name'] for r in receipts),['run2','run3','run4'])
  self.assertEqual(expired,[0,0,0,1,1]);self.assertEqual(len(list(trash.iterdir())),3)
 def test_fifo_purge_does_not_follow_symlinks_to_original_data(self):
  external=self.root/'source.bag';external.write_bytes(b'preserved')
  for i in range(4):
   folder,key=self.recording()
   if i==0:(folder/'original-link.bag').symlink_to(external)
   preview=h.deletion_preview(key);h.delete(key,preview['revision'])
  self.assertEqual(external.read_bytes(),b'preserved')
  self.assertEqual(len(list((self.root/'.trash/replay-recordings').iterdir())),3)
if __name__=='__main__':unittest.main()
