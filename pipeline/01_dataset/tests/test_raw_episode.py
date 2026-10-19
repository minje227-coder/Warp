from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import zipfile
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from raw_episode import FILES, digest, import_raw_episode

class RawEpisodeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name).resolve()
        self.src=self.root/'source';self.src.mkdir();self.out=self.root/'out'
        self.episode_id='a'*32
        import av,numpy as np
        with av.open(str(self.src/'camera.mp4'),'w') as v:
            s=v.add_stream('libx264',rate=30);s.width=s.height=1280;s.pix_fmt='yuv420p'
            for i in range(2):
                f=av.VideoFrame.from_ndarray(np.full((1280,1280,3),i*40,dtype=np.uint8),format='rgb24')
                for packet in s.encode(f):v.mux(packet)
            for packet in s.encode():v.mux(packet)
        for name in FILES-{'camera.mp4'}:(self.src/name).write_text('{}\n')
        self.manifest={'created_unix_ms':int(datetime.fromisoformat('2026-10-14T23:55:06.214+09:00').timestamp()*1000),'schema_version':3,'episode_id':self.episode_id,'width':1280,'height':1280,'sensor_frames':2,'encoded_frames':2,'tracking_samples':1,
                       'files':{n:{'bytes':(self.src/n).stat().st_size,'sha256':digest(self.src/n)} for n in FILES}}
        self.archive=self.root/'upload.zip';self.pack()
    def tearDown(self):self.tmp.cleanup()
    def pack(self,extra=None):
        with zipfile.ZipFile(self.archive,'w') as z:
            for n in FILES:z.write(self.src/n,n)
            z.writestr('manifest.json',json.dumps(self.manifest))
            if extra:z.writestr(extra,'bad')
    def ingest(self):return import_raw_episode(self.archive,self.out,self.episode_id,digest(self.archive))
    def test_original_video_preserved_without_resampling(self):
        r=self.ingest();p=Path(r['episode']);self.assertEqual(digest(p/'camera.mp4'),digest(self.src/'camera.mp4'));self.assertFalse(r['dataset_ready']);self.assertEqual(r['processing_state'],'pending')
        self.assertEqual(p.parent.parent,self.out)
        self.assertEqual(p.name,'23-55-06_'+self.episode_id)
        self.assertEqual(p.parent.name,'2026-10-14')
        self.assertEqual(json.loads((p/'manifest.json').read_text()),self.manifest)
        self.assertFalse((self.out/'raw').exists())
    def test_retry_returns_same_episode(self):
        first=self.ingest();self.assertEqual(first,self.ingest());self.assertEqual(len(list(self.out.glob('*/*/manifest.json'))),1)
    def test_receipt_commit_crash_recovery(self):
        first=self.ingest();(self.out/'.raw_uploads'/(self.episode_id+'.json')).unlink();self.assertEqual(first,self.ingest())
    def test_legacy_receipt_crash_recovery(self):
        first=self.ingest();folder=Path(first['episode']);legacy=self.out/'raw'/folder.parent.name/self.episode_id
        legacy.parent.mkdir(parents=True);folder.rename(legacy)
        first['episode']=str(legacy);(legacy/'transfer.json').write_text(json.dumps(first))
        (self.out/'.raw_uploads'/(self.episode_id+'.json')).unlink()
        self.assertEqual(first,self.ingest())
        self.assertFalse(folder.exists())
    def test_capture_date_and_time_are_korean_not_upload_time(self):
        self.manifest['created_unix_ms']=int(datetime.fromisoformat('2026-10-14T15:00:01+00:00').timestamp()*1000)
        self.pack();r=self.ingest();p=Path(r['episode'])
        self.assertEqual(p.parent.name,'2026-10-15')
        self.assertEqual(p.name,'00-00-01_'+self.episode_id)
    def test_same_second_keeps_unique_uuid(self):
        first=self.ingest();self.episode_id='b'*32;self.manifest['episode_id']=self.episode_id;self.pack();second=self.ingest()
        self.assertNotEqual(first['episode'],second['episode'])
        self.assertEqual(Path(second['episode']).name,'23-55-06_'+'b'*32)
    def test_old_uuid_folder_receipt_crash_recovery(self):
        first=self.ingest();folder=Path(first['episode']);old=folder.with_name(self.episode_id);folder.rename(old)
        first['episode']=str(old);(old/'transfer.json').write_text(json.dumps(first))
        (self.out/'.raw_uploads'/(self.episode_id+'.json')).unlink()
        self.assertEqual(first,self.ingest())
        self.assertFalse(folder.exists())
    def test_prefixed_legacy_folder_receipt_crash_recovery(self):
        first=self.ingest();folder=Path(first['episode']);legacy=self.out/'raw'/folder.parent.name/folder.name
        legacy.parent.mkdir(parents=True);folder.rename(legacy)
        first['episode']=str(legacy);(legacy/'transfer.json').write_text(json.dumps(first))
        (self.out/'.raw_uploads'/(self.episode_id+'.json')).unlink()
        self.assertEqual(first,self.ingest())
    def test_duplicate_uuid_across_old_and_new_paths_rejected(self):
        first=self.ingest();folder=Path(first['episode']);other=folder.with_name(self.episode_id)
        shutil.copytree(folder,other);(self.out/'.raw_uploads'/(self.episode_id+'.json')).unlink()
        with self.assertRaisesRegex(ValueError,'Duplicate raw episode identity'):self.ingest()
    def test_invalid_recording_timestamp_rejected_without_receipt(self):
        for value in (None,True,-1,'bad',1e20,10**100):
            with self.subTest(value=value):
                self.manifest['created_unix_ms']=value;self.pack()
                with self.assertRaisesRegex(ValueError,'recording start timestamp'):self.ingest()
                self.assertFalse((self.out/'.raw_uploads'/(self.episode_id+'.json')).exists())
    def test_existing_destination_is_preserved(self):
        target=self.out/'2026-10-14'/('23-55-06_'+self.episode_id);target.mkdir(parents=True)
        (target/'original').write_text('keep')
        with self.assertRaisesRegex(ValueError,'already exists'):self.ingest()
        self.assertEqual((target/'original').read_text(),'keep')
    def test_same_id_different_bytes_rejected(self):
        self.ingest();self.manifest['note']='different';self.pack()
        with self.assertRaisesRegex(ValueError,'different bytes'):self.ingest()
    def test_wrong_upload_checksum_rejected(self):
        with self.assertRaisesRegex(ValueError,'checksum'):import_raw_episode(self.archive,self.out,self.episode_id,'0'*64)
    def test_source_checksum_failure_leaves_no_receipt(self):
        self.manifest['files']['tracking.jsonl']['sha256']='0'*64;self.pack()
        with self.assertRaisesRegex(ValueError,'Source checksum'):self.ingest()
        self.assertFalse((self.out/'.raw_uploads'/(self.episode_id+'.json')).exists())
    def test_path_traversal_rejected(self):
        self.pack('../outside')
        with self.assertRaisesRegex(ValueError,'archive entries'):self.ingest()
        self.assertFalse((self.root/'outside').exists())
    def test_duplicate_member_rejected(self):
        self.pack('tracking.jsonl')
        with self.assertRaisesRegex(ValueError,'duplicate'):self.ingest()
    def test_missing_confirmed_pc_data_rejected(self):
        r=self.ingest();shutil.rmtree(r['episode'])
        with self.assertRaisesRegex(ValueError,'PC episode missing'):self.ingest()
    def test_empty_tracking_rejected(self):
        self.manifest['tracking_samples']=0;self.pack()
        with self.assertRaisesRegex(ValueError,'Missing camera/tracking'):self.ingest()
    def test_wrong_manifest_resolution_rejected(self):
        self.manifest['height']=960;self.pack()
        with self.assertRaisesRegex(ValueError,'resolution'):self.ingest()
if __name__=='__main__':unittest.main()
