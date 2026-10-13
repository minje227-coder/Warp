import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE))
spec=importlib.util.spec_from_file_location('raw_worker',HERE/'raw_processing_worker.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
ID='a'*32

class WorkerTest(unittest.TestCase):
    def source(self,root,schema=3,legacy=False):
        src=(root/'raw' if legacy else root)/'2026-10-13'/ID;src.mkdir(parents=True)
        (src/'manifest.json').write_text(json.dumps({'schema_version':schema,'episode_id':ID}))
        (src/'transfer.json').write_text(json.dumps({'state':'saved'}));return src
    def processor(self,source,out,**settings):
        out.mkdir();(out/'quality.json').write_text('{"state":"needs_clock_validation"}');(out/'processing_manifest.json').write_text('{"state":"complete"}');return {'state':'needs_clock_validation'}
    def test_persist_complete_and_skip_duplicate_source_immutable(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);src=self.source(root);before={p.name:p.read_bytes() for p in src.iterdir()}
            self.assertEqual(m.run_once(root,self.processor),1)
            self.assertEqual(m.run_once(root,lambda *a,**k:self.fail('Already processed')),0)
            self.assertEqual(before,{p.name:p.read_bytes() for p in src.iterdir() if p.is_file()})
            self.assertTrue((src/'processed'/'quality.json').is_file())
            self.assertFalse((root/'derived').exists())
            self.assertEqual(json.loads((root/'.raw_processing'/(ID+'.json')).read_text())['state'],'complete')
    def test_crash_recovery_cleans_only_partial_derived(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);src=self.source(root);partial=src/'processed.partial';partial.mkdir();(partial/'unfinished').write_text('x')
            jobs=root/'.raw_processing';jobs.mkdir();(jobs/(ID+'.json')).write_text('{"state":"running","attempts":1}')
            self.assertEqual(m.run_once(root,self.processor),1);self.assertTrue(src.exists());self.assertFalse(partial.exists())
            self.assertEqual(json.loads((jobs/(ID+'.json')).read_text())['attempts'],2)
    def test_legacy_complete_output_is_reused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);src=self.source(root,legacy=True)
            out=root/'derived'/'2026-10-13'/ID;out.parent.mkdir(parents=True)
            self.processor(src,out)
            self.assertEqual(m.run_once(root,lambda *a,**k:self.fail('Already processed')),0)
            self.assertFalse((src/'processed').exists())
    def test_legacy_pending_episode_is_processed(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);self.source(root,legacy=True)
            self.assertEqual(m.run_once(root,self.processor),1)
            self.assertTrue((root/'derived'/'2026-10-13'/ID/'quality.json').is_file())
    def test_error_no_infinite_retry_legacy_skip(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);self.source(root);calls=[]
            def bad(*a,**k):calls.append(1);raise ValueError('deterministic QC error')
            m.run_once(root,bad);m.run_once(root,bad);self.assertEqual(len(calls),1)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);self.source(root,1);self.assertEqual(m.run_once(root,self.processor),0)

if __name__=='__main__':unittest.main()
