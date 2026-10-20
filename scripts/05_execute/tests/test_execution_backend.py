"""Standalone lifecycle/storage tests: no dashboard, ROS, network or hardware."""
import importlib
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kuavo_execution import session, capture, history, first_state, package


class Process:
    returncode=0
    def __init__(self,rows):
        self.stdin=NS(write=lambda _:None,flush=lambda:None,close=lambda:None)
        self.stdout=io.BytesIO(b''.join((json.dumps(r)+'\n').encode() for r in rows))
        self.stderr=io.BytesIO()
    def poll(self):return 0
    def wait(self,timeout=None):return 0


class BackendTests(unittest.TestCase):
    def test_imports_do_not_require_dashboard_ros_or_start_processes(self):
        script='''import importlib.abc,sys
sys.path.insert(0,sys.argv[1])
class Deny(importlib.abc.MetaPathFinder):
 def find_spec(self,name,*args):
  if name.split('.')[0] in {'tabs','core','datadir','rospy'}:raise RuntimeError('host import: '+name)
sys.meta_path.insert(0,Deny())
import subprocess,socket
subprocess.Popen=lambda *a,**k:(_ for _ in ()).throw(RuntimeError('process on import'))
socket.socket=lambda *a,**k:(_ for _ in ()).throw(RuntimeError('network on import'))
from kuavo_execution import session,capture,history,catalog,timeline,validation,compute,standard_rack
assert session.common is None
'''
        done=subprocess.run([sys.executable,'-I','-c',script,str(ROOT)],capture_output=True,text=True)
        self.assertEqual(done.returncode,0,done.stderr)

    def run_session(self,ready=True,valid=True,tracking_error=.1):
        calls=[]
        class Capture:
            def __init__(self,*args):self.ready=threading.Event();self.info={}
            def start(self):calls.append('capture.start')
            def wait_ready(self,check):
                check();calls.append('capture.ready')
                if not ready:raise ValueError('missing camera')
                self.ready.set()
            def ingest(self,row):pass
            def finish(self):calls.append('capture.finish');return dict(valid=valid,errors=[] if valid else ['missing depth'])
        plan=dict(interval=[0.,3.],payload_sha256='payload',artifact_sha256={},standard_status='verified')
        detail=dict(id='take',fingerprint='fp')
        proc=Process([dict(event='replay_start',ros_time_ns=1),
                      dict(sample=True,elapsed_s=3.,actual_q=[tracking_error]*18,position=[0.]*21),
                      dict(event='replay_end',ros_time_ns=4,success=True),dict(done=True,success=True)])
        def spawn(*args,**kwargs):calls.append('motion.spawn');return proc
        control=NS(LINK=NS(popen_script=spawn),ACTIONS={})
        episodes=NS(selection_detail=lambda *a:detail,entry=lambda *a:{})
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        data=history.DataDirectory(tmp.name)
        session.configure(control=control,episode_source=episodes,paths=data)
        s=session.Session(detail,'sid',plan);s.beat('sid','take','fp')
        driver=NS(ReplayPath=lambda _:NS(elapsed=lambda _:[None,None,None,[0.,0.]]))
        with patch.object(capture,'Capture',Capture),patch.object(first_state,'target',return_value={}),\
             patch.object(package,'prepare',return_value=plan),patch.object(package,'driver',return_value=driver):
            s.run()
        meta=json.loads((s.run_dir/'manifest.json').read_text())
        return s,meta,calls

    def test_standalone_session_owns_order_trace_manifest_and_tracking(self):
        s,meta,calls=self.run_session()
        self.assertEqual(calls,['capture.start','capture.ready','motion.spawn','capture.finish'])
        self.assertEqual(meta['status'],'completed');self.assertTrue(meta['collection_valid'])
        self.assertTrue(meta['execution_completed']);self.assertIsNone(meta['success'])
        self.assertEqual(meta['tracking']['samples'],1)
        self.assertAlmostEqual(meta['tracking']['maximum_joint_error_deg'],math.degrees(.1))
        self.assertAlmostEqual(meta['tracking']['rms_joint_error_deg'],math.degrees(.1))
        self.assertEqual(len((s.run_dir/'trace.jsonl').read_text().splitlines()),1)

    def test_readiness_failure_never_launches_motion(self):
        _,meta,calls=self.run_session(ready=False,valid=False)
        self.assertNotIn('motion.spawn',calls)
        self.assertFalse(meta['execution_completed']);self.assertFalse(meta['collection_valid'])

    def test_collection_failure_cannot_be_hidden_by_completed_motion(self):
        _,meta,_=self.run_session(valid=False)
        self.assertTrue(meta['execution_completed']);self.assertFalse(meta['collection_valid'])
        self.assertEqual(meta['status'],'failed');self.assertIsNone(meta['success'])

    def test_standalone_history_labels_task_without_changing_qc(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(history,'datadir',history.DataDirectory(tmp)):
            folder=Path(tmp)/'retargeting/runs/take';folder.mkdir(parents=True)
            (folder/'manifest.json').write_text(json.dumps(dict(source_episode='take',status='completed',collection_valid=False,execution_completed=True)))
            key=history.listing()['recordings'][0]['id']
            result=history.label(key,True,'picked box')
            self.assertTrue(result['success']);self.assertFalse(result['collection_valid'])
            self.assertTrue(result['execution_completed']);self.assertEqual(result['task_note'],'picked box')

    def test_bad_tracking_trace_marks_collection_invalid(self):
        _,meta,_=self.run_session(tracking_error=float('nan'))
        self.assertFalse(meta['collection_valid'])
        self.assertIn('실측 추종 기록 오류',meta['collection']['errors'])

    def test_storage_root_must_be_explicit(self):
        with patch.dict('os.environ',{},clear=True):
            with self.assertRaisesRegex(ValueError,'root'):history.DataDirectory().path('retargeting')


if __name__=='__main__':unittest.main()
