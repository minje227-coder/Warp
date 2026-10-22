"""Timestamp matching and startup readiness; pure fake TF, no ROS."""
import importlib.util,unittest
from pathlib import Path
from types import SimpleNamespace as NS
spec=importlib.util.spec_from_file_location('warp_camera_tf',Path(__file__).resolve().parents[1]/'observe_replay.py')
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)

class TfReadinessTests(unittest.TestCase):
    def test_distinct_matched_frames_required_and_failures_reset(self):
        g=w.TfReadiness()
        self.assertFalse(g.update(1,True,0.))
        self.assertFalse(g.update(1,True,1.))
        self.assertFalse(g.update(2,True,1.1))
        self.assertTrue(g.update(3,True,1.2))
        self.assertFalse(g.update(4,False,1.3))
        self.assertFalse(g.update(5,True,1.4))
        self.assertFalse(g.update(6,True,1.9))
        self.assertTrue(g.update(7,True,2.4))

    def test_clock_reset_requires_new_warmup(self):
        g=w.TfReadiness()
        for stamp,now in [(10,0.),(11,.5),(12,1.)]:g.update(stamp,True,now)
        self.assertFalse(g.update(1,True,2.))

    def test_future_tf_retries_exact_same_timestamp(self):
        now=[0.];calls=[]
        def lookup(target,frame,stamp,duration):
            calls.append((target,frame,stamp))
            if len(calls)<3:raise RuntimeError('extrapolation into the future')
            return NS(transform='matched')
        got=w.image_transform(NS(lookup_transform=lookup),'camera',123.,lambda x:x,lambda:False,
                              clock=lambda:now[0],wait=lambda dt:now.__setitem__(0,now[0]+dt))
        self.assertEqual(got,'matched');self.assertEqual(calls,[('base_link','camera',123.)]*3)

    def test_past_image_is_discarded_without_retry_or_latest_fallback(self):
        calls=[]
        def lookup(*args):
            calls.append(args);raise RuntimeError('extrapolation into the past')
        with self.assertRaisesRegex(RuntimeError,'past'):
            w.image_transform(NS(lookup_transform=lookup),'camera',123.,lambda x:x,lambda:False)
        self.assertEqual(len(calls),1);self.assertEqual(calls[0][2],123.)

    def test_missing_tf_has_bounded_retry(self):
        now=[0.];calls=[]
        def lookup(*args):
            calls.append(args);raise RuntimeError('frame unavailable')
        with self.assertRaisesRegex(RuntimeError,'unavailable'):
            w.image_transform(NS(lookup_transform=lookup),'camera',123.,lambda x:x,lambda:False,
                              clock=lambda:now[0],wait=lambda dt:now.__setitem__(0,now[0]+dt))
        self.assertLessEqual(now[0],.62);self.assertTrue(all(c[2]==123. for c in calls))
