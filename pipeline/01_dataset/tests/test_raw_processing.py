import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

PATH=Path(__file__).resolve().parents[1]/'process_raw_episode.py'
spec=importlib.util.spec_from_file_location('raw_processing',PATH);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def body(ns, x=0):
    return {'kind':'body','timestamp_ticks':m.UNIX_TICKS+ns//100,'valid':True,'coordinate_frame':'native','tracking_origin_pose':{'position':0},'bones':[{'id':1,'name':'wrist','position':{'x':x,'y':0,'z':0},'rotation':{'x':0,'y':0,'z':0,'w':1}}]}


class ProcessingTest(unittest.TestCase):
    def test_container_pts_normalization_full_sequence(self):
        encoded=[{'pts_us':9000000+t} for t in [0,33333,66667]]
        result=m.verify_container_pts([0,33333,66667],encoded)
        self.assertTrue(result['verified'])
        self.assertEqual(result['container_to_encoder_shift_us'],9000000)
        self.assertEqual(result['maximum_residual_us'],0)
    def test_container_pts_mismatch_no_reorder(self):
        encoded=[{'pts_us':t} for t in [0,33333,66667]]
        self.assertFalse(m.verify_container_pts([0,33333],encoded)['verified'])
        self.assertFalse(m.verify_container_pts([0,33333,70000],encoded)['verified'])
        self.assertFalse(m.verify_container_pts([0,66667,33333],encoded)['verified'])
    def test_unique_exposure_match_and_gap_not_padded(self):
        e=[{'index':i,'pts_us':t} for i,t in enumerate([1000,2000,4000,5000])]
        s=[{'sensor_timestamp_ns':t} for t in [1000001,2001999,4002001]]
        r=m.reconcile(e,s)
        self.assertEqual([v['sensor_index'] for v in r],[0,1,None,None])
    def test_duplicate_and_equidistant_matches_rejected(self):
        e=[{'index':0,'pts_us':1},{'index':1,'pts_us':3},{'index':2,'pts_us':3}]
        s=[{'sensor_timestamp_ns':0},{'sensor_timestamp_ns':2000},{'sensor_timestamp_ns':3000}]
        self.assertTrue(all(r['sensor_index'] is None for r in m.reconcile(e,s)))
    def test_no_implicit_clock_alignment(self):
        self.assertEqual(m.align_body([{'encoded_index':0,'sensor_timestamp_ns':100}], [body(100)],[],None),[])
    def test_alignment_bracket_slerp_no_extrapolation(self):
        track=[body(10000000,0),body(30000000,2)]
        track[1]['bones'][0]['rotation']={'x':0,'y':0,'z':1,'w':0}
        rows=m.align_body([{'encoded_index':i,'sensor_timestamp_ns':t} for i,t in enumerate([0,20000000,40000000])],track,[],0)
        self.assertEqual([r['valid'] for r in rows],[False,True,False])
        self.assertEqual(rows[1]['bones'][0]['position']['x'],1)
        self.assertAlmostEqual(rows[1]['bones'][0]['rotation']['z'],2**-0.5)
    def test_recenter_and_large_gap_rejected(self):
        match=[{'encoded_index':0,'sensor_timestamp_ns':20000000}]
        event=[{'unix_ticks':m.UNIX_TICKS+200000,'event':'tracking_origin_recentered'}]
        self.assertEqual(m.align_body(match,[body(10000000),body(30000000)],event,0)[0]['reason'],'tracking_discontinuity')
        self.assertEqual(m.align_body(match,[body(0),body(60000000)],[],0)[0]['reason'],'tracking_gap')
    def test_missing_hand_coverage(self):
        self.assertEqual(m.coverage([body(0)])['missing_types'],['left_hand','right_hand'])
    def test_raw_preserved_and_qc_real_video(self):
        import av
        import numpy as np
        with tempfile.TemporaryDirectory() as d:
            raw=Path(d)/'raw';raw.mkdir();out=Path(d)/'derived'
            with av.open(str(raw/'camera.mp4'),'w') as v:
                st=v.add_stream('libx264',rate=30);st.width=64;st.height=64;st.pix_fmt='yuv420p'
                for i in range(3):
                    f=av.VideoFrame.from_ndarray(np.full((64,64,3),i*50,dtype=np.uint8),format='rgb24')
                    for packet in st.encode(f):v.mux(packet)
                for packet in st.encode():v.mux(packet)
            with av.open(str(raw/'camera.mp4')) as v:
                pts=[round(f.pts*f.time_base*1000000) for f in v.decode(video=0)]
            payloads={'camera_frames.jsonl':[{'sensor_timestamp_ns':p*1000} for p in pts],'encoded_frames.jsonl':[{'index':i,'pts_us':p} for i,p in enumerate(pts)],'tracking.jsonl':[body(0)],'events.jsonl':[]}
            for name,rows in payloads.items():(raw/name).write_text(''.join(json.dumps(x)+'\n' for x in rows))
            (raw/'calibration.json').write_text('{}')
            files={p.name:{'sha256':m.sha256(p),'bytes':p.stat().st_size} for p in raw.iterdir()}
            (raw/'manifest.json').write_text(json.dumps({'schema_version':3,'width':64,'height':64,'files':files}))
            before={p.name:m.sha256(p) for p in raw.iterdir()}
            qc=m.process(raw,out,detect_tags=False)
            self.assertEqual(qc['state'],'needs_clock_validation');self.assertEqual(qc['decoded_frames'],3)
            self.assertEqual(qc['unmatched_decoded_frames'],0)
            self.assertEqual(before,{p.name:m.sha256(p) for p in raw.iterdir()})
            with self.assertRaises(ValueError):m.process(raw,raw/'derived',detect_tags=False)

if __name__=='__main__':unittest.main()
