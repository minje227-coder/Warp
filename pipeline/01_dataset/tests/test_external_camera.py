import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from external_camera import validate_external_camera, process_external_camera, align_external_frames, export_paired_videos


def external_source(path):
    import av
    import numpy as np
    raw = bytearray()
    rows = []
    for i, dt in enumerate([0, 66_667_000, 180_000_000]):
        buffer = io.BytesIO()
        with av.open(buffer, 'w', format='mjpeg') as out:
            stream = out.add_stream('mjpeg', rate=30)
            stream.width = stream.height = 16
            stream.pix_fmt = 'yuvj420p'
            frame = av.VideoFrame.from_ndarray(np.full((16,16,3), i*70, dtype=np.uint8), format='rgb24')
            for packet in stream.encode(frame): out.mux(packet)
            for packet in stream.encode(): out.mux(packet)
        jpeg = buffer.getvalue()
        rows.append({'index':i, 'offset':len(raw), 'bytes':len(jpeg),
                     'host_first_arrival_elapsed_ns':1_000_000_000+dt,
                     'host_last_arrival_elapsed_ns':1_001_000_000+dt,
                     'time_source':'usb_payload_arrival_not_exposure'})
        raw.extend(jpeg)
    (path/'external_camera.mjpeg').write_bytes(raw)
    (path/'external_camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    meta={'schema_version':1,'state':'complete','codec':'mjpeg','width':16,'height':16,
          'negotiated_interval_100ns':333333,'time_source':'usb_payload_arrival_not_exposure',
          'capture_result':{'ok':True,'frames':3,'payload_errors':0}}
    (path/'external_camera.json').write_text(json.dumps(meta))
    return rows


class ExternalCameraTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.source=self.root/'source';self.source.mkdir();self.rows=external_source(self.source)
    def tearDown(self):self.tmp.cleanup()
    def quest_source(self, offsets):
        import av
        import numpy as np
        from fractions import Fraction
        with av.open(str(self.source/'camera.mp4'), 'w') as out:
            stream=out.add_stream('libx264',rate=30)
            stream.width=stream.height=16;stream.pix_fmt='yuv420p'
            stream.time_base=stream.codec_context.time_base=Fraction(1,1_000_000)
            stream.options={'bf':'0','crf':'0'}
            for i, offset in enumerate(offsets):
                frame=av.VideoFrame.from_ndarray(np.full((16,16,3),i*40,dtype=np.uint8),format='rgb24')
                frame.pts=offset;frame.time_base=Fraction(1,1_000_000)
                for packet in stream.encode(frame):out.mux(packet)
            for packet in stream.encode():out.mux(packet)
        sensor=[{'sensor_timestamp_ns':999999+i,'callback_elapsed_ns':1_001_000_000+offset*1000} for i,offset in enumerate(offsets)]
        decoded=[{'decoded_index':i,'camera_match':{'sensor_index':i,'encoded_index':i}} for i in range(len(offsets))]
        return decoded,sensor
    def test_paired_videos_have_equal_pts_and_unique_original_observations(self):
        import av
        decoded,sensor=self.quest_source([0,33333,66667,100000,180000])
        destination=self.root/'paired';process_external_camera(self.source,destination)
        quality=export_paired_videos(self.source,destination,decoded,sensor,{'timestamp_source':0})
        pairs=[json.loads(l) for l in (destination/'paired_frames.jsonl').read_text().splitlines()]
        self.assertEqual([p['quest_decoded_index'] for p in pairs],[0,2,4])
        self.assertEqual([p['external_index'] for p in pairs],[0,1,2])
        means=[]
        for name in ['paired_quest.mp4','paired_external.mp4']:
            with av.open(str(destination/name)) as video:frames=list(video.decode(video=0))
            self.assertEqual([round(f.pts*f.time_base*1_000_000) for f in frames],[0,66667,180000])
            means.append([float(f.to_ndarray(format='rgb24').mean()) for f in frames])
        for actual,wanted in zip(means[0],[0,80,160]):self.assertAlmostEqual(actual,wanted,delta=3)
        with av.open(str(destination/'external_camera.mp4')) as video:
            external_means=[float(f.to_ndarray(format='rgb24').mean()) for f in video.decode(video=0)]
        for actual,wanted in zip(means[1],external_means):self.assertAlmostEqual(actual,wanted,delta=3)
        self.assertEqual(quality['frames_per_video'],3)
        self.assertTrue(quality['equal_video_counts_and_pts_verified'])
        self.assertEqual(quality['reused_source_observations'],0)
        self.assertFalse(quality['exposure_alignment_verified'])
    def test_paired_export_excludes_external_frame_instead_of_reusing_quest_pixels(self):
        decoded,sensor=self.quest_source([0,80000,180000])
        self.rows[1]['host_first_arrival_elapsed_ns']=1_010_000_000
        self.rows[1]['host_last_arrival_elapsed_ns']=1_011_000_000
        (self.source/'external_camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        destination=self.root/'paired';process_external_camera(self.source,destination)
        quality=export_paired_videos(self.source,destination,decoded,sensor,{'timestamp_source':0})
        pairs=[json.loads(l) for l in (destination/'paired_frames.jsonl').read_text().splitlines()]
        self.assertEqual([p['external_index'] for p in pairs],[0,2])
        self.assertEqual([p['quest_decoded_index'] for p in pairs],[0,2])
        self.assertEqual(quality['exclusion_reasons'],{'quest_observation_already_used':1})
    def test_paired_export_does_not_create_videos_without_valid_quest_times(self):
        destination=self.root/'paired';process_external_camera(self.source,destination)
        quality=export_paired_videos(self.source,destination,[],[],{'timestamp_source':0})
        self.assertEqual(quality['state'],'no_valid_pairs')
        self.assertEqual(quality['exclusion_reasons'],{'missing_quest_time':3})
        self.assertFalse((destination/'paired_quest.mp4').exists())
        self.assertFalse((destination/'paired_external.mp4').exists())
    def test_variable_timestamps_preserved_without_frame_padding(self):
        quality=process_external_camera(self.source,self.root/'processed')
        rows=[json.loads(l) for l in (self.root/'processed/external_decoded_frames.jsonl').open()]
        self.assertEqual([r['pts_us'] for r in rows],[0,66667,180000])
        self.assertEqual(quality['frames'],3);self.assertEqual(quality['decoded_frames'],3)
        self.assertAlmostEqual(quality['maximum_arrival_gap_ms'],113.333)
        self.assertEqual(quality['repeated_frame_padding'],0);self.assertFalse(quality['dataset_ready'])
    def test_common_clock_mapping_without_video_duplication(self):
        destination=self.root/'matches';destination.mkdir()
        sensor=[{'sensor_timestamp_ns':t,'callback_elapsed_ns':t+9_000_000} for t in [1_001_000_000,1_030_000_000,1_067_667_000,1_181_000_000,1_190_000_000]]
        decoded=[{'decoded_index':i,'camera_match':{'sensor_index':i,'encoded_index':i}} for i in range(len(sensor))]
        quality=align_external_frames(self.source,destination,decoded,sensor,{'timestamp_source':1})
        rows=[json.loads(l) for l in (destination/'external_timestep_matches.jsonl').open()]
        self.assertEqual([r['external_index'] for r in rows],[0,0,1,2,None])
        self.assertEqual(quality['matched_frames'],4);self.assertEqual(quality['reused_external_references'],1)
        self.assertEqual(rows[-1]['reason'],'outside_external_range')
        self.assertFalse(quality['exposure_alignment_verified'])
        self.assertEqual(len(validate_external_camera(self.source)[1]),3)
    def test_unknown_camera_clock_uses_callback_instead_of_sensor_clock(self):
        destination=self.root/'matches';destination.mkdir()
        quality=align_external_frames(self.source,destination,[{'decoded_index':0,'camera_match':{'sensor_index':0}}],
                                      [{'sensor_timestamp_ns':999999,'callback_elapsed_ns':1_067_667_000}],{'timestamp_source':0})
        self.assertEqual(quality['matched_frames'],1)
        self.assertIn('callback_elapsed',quality['time_basis'])
    def test_gap_is_left_unmatched(self):
        destination=self.root/'matches';destination.mkdir()
        quality=align_external_frames(self.source,destination,[{'decoded_index':0,'camera_match':{'sensor_index':0}}],
                                      [{'sensor_timestamp_ns':1_123_000_000}],{'timestamp_source':1})
        self.assertEqual(quality['matched_frames'],0);self.assertEqual(quality['reasons'],{'external_gap':1})
    def test_nonmonotonic_timestamps_rejected(self):
        self.rows[2]['host_last_arrival_elapsed_ns']=self.rows[1]['host_last_arrival_elapsed_ns']
        self.rows[2]['host_first_arrival_elapsed_ns']=self.rows[1]['host_first_arrival_elapsed_ns']
        (self.source/'external_camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        with self.assertRaisesRegex(ValueError,'timestamps'):validate_external_camera(self.source)
    def test_truncated_frame_rejected(self):
        p=self.source/'external_camera.mjpeg';p.write_bytes(p.read_bytes()[:-3])
        with self.assertRaisesRegex(ValueError,'bounds'):validate_external_camera(self.source)
    def test_incomplete_capture_rejected(self):
        p=self.source/'external_camera.json';m=json.loads(p.read_text());m['state']='recording';p.write_text(json.dumps(m))
        with self.assertRaisesRegex(ValueError,'incomplete'):validate_external_camera(self.source)
    def test_receiver_accepts_complete_extension_and_preserves_bytes(self):
        from test_raw_episode import RawEpisodeTests
        from raw_episode import FILES, USB_FILES, digest
        fixture=RawEpisodeTests('runTest');fixture.setUp()
        try:
            external_source(fixture.src)
            fixture.manifest['external_camera']={'enabled':True,'schema_version':1}
            fixture.manifest['files'].update({n:{'bytes':(fixture.src/n).stat().st_size,'sha256':digest(fixture.src/n)} for n in USB_FILES})
            with zipfile.ZipFile(fixture.archive,'w') as z:
                for n in FILES|USB_FILES:z.write(fixture.src/n,n)
                z.writestr('manifest.json',json.dumps(fixture.manifest))
            result=fixture.ingest();saved=Path(result['episode'])
            for n in USB_FILES:self.assertEqual(digest(saved/n),digest(fixture.src/n))
        finally:fixture.tearDown()
    def test_receiver_to_worker_generates_both_videos_and_timestep_manifest(self):
        import av
        from test_raw_episode import RawEpisodeTests
        from raw_episode import FILES, USB_FILES, digest
        from raw_processing_worker import run_once
        from process_raw_episode import process
        fixture=RawEpisodeTests('runTest');fixture.setUp()
        try:
            external_source(fixture.src)
            with av.open(str(fixture.src/'camera.mp4')) as video:
                pts=[round(f.pts*f.time_base*1_000_000) for f in video.decode(video=0)]
            sensor=[{'sensor_timestamp_ns':1_001_000_000+p*1000} for p in pts]
            encoded=[{'index':i,'pts_us':1_001_000+p} for i,p in enumerate(pts)]
            (fixture.src/'camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sensor))
            (fixture.src/'encoded_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in encoded))
            (fixture.src/'calibration.json').write_text(json.dumps({'timestamp_source':1}))
            (fixture.src/'tracking.jsonl').write_text(json.dumps({'kind':'body','valid':False})+'\n')
            fixture.manifest['external_camera']={'enabled':True,'schema_version':1}
            fixture.manifest['files']={n:{'bytes':(fixture.src/n).stat().st_size,'sha256':digest(fixture.src/n)} for n in FILES|USB_FILES}
            with zipfile.ZipFile(fixture.archive,'w') as z:
                for n in FILES|USB_FILES:z.write(fixture.src/n,n)
                z.writestr('manifest.json',json.dumps(fixture.manifest))
            saved=Path(fixture.ingest()['episode'])
            self.assertEqual(run_once(fixture.out,processor=lambda source,output,**kw:process(source,output,detect_tags=False,**kw)),1)
            output=saved/'processed'
            quality=json.loads((output/'quality.json').read_text())
            self.assertEqual(quality['external_camera']['decoded_frames'],3)
            self.assertEqual(quality['external_camera']['timestep_matching']['matched_frames'],2)
            self.assertEqual(quality['external_camera']['paired_export']['frames_per_video'],1)
            manifest=json.loads((output/'processing_manifest.json').read_text())
            for name in ['external_camera.mp4','external_decoded_frames.jsonl','external_timestep_matches.jsonl','paired_quest.mp4','paired_external.mp4','paired_frames.jsonl','paired_quality.json']:
                self.assertEqual(manifest['outputs'][name],digest(output/name))
            for name in FILES|USB_FILES:self.assertEqual(digest(saved/name),fixture.manifest['files'][name]['sha256'])
            self.assertFalse(quality['dataset_ready'])
        finally:fixture.tearDown()
    def test_receiver_rejects_extension_without_complete_triplet(self):
        from test_raw_episode import RawEpisodeTests
        fixture=RawEpisodeTests('runTest');fixture.setUp()
        try:
            fixture.manifest['external_camera']={'enabled':True,'schema_version':1};fixture.pack()
            with self.assertRaisesRegex(ValueError,'provenance'):fixture.ingest()
        finally:fixture.tearDown()


class D435YuyvTests(unittest.TestCase):
    quest_source=ExternalCameraTests.quest_source
    tearDown=ExternalCameraTests.tearDown
    test_paired_videos_have_equal_pts_and_unique_original_observations=ExternalCameraTests.test_paired_videos_have_equal_pts_and_unique_original_observations
    test_variable_timestamps_preserved_without_frame_padding=ExternalCameraTests.test_variable_timestamps_preserved_without_frame_padding
    # Reuse meaningful end-to-end timing tests with uncompressed camera pixels.
    def setUp(self):
        ExternalCameraTests.setUp(self)
        import av
        raw=bytearray()
        with av.open(str(self.source/'external_camera.mjpeg'),format='mjpeg') as video:
            for i,frame in enumerate(video.decode(video=0)):
                data=frame.to_ndarray(format='yuyv422').tobytes()
                self.rows[i].update(offset=len(raw),bytes=len(data))
                raw.extend(data)
        (self.source/'external_camera.mjpeg').unlink()
        (self.source/'external_camera.yuyv').write_bytes(raw)
        (self.source/'external_camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        meta=json.loads((self.source/'external_camera.json').read_text())
        meta.update(codec='yuyv422',raw_file='external_camera.yuyv',transport='bulk')
        (self.source/'external_camera.json').write_text(json.dumps(meta))

    def test_yuyv_bad_frame_length_is_rejected(self):
        self.rows[0]['bytes']-=2
        (self.source/'external_camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        with self.assertRaisesRegex(ValueError,'YUYV frame length'):
            validate_external_camera(self.source)

    def test_yuyv_odd_width_is_rejected(self):
        meta=json.loads((self.source/'external_camera.json').read_text());meta['width']=15
        (self.source/'external_camera.json').write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError,'YUYV dimensions'):
            validate_external_camera(self.source)

    def test_yuyv_raw_filename_cannot_escape(self):
        meta=json.loads((self.source/'external_camera.json').read_text());meta['raw_file']='../external_camera.yuyv'
        (self.source/'external_camera.json').write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError,'codec/raw file'):
            validate_external_camera(self.source)

    def test_yuyv_receiver_and_processing_preserve_original_pixels(self):
        import av
        from test_raw_episode import RawEpisodeTests
        from raw_episode import FILES, USB_YUYV_FILES, digest
        from raw_processing_worker import run_once
        from process_raw_episode import process
        fixture=RawEpisodeTests('runTest');fixture.setUp()
        try:
            with av.open(str(fixture.src/'camera.mp4')) as video:
                pts=[round(f.pts*f.time_base*1_000_000) for f in video.decode(video=0)]
            sensor=[{'sensor_timestamp_ns':1_001_000_000+p*1000} for p in pts]
            encoded=[{'index':i,'pts_us':1_001_000+p} for i,p in enumerate(pts)]
            (fixture.src/'camera_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sensor))
            (fixture.src/'encoded_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in encoded))
            (fixture.src/'calibration.json').write_text(json.dumps({'timestamp_source':1}))
            (fixture.src/'tracking.jsonl').write_text(json.dumps({'kind':'body','valid':False})+'\n')
            for name in USB_YUYV_FILES:
                (fixture.src/name).write_bytes((self.source/name).read_bytes())
            fixture.manifest['external_camera']={'enabled':True,'schema_version':1}
            fixture.manifest['files']={n:{'bytes':(fixture.src/n).stat().st_size,'sha256':digest(fixture.src/n)} for n in FILES|USB_YUYV_FILES}
            with zipfile.ZipFile(fixture.archive,'w') as archive:
                for name in FILES|USB_YUYV_FILES:archive.write(fixture.src/name,name)
                archive.writestr('manifest.json',json.dumps(fixture.manifest))
            saved=Path(fixture.ingest()['episode'])
            self.assertEqual(run_once(fixture.out,processor=lambda source,output,**kw:process(source,output,detect_tags=False,**kw)),1)
            quality=json.loads((saved/'processed/quality.json').read_text())
            self.assertEqual(quality['external_camera']['decoded_frames'],3)
            self.assertFalse(quality['dataset_ready'])
            for name in FILES|USB_YUYV_FILES:
                self.assertEqual(digest(saved/name),fixture.manifest['files'][name]['sha256'])
        finally:fixture.tearDown()
