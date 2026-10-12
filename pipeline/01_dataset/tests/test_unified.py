"""Offline protocol/artifact tests. Synthetic frames are never Quest evidence."""
import copy
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from unified_capture import HEADER, CHUNK_BYTES, Reassembler, validate_bundle, BundleWriter
from validate_csv import SEW_BONES
from validate_episode import inspect_episode

def metadata(frame_id=1):
    def bone(identifier, x):
        return {'id':identifier,'position':{'x':x,'y':0,'z':0},'rotation':{'x':0,'y':0,'z':0,'w':1}}
    stamp = 638900000000000000 + frame_id * 1000000
    ids = list(SEW_BONES.values())
    return {'version':1,'frame_id':frame_id,'clock':'quest_utc_ticks_100ns',
        'timestamp_ticks':stamp,'skeleton_time_source':'provider_query_utc',
        'alignment':'interpolated_between_provider_samples', 'width':16,'height':16,'eye':'right',
        'fx':8,'fy':8,'cx':8,'cy':8,'tag_family':'tag36h11','tag_size_m':0.0784,'tags':[],
        'before':{'timestamp_ticks':stamp-100000,'bones':[bone(i,0) for i in ids]},
        'after':{'timestamp_ticks':stamp+100000,'bones':[bone(i,2) for i in ids]},
        'bones':[bone(i,1) for i in ids]}

def packets(meta,jpeg):
    raw_json = json.dumps(meta).encode()
    payload = struct.pack('<i',len(raw_json))+raw_json+jpeg
    count = (len(payload)+CHUNK_BYTES-1)//CHUNK_BYTES
    return [HEADER.pack(b'WRP1',meta['frame_id'],i,count,len(payload))+
        payload[i*CHUNK_BYTES:(i+1)*CHUNK_BYTES] for i in range(count)]

def synthetic_jpeg():
    import av
    frame = av.VideoFrame(16,16,'rgb24')
    for plane in frame.planes: plane.update(bytes(plane.buffer_size))
    stream = io.BytesIO()
    with av.open(stream,'w',format='mjpeg') as out:
        codec = out.add_stream('mjpeg',rate=10)
        codec.width = 16; codec.height = 16; codec.pix_fmt = 'yuvj420p'
        for packet in codec.encode(frame): out.mux(packet)
        for packet in codec.encode(): out.mux(packet)
    return stream.getvalue()

class UnifiedTest(unittest.TestCase):
    def test_fragment_reassembly_preserves_original_timestamp(self):
        meta = metadata(); jpeg = b'\xff\xd8'+b'x'*45000+b'\xff\xd9'
        receiver = Reassembler()
        result = None
        for packet in reversed(packets(meta,jpeg)): result = receiver.feed(packet) or result
        self.assertEqual(result,(meta,jpeg))
        validate_bundle(*result)

    def test_missing_chunk_does_not_emit_partial_frame(self):
        receiver = Reassembler()
        parts = packets(metadata(),b'\xff\xd8'+b'x'*24000+b'\xff\xd9')
        for packet in parts[:-1]: self.assertIsNone(receiver.feed(packet))
        self.assertTrue(receiver.pending)

    def test_duplicate_fragment_is_rejected(self):
        receiver = Reassembler(); first = packets(metadata(),b'x'*20000)[0]
        receiver.feed(first)
        with self.assertRaisesRegex(ValueError,'Conflicting'): receiver.feed(first)

    def test_clock_and_bracket_provenance_fail_closed(self):
        jpeg = b'\xff\xd8\xff\xd9'
        for mutate in ('clock','before','bones','tags'):
            meta = metadata()
            if mutate == 'clock': meta['clock'] = 'pc_receive_monotonic'
            if mutate == 'before': meta['before']['timestamp_ticks'] = meta['timestamp_ticks']-1000000
            if mutate == 'bones': meta['bones'][0]['position']['x'] = 9
            if mutate == 'tags': meta['tags'] = None
            with self.subTest(mutate=mutate), self.assertRaises(ValueError): validate_bundle(meta,jpeg)

    def test_zero_detected_tags_is_a_completed_detection(self):
        validate_bundle(metadata(),b'\xff\xd8\xff\xd9')

    def test_shared_artifacts_video_and_tampered_timestamp(self):
        try: import av
        except ImportError: self.skipTest('PyAV required for artifact test')
        jpeg = synthetic_jpeg()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            writer = BundleWriter(root)
            for i in range(1,4): writer.write(metadata(i),jpeg)
            writer.close(); self.assertEqual(writer.make_video()['decoded_frames'],3)
            (root/'metadata.json').write_text(json.dumps({'status':'complete',
                'capture_protocol':'warp_capture_v1','shared_frames':3,'clock':'quest_utc_ticks_100ns'}))
            self.assertTrue(inspect_episode(root,min_frames=3,strict_sew=True)['ok'])
            path = root/'rgb/timestamps.csv'
            rows = path.read_text().splitlines()
            cells = rows[1].split(','); cells[3] = str(int(cells[3])+1)
            rows[1] = ','.join(cells); path.write_text('\n'.join(rows)+'\n')
            result = inspect_episode(root,min_frames=3,strict_sew=True)
            self.assertFalse(result['ok']); self.assertIn('timestamps disagree',' '.join(result['messages']))

class WebRTCTest(unittest.IsolatedAsyncioTestCase):
    async def test_ready_start_stop_saved_over_actual_webrtc(self):
        try:
            from aiortc import RTCPeerConnection, RTCSessionDescription
            import av, fastapi, uvicorn
        except ImportError: self.skipTest('Receiver dependencies required for WebRTC test')
        import asyncio
        import socket
        import urllib.request
        from record_unified_quest import capture, parse_args
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',0)); port = probe.getsockname()[1]
        with tempfile.TemporaryDirectory() as temp:
            args = parse_args(['--session','synthetic_test','--out-root',temp,'--host','127.0.0.1',
                '--port',str(port),'--min-frames','3','--wait-seconds','15'])
            server = asyncio.create_task(capture(args))
            client = RTCPeerConnection()
            channel = client.createDataChannel('warp_capture')
            received = []
            configured = asyncio.Event()
            recording = asyncio.Event()
            @channel.on('message')
            def on_message(raw):
                message = json.loads(raw); received.append(message)
                if message['type'] == 'config': configured.set()
                if message.get('state') == 'recording': recording.set()
            try:
                await asyncio.sleep(0.2)
                await client.setLocalDescription(await client.createOffer())
                body = json.dumps({'type':client.localDescription.type,'sdp':client.localDescription.sdp}).encode()
                def post():
                    request = urllib.request.Request(f'http://127.0.0.1:{port}/offer', data=body,
                        headers={'Content-Type':'application/json'})
                    with urllib.request.urlopen(request,timeout=10) as response: return json.load(response)
                answer = await asyncio.to_thread(post)
                await client.setRemoteDescription(RTCSessionDescription(**answer))
                await asyncio.wait_for(configured.wait(),5)
                # Connected/ready alone cannot start recording.
                await asyncio.sleep(0.1)
                self.assertFalse(recording.is_set())
                channel.send(b'WARP_REQUEST_START')
                await asyncio.wait_for(recording.wait(),5)
                jpeg = synthetic_jpeg()
                for i in range(1,4):
                    for packet in packets(metadata(i),jpeg): channel.send(packet)
                    await asyncio.sleep(0.1)
                channel.send(b'WARP_REQUEST_STOP')
                self.assertEqual(await asyncio.wait_for(server,8),0)
                states = [message.get('state') for message in received]
                self.assertIn('saving',states); self.assertIn('saved',states)
                result = inspect_episode(Path(temp)/'synthetic_test',min_frames=3,strict_sew=True)
                self.assertTrue(result['ok'],result['messages'])
            finally:
                await client.close()
                if not server.done():
                    server.cancel(); await asyncio.gather(server,return_exceptions=True)

if __name__ == '__main__': unittest.main()
