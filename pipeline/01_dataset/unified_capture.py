"""One XRT camera-frame bundle: physical RGB, aligned skeleton and same-frame tags."""
from __future__ import annotations
import csv
from fractions import Fraction
import io
import json
import math
from pathlib import Path
import struct
import time

HEADER = struct.Struct('<4sqiii')
CHUNK_BYTES = 12000
MAX_BYTES = 8 * 1024 * 1024
TICKS_PER_SECOND = 10_000_000

class Reassembler:
    def __init__(self):
        self.pending = {}
        self.last_id = 0

    def feed(self, packet):
        if not isinstance(packet, bytes) or len(packet) < HEADER.size:
            raise ValueError('Invalid capture packet')
        magic, frame_id, index, count, total = HEADER.unpack_from(packet)
        chunk = packet[HEADER.size:]
        if magic != b'WRP1' or not 0 < total <= MAX_BYTES or frame_id <= 0:
            raise ValueError('Invalid capture header')
        if count != (total + CHUNK_BYTES - 1) // CHUNK_BYTES or not 0 <= index < count:
            raise ValueError('Invalid capture fragmentation')
        expected = min(CHUNK_BYTES, total - index * CHUNK_BYTES)
        if len(chunk) != expected:
            raise ValueError('Invalid capture chunk length')
        if frame_id <= self.last_id:
            raise ValueError('Duplicate or regressed frame ID')
        now = time.monotonic()
        if any(now - record[0] > 2 for record in self.pending.values()):
            raise ValueError('Incomplete capture bundle timed out')
        if frame_id not in self.pending:
            if len(self.pending) >= 4:
                raise ValueError('Too many incomplete bundles')
            self.pending[frame_id] = (now, total, count, {})
        _, saved_total, saved_count, chunks = self.pending[frame_id]
        if saved_total != total or saved_count != count or index in chunks:
            raise ValueError('Conflicting capture fragments')
        chunks[index] = chunk
        if len(chunks) != count: return None
        raw = b''.join(chunks[i] for i in range(count))
        del self.pending[frame_id]
        self.last_id = frame_id
        if len(raw) < 4: raise ValueError('Missing capture metadata')
        length, = struct.unpack_from('<i', raw)
        if not 0 < length < len(raw) - 4: raise ValueError('Invalid metadata length')
        meta = json.loads(raw[4:4 + length])
        if meta.get('frame_id') != frame_id: raise ValueError('Frame ID mismatch')
        return meta, raw[4 + length:]

def validate_bundle(meta, jpeg=None):
    if meta.get('version') != 1 or meta.get('clock') != 'quest_utc_ticks_100ns':
        raise ValueError('Unsupported shared clock/protocol')
    if meta.get('skeleton_time_source') != 'provider_query_utc' or meta.get('alignment') != 'interpolated_between_provider_samples':
        raise ValueError('Unknown skeleton timing provenance')
    timestamp = meta['timestamp_ticks']
    before, after = meta['before'], meta['after']
    if not isinstance(timestamp, int) or timestamp <= 0:
        raise ValueError('Invalid capture timestamp')
    if not (before['timestamp_ticks'] <= timestamp <= after['timestamp_ticks']) or \
            max(timestamp - before['timestamp_ticks'], after['timestamp_ticks'] - timestamp) > 500000:
        raise ValueError('Skeleton does not bracket camera capture within 50 ms')
    if meta.get('tag_family') != 'tag36h11' or not isinstance(meta.get('tags'), list):
        raise ValueError('AprilTag detection not completed for this frame')
    if meta.get('eye') not in ('left','right'): raise ValueError('Unknown physical camera eye')
    for tag in meta['tags']:
        for key, coordinates in (('position','xyz'),('rotation','xyzw')):
            if not all(math.isfinite(tag[key][axis]) for axis in coordinates):
                raise ValueError('Non-finite AprilTag pose')
    if not math.isfinite(meta['tag_size_m']) or meta['tag_size_m'] <= 0:
        raise ValueError('Invalid configured tag size')
    if not 0 < meta['width'] <= 4096 or not 0 < meta['height'] <= 4096:
        raise ValueError('Invalid RGB dimensions')
    if jpeg is not None and (not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9')):
        raise ValueError('Invalid physical RGB JPEG')
    for name in ('fx', 'fy', 'cx', 'cy'):
        if not math.isfinite(meta[name]): raise ValueError('Invalid intrinsics')
    if meta['fx'] <= 0 or meta['fy'] <= 0: raise ValueError('Unavailable intrinsics')
    bones = meta['bones']
    if not bones or len(bones) != len(before['bones']) or len(bones) != len(after['bones']):
        raise ValueError('Incomplete skeleton')
    for sample in (bones, before['bones'], after['bones']):
        ids = [bone['id'] for bone in sample]
        if len(ids) != len(set(ids)) or ids != [bone['id'] for bone in bones]:
            raise ValueError('Skeleton ID mismatch')
        for bone in sample:
            for key, coordinates in (('position', 'xyz'), ('rotation', 'xyzw')):
                if not all(math.isfinite(bone[key][axis]) for axis in coordinates):
                    raise ValueError('Non-finite skeleton data')
    # Verify the saved aligned positions really correspond to the source brackets.
    span = after['timestamp_ticks'] - before['timestamp_ticks']
    alpha = (timestamp - before['timestamp_ticks']) / span if span else 0
    for bone, left, right in zip(bones, before['bones'], after['bones']):
        for axis in 'xyz':
            expected = left['position'][axis] * (1 - alpha) + right['position'][axis] * alpha
            if not math.isclose(bone['position'][axis], expected, abs_tol=1e-5):
                raise ValueError('Skeleton interpolation mismatch')

class BundleWriter:
    """Direct CFR MP4 plus the same fixed timestep for every saved modality.

    Real camera timestamps/brackets are retained. Missing slots hold the prior
    complete bundle, explicitly labelled repeated; no new detection is invented.
    """
    def __init__(self, folder, fps=30):
        self.folder = Path(folder)
        (self.folder / 'rgb').mkdir()
        self.fps = Fraction(str(fps))
        self.frames = self.source_frames = self.repeated_frames = self.coalesced_frames = 0
        self.tag_detections = 0
        self.last_timestamp = self.origin = None
        self.previous = self.video = self.stream = None
        self.closed = False
        self.files = []
        self.meta_file = self._open('capture.jsonl')
        self.body = self._csv('body.csv', ['time_elapsed','data_type','id','pos_x','pos_y','pos_z',
            'rot_x','rot_y','rot_z','rot_w','timestamp_ticks','frame_id','source_frame_id','repeated'])
        self.tags = self._csv('apriltag.csv', ['timestamp_ticks','frame_id','tag_id','pos_x','pos_y','pos_z',
            'rot_x','rot_y','rot_z','rot_w','tag_size_m','time_elapsed','source_frame_id','repeated'])
        self.times = self._csv('rgb/timestamps.csv', ['file','frame_id','timestamp_ticks','body_timestamp_ticks',
            'apriltag_timestamp_ticks','skeleton_before_ticks','skeleton_after_ticks',
            'video_frame_index','time_elapsed','timestep_ticks','source_frame_id','repeated'])

    def _open(self, name):
        file = (self.folder / name).open('x', encoding='utf-8', newline='')
        self.files.append(file)
        return file

    def _csv(self, name, columns):
        writer = csv.writer(self._open(name)); writer.writerow(columns); return writer

    def write(self, meta, jpeg):
        if self.closed: raise ValueError('Writer already closed')
        validate_bundle(meta, jpeg)
        stamp = meta['timestamp_ticks']
        if self.last_timestamp is not None and stamp <= self.last_timestamp:
            raise ValueError('Camera timestamp regressed')
        if self.last_timestamp is not None and stamp - self.last_timestamp > 2 * TICKS_PER_SECOND:
            raise ValueError('Camera gap exceeds 2 seconds')
        import av
        with av.open(io.BytesIO(jpeg), format='mjpeg') as image:
            decoded = next(image.decode(video=0)).to_ndarray(format='rgb24')
        if decoded.shape[:2] != (meta['height'], meta['width']):
            raise ValueError('JPEG dimensions disagree with metadata')
        if self.origin is None:
            self.origin = stamp
            self.video = av.open(str(self.folder / 'rgb/video.mp4'), 'w')
            self.stream = self.video.add_stream('libx264', rate=self.fps)
            self.stream.width, self.stream.height = meta['width'], meta['height']
            self.stream.pix_fmt = 'yuv420p'
            self.stream.options = {'preset':'veryfast','crf':'20'}
            self.stream.time_base = self.stream.codec_context.time_base = 1 / self.fps
        if (meta['width'], meta['height']) != (self.stream.width, self.stream.height):
            raise ValueError('Camera dimensions changed during recording')
        # Rational arithmetic avoids accumulated drift (30 Hz is not integer ticks).
        target = int(Fraction((stamp - self.origin), TICKS_PER_SECOND) * self.fps + Fraction(1,2))
        self.source_frames += 1
        self.last_timestamp = stamp
        if target < self.frames:
            self.coalesced_frames += 1
            return
        while self.frames < target:
            self._emit(*self.previous, repeated=True)
        self._emit(meta, decoded, repeated=False)
        self.previous = (meta, decoded)

    def _emit(self, source, pixels, repeated):
        import av
        index = self.frames
        elapsed = float(Fraction(index, 1) / self.fps)
        stamp = source['timestamp_ticks']
        meta = dict(source, frame_id=index+1, source_frame_id=source['frame_id'],
                    video_frame_index=index, time_elapsed=elapsed,
                    timestep_ticks=round(Fraction(index*TICKS_PER_SECOND,1)/self.fps),
                    repeated=repeated)
        frame = av.VideoFrame.from_ndarray(pixels, format='rgb24')
        frame.pts = index
        frame.time_base = 1 / self.fps
        for packet in self.stream.encode(frame): self.video.mux(packet)
        for bone in meta['bones']:
            self.body.writerow([elapsed,'bone',bone['id'],*[bone['position'][axis] for axis in 'xyz'],
                *[bone['rotation'][axis] for axis in 'xyzw'],stamp,meta['frame_id'],source['frame_id'],repeated])
        for tag in meta['tags']:
            self.tags.writerow([stamp,meta['frame_id'],tag['id'],*[tag['position'][axis] for axis in 'xyz'],
                *[tag['rotation'][axis] for axis in 'xyzw'],meta['tag_size_m'],elapsed,source['frame_id'],repeated])
        self.times.writerow(['video.mp4',meta['frame_id'],stamp,stamp,stamp,
            meta['before']['timestamp_ticks'],meta['after']['timestamp_ticks'],index,elapsed,
            meta['timestep_ticks'],source['frame_id'],repeated])
        self.meta_file.write(json.dumps(meta, separators=(',',':')) + '\n')
        self.frames += 1
        self.repeated_frames += int(repeated)
        self.tag_detections += len(meta['tags'])

    def close(self):
        if self.closed: return
        self.closed = True
        try:
            if self.video is not None:
                try:
                    for packet in self.stream.encode(): self.video.mux(packet)
                finally: self.video.close()
        finally:
            for file in self.files: file.close()

    def make_video(self):
        self.close()
        import av
        decoded = 0
        with av.open(str(self.folder / 'rgb/video.mp4')) as video:
            for index, frame in enumerate(video.decode(video=0)):
                if abs(float(frame.pts * frame.time_base) - float(Fraction(index,1)/self.fps)) > 1e-6:
                    raise ValueError('MP4 timestep differs from shared data')
                decoded += 1
        if decoded != self.frames: raise ValueError('MP4 frame count differs from capture bundles')
        return {'decoded_frames':decoded, 'fps':float(self.fps), 'storage':'direct_mp4',
                'source_frames':self.source_frames, 'repeated_frames':self.repeated_frames,
                'coalesced_frames':self.coalesced_frames}
