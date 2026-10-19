"""Preserve UVC MJPEG/YUYV provenance and derive timestamped MP4s without padding."""
from fractions import Fraction
import io
import json
from pathlib import Path


def external_raw_path(source, meta):
    name = {'mjpeg': 'external_camera.mjpeg', 'yuyv422': 'external_camera.yuyv'}.get(meta.get('codec'))
    if name is None or meta.get('raw_file', name) != name:
        raise ValueError('Invalid external camera codec/raw file')
    return Path(source) / name


def _decode_external_frame(data, meta):
    import av
    if meta['codec'] == 'yuyv422':
        import numpy as np
        pixels = np.frombuffer(data, dtype=np.uint8).reshape(meta['height'], meta['width'], 2)
        return av.VideoFrame.from_ndarray(pixels, format='yuyv422')
    with av.open(io.BytesIO(data), format='mjpeg') as capture:
        frames = list(capture.decode(video=0))
    if len(frames) != 1 or (frames[0].width, frames[0].height) != (meta['width'], meta['height']):
        raise ValueError('External JPEG decode/dimensions differ from source metadata')
    return frames[0]


def validate_external_camera(source):
    source = Path(source)
    meta = json.loads((source / 'external_camera.json').read_text())
    result = meta.get('capture_result', {})
    if meta.get('schema_version') != 1 or meta.get('state') != 'complete' or result.get('ok') is not True:
        raise ValueError('External camera source is incomplete')
    if meta.get('codec') not in ('mjpeg', 'yuyv422') or not all(type(meta.get(k)) is int and 0 < meta[k] <= 8192 for k in ('width', 'height')):
        raise ValueError('Invalid external camera dimensions/codec')
    if meta['codec'] == 'yuyv422' and (meta['width'] % 2 or meta['width'] * meta['height'] * 2 > 8*1024**2):
        raise ValueError('Invalid YUYV dimensions')
    for key in ('negotiated_interval_100ns', 'requested_interval_100ns'):
        if key in meta and (type(meta[key]) is not int or meta[key] <= 0):
            raise ValueError('Invalid external camera frame interval')
    rows = []
    end = 0
    previous = None
    raw = external_raw_path(source, meta)
    total_bytes = raw.stat().st_size
    with raw.open('rb') as video, (source / 'external_camera_frames.jsonl').open() as index:
        for line in index:
            if len(line) > 8192 or len(rows) >= 1_000_000:
                raise ValueError('External camera index exceeds limits')
            row = json.loads(line)
            keys = ('index', 'offset', 'bytes', 'host_first_arrival_elapsed_ns', 'host_last_arrival_elapsed_ns')
            if any(type(row.get(k)) is not int for k in keys):
                raise ValueError('External camera index fields must be integers')
            first, last = row['host_first_arrival_elapsed_ns'], row['host_last_arrival_elapsed_ns']
            size = row['bytes']
            if row['index'] != len(rows) or row['offset'] != end or not 4 <= size <= 8*1024**2:
                raise ValueError('External camera index offsets/lengths are invalid')
            if end + size > total_bytes or first <= 0 or first > last or (previous is not None and last <= previous):
                raise ValueError('External camera timestamps or file bounds are invalid')
            if row.get('time_source') != 'usb_payload_arrival_not_exposure':
                raise ValueError('Unsupported external camera time provenance')
            if meta['codec'] == 'yuyv422':
                if size != meta['width'] * meta['height'] * 2:
                    raise ValueError('External YUYV frame length differs from dimensions')
            else:
                video.seek(end)
                if video.read(2) != b'\xff\xd8':
                    raise ValueError('External camera frame lacks JPEG start marker')
                video.seek(end + size - 2)
                if video.read(2) != b'\xff\xd9':
                    raise ValueError('External camera frame lacks JPEG end marker')
            rows.append(row)
            end += size
            previous = last
    if not rows or end != total_bytes or type(result.get('frames')) is not int or len(rows) != result['frames']:
        raise ValueError('External camera frame count or byte count mismatch')
    return meta, rows


def process_external_camera(source, destination):
    import av
    source, destination = Path(source), Path(destination)
    meta, rows = validate_external_camera(source)
    destination.mkdir(parents=True, exist_ok=True)
    video_path = destination / 'external_camera.mp4'
    frame_path = destination / 'external_decoded_frames.jsonl'
    if video_path.exists() or frame_path.exists():
        raise ValueError('External camera derived outputs already exist')
    t0 = rows[0]['host_last_arrival_elapsed_ns']
    encoded = []
    tb = Fraction(1, 1_000_000)
    try:
        with av.open(str(video_path), 'w') as output, external_raw_path(source, meta).open('rb') as raw, frame_path.open('w') as index:
            stream = output.add_stream('libx264', rate=30)
            stream.width, stream.height = meta['width'], meta['height']
            stream.pix_fmt = 'yuv420p'
            stream.time_base = stream.codec_context.time_base = tb
            stream.options = {'crf': '18', 'preset': 'veryfast', 'bf': '0'}
            for row in rows:
                raw.seek(row['offset'])
                frame = _decode_external_frame(raw.read(row['bytes']), meta).reformat(format='yuv420p')
                pts_us = round((row['host_last_arrival_elapsed_ns']-t0)/1000)
                if encoded and pts_us <= encoded[-1]:
                    raise ValueError('External camera MP4 PTS would not be strictly monotonic')
                frame.pts, frame.time_base = pts_us, tb
                for packet in stream.encode(frame):
                    output.mux(packet)
                encoded.append(pts_us)
                index.write(json.dumps(dict(row, pts_us=pts_us, timestamp_basis='host_last_usb_payload_arrival'))+'\n')
            for packet in stream.encode():
                output.mux(packet)
        with av.open(str(video_path)) as rendered:
            decoded = [round(f.pts*f.time_base*1_000_000) for f in rendered.decode(video=0)]
        if decoded != encoded:
            raise ValueError('External MP4 decode timestamps/count differ from source')
        gaps = [(b-a)/1e6 for a,b in zip(encoded, encoded[1:])]
        observed = None if len(rows)<2 else (len(rows)-1)*1e9/(rows[-1]['host_last_arrival_elapsed_ns']-t0)
        quality = {'state': 'needs_exposure_clock_validation', 'frames': len(rows), 'decoded_frames': len(decoded),
                   'width': meta['width'], 'height': meta['height'], 'source_codec': meta['codec'], 'source_arrival_fps': observed,
                   'requested_fps': 1e7/meta.get('requested_interval_100ns', meta['negotiated_interval_100ns']),
                   'negotiated_fps': 1e7/meta['negotiated_interval_100ns'],
                   'maximum_arrival_gap_ms': max(gaps)*1000 if gaps else None,
                   'payload_errors': meta['capture_result'].get('payload_errors'), 'repeated_frame_padding': 0,
                   'time_source': meta['time_source'], 'hardware_synchronized': False, 'dataset_ready': False}
        (destination/'external_quality.json').write_text(json.dumps(quality, indent=2)+'\n')
        return quality
    except Exception:
        video_path.unlink(missing_ok=True)
        frame_path.unlink(missing_ok=True)
        raise




def align_external_frames(source, destination, decoded, sensor, calibration, max_delta_ms=50):
    """Map decoded Quest timesteps to USB source indices on the same host clock.

    REALTIME Camera2 sensor timestamps share elapsedRealtimeNanos. Unknown
    sensor clocks instead use the recorded callback elapsed time. Neither case
    turns USB packet arrival timestamps into camera exposure timestamps.
    """
    from bisect import bisect_left
    from collections import Counter
    if not 0 < max_delta_ms <= 50:
        raise ValueError('External matching tolerance must be in (0,50] ms')
    _, rows = validate_external_camera(source)
    times = [r['host_last_arrival_elapsed_ns'] for r in rows]
    realtime = calibration.get('timestamp_source') == 1
    field = 'sensor_timestamp_ns' if realtime else 'callback_elapsed_ns'
    basis = 'quest_sensor_realtime_to_usb_arrival' if realtime else 'quest_callback_elapsed_to_usb_arrival'
    used, deltas, reasons = set(), [], Counter()
    with (Path(destination)/'external_timestep_matches.jsonl').open('w') as output:
        for frame in decoded:
            match = frame.get('camera_match') or {}
            i = match.get('sensor_index')
            row = {'quest_decoded_index': frame['decoded_index'], 'quest_encoded_index': match.get('encoded_index'),
                   'quest_timestep_elapsed_ns': None, 'external_index': None, 'matched': False,
                   'time_basis': basis, 'exposure_alignment_verified': False}
            if type(i) is not int or not 0 <= i < len(sensor) or type(sensor[i].get(field)) is not int:
                row['reason'] = 'missing_quest_time'
            else:
                target = sensor[i][field]
                row['quest_timestep_elapsed_ns'] = target
                if target < times[0] or target > times[-1]:
                    row['reason'] = 'outside_external_range'
                else:
                    j = bisect_left(times, target)
                    candidates = sorted((k for k in (j-1,j) if 0 <= k < len(times)), key=lambda k: abs(times[k]-target))
                    k = candidates[0]
                    if len(candidates)>1 and abs(times[candidates[1]]-target)==abs(times[k]-target):
                        row['reason'] = 'ambiguous_external_time'
                    elif abs(times[k]-target) > max_delta_ms*1e6:
                        row['reason'] = 'external_gap'
                    else:
                        delta = times[k]-target
                        row.update(matched=True, reason='software_time_match', external_index=rows[k]['index'],
                                   external_timestep_elapsed_ns=times[k], delta_ns=delta,
                                   external_mp4_pts_us=round((times[k]-times[0])/1000),
                                   reused_external_reference=k in used)
                        used.add(k); deltas.append(abs(delta)/1e6)
            reasons[row['reason']] += 1
            output.write(json.dumps(row)+'\n')
    return {'state':'software_time_matched_exposure_alignment_unverified', 'time_basis':basis,
            'quest_frames':len(decoded), 'matched_frames':len(deltas), 'unique_external_frames_used':len(used),
            'reused_external_references':len(deltas)-len(used), 'maximum_match_delta_ms':max(deltas,default=None),
            'match_tolerance_ms':max_delta_ms, 'reasons':dict(reasons), 'exposure_alignment_verified':False,
            'hardware_synchronized':False, 'dataset_ready':False}


def export_paired_videos(source, destination, decoded, sensor, calibration, max_delta_ms=50):
    """Select unique observations onto the external arrival timeline, never pad.

    Both output videos have identical PTS and counts. Original timestamps and
    pixel indices remain explicit; equal output PTS do not verify exposure sync.
    """
    from bisect import bisect_left
    from collections import Counter
    if not 0 < max_delta_ms <= 50:
        raise ValueError('Paired matching tolerance must be in (0,50] ms')
    source, destination = Path(source), Path(destination)
    external_meta, external = validate_external_camera(source)
    field = 'sensor_timestamp_ns' if calibration.get('timestamp_source') == 1 else 'callback_elapsed_ns'
    quest = []
    for frame in decoded:
        match = frame.get('camera_match') or {}
        i = match.get('sensor_index')
        if type(i) is int and 0 <= i < len(sensor) and type(sensor[i].get(field)) is int:
            quest.append((sensor[i][field], frame, sensor[i]))
    times = [q[0] for q in quest]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Paired Quest timestamps are not strictly monotonic')
    pairs, reasons = [], Counter()
    previous = -1
    for row in external:
        target = row['host_last_arrival_elapsed_ns']
        if not times:
            reasons['missing_quest_time'] += 1
            continue
        if target < times[0] or target > times[-1]:
            reasons['outside_quest_range'] += 1
            continue
        j = bisect_left(times, target)
        candidates = sorted((k for k in (j-1, j) if 0 <= k < len(times)), key=lambda k: abs(times[k]-target))
        k = candidates[0]
        if len(candidates) > 1 and abs(times[candidates[1]]-target) == abs(times[k]-target):
            reasons['ambiguous_quest_time'] += 1
            continue
        if abs(times[k]-target) > max_delta_ms*1e6:
            reasons['quest_gap'] += 1
            continue
        if k <= previous:
            reasons['quest_observation_already_used'] += 1
            continue
        previous = k
        _, frame, observation = quest[k]
        pair = {'index': len(pairs), 'common_elapsed_ns': target,
                'quest_decoded_index': frame['decoded_index'],
                'quest_sensor_index': frame['camera_match']['sensor_index'],
                'quest_time_ns': times[k], 'quest_time_field': field,
                'quest_sensor_timestamp_ns': observation.get('sensor_timestamp_ns'),
                'quest_callback_elapsed_ns': observation.get('callback_elapsed_ns'),
                'external_index': row['index'], 'external_arrival_elapsed_ns': target,
                'quest_minus_external_ns': times[k]-target,
                'exposure_alignment_verified': False}
        pairs.append(pair)
    for pair in pairs:
        pair['common_pts_us'] = round((pair['common_elapsed_ns']-pairs[0]['common_elapsed_ns'])/1000)
    if any(b['common_pts_us'] <= a['common_pts_us'] for a, b in zip(pairs, pairs[1:])):
        raise ValueError('Paired output PTS would not be strictly monotonic')
    paths = [destination/name for name in ('paired_quest.mp4', 'paired_external.mp4', 'paired_frames.jsonl', 'paired_quality.json')]
    if any(p.exists() for p in paths):
        raise ValueError('Paired outputs already exist')
    try:
        if pairs:
            _encode_selected_video(source/'camera.mp4', paths[0], pairs, 'quest_decoded_index')
            _encode_selected_video(external_raw_path(source, external_meta), paths[1], pairs, 'external_index', external_meta)
        paths[2].write_text(''.join(json.dumps(p, allow_nan=False)+'\n' for p in pairs))
        quality = {'state': 'paired_software_timeline' if pairs else 'no_valid_pairs',
                   'frames_per_video': len(pairs), 'equal_video_counts_and_pts_verified': bool(pairs),
                   'external_source_frames': len(external), 'quest_source_frames': len(decoded),
                   'excluded_external_frames': len(external)-len(pairs), 'exclusion_reasons': dict(reasons),
                   'unused_quest_frames': len(decoded)-len(pairs), 'reused_source_observations': 0,
                   'repeated_frame_padding': 0, 'timeline': 'external_host_arrival_variable_timestamps',
                   'observed_pair_rate_hz': None if len(pairs) < 2 else (len(pairs)-1)*1e9/(pairs[-1]['common_elapsed_ns']-pairs[0]['common_elapsed_ns']),
                   'quest_time_field': field, 'maximum_match_delta_ms': max((abs(p['quest_minus_external_ns'])/1e6 for p in pairs), default=None),
                   'match_tolerance_ms': max_delta_ms, 'exposure_alignment_verified': False,
                   'hardware_synchronized': False, 'dataset_ready': False}
        paths[3].write_text(json.dumps(quality, indent=2, allow_nan=False)+'\n')
        return quality
    except Exception:
        for p in paths:
            p.unlink(missing_ok=True)
        raise


def _encode_selected_video(source, destination, pairs, index_field, external_meta=None):
    import av
    tb = Fraction(1, 1_000_000)
    selected = iter(pairs)
    wanted = next(selected)
    count = 0
    options = None
    capture_format = 'mjpeg' if source.suffix == '.mjpeg' else None
    if external_meta and external_meta['codec'] == 'yuyv422':
        capture_format = 'rawvideo'
        options = {'pixel_format': 'yuyv422', 'video_size': f"{external_meta['width']}x{external_meta['height']}"}
    with av.open(str(source), format=capture_format, options=options) as capture, av.open(str(destination), 'w') as output:
        stream = None
        for index, frame in enumerate(capture.decode(video=0)):
            if index != wanted[index_field]:
                continue
            if stream is None:
                rate = Fraction((len(pairs)-1)*1_000_000, pairs[-1]['common_pts_us']).limit_denominator(100000) if len(pairs) > 1 else Fraction(30)
                stream = output.add_stream('libx264', rate=rate)
                stream.width, stream.height = frame.width, frame.height
                stream.pix_fmt = 'yuv420p'
                stream.time_base = stream.codec_context.time_base = tb
                stream.options = {'crf': '18', 'preset': 'veryfast', 'bf': '0'}
            frame = frame.reformat(format='yuv420p')
            frame.pts, frame.time_base = wanted['common_pts_us'], tb
            for packet in stream.encode(frame):
                output.mux(packet)
            count += 1
            wanted = next(selected, None)
            if wanted is None:
                break
        if count != len(pairs):
            raise ValueError('Paired source observation missing during decode')
        for packet in stream.encode():
            output.mux(packet)
    with av.open(str(destination)) as rendered:
        pts = [round(f.pts*f.time_base*1_000_000) for f in rendered.decode(video=0)]
    if pts != [p['common_pts_us'] for p in pairs]:
        raise ValueError('Paired output frame counts/PTS differ from common timeline')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    args = parser.parse_args()
    print(json.dumps(process_external_camera(args.source, args.destination), indent=2))
