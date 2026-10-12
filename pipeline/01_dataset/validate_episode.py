#!/usr/bin/env python3
"""Validate skeleton CSV plus required physical RGB episode artifacts."""
import argparse
import csv
import json
from pathlib import Path
from validate_csv import inspect_csv


def inspect_episode(folder: Path, *, min_frames=30, strict_sew=False):
    result = inspect_csv(folder / 'body.csv', min_frames=min_frames, strict_sew=strict_sew)
    meta = json.loads((folder / 'metadata.json').read_text())
    issues = list(result['messages'])
    if meta.get('status') != 'complete':
        issues.append('Episode metadata status is not complete: ' + str(meta.get('error')))
    if meta.get('capture_protocol') in ('warp_capture_v1','warp_capture_v2'):
        try:
            from unified_capture import validate_bundle
            bundles = [json.loads(line) for line in (folder/'capture.jsonl').read_text().splitlines()]
            with (folder/'rgb/timestamps.csv').open() as file:
                rows = list(csv.DictReader(file))
            if len(rows) != len(bundles) or len(rows) != meta['shared_frames']:
                raise ValueError('Shared-frame count mismatch')
            expected_body = {}
            expected_tags = []
            previous = -1
            for bundle, row in zip(bundles, rows):
                if Path(row['file']).name != row['file']: raise ValueError('Invalid RGB filename')
                jpeg = None if meta.get('rgb_storage') == 'direct_mp4' else (folder/'rgb'/row['file']).read_bytes()
                validate_bundle(bundle,jpeg)
                stamp, frame_id = bundle['timestamp_ticks'], bundle['frame_id']
                if stamp < previous or (stamp == previous and not bundle.get('repeated')):
                    raise ValueError('Capture clock regressed or unmarked repeated sample')
                if meta.get('rgb_storage') == 'direct_mp4':
                    index = len(expected_body) // len(bundle['bones'])
                    if (bundle['video_frame_index'] != index or int(row['video_frame_index']) != index
                            or int(row['source_frame_id']) != bundle['source_frame_id']
                            or abs(float(row['time_elapsed']) - bundle['time_elapsed']) > 1e-9
                            or int(row['timestep_ticks']) != bundle['timestep_ticks']
                            or bundle['timestep_ticks'] != round(index*10_000_000/meta['recording_fps'])
                            or abs(bundle['time_elapsed'] - index/meta['recording_fps']) > 1e-9):
                        raise ValueError('Shared fixed timestep mismatch')
                previous = stamp
                if int(row['frame_id']) != frame_id or any(int(row[name]) != stamp for name in
                        ('timestamp_ticks','body_timestamp_ticks','apriltag_timestamp_ticks')):
                    raise ValueError('RGB/skeleton/AprilTag timestamps disagree')
                for bone in bundle['bones']:
                    expected_body[(frame_id,bone['id'])] = (stamp,bone)
                expected_tags.extend((frame_id,tag['id'],stamp) for tag in bundle['tags'])
            with (folder/'body.csv').open() as file:
                body = list(csv.DictReader(file))
            if len(body) != len(expected_body): raise ValueError('Skeleton rows differ from shared bundles')
            seen = set()
            for row in body:
                key = int(row['frame_id']),int(row['id'])
                if key in seen: raise ValueError('Duplicate skeleton row')
                seen.add(key)
                stamp,bone = expected_body[key]
                if meta.get('rgb_storage') == 'direct_mp4' and abs(float(row['time_elapsed']) - (key[0]-1)/meta['recording_fps']) > 1e-9:
                    raise ValueError('Skeleton/data timestep mismatch')
                if int(row['timestamp_ticks']) != stamp: raise ValueError('Skeleton timestamp differs from RGB')
                for axis in 'xyz':
                    if abs(float(row['pos_'+axis])-bone['position'][axis]) > 1e-5:
                        raise ValueError('Skeleton CSV pose differs from shared bundle')
            with (folder/'apriltag.csv').open() as file:
                tags = [(int(row['frame_id']),int(row['tag_id']),int(row['timestamp_ticks'])) for row in csv.DictReader(file)]
            if tags != expected_tags: raise ValueError('AprilTag rows differ from shared RGB frames')
            import av
            with av.open(str(folder/'rgb/video.mp4')) as video:
                decoded = 0
                for index, frame in enumerate(video.decode(video=0)):
                    if meta.get('rgb_storage') == 'direct_mp4':
                        if (frame.width,frame.height) != (bundles[index]['width'],bundles[index]['height']):
                            raise ValueError('Video dimensions disagree with bundle')
                        if abs(float(frame.pts*frame.time_base) - bundles[index]['time_elapsed']) > 1e-6:
                            raise ValueError('Video/data timestep mismatch')
                    decoded += 1
            if decoded != len(bundles): raise ValueError('Video frame count differs from shared capture')
            result['unified'] = {'shared_frames':len(bundles),'tag_detections':len(tags),
                'same_timestamp':True,'clock':meta['clock'],'hardware_synchronized':False}
        except Exception as exc:
            issues.append('Unified validation failed: ' + str(exc))
        result['messages'] = issues
        result['ok'] = result['ok'] and not issues
        return result
    if meta.get('rgb_required'):
        rgb = folder / 'rgb'
        try:
            calibration = json.loads((rgb / 'calibration.json').read_text())
            if calibration.get('source') != 'quest_passthrough_camera2':
                issues.append('RGB source is not the physical Quest Camera2 camera')
            with (rgb / 'timestamps.csv').open() as f:
                rows = list(csv.DictReader(f))
            if len(rows) < 5 or len(rows) != meta['rgb']['frames']:
                issues.append('RGB frame count mismatch or too few frames')
            previous_sensor = previous_receive = -1
            for row in rows:
                name = row['file']
                if Path(name).name != name:
                    raise ValueError('Invalid RGB filename')
                jpeg = (rgb / name).read_bytes()
                if not (jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
                    issues.append('Invalid JPEG: ' + name)
                sensor, received = int(row['sensor_timestamp_ns']), int(row['pc_receive_monotonic_ns'])
                if sensor <= previous_sensor or received <= previous_receive:
                    issues.append('RGB timestamps not strictly increasing')
                previous_sensor, previous_receive = sensor, received
                if calibration.get('timestamp_source') != 1 and row['capture_pc_estimate_ns']:
                    issues.append('Unknown camera clock was assigned a false capture clock alignment')
            import av
            with av.open(str(rgb / 'video.mp4')) as video:
                decoded = sum(1 for _ in video.decode(video=0))
            if decoded != len(rows):
                issues.append('RGB MP4 decoded frame count mismatch')
            result['rgb'] = {'frames':len(rows), 'mp4_decoded_frames':decoded, 'eye':calibration['eye'],
                             'timestamp_source':calibration.get('timestamp_source'), 'hardware_synchronized':False}
        except (OSError, ValueError, KeyError, StopIteration) as exc:
            issues.append('RGB artifact validation failed: ' + str(exc))
    result['messages'] = issues
    result['ok'] = result['ok'] and not issues
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('path',type=Path); p.add_argument('--min-frames',type=int,default=30); p.add_argument('--strict-sew',action='store_true')
    args = p.parse_args()
    try:
        if args.path.is_file():
            result = inspect_csv(args.path,min_frames=args.min_frames,strict_sew=args.strict_sew)
        else:
            result = inspect_episode(args.path,min_frames=args.min_frames,strict_sew=args.strict_sew)
    except Exception as exc:
        result = {'ok':False,'messages':[str(exc)]}
    print('[PASS]' if result['ok'] else '[FAIL]',args.path)
    print(json.dumps(result,indent=2))
    return 0 if result['ok'] else 2

if __name__ == '__main__':
    raise SystemExit(main())
