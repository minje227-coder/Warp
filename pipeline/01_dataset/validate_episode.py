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
