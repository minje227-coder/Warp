"""Review two virtual gripper targets on raw XR hands alongside Quest RGB.

The 3-D hand views are wrist-centered with fixed world axes. They are NOT a
calibrated projection into RGB pixels or verified robot contact geometry.
"""
import argparse
from bisect import bisect_left
import json
from pathlib import Path
import time
import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps
from .geometry import UNITY_TO_ROS, rotation_error
from .hand_mapping import calibrate_hand, hand_candidates
from .input import interpolate, read_episode, sha256

HAND_CHAINS = ((1, 2, 3, 4, 5), (1, 6, 7, 8, 9, 10),
    (1, 11, 12, 13, 14, 15), (1, 16, 17, 18, 19, 20), (1, 21, 22, 23, 24, 25))
ORANGE, CYAN, PINK = '#ffab40', '#22d3ee', '#f472b6'


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(type(value).__name__)


def dump(path, data):
    path.write_text(json.dumps(data, indent=2, default=serial, allow_nan=False) + '\n')


def world_points(points):
    return {i: UNITY_TO_ROS @ p for i, p in points.items()}


def camera_times(source):
    camera = [json.loads(l) for l in (source / 'camera_frames.jsonl').open()]
    encoded = [json.loads(l) for l in (source / 'encoded_frames.jsonl').open()]
    offsets = [int(r['callback_unix_ms']) * 1000000 - int(r['callback_uptime_ns']) for r in camera]
    offset = int(np.median(offsets))
    sensor = [int(r['sensor_timestamp_ns']) for r in camera]
    result = []
    for i, frame in enumerate(encoded):
        ns = int(frame['pts_us']) * 1000
        j = bisect_left(sensor, ns)
        candidates = [k for k in (j - 1, j) if 0 <= k < len(sensor)]
        match = min(candidates, key=lambda k: abs(sensor[k] - ns))
        if abs(sensor[match] - ns) > 2000:
            raise ValueError('encoded_sensor_identity_mismatch')
        utc = sensor[match] + offset
        result.append({'encoded_index': i, 'camera_frame_number': camera[match]['frame_number'],
            'sensor_timestamp_ns': sensor[match], 'pts_us': int(frame['pts_us']),
            'preview_utc_ns': utc, 'tracking_target_ticks': utc // 100 + 621355968000000000})
    return result, {'method': 'median_callback_unix_minus_uptime; preview_only', 'offset_ns': offset,
        'offset_range_ns': max(offsets) - min(offsets), 'clock_verified': False}


def percentile(values):
    return None if not values else {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)), 'max': float(max(values))}


def hand_statistics(rows, side):
    valid = [r['hands'][side] for r in rows if r['hands'][side]['valid']]
    disagreements = [float(np.linalg.norm(np.array(h['candidates']['pinch_palm']['position']) - h['candidates']['palm_fixed']['position'])) for h in valid]
    orientation = [np.degrees(rotation_error(np.array(h['candidates']['original_pinch']['rotation']), np.array(h['candidates']['pinch_palm']['rotation']))) for h in valid]
    # Compare ~250ms windows, excluding invalid samples. Match small wrist/palm
    # motion with meaningful aperture change; this is a conditional observation.
    trials = []
    for a, b in zip(rows, rows[8:]):
        x, y = a['hands'][side], b['hands'][side]
        if not x['valid'] or not y['valid'] or any(not r['hands'][side]['valid'] for r in rows[a['index']:b['index'] + 1]):
            continue
        dt = b['elapsed_s'] - a['elapsed_s']
        if not .2 <= dt <= .35:
            continue
        xa, ya = x['candidates'], y['candidates']
        wrist_delta = np.linalg.norm(np.array(y['points']['1']) - x['points']['1'])
        palm_angle = rotation_error(np.array(xa['palm_rotation']), np.array(ya['palm_rotation']))
        aperture_delta = abs(ya['pinch_width_m'] - xa['pinch_width_m'])
        if wrist_delta <= .01 and palm_angle <= np.radians(5) and aperture_delta >= .008:
            trials.append({'start_s': a['elapsed_s'], 'end_s': b['elapsed_s'], 'width_change_mm': aperture_delta * 1000,
                'wrist_move_mm': wrist_delta * 1000, 'palm_rotation_deg': np.degrees(palm_angle),
                'old_tcp_rotation_deg': np.degrees(rotation_error(np.array(xa['original_pinch']['rotation']), np.array(ya['original_pinch']['rotation']))),
                'stable_tcp_rotation_deg': np.degrees(rotation_error(np.array(xa['pinch_palm']['rotation']), np.array(ya['pinch_palm']['rotation']))),
                'pinch_tcp_move_mm': float(np.linalg.norm(np.array(xa['pinch_palm']['position']) - ya['pinch_palm']['position']) * 1000),
                'fixed_tcp_move_mm': float(np.linalg.norm(np.array(xa['palm_fixed']['position']) - ya['palm_fixed']['position']) * 1000)})
    return {'valid_review_frames': len(valid), 'total_review_frames': len(rows),
        'pinch_width_mm': percentile([h['candidates']['pinch_width_m'] * 1000 for h in valid]),
        'candidate_position_difference_mm': percentile([v * 1000 for v in disagreements]),
        'old_vs_palm_orientation_difference_deg': percentile(orientation),
        'palm_closing_axis_vs_tip_chord_deg': percentile([h['candidates']['closing_axis_tip_chord_deg'] for h in valid if h['candidates']['closing_axis_tip_chord_deg'] is not None]),
        'small_wrist_palm_motion_aperture_change_windows': {'criteria': '0.2-0.35s; wrist<=10mm; palm<=5deg; width_change>=8mm; all samples valid',
            'count': len(trials), 'metrics': {key: percentile([t[key] for t in trials]) for key in ('old_tcp_rotation_deg', 'stable_tcp_rotation_deg', 'pinch_tcp_move_mm', 'fixed_tcp_move_mm')},
            'examples': sorted(trials, key=lambda t: t['old_tcp_rotation_deg'], reverse=True)[:6]},
        'note': 'Overlapping observational windows, not independent trials, ground-truth contact errors, or IK failure attribution.'}


def panel_view(rows, side):
    right = np.array([.70710678, -.70710678, 0.])
    up = np.array([.24184476, .24184476, .93969262])
    projected = []
    for row in rows:
        h = row['hands'][side]
        if not h['valid']:
            continue
        wrist = np.array(h['points']['1'])
        for p in h['points'].values():
            q = np.asarray(p) - wrist;projected.append([q @ right, q @ up])
        for mode in ('pinch_palm', 'palm_fixed'):
            c = h['candidates'][mode];p, r = np.array(c['position']), np.array(c['rotation'])
            for sign in (-1, 1):
                end = p + sign * r[:, 0] * h['candidates']['pinch_width_m'] / 2
                for point in (end, end + r[:, 2] * .07):
                    q = point - wrist;projected.append([q @ right, q @ up])
    lo, hi = np.min(projected, axis=0), np.max(projected, axis=0)
    scale = min(510 / max(hi[0]-lo[0], .01), 370 / max(hi[1]-lo[1], .01))
    return {'scale_px_per_m': scale, 'center': ((lo + hi) / 2).tolist()}


def hand_panel(image, origin, hand, side, font, small, view, frame_method):
    draw = ImageDraw.Draw(image)
    ox, oy = origin
    draw.rounded_rectangle((ox + 8, oy + 48, ox + 632, oy + 735), radius=16, fill='#172337')
    draw.text((ox + 20, oy + 12), f'{side.upper()} HAND - wrist-centered 3D', font=font, fill='white')
    if not hand['valid']:
        draw.text((ox + 35, oy + 240), 'NO VALID HAND SAMPLE', font=font, fill=PINK)
        draw.text((ox + 35, oy + 280), hand['reason'], font=small, fill=PINK)
        return
    points = {int(k): np.array(p) for k, p in hand['points'].items()}
    wrist = points[1]
    # Fixed, orthographic world camera. Positive world up remains screen up.
    camera_right = np.array([.70710678, -.70710678, 0.])
    camera_up = np.array([.24184476, .24184476, .93969262])
    def project(p):
        q = np.asarray(p) - wrist
        scale = view['scale_px_per_m'];cx, cy = view['center']
        return (float(ox + 320 + scale * (np.dot(q, camera_right)-cx)), float(oy + 290 - scale * (np.dot(q, camera_up)-cy)))
    def line(a, b, color, width=3):
        draw.line((project(a), project(b)), fill=color, width=width)
    for chain in HAND_CHAINS:
        for a, b in zip(chain, chain[1:]):
            if a in points and b in points:
                line(points[a], points[b], '#94a3b8', 3)
    for j, p in points.items():
        x, y = project(p);r = 4 if j in (1, 5, 7, 10, 17) else 2
        draw.ellipse((x-r, y-r, x+r, y+r), fill='white' if j not in (5, 10) else '#fde68a')
    for j, label in ((1, 'wrist'), (5, 'thumb tip'), (10, 'index tip')):
        x, y = project(points[j]);draw.text((x + 7, y + 2), label, font=small, fill='#e2e8f0')
    candidates = hand['candidates']
    line(points[5], points[10], '#fde047', 3)
    for mode, color in (('palm_fixed', CYAN), ('pinch_palm', ORANGE)):
        target = candidates[mode]
        p, r = np.array(target['position']), np.array(target['rotation'])
        closing, approach = r[:, 0], -r[:, 2]
        width = candidates['pinch_width_m']
        # Abstract jaw contact markers. Length is for visibility, not robot STL.
        ends = [p + sign * closing * width / 2 for sign in (-1, 1)]
        backs = [e - approach * .055 for e in ends]
        line(ends[0], backs[0], color, 4);line(ends[1], backs[1], color, 4)
        line(backs[0], backs[1], color, 3)
        line(p - approach * .07, p + approach * .035, color, 2)
        x, y = project(p);draw.ellipse((x-6, y-6, x+6, y+6), outline=color, width=3)
    old = candidates['original_pinch'];p = np.array(old['position'])
    line(p, p - np.array(old['rotation'])[:, 2] * .07, PINK, 3)
    old_angle = np.degrees(rotation_error(np.array(old['rotation']), np.array(candidates['pinch_palm']['rotation'])))
    difference = np.linalg.norm(np.array(candidates['pinch_palm']['position']) - candidates['palm_fixed']['position']) * 1000
    chord = candidates['closing_axis_tip_chord_deg']
    chord_label = 'undefined near closure' if chord is None else f'{chord:.1f}deg'
    frame_label = 'thumb/index MCP' if frame_method == 'mcp_pinch' else 'palm orientation'
    for y, text, color in ((550, 'ORANGE: fingertip midpoint + ' + frame_label, ORANGE),
        (576, 'CYAN: fixed virtual TCP + ' + frame_label, CYAN),
        (602, 'PINK: old fingertip-derived approach direction', PINK),
        (628, 'YELLOW: thumb-to-index fingertip chord', '#fde047'),
        (660, f'aperture {candidates["pinch_width_m"] * 1000:.1f}mm | TCP gap {difference:.1f}mm', 'white'),
        (688, f'old / stable rotation difference {old_angle:.1f}deg', 'white'),
        (714, f'closing axis / fingertip chord: {chord_label}', 'white')):
        draw.text((ox + 20, oy + y), text, font=small, fill=color)
    # Scale: 50mm, fixed view throughout the recording.
    draw.line((ox + 30, oy + 510, ox + 30 + .05 * view['scale_px_per_m'], oy + 510), fill='white', width=3)
    draw.text((ox + 30, oy + 515), '50mm', font=small, fill='white')


def review(source, output, config, reference_start=5., reference_end=7., frame_method='palm_reference'):
    started = time.monotonic()
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError('Review output must be outside raw episode')
    if output.exists():
        raise FileExistsError(output)
    cfg = json.loads(Path(config).read_text())
    raw, streams, discontinuities = read_episode(source)
    frames, clock = camera_times(source)
    reference = {}
    calibration = {}
    for side, kind in (('left', 'XRHandLeft'), ('right', 'XRHandRight')):
        examples, provenance = [], []
        for frame in frames:
            elapsed = (frame['sensor_timestamp_ns'] - frames[0]['sensor_timestamp_ns']) * 1e-9
            if reference_start <= elapsed <= reference_end:
                try:
                    points, origin = interpolate(streams[kind], frame['tracking_target_ticks'], .05, discontinuities)
                    points = world_points(points)
                    # Both constructions must be non-degenerate before fitting.
                    calibrate_hand([points], cfg['hand_to_tcp_rotation'][side], frame_method)
                    examples.append(points);provenance.append(origin)
                except ValueError:
                    pass
        calibration[side] = calibrate_hand(examples, cfg['hand_to_tcp_rotation'][side], frame_method)
        reference[side] = provenance
    output.mkdir(parents=True)
    dump(output / 'reference.json', {'window_s': [reference_start, reference_end], 'calibration': calibration, 'source': reference})
    rows = []
    for frame in frames:
        elapsed = (frame['sensor_timestamp_ns'] - frames[0]['sensor_timestamp_ns']) * 1e-9
        row = {'index': frame['encoded_index'], 'elapsed_s': elapsed, 'frame': frame, 'hands': {}}
        for side, kind in (('left', 'XRHandLeft'), ('right', 'XRHandRight')):
            try:
                points, origin = interpolate(streams[kind], frame['tracking_target_ticks'], .05, discontinuities)
                points = world_points(points)
                candidates = hand_candidates(points, calibration[side], cfg['hand_to_tcp_rotation'][side])
                row['hands'][side] = {'valid': True, 'points': {str(k): v for k, v in points.items()}, 'source': origin, 'candidates': candidates}
            except ValueError as exc:
                row['hands'][side] = {'valid': False, 'reason': str(exc)}
        rows.append(row)
    views = {side: panel_view(rows, side) for side in ('left', 'right')}
    with (output / 'hand_targets.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row, default=serial, allow_nan=False) + '\n')
    report = {'episode': str(source), 'duration_s': rows[-1]['elapsed_s'], 'sensor_frames': raw['sensor_frames'],
        'source_checksums': {k: v['sha256'] for k, v in raw['files'].items()}, 'clock': clock,
        'reference_window_s': [reference_start, reference_end], 'frame_method': frame_method, 'calibration': calibration, 'panel_views': views,
        'hands': {side: hand_statistics(rows, side) for side in ('left', 'right')},
        'camera_pixel_projection_verified': False, 'contact_mapping_verified': False, 'physical_motion': False,
        'dataset_ready': False, 'implementation_sha256': {p.name: sha256(p) for p in (Path(__file__), Path(__file__).with_name('hand_mapping.py'))},
        'note': 'Diagnostic virtual jaw markers, no URDF jaw linkage calibration or IK. Source cadence preserved; camera-tracking UTC join is estimated.'}
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 23)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 17)
    video_path = output / 'hand-comparison.mp4'
    writer = av.open(str(video_path), mode='w');stream = writer.add_stream('libx264', rate=30)
    stream.width, stream.height, stream.pix_fmt = 1920, 880, 'yuv420p'
    stream.options = {'crf': '20', 'preset': 'fast'}
    # Preserve exposure-derived variable cadence; no frame padding/repetition.
    from fractions import Fraction
    stream.time_base = Fraction(1, 1000000)
    stream.codec_context.time_base = Fraction(1, 1000000)
    decoded = 0
    with av.open(str(source / 'camera.mp4')) as container:
        for frame, row in zip(container.decode(video=0), rows):
            canvas = Image.new('RGB', (1920, 880), '#0f172a');draw = ImageDraw.Draw(canvas)
            draw.text((20, 12), 'ORIGINAL QUEST RGB', font=font, fill='white')
            rgb = ImageOps.contain(frame.to_image(), (640, 640));canvas.paste(rgb, ((640-rgb.width)//2, 65))
            draw.text((20, 726), f'source t={row["elapsed_s"]:.2f}s | frame {row["index"]}', font=font, fill='white')
            hand_panel(canvas, (640, 0), row['hands']['left'], 'left', font, small, views['left'], frame_method)
            hand_panel(canvas, (1280, 0), row['hands']['right'], 'right', font, small, views['right'], frame_method)
            draw.text((20, 787), f'Frame: {frame_method} | Reference: grasp pose {reference_start:g}-{reference_end:g}s. Fixed world view; wrist translation removed.', font=small, fill='white')
            draw.text((20, 820), 'VIRTUAL TARGET REVIEW: camera timing estimated | no calibrated RGB overlay | jaw markers are schematic | tool/contact axes uncalibrated', font=small, fill='#fbbf24')
            draw.text((20, 848), 'No robot commands. Missing/invalid tracking is shown empty. Production IK mapping is unchanged.', font=small, fill='#cbd5e1')
            video_frame = av.VideoFrame.from_image(canvas)
            video_frame.pts = round(row['elapsed_s'] * 1000000);video_frame.time_base = Fraction(1, 1000000)
            for packet in stream.encode(video_frame):
                writer.mux(packet)
            if row['index'] in (150, 450, 900, 1200):
                canvas.save(output / f'frame-{row["index"]:04d}.jpg')
            decoded += 1
            if decoded % 300 == 0:
                print(json.dumps({'frames_rendered': decoded, 'total': len(rows), 'wall_s': round(time.monotonic()-started, 2)}), flush=True)
    for packet in stream.encode():
        writer.mux(packet)
    writer.close()
    if decoded != len(rows):
        raise ValueError('decoded_frame_count_mismatch')
    report.update(rendered_frames=decoded, wall_s=time.monotonic()-started,
        output_checksums={p.name: sha256(p) for p in (video_path, output / 'hand_targets.jsonl', output / 'reference.json')})
    dump(output / 'report.json', report)
    print(json.dumps(report, default=serial), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source');parser.add_argument('output');parser.add_argument('--config', required=True)
    parser.add_argument('--reference-start', type=float, default=5.)
    parser.add_argument('--reference-end', type=float, default=7.)
    parser.add_argument('--frame-method', choices=('palm_reference', 'mcp_pinch'), default='palm_reference')
    args = parser.parse_args()
    if not 0 <= args.reference_start < args.reference_end:
        raise ValueError('Invalid reference window')
    review(args.source, args.output, args.config, args.reference_start, args.reference_end, args.frame_method)


if __name__ == '__main__':
    main()
