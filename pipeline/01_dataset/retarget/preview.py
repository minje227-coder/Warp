"""Headless URDF + human skeleton + original Quest RGB diagnostic MP4."""
import argparse
from bisect import bisect_right
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL', 'egl')
import av
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps
from .input import sha256
from .model import JOINT_NAMES

BODY_EDGES = [(7, 6), (6, 5), (5, 3), (3, 1), (5, 10), (10, 11), (11, 19),
    (5, 15), (15, 16), (16, 45), (1, 70), (70, 71), (71, 73), (73, 76),
    (1, 77), (77, 78), (78, 80), (80, 83)]
HAND_EDGES = [(1, 3), (3, 5), (1, 7), (7, 10)]


def render_model(urdf, cache, binary_gripper=False):
    import trimesh
    tree = ET.parse(urdf)
    root = tree.getroot()
    assets = Path(urdf).resolve().parent
    cache.mkdir(parents=True, exist_ok=True)
    from .gripper_signal import linkage_angles
    gripper_names = set(linkage_angles('left', True)) | set(linkage_angles('right', True)) if binary_gripper else set()
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed' and joint.get('name') not in set(JOINT_NAMES) | gripper_names:
            joint.set('type', 'fixed')
            for child in list(joint):
                if child.tag not in ('origin', 'parent', 'child'):
                    joint.remove(child)
    for link in root.findall('link'):
        for collision in list(link.findall('collision')):
            link.remove(collision)  # Rendering uses visual meshes; solver keeps collisions.
    for mesh in root.iter('mesh'):
        source = (assets / mesh.get('filename')).resolve()
        target = cache / (sha256(source) + '.obj')
        if not target.exists():
            trimesh.load_mesh(source).export(target, file_type='obj')
        mesh.set('filename', str(target))
    extension = root.find('mujoco')
    if extension is None:
        extension = ET.SubElement(root, 'mujoco')
    compiler = extension.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(extension, 'compiler')
    compiler.attrib.update(discardvisual='false', fusestatic='false', balanceinertia='false')
    raw = cache / 'visual.urdf'
    tree.write(raw, encoding='unicode')
    initial = mujoco.MjModel.from_xml_path(str(raw))
    converted = cache / 'visual.xml'
    mujoco.mj_saveLastXML(str(converted), initial)
    tree = ET.parse(converted)
    root = tree.getroot()
    world = root.find('worldbody')
    robot = world.find("body[@name='base_link']")
    if robot is None:
        raise ValueError('URDF renderer did not preserve base_link')
    ET.SubElement(robot, 'freejoint', name='preview_base')
    ET.SubElement(world, 'geom', name='preview_floor', type='plane', size='8 8 .1', rgba='.18 .22 .28 1', contype='0', conaffinity='0')
    ET.SubElement(world, 'light', pos='0 0 5', dir='0 0 -1', diffuse='.8 .8 .8')
    visual = root.find('visual')
    if visual is None:
        visual = ET.SubElement(root, 'visual')
    global_settings = visual.find('global')
    if global_settings is None:
        global_settings = ET.SubElement(visual, 'global')
    global_settings.attrib.update(offwidth='640', offheight='600')
    tree.write(converted, encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(converted))


def geom(scene, kind, position, size, color, end=None):
    if scene.ngeom >= scene.maxgeom:
        raise ValueError('Preview scene capacity exceeded')
    value = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(value, kind, np.array(size, dtype=float), np.array(position, dtype=float), np.eye(3).ravel(), np.array(color, dtype=np.float32))
    if end is not None:
        mujoco.mjv_connector(value, kind, size[0], np.array(position), np.array(end))
    scene.ngeom += 1


def skeleton(scene, points, edges, color, radius):
    for a, b in edges:
        if str(a) in points and str(b) in points:
            geom(scene, mujoco.mjtGeom.mjGEOM_CAPSULE, points[str(a)], [radius, 0, 0], color, points[str(b)])
    for position in points.values():
        geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE, position, [radius * 1.3] * 3, color)


def preview(trajectory, output, maximum_seconds=None, show_diagnostics=False, binary_gripper=False):
    trajectory, output = Path(trajectory).resolve(), Path(output).resolve()
    manifest = json.loads((trajectory / 'manifest.json').read_text())
    source = Path(manifest['source_episode'])
    urdf = Path(manifest['model']['source_urdf'])
    if sha256(urdf) != manifest['model']['source_urdf_sha256']:
        raise ValueError('Source URDF changed since conversion')
    if sha256(trajectory / 'trajectory.jsonl') != manifest['output_checksums']['trajectory.jsonl']:
        raise ValueError('Trajectory checksum mismatch')
    rows = [json.loads(line) for line in (trajectory / 'trajectory.jsonl').open()]
    if not rows:
        raise ValueError('Empty trajectory')
    camera = [json.loads(line) for line in (source / 'camera_frames.jsonl').open()]
    encoded = [json.loads(line) for line in (source / 'encoded_frames.jsonl').open()]
    if sha256(source / 'camera.mp4') != manifest['source_checksums']['camera.mp4']:
        raise ValueError('Source video checksum mismatch')
    # For preview only: callback UNIX minus monotonic uptime, not elapsed-realtime.
    # SENSOR_TIMESTAMP and callback_uptime share this device's uptime basis.
    # Offset/drift still need independent calibration before a training join.
    offsets = [int(r['callback_unix_ms']) * 1000000 - int(r['callback_uptime_ns']) for r in camera]
    offset = int(np.median(offsets))
    sensor = np.array([r['sensor_timestamp_ns'] for r in camera], dtype=np.int64)
    video_ns = []
    for row in encoded:
        ns = int(row['pts_us']) * 1000
        i = int(np.argmin(np.abs(sensor - ns)))
        if abs(int(sensor[i]) - ns) > 2000:
            raise ValueError('Cannot associate encoded frame with exposure')
        video_ns.append(int(sensor[i]) + offset)
    signals, grip_calibration = None, None
    if binary_gripper:
        from .gripper_signal import binary_signals
        grip_rows = []
        for row in rows:
            hands = {}
            for index, side in enumerate(('left','right')):
                available = row.get('grippers') is not None
                hands[side] = {'valid': available, 'reason':row['reason'], 'candidates': {'pinch_width_m':row['grippers'][index]['human_pinch_m']} if available else {}}
            grip_rows.append(dict(index=row['index'],elapsed_s=row['elapsed_s'],hands=hands))
        signals, grip_calibration = binary_signals(grip_rows)
        output.with_suffix('.gripper-signals.json').parent.mkdir(parents=True,exist_ok=True)
        output.with_suffix('.gripper-signals.json').write_text(json.dumps({'calibration':grip_calibration,'frames':signals},indent=2)+'\n')
    model = render_model(urdf, trajectory / ('preview_model_binary' if binary_gripper else 'preview_model'), binary_gripper)
    data = mujoco.MjData(model)
    human_model = mujoco.MjModel.from_xml_string('<mujoco><visual><global offwidth="640" offheight="600"/></visual><worldbody><light pos="0 0 5"/><geom type="plane" size="8 8 .1" rgba=".18 .22 .28 1"/></worldbody></mujoco>')
    human_data = mujoco.MjData(human_model)
    mujoco.mj_forward(human_model, human_data)
    robot_renderer = mujoco.Renderer(model, height=600, width=640)
    human_renderer = mujoco.Renderer(human_model, height=600, width=640)
    indices = [model.joint(n).qposadr[0] for n in JOINT_NAMES]
    camera_view = mujoco.MjvCamera()
    camera_view.azimuth, camera_view.elevation, camera_view.distance = 135., -15., 3.9
    centers = [np.asarray(r['human_points']['body']['1']) for r in rows if 'human_points' in r]
    center = np.median(centers, axis=0) if centers else np.zeros(3)
    camera_view.lookat = [center[0], center[1], .85]
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 17)
    timestamps = [r['utc_ns'] for r in rows]
    fps = 30
    time_scale = manifest.get('execution_time_scale', 1.)
    duration = (timestamps[-1] - timestamps[0]) * 1e-9 * time_scale
    if maximum_seconds is not None:
        duration = min(duration, maximum_seconds)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    temp = output.with_name(output.stem + '.partial.mp4')
    if temp.exists():
        raise FileExistsError(temp)
    source_container = av.open(str(source / 'camera.mp4'))
    decoder = iter(source_container.decode(video=0))
    source_index, image = -1, None
    writer = av.open(str(temp), mode='w')
    stream = writer.add_stream('libx264', rate=fps)
    stream.width, stream.height, stream.pix_fmt = 1920, 800, 'yuv420p'
    stream.options = {'crf': '20', 'preset': 'fast'}
    last_q = None
    counts = {'verified': 0, 'invalid_held': 0, 'diagnostic_only': 0}
    for frame_index in range(int(duration * fps) + 1):
        target_ns = timestamps[0] + round(frame_index * 1e9 / fps / time_scale)
        row_index = max(0, bisect_right(timestamps, target_ns) - 1)
        row = rows[row_index]
        wanted = min(len(video_ns) - 1, max(0, bisect_right(video_ns, target_ns) - 1))
        while source_index < wanted:
            image = next(decoder).to_image()
            source_index += 1
        if image is None:
            image = next(decoder).to_image()
            source_index = 0
        canvas = Image.new('RGB', (1920, 800), '#0f172a')
        draw = ImageDraw.Draw(canvas)
        for i, title in enumerate(('Original Quest RGB', 'Recorded human skeleton', 'Retargeted KUAVO S63 URDF')):
            draw.text((i * 640 + 18, 20), title, fill='white', font=font)
        canvas.paste(ImageOps.contain(image, (640, 600)), (0, 80))
        if row['valid']:
            last_q = (row['q'], row['base_pose'])
            counts['verified'] += 1
        elif row.get('diagnostic_q') is not None and show_diagnostics:
            last_q = (row['diagnostic_q'], row['diagnostic_base_pose'])
            counts['diagnostic_only'] += 1
        else:
            counts['invalid_held'] += 1
        if last_q is not None:
            q, base = last_q
            q = np.array(q)
            if row.get('torso_q') is not None:
                q[:4] = row['torso_q']
            data.qpos[model.joint('preview_base').qposadr[0]:model.joint('preview_base').qposadr[0] + 7] = [base[0], base[1], 0., np.cos(base[2] / 2), 0., 0., np.sin(base[2] / 2)]
            data.qpos[indices] = q
        if signals is not None:
            for side, grip in signals[row_index]['hands'].items():
                if grip['valid']:
                    for name, angle in grip['joint_targets'].items():
                        data.qpos[model.joint(name).qposadr[0]] = angle
        mujoco.mj_forward(model, data)
        human_renderer.update_scene(human_data, camera=camera_view)
        robot_renderer.update_scene(data, camera=camera_view)
        points = row.get('human_points', {})
        skeleton(human_renderer.scene, points.get('body', {}), BODY_EDGES, [.2, .8, 1., 1.], .012)
        for kind in ('XRHandLeft', 'XRHandRight'):
            skeleton(human_renderer.scene, points.get(kind, {}), HAND_EDGES, [1., .7, .15, 1.], .006)
        for goal in row.get('tcp_targets', []):
            for scene in (human_renderer.scene, robot_renderer.scene):
                geom(scene, mujoco.mjtGeom.mjGEOM_SPHERE, goal['position'], [.015] * 3, [1., .4, .1, 1.])
                rot = np.array(goal['rotation'])
                for axis, color in enumerate(([1., 0., 0., 1.], [0., 1., 0., 1.], [.2, .5, 1., 1.])):
                    geom(scene, mujoco.mjtGeom.mjGEOM_CAPSULE, goal['position'], [.003, 0, 0], color, np.array(goal['position']) + rot[:, axis] * .075)
        canvas.paste(Image.fromarray(human_renderer.render()), (640, 80))
        canvas.paste(Image.fromarray(robot_renderer.render()), (1280, 80))
        diagnostic_shown = row.get('diagnostic_q') is not None and show_diagnostics
        status = 'VERIFIED GEOMETRIC FRAME' if row['valid'] else 'INVALID: ' + row['reason'] + (' | fixed-torso diagnostic' if diagnostic_shown else ' | arm pose held')
        draw.text((20, 694), f'source t={row["elapsed_s"]:.2f}s | playback {1 / time_scale:.2f}x | {status}', font=font, fill='#86efac' if row['valid'] else '#fca5a5')
        if row['valid']:
            errors = row['verification']['tcp_errors']
            text = ' | '.join(f'{side}: {e["position_m"] * 1000:.1f}mm / {np.degrees(e["orientation_rad"]):.2f}deg' for side, e in zip(('L TCP', 'R TCP'), errors))
            draw.text((20, 728), text, font=small, fill='white')
        if signals is not None:
            states = ' | '.join(side[0].upper()+': '+str(grip.get('state','unknown')) for side,grip in signals[row_index]['hands'].items())
            draw.text((1300,728), states+' (preview)',font=small,fill='#fbbf24')
        draw.text((20, 766), 'PREVIEW ONLY: camera/tracking alignment estimated; hand axes uncalibrated; no robot commands; not training-ready', font=small, fill='#fbbf24')
        video_frame = av.VideoFrame.from_image(canvas)
        for packet in stream.encode(video_frame):
            writer.mux(packet)
        if frame_index % 150 == 0:
            print(json.dumps({'preview_frame': frame_index, 'duration_s': duration}), flush=True)
        if frame_index == fps * 3:
            canvas.save(output.with_suffix('.png'))
    for packet in stream.encode():
        writer.mux(packet)
    writer.close()
    source_container.close()
    robot_renderer.close()
    human_renderer.close()
    temp.rename(output)
    result = {'video': str(output), 'video_sha256': sha256(output), 'fps': fps, 'duration_s': duration, 'execution_time_scale': time_scale,
        'camera_to_utc_offset_ns_preview_estimate': offset, 'clock_offset_range_ns': max(offsets) - min(offsets),
        'camera_tracking_clock_verified': False, 'frame_counts': counts, 'physical_motion': False,
        'binary_gripper_preview':binary_gripper,'gripper_calibration':grip_calibration,'opened_gripper_collision_verified':False,
        'note': 'CFR is a visualization only. Invalid poses/held panels are labeled and never inserted into the trajectory.'}
    output.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory')
    parser.add_argument('output')
    parser.add_argument('--max-seconds', type=float)
    parser.add_argument('--show-diagnostics', action='store_true', help='Explicitly show invalid diagnostic IK poses; off by default')
    parser.add_argument('--binary-gripper', action='store_true', help='Preview-only binary linkage; arm IK collision model remains neutral gripper')
    args = parser.parse_args()
    print(json.dumps(preview(args.trajectory, args.output, args.max_seconds, args.show_diagnostics, args.binary_gripper), indent=2))


if __name__ == '__main__':
    main()
