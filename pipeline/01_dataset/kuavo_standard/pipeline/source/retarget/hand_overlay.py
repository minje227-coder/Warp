"""Only the URDF grippers over original RGB, with recorded Quest hand targets.

Camera2's gyro-reference pose is composed with recorded head pose. A constant
preview registration is fitted to image hand landmarks, with temporal holdout.
This is not a verified headset extrinsic, robot action, or contact calibration.
"""
import argparse
from bisect import bisect_left
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import av
import cv2
import mediapipe as mp
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
import trimesh
try:
    from .gripper_signal import binary_signals, linkage_angles
except ImportError:
    from gripper_signal import binary_signals, linkage_angles

C = np.array([[0., 0., 1.], [-1., 0., 0.], [0., 1., 0.]])
D = np.diag([1., 1., -1.])  # Android gyro right-handed -> Unity head axes.
MP_TO_XR = {0: 1, 2: 3, 5: 7, 9: 12, 13: 17, 17: 22}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def serial(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    raise TypeError(type(v).__name__)


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=serial, allow_nan=False) + '\n')


class Camera:
    def __init__(self, calibration, heads):
        if calibration['pose_reference'] != 1:
            raise ValueError('Only gyro-referenced Camera2 extrinsics supported')
        self.heads = heads
        self.ticks = [h['timestamp_ticks'] for h in heads]
        self.lens_rotation = Rotation.from_quat(calibration['lens_pose_rotation']).as_matrix()
        self.lens_translation = np.array(calibration['lens_pose_translation'])
        fx, fy, cx, cy, skew = calibration['intrinsics']
        self.k = np.array([[fx, skew, cx], [0., fy, cy], [0., 0., 1.]])

    def pose(self, ticks):
        i = bisect_left(self.ticks, ticks)
        if i < len(self.ticks) and self.ticks[i] == ticks:
            lo = hi = i
        elif not 0 < i < len(self.ticks):
            raise ValueError('outside_head_tracking')
        else:
            lo, hi = i-1, i
        a, b = self.heads[lo], self.heads[hi]
        if not a['valid'] or not b['valid'] or (self.ticks[hi]-self.ticks[lo])*1e-7 > .05 or a['tracking_origin_pose'] != b['tracking_origin_pose']:
            raise ValueError('invalid_head_tracking')
        w = 0. if lo == hi else (ticks-self.ticks[lo])/(self.ticks[hi]-self.ticks[lo])
        pa = np.array([a['head_pose']['position'][k] for k in 'xyz'])
        pb = np.array([b['head_pose']['position'][k] for k in 'xyz'])
        qa = np.array([a['head_pose']['rotation'][k] for k in 'xyzw'])
        qb = np.array([b['head_pose']['rotation'][k] for k in 'xyzw'])
        rh = Rotation.from_quat(qa).as_matrix() if lo == hi else Slerp([0., 1.], Rotation.from_quat([qa, qb]))([w]).as_matrix()[0]
        # Camera2 documents lens rotation as gyro->camera (not the reverse).
        r = C @ rh @ D @ self.lens_rotation.T
        p = C @ ((1-w)*pa+w*pb+rh @ D @ self.lens_translation)
        return r, p, {'head_lines': [lo+1, hi+1], 'head_ticks': [a['timestamp_ticks'], b['timestamp_ticks']], 'weight': w}

    def project(self, points):
        pixels = np.asarray(points) @ self.k.T
        return pixels[:, :2] / pixels[:, 2:3]


def register(source, rows, camera, model, output):
    """Fit one fixed optical-frame correction; do not snap targets per frame."""
    cache = output / 'registration-observations.json'
    if cache.exists():
        observations = json.loads(cache.read_text())
    else:
        observations = []
        options = mp.tasks.vision.HandLandmarkerOptions(base_options=mp.tasks.BaseOptions(model_asset_path=str(model)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=2,
            min_hand_detection_confidence=.6, min_hand_presence_confidence=.6, min_tracking_confidence=.6)
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as detector, av.open(str(source / 'camera.mp4')) as container:
            for i, frame in enumerate(container.decode(video=0)):
                if i % 15:
                    continue
                row = rows[i]
                try:
                    r, p, _ = camera.pose(row['frame']['tracking_target_ticks'])
                except ValueError:
                    continue
                rgb = frame.to_ndarray(format='rgb24')
                detected = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), round(row['elapsed_s']*1000))
                predictions = {}
                for side, h in row['hands'].items():
                    if h['valid']:
                        wrist = r.T @ (np.array(h['points']['1'])-p)
                        if wrist[2] > .03:
                            predictions[side] = camera.project([wrist])[0]
                available = set(predictions)
                for landmarks in sorted(detected.hand_landmarks, key=lambda l: l[0].x):
                    uv0 = np.array([landmarks[0].x*rgb.shape[1], landmarks[0].y*rgb.shape[0]])
                    if not available:
                        break
                    side = min(available, key=lambda s: np.linalg.norm(predictions[s]-uv0))
                    if np.linalg.norm(predictions[side]-uv0) > 350:
                        continue
                    available.remove(side)
                    hand = row['hands'][side]
                    x, y = [], []
                    for mi, xi in MP_TO_XR.items():
                        if str(xi) not in hand['points']:
                            continue
                        landmark = landmarks[mi]
                        if not 0 <= landmark.x <= 1 or not 0 <= landmark.y <= 1:
                            continue
                        x.append(r.T @ (np.array(hand['points'][str(xi)])-p))
                        y.append([landmark.x*rgb.shape[1], landmark.y*rgb.shape[0]])
                    if len(x) >= 5:
                        observations.append({'frame': i, 'elapsed_s': row['elapsed_s'], 'side': side, 'optical_points': x, 'image_pixels': y})
        save(cache, observations)
    if len(observations) < 12:
        raise ValueError('Insufficient image registration observations')
    # Alternating one-second time blocks. All points from a frame stay together.
    train = [o for o in observations if int(o['elapsed_s']) % 2 == 0]
    test = [o for o in observations if int(o['elapsed_s']) % 2 == 1]
    x = np.concatenate([o['optical_points'] for o in train]);y = np.concatenate([o['image_pixels'] for o in train])
    def residual(params, xx=x, yy=y):
        corrected = xx @ Rotation.from_rotvec(params[:3]).as_matrix().T + params[3:]
        return (camera.project(corrected)-yy).ravel()
    fit = least_squares(residual, np.zeros(6), bounds=(np.r_[[-.5]*3, [-.15]*3], np.r_[[.5]*3, [.15]*3]),
        loss='soft_l1', f_scale=8., max_nfev=200)
    rr, tt = Rotation.from_rotvec(fit.x[:3]).as_matrix(), fit.x[3:]
    metrics = {}
    for label, data in (('train', train), ('heldout_time_blocks', test)):
        error = [np.linalg.norm(camera.project(np.array(o['optical_points']) @ rr.T + tt)-o['image_pixels'], axis=1) for o in data]
        error = np.concatenate(error) if error else np.array([])
        metrics[label] = {'hand_observations': len(data), 'points': len(error), 'median_px': float(np.median(error)), 'p95_px': float(np.percentile(error,95))}
    result = {'rotation': rr, 'translation_m': tt, 'parameters': fit.x, 'metrics': metrics,
        'method': 'constant RGB hand-landmark registration; alternating one-second holdout; no per-frame snapping',
        'calibration_verified': False, 'sensor_clock_verified': False,
        'note': 'Image landmarks are estimated by MediaPipe, not independent ground-truth extrinsics or contact measurements.'}
    save(output / 'projection-registration.json', result)
    return result


def gripper_meshes(urdf, mesh_root, side):
    root = ET.parse(urdf).getroot();s = side[0]
    tcp = f'zarm_{s}7_endeffector_center';tf = {f'zarm_{s}7_link': np.eye(4)}
    def transforms(opened):
        angles=linkage_angles(side,opened);result={f'zarm_{s}7_link':np.eye(4)}
        for _ in range(15):
            for j in root.findall('joint'):
                parent,child=j.find('parent').get('link'),j.find('child').get('link')
                if parent not in result or child in result:continue
                origin=j.find('origin');t=np.eye(4)
                if origin is not None:
                    t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                    t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                if j.get('type')=='revolute' and j.get('name') in angles:
                    a=angles[j.get('name')];limits=j.find('limit')
                    if not float(limits.get('lower'))<=a<=float(limits.get('upper')):raise ValueError('Gripper joint limit')
                    axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(axis*a).as_matrix()
                result[child]=result[parent]@t
        return result
    tf_open=transforms(True)
    for _ in range(15):
        for j in root.findall('joint'):
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent in tf and child not in tf:
                origin = j.find('origin');t = np.eye(4)
                if origin is not None:
                    t[:3,3] = np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                    t[:3,:3] = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                tf[child] = tf[parent] @ t  # All gripper revolute joints at neutral 0.
    meshes, hashes = [], {}
    for link in root.findall('link'):
        name = link.get('name')
        if not name.startswith(s+'_') or not any(k in name for k in ('twofinger', '_bar_', '_finger')):
            continue
        visual = link.find('visual');node = visual.find('geometry/mesh') if visual is not None else None
        if node is None or name not in tf:
            continue
        path = Path(mesh_root)/Path(node.get('filename')).name
        mesh = trimesh.load_mesh(path);hashes[name] = {'path': str(path), 'sha256': digest(path)}
        desired = 400 if 'twofinger' in name else 260 if '_finger' in name else 80
        if len(mesh.faces) > desired:
            mesh = mesh.simplify_quadric_decimation(face_count=desired)
        t = np.linalg.inv(tf[tcp]) @ tf[name]
        origin = visual.find('origin')
        if origin is not None:
            v = np.eye(4);v[:3,3] = np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            v[:3,:3] = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix();t=t@v
        opening=np.linalg.inv(tf_open[tcp])@tf_open[name]
        if origin is not None:opening=opening@v
        open_mesh=mesh.copy();open_mesh.apply_transform(opening)
        mesh.apply_transform(t)
        meshes.append({'name': name, 'vertices': mesh.vertices.copy(), 'open_vertices':open_mesh.vertices.copy(), 'faces': mesh.faces.copy()})
    if not meshes:
        raise ValueError('No gripper meshes')
    return meshes, hashes


def render(rgb, hands, camera_rotation, camera_position, registration, camera, meshes):
    canvas = rgb.copy();mask = np.zeros(rgb.shape[:2],np.uint8);triangles=[];shown=[]
    for side, h in hands.items():
        if not h['valid']:
            continue
        goal = h['candidates']['pinch_palm'];rg,pg=np.array(goal['rotation']),np.array(goal['position'])
        for mesh in meshes[side]:
            world = mesh['open_vertices' if h.get('gripper_open') else 'vertices'] @ rg.T + pg
            optical = (world-camera_position) @ camera_rotation
            optical = optical @ np.array(registration['rotation']).T + registration['translation_m']
            faces=mesh['faces'];valid=np.min(optical[faces][:,:,2],axis=1)>.04
            faces=faces[valid]
            if not len(faces):continue
            vertices = optical[faces];uv=camera.project(vertices.reshape(-1,3)).reshape(-1,3,2)
            normals=np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
            shade=.5+.5*np.abs(normals@np.array([-.3,-.4,-.866]))
            base=np.array([78,202,238]) if side=='left' else np.array([255,164,70])
            for xy,z,brightness in zip(uv,np.mean(vertices[:,:,2],axis=1),shade):
                if not np.isfinite(xy).all() or np.max(np.abs(xy))>20000:continue
                if xy[:,0].max()<0 or xy[:,0].min()>=rgb.shape[1] or xy[:,1].max()<0 or xy[:,1].min()>=rgb.shape[0]:continue
                triangles.append((float(z),np.rint(xy).astype(np.int32),(base*brightness).astype(np.uint8)))
        shown.append(side)
    for _,polygon,color in sorted(triangles,key=lambda t:t[0],reverse=True):
        cv2.fillConvexPoly(canvas,polygon,tuple(int(v) for v in color),lineType=cv2.LINE_AA)
        cv2.fillConvexPoly(mask,polygon,255,lineType=cv2.LINE_AA)
    blended=cv2.addWeighted(rgb,.4,canvas,.6,0)
    result=rgb.copy();result[mask>0]=blended[mask>0]
    return result, shown


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source');parser.add_argument('output');parser.add_argument('--mesh-root',required=True)
    parser.add_argument('--hand-model',required=True);parser.add_argument('--max-seconds',type=float)
    parser.add_argument('--binary-gripper',action='store_true')
    args=parser.parse_args();source,output=Path(args.source),Path(args.output);output.mkdir(parents=True,exist_ok=True)
    video=output/'gripper-on-hands.mp4'
    if video.exists():raise FileExistsError(video)
    started=time.monotonic();cal=json.loads((source/'calibration.json').read_text());manifest=json.loads((source/'manifest.json').read_text())
    if digest(source/'camera.mp4')!=manifest['files']['camera.mp4']['sha256']:raise ValueError('Source RGB checksum mismatch')
    rows=[json.loads(l) for l in (source/'hand_targets.jsonl').open()];heads=[json.loads(l) for l in (source/'head.jsonl').open()]
    gripper_calibration=None
    if args.binary_gripper:
        signals,gripper_calibration=binary_signals(rows);save(output/'gripper-signals.json',signals)
        for row,signal in zip(rows,signals):
            for side,h in row['hands'].items():
                g=signal['hands'][side]
                h['valid']=h['valid'] and g['valid'];h['gripper_open']=g.get('open_target')==1
    camera=Camera(cal,heads);registration=register(source,rows,camera,Path(args.hand_model),output)
    print(json.dumps({'registration':registration},default=serial),flush=True)
    meshes,hashes={},{}
    for side in ('left','right'):meshes[side],hashes[side]=gripper_meshes(source/'kuavo_s63_twofinger.urdf',args.mesh_root,side)
    writer=av.open(str(video),mode='w');stream=writer.add_stream('libx264',rate=30)
    stream.width,stream.height,stream.pix_fmt=cal['width'],cal['height'],'yuv420p'
    stream.time_base=stream.codec_context.time_base=Fraction(1,1000000);stream.options={'crf':'19','preset':'fast'}
    counters={'frames':0,'left':0,'right':0,'invalid_head':0};projections=[]
    with av.open(str(source/'camera.mp4')) as container:
        for i,frame in enumerate(container.decode(video=0)):
            row=rows[i]
            if args.max_seconds is not None and row['elapsed_s']>args.max_seconds:break
            rgb=frame.to_ndarray(format='rgb24');shown=[]
            try:
                rotation,position,head_source=camera.pose(row['frame']['tracking_target_ticks'])
                rgb,shown=render(rgb,row['hands'],rotation,position,registration,camera,meshes)
            except ValueError as exc:
                counters['invalid_head']+=1;head_source={'invalid':str(exc)}
            projections.append({'frame':i,'sensor_timestamp_ns':row['frame']['sensor_timestamp_ns'],'tracking_target_ticks':row['frame']['tracking_target_ticks'],'head_source':head_source,'shown':shown})
            for side in shown:counters[side]+=1
            counters['frames']+=1
            if i in (150,450,900,1200):
                from PIL import Image
                Image.fromarray(rgb).save(output/f'frame-{i:04d}.jpg')
            out=av.VideoFrame.from_ndarray(rgb,format='rgb24');out.pts=round(row['elapsed_s']*1000000);out.time_base=Fraction(1,1000000)
            for packet in stream.encode(out):writer.mux(packet)
            if i%150==0:print(json.dumps({'frame':i,'wall_s':round(time.monotonic()-started,1)}),flush=True)
    for packet in stream.encode():writer.mux(packet)
    writer.close();save(output/'projection-frames.json',projections)
    report={'schema_version':1,'source_episode_id':manifest['episode_id'],'source_rgb_sha256':digest(source/'camera.mp4'),
        'head_sha256':digest(source/'head.jsonl'),'hand_targets_sha256':digest(source/'hand_targets.jsonl'),
        'urdf_sha256':digest(source/'kuavo_s63_twofinger.urdf'),'meshes':hashes,'registration':registration,
        'hand_target':'recorded Quest MCP frame / thumb-index midpoint; no per-frame RGB target snapping',
        'gripper_aperture':'binary source-derived preview; revolute-only KUAVO linkage table; fixed URDF joints remain fixed' if args.binary_gripper else 'neutral 0',
        'gripper_calibration':gripper_calibration,'opened_gripper_collision_verified':False,
        'render':'actual gripper mesh subset, decimated, 60% opacity, left cyan/right orange; no arms or skeleton',
        'counts':counters,'wall_s':time.monotonic()-started,'physical_motion':False,'dataset_ready':False,
        'projection_verified':False,'clock_verified':False,'script_sha256':digest(__file__),'video_sha256':digest(video)}
    save(output/'report.json',report);print(json.dumps({'done':str(video),'counts':counters}),flush=True)


if __name__=='__main__':main()
