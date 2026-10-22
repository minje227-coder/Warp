#!/usr/bin/env python3
"""Prepare IK inputs in an observed robot base_link frame; never control a robot.

Calibration contract: T_base_recording maps recording XYZ to observed base_link.
Only the wheeled base is projected onto the physical ground (z=0, yaw only).
The physical floor stays z=0; recording objects/targets receive the same SE(3).
"""
import argparse
import copy
import hashlib
import json
import math
import bisect
import sys
import subprocess
import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def transform_matrix(value):
    T = np.asarray(value, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-6)):
        raise ValueError('T_base_recording must be a finite rigid 4x4 transform')
    return T


def transform_row(row, T):
    r = copy.deepcopy(row)
    R, t = T[:3, :3], T[:3, 3]
    def point(p): return (R @ np.asarray(p) + t).tolist()
    def vector(v): return (R @ np.asarray(v)).tolist()
    def pose(p):
        for key in ('position', 'raw_position'):
            if key in p: p[key] = point(p[key])
        for key in ('rotation', 'raw_rotation'):
            if key in p: p[key] = (R @ np.asarray(p[key])).tolist()
    for target in r.get('tcp_targets', []):
        pose(target)
        for key in ('body_down_direction', 'body_forward_direction', 'target_offset_world_m'):
            if key in target: target[key] = vector(target[key])
        # camera_mount_side_axis_tcp is tool-local, not a world vector.
    for key in ('torso_placement_target', 'chest_target'):
        if r.get(key): pose(r[key])
    if r.get('adaptive_torso_offset_m') is not None:
        r['adaptive_torso_offset_m'] = vector(r['adaptive_torso_offset_m'])
    for points in r.get('human_points', {}).values():
        for key, value in points.items(): points[key] = point(value)
    if r.get('base_pose') is not None:
        x, y, yaw = r['base_pose']
        p = point([x, y, 0])
        forward = R @ np.array([np.cos(yaw), np.sin(yaw), 0])
        if np.linalg.norm(forward[:2]) < 1e-6:
            raise ValueError('Cannot project recording base heading onto ground')
        r['base_pose'] = [p[0], p[1], float(np.arctan2(forward[1], forward[0]))]
    return r


def transform_world(world, T):
    w = copy.deepcopy(world)
    for sample in w['samples']:
        for obj in sample['world']:
            if obj['id'] == 'ground':
                # This is the actual robot floor, not a tracked recording object.
                obj['position'] = [0, 0, -0.05]
                obj['quaternion_xyzw'] = [0, 0, 0, 1]
                obj['dimensions'] = [100, 100, 0.1]
                continue
            obj['position'] = (T[:3, :3] @ np.asarray(obj['position']) + T[:3, 3]).tolist()
            obj['quaternion_xyzw'] = Rotation.from_matrix(
                T[:3, :3] @ Rotation.from_quat(obj['quaternion_xyzw']).as_matrix()).as_quat().tolist()
    return w


def grasp_lowering_cm(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 30:
        raise ValueError('TCP 하향 보정은 0–30 cm 사이 숫자로 입력하세요.')
    return float(value)


def box_tag_tracks(spec, descriptor, T):
    # PnP lives in the adopted vision runtime; Drake's runtime has no OpenCV.
    runtime=Path('/home/lab6/Warp-review/wilor-kuavo-standard')
    policy=json.loads((runtime/'object_overlay_config.json').read_text())
    request=json.dumps(dict(spec=spec,descriptor=str(descriptor),T=np.asarray(T).tolist()))
    result=subprocess.run([policy['tag_pose_python'],str(Path(__file__).resolve()),'--box-tag-tracks'],
        input=request,text=True,capture_output=True,
        env=dict(os.environ,PYTHONPATH=str(Path(sys.prefix)/'lib'/('python%d.%d'%sys.version_info[:2])/'site-packages')))
    if result.returncode:
        raise ValueError('박스 태그 아래축 추정 실패: '+result.stderr[-2400:])
    tracks,provenance=json.loads(result.stdout)
    for values in tracks.values():
        for row in values:
            for key in ('center','rotation','down','half'):row[key]=np.asarray(row[key])
    return tracks,provenance


def _box_tag_tracks(spec, descriptor, T):
    """Recover full accepted tag axes; never derive them from gravity-flattened boxes.

    Uses the adopted multi-tag estimator and its gates, camera poses at the
    recorded image times, calibrated tag bundles and existing +/-0.3s smoothing.
    No global-up projection is applied to the tag's in-plane downward axis.
    """
    runtime = Path('/home/lab6/Warp-review/wilor-kuavo-standard')
    sys.path.insert(0, str(runtime/'tools'))
    from multi_tag_pose import estimate_tag_bundle
    policy_path = runtime/'object_overlay_config.json'
    policy = json.loads(policy_path.read_text())
    episode = Path(spec['episode'])
    provider = Path(spec['display_source'])
    if not provider.is_absolute():provider = descriptor.parent/provider
    scenes_path = provider/'data/frame-scenes.jsonl'
    tags_path = episode/'processed/apriltag.jsonl'
    calibration_path = episode/'calibration.json'
    cal = json.loads(calibration_path.read_text())
    fx,fy,cx,cy,skew = cal['intrinsics']
    K = np.array([[fx,skew,cx],[0.,fy,cy],[0.,0.,1.]])
    detections = {}
    for line in tags_path.open():
        d=json.loads(line);detections.setdefault(d['decoded_index'],{})[d['tag_id']]=d
    box=policy['box'];settings=policy['multi_tag_pose'];selection=policy['pose_selection']
    tracks={g['id']:[] for g in box['instances']}
    for line in scenes_path.open():
        scene=json.loads(line)
        if not all(k in scene for k in ('camera_rotation','camera_position_m','elapsed_s')):continue
        ds=detections.get(scene['decoded_index'],{})
        for group in box['instances']:
            q=estimate_tag_bundle(ds,{int(k):np.array(v) for k,v in group['tag_offsets_in_front_face_axes_m'].items()},
                box['tag_size_m'],K,np.array(scene['camera_rotation']),np.array(scene['camera_position_m']),
                max_rms_px=selection['maximum_reprojection_rms_px'],min_up=selection['minimum_tag_y_up_component'],
                ransac_error_px=settings['ransac_reprojection_error_px'],ransac_iterations=settings['ransac_iterations'],
                ransac_confidence=settings['ransac_confidence'],
                tag_inplane_rotations_rad={int(k):v for k,v in group.get('tag_inplane_rotations_rad',{}).items()})
            if q is None:continue
            tag_R=np.asarray(q['rotation']);front=tag_R[:,2]
            up=np.array([0.,0.,1.])-front*front[2];up/=np.linalg.norm(up)
            # Same box envelope as the adopted collision world, selection only.
            box_R=np.column_stack((np.cross(up,front),-front,up))
            center=np.asarray(q['position'])-front*box['size_m'][1]/2
            tracks[group['id']].append(dict(t=float(scene['elapsed_s']),center=center,
                box_R=box_R,tag_R=tag_R,tag_ids=q['tag_ids'],method=q['method']))
    smoothed={}
    for key,observations in tracks.items():
        ordered=sorted(observations,key=lambda o:o['t']);records=[]
        for a in ordered:
            nearby=[b for b in ordered if abs(b['t']-a['t'])<=box['half_window_s']+1e-8]
            tag_R=Rotation.from_matrix([b['tag_R'] for b in nearby]).mean().as_matrix()
            box_R=Rotation.from_matrix([b['box_R'] for b in nearby]).mean().as_matrix()
            records.append(dict(t=a['t'],center=T[:3,:3]@np.mean([b['center'] for b in nearby],axis=0)+T[:3,3],
                rotation=T[:3,:3]@box_R,down=-(T[:3,:3]@tag_R[:,1]),
                half=np.array(box['size_m'])/2,tag_ids=a['tag_ids'],method=a['method']))
        smoothed[key]=records
    files=[scenes_path,tags_path,calibration_path,policy_path,runtime/'box_tag_calibration.json',runtime/'tools/multi_tag_pose.py']
    provenance=dict(axis='canonical tag bundle -Y; parallel to tag face',
        smoothing_half_window_s=box['half_window_s'],missing_policy='hold last accepted pose; no prefix extrapolation',
        association='nearest box envelope per hand with 30mm hysteresis',
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    return smoothed,provenance


def lower_along_box_tags(rows, tracks, amount):
    selected={};switches=0
    times={k:[o['t'] for o in v] for k,v in tracks.items()}
    for row in rows:
        if not row.get('input_available',row.get('valid',False)):continue
        candidates={}
        for key,values in tracks.items():
            j=bisect.bisect_right(times[key],float(row['elapsed_s'])+1e-9)-1
            if j>=0:candidates[key]=values[j]
        if row.get('tcp_targets') and not candidates:
            raise ValueError('박스 태그 아래축 없음: source %.6fs; 수직 방향으로 대체하지 않습니다.'%row['elapsed_s'])
        for hand,target in enumerate(row.get('tcp_targets',[])):
            original=np.asarray(target['position'],dtype=float)
            distances={k:float(np.linalg.norm(np.maximum(abs(v['rotation'].T@(original-v['center']))-v['half'],0.))) for k,v in candidates.items()}
            best=min(distances,key=distances.get);old=selected.get(hand)
            if old in distances and distances[old]<=distances[best]+.03:best=old
            if old is not None and old!=best:switches+=1
            selected[hand]=best;pose=candidates[best];offset=pose['down']*amount
            target.setdefault('raw_position',original.tolist())
            target['position']=(original+offset).tolist()
            target['target_offset_world_m']=(np.asarray(target.get('target_offset_world_m',[0.,0.,0.]))+offset).tolist()
            target['tcp_lowering']=dict(frame='box_tag',box_id=best,axis='-Y',
                offset_base_m=offset.tolist(),observation_source_s=pose['t'],
                observation_age_s=max(0.,row['elapsed_s']-pose['t']),tag_ids=pose['tag_ids'],pose_method=pose['method'])
    return switches


def prepare(descriptor, calibration, output, tcp_lowering_cm=0.):
    descriptor, calibration, output = map(lambda p: Path(p).expanduser().resolve(),
                                          (descriptor, calibration, output))
    tcp_lowering_cm = grasp_lowering_cm(tcp_lowering_cm)
    spec = json.loads(descriptor.read_text())
    cal = json.loads(calibration.read_text())
    episode = Path(spec['episode']).resolve()
    if cal.get('episode_id') != episode.name:
        raise ValueError('Calibration belongs to a different recording')
    T = transform_matrix(cal['T_base_recording'])
    def source(key):
        p = Path(spec[key]); return p if p.is_absolute() else descriptor.parent / p
    rows = [json.loads(line) for line in source('reference').read_text().splitlines() if line.strip()]
    transformed = [transform_row(r, T) for r in rows]
    axis_provenance = None
    if tcp_lowering_cm:
        tracks, axis_provenance = box_tag_tracks(spec, descriptor, T)
        axis_provenance['box_association_switches'] = lower_along_box_tags(transformed, tracks, tcp_lowering_cm / 100.)
    world = transform_world(json.loads(source('world').read_text()), T)
    config = json.loads(source('reference_config').read_text())
    # Per-frame poses are transformed above; these offsets are world vectors.
    if 'tcp_target_offset_world_m' in config:
        config['tcp_target_offset_world_m'] = (T[:3, :3] @ config['tcp_target_offset_world_m']).tolist()
    if config.get('world', {}).get('initial_base_pose') is not None:
        config['world']['initial_base_pose'] = transform_row(
            {'base_pose': config['world']['initial_base_pose']}, T)['base_pose']
    config.setdefault('ik', {})['environment_world_json'] = str(output / 'world.json')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'reference.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in transformed))
    (output / 'world.json').write_text(json.dumps(world))
    (output / 'reference-config.json').write_text(json.dumps(config, indent=2))
    result = dict(spec, reference=str(output/'reference.jsonl'), world=str(output/'world.json'),
                  reference_config=str(output/'reference-config.json'))
    result['coordinate_preparation'] = dict(frame=cal.get('frame', 'observed_base_link'),
        T_base_recording=T.tolist(), calibration=str(calibration),
        tcp_lowering_cm=tcp_lowering_cm, tcp_lowering_frame='box_tag',
        tcp_lowering_scope='both TCP targets, all source phases',
        calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest(),
        source_prepared=str(descriptor), base='ground-projected XY/yaw',
        floor='surveyed rack height datum z=0' if cal.get('frame') == 'fixed_rack_height' else 'physical base_link z=0',
        fixed_tag_heights_m=cal.get('fixed_tag_heights_m'),
        height_residual_m=cal.get('height_residual_m'),
        robot_observation_used=cal.get('robot_observation_used', True),
        render='adopted renderer uses prepared coordinate frame')
    if axis_provenance is not None:
        result['coordinate_preparation']['box_tag_lowering'] = axis_provenance
    config['coordinate_preparation'] = result['coordinate_preparation']
    (output/'reference-config.json').write_text(json.dumps(config, indent=2))
    (output/'prepared.json').write_text(json.dumps(result, indent=2))
    return output/'prepared.json'


def main():
    if sys.argv[1:] == ['--box-tag-tracks']:
        request = json.load(sys.stdin)
        result = _box_tag_tracks(request['spec'], Path(request['descriptor']), np.array(request['T']))
        print(json.dumps(result, default=lambda v: v.tolist() if isinstance(v, np.ndarray) else v))
        return
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared', required=True)
    p.add_argument('--calibration', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--tcp-lowering-cm', type=float, default=0.)
    a = p.parse_args()
    print(prepare(a.prepared, a.calibration, a.output, a.tcp_lowering_cm))

if __name__ == '__main__': main()
