"""Prepare all configured boxes from full-episode tags, then hold missing poses."""
from pathlib import Path
import argparse
import bisect
import collections
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from box_shell import box_shell
from ros_tag_pose import estimate_tag_bundle


def read(path):
    return [json.loads(line) for line in path.open() if line.strip()]


def instances(policy):
    box = policy['box']
    return box.get('instances', [dict(id='box_100_A13',
        tag_offsets_in_front_face_axes_m=box['tag_offsets_in_front_face_axes_m'],
        tag_inplane_rotations_rad=box.get('tag_inplane_rotations_rad', {}))])


def hold_at(observations, t):
    """Never fill from the future or interpolate across an unobserved interval."""
    index = bisect.bisect_right([x[0] for x in observations], t) - 1
    if index < 0:
        return None, dict(status='unknown', last_observed_s=None, age_s=None)
    observed_s, value = observations[index]
    age = max(0., t-observed_s)
    return value, dict(status='observed' if age < 1e-8 else 'held',
                      last_observed_s=observed_s, age_s=age)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['root', 'raw', 'manifest', 'fused-root', 'reference-root']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--end', type=float, default=20.)
    args = parser.parse_args()
    root = args.root.resolve()
    (root/'data').mkdir(exist_ok=True, parents=True)
    raw = args.raw.resolve()
    policy_root = Path(__file__).resolve().parent.parent
    policy = json.loads((policy_root/'object_overlay_config.json').read_text())
    box, rack = policy['box'], policy['rack']
    groups = instances(policy)
    assert len({g['id'] for g in groups}) == len(groups)
    box_ids = {int(tid) for g in groups for tid in g['tag_offsets_in_front_face_axes_m']}
    assert len(box_ids) == sum(len(g['tag_offsets_in_front_face_axes_m']) for g in groups)
    manifest = json.loads(args.manifest.read_text())
    alignment = manifest['world_alignment']
    A, offset = np.array(alignment['position_basis']), np.array(alignment['translation_m'])
    cal = json.loads((raw/'calibration.json').read_text())
    fx, fy, cx, cy, skew = cal['intrinsics']
    K = np.array([[fx,skew,cx],[0.,fy,cy],[0.,0.,1.]])
    lens_R = Rotation.from_quat(cal['lens_pose_rotation']).as_matrix()
    lens_t = np.array(cal['lens_pose_translation'])
    flip = np.diag([1.,1.,-1.])
    heads = sorted((h for h in read(raw/'tracking.jsonl') if h['kind']=='head_and_tracking_status'),
                   key=lambda h:h['timestamp_ticks'])
    ht = [h['timestamp_ticks'] for h in heads]
    sensors = read(raw/'camera_frames.jsonl')
    refs = {r['external_index']:r['reference'] for r in read(args.reference_root/raw.name/'meta_frame_reference.jsonl')}
    fused = read(args.fused_root/raw.name/'fused_both.jsonl')
    def tick(index):
        body = refs[index]['body']
        return body['sample']['timestamp_ticks'] - round(body['delta_ms']*10000)
    first = tick(fused[0]['external_index'])
    elapsed = lambda index:(tick(index)-first)*1e-7
    xyz = lambda v:np.array([v[k] for k in 'xyz'])
    def camera(sensor_index):
        target = 621355968000000000 + int(sensors[sensor_index]['callback_unix_ms'])*10000
        i = bisect.bisect_left(ht, target)
        if not 0<i<len(heads):
            raise ValueError('outside_head_tracking')
        left,right = heads[i-1:i+1]
        dt = right['timestamp_ticks']-left['timestamp_ticks']
        if not (left['valid'] and right['valid'] and 0<dt<=500000 and
                left['tracking_origin_pose']==right['tracking_origin_pose']):
            raise ValueError('head_tracking_gap')
        u = (target-left['timestamp_ticks'])/dt
        rh = Slerp([0.,1.],Rotation.from_quat([
            [h['head_pose']['rotation'][k] for k in 'xyzw'] for h in [left,right]
        ]))(u).as_matrix()
        pos = (1-u)*xyz(left['head_pose']['position'])+u*xyz(right['head_pose']['position'])
        return A@rh@flip@lens_R.T,A@(pos+rh@flip@lens_t)+offset
    def estimate(ds, centers, size, Rwc, twc, angles=None):
        settings, selection = policy['multi_tag_pose'],policy['pose_selection']
        return estimate_tag_bundle(ds,centers,size,K,Rwc,twc,
            max_rms_px=selection['maximum_reprojection_rms_px'],
            min_up=selection['minimum_tag_y_up_component'],
            ransac_error_px=settings['ransac_reprojection_error_px'],
            ransac_iterations=settings['ransac_iterations'],ransac_confidence=settings['ransac_confidence'],
            tag_inplane_rotations_rad=angles)
    def audit(q):
        return {k:q[k] for k in ['method','tag_ids','observed_tag_ids','rejected_tag_ids',
                                'corner_count','per_tag_rms_px','joint_failure_reason'] if k in q}
    detections = collections.defaultdict(dict)
    for detection in read(raw/'processed/apriltag.jsonl'):
        if detection.get('detector')!='apt_apriltag_ros':
            raise ValueError('Reprocess tags with apt apriltag_ros; legacy detector data is not accepted')
        detections[detection['decoded_index']][detection['tag_id']] = detection
    decoded = read(raw/'processed/decoded_frames.jsonl')
    rack_offsets = {int(k):np.array(v) for k,v in rack['tag_offsets_in_geometry_tag_axes_m'].items()}
    rack_anchor = rack_offsets[int(rack['tag_id'])]
    rack_centers = {tid:off-rack_anchor for tid,off in rack_offsets.items()}
    rack_observations, frames = [],[]
    tracks = {g['id']:[] for g in groups}
    L,W,H = box['size_m']
    for frame in decoded:
        match = frame.get('camera_match')
        rec = dict(decoded_index=frame['decoded_index'],container_pts_us=frame['container_pts_us'])
        if match is None:
            rec['camera_error']='unmatched_camera_frame'
            frames.append(rec)
            continue
        sensor_index = match['sensor_index']
        t = (621355968000000000+int(sensors[sensor_index]['callback_unix_ms'])*10000-first)*1e-7
        rec['elapsed_s']=t
        try:
            Rwc,twc = camera(sensor_index)
        except ValueError as error:
            rec['camera_error']=str(error)
            frames.append(rec)
            continue
        rec.update(camera_rotation=Rwc.tolist(),camera_position_m=twc.tolist())
        ds = detections[frame['decoded_index']]
        q = estimate(ds,rack_centers,rack['tag_size_m'],Rwc,twc)
        if q is not None:
            rack_observations.append(q)
            rec['rack_observation']=dict(audit(q),rms_px=None,rms_m=q['rms_m'])
        rec['boxes_raw']={}
        for group in groups:
            centers={int(k):np.array(v) for k,v in group['tag_offsets_in_front_face_axes_m'].items()}
            angles={int(k):v for k,v in group.get('tag_inplane_rotations_rad',{}).items()}
            q = estimate(ds,centers,box['tag_size_m'],Rwc,twc,angles)
            if q is None:
                continue
            front=q['rotation'][:,2].copy();front/=np.linalg.norm(front)
            up=np.array([0.,0.,1.])-front*front[2];up/=np.linalg.norm(up)
            rotation=np.column_stack((np.cross(up,front),-front,up))
            value=dict(id=group['id'],position=(q['position']-front*W/2).tolist(),
                       quaternion_xyzw=Rotation.from_matrix(rotation).as_quat().tolist(),
                       dimensions=[L,W,H],**audit(q),rms_px=q['rms'])
            rec['boxes_raw'][group['id']]=value
            tracks[group['id']].append((t,value))
        frames.append(rec)
    assert len(rack_observations)>=10,('rack_observations',len(rack_observations))
    ps=np.array([q['position'] for q in rack_observations]);median=np.median(ps,axis=0)
    distances=np.linalg.norm(ps-median,axis=1);keep=distances<max(.025,3*np.median(distances))
    position=np.median(ps[keep],axis=0)
    rotation=Rotation.from_matrix([q['rotation'] for q,k in zip(rack_observations,keep) if k]).mean()
    position=position-rotation.as_matrix()@rack_anchor
    sys.path.insert(0,str(policy_root/'object_assets'))
    from rack_geometry import load_rack_geometry,build_rack_cuboids,RackPose
    quat=rotation.as_quat()
    geometry=build_rack_cuboids(load_rack_geometry(policy_root/'object_assets/rack.json'),
                              RackPose(tuple(position),tuple(quat[[3,0,1,2]])))
    (root/'data/rack-preview-anchor.json').write_text(json.dumps(dict(position=position.tolist(),
        rotation=rotation.as_matrix().tolist(),status='full_episode_fixed_rack',
        tag_offsets=rack['tag_offsets_in_geometry_tag_axes_m'])))
    fixed=[]
    for name,g in geometry.items():
        q=np.array(g['pose'][3:])
        fixed.append(dict(id='rack_'+name,position=g['pose'][:3],quaternion_xyzw=q[[1,2,3,0]].tolist(),dimensions=g['dims']))
    fixed.append(dict(id='ground',position=[0,0,-.05],quaternion_xyzw=[0,0,0,1],dimensions=[100,100,.1]))
    smoothed={}
    for object_id,observations in tracks.items():
        assert observations,('no_box_observations',object_id)
        smoothed[object_id]=[]
        for t,value in observations:
            nearby=[v for tt,v in observations if abs(tt-t)<=box['half_window_s']+1e-8]
            value=dict(value)
            value['position']=np.mean([v['position'] for v in nearby],axis=0).tolist()
            value['quaternion_xyzw']=Rotation.from_quat([v['quaternion_xyzw'] for v in nearby]).mean().as_quat().tolist()
            smoothed[object_id].append((t,value))
    def scene_at(t, collision=False):
        world=list(fixed); states={}
        for object_id,observations in smoothed.items():
            value,state=hold_at(observations,t)
            states[object_id]=state
            if value is not None:
                world.extend(box_shell(value,box['shell_thickness_m']) if collision else [value])
        return world,states
    scenes=[]
    by_decoded={r['decoded_index']:r for r in frames}
    for pair in read(raw/'processed/paired_frames.jsonl'):
        if pair['external_index'] not in refs or elapsed(pair['external_index'])>args.end+.1:
            continue
        t=elapsed(pair['external_index'])
        rec=dict(by_decoded[pair['quest_decoded_index']])
        rec.update(external_index=pair['external_index'],elapsed_s=t,pts_us=pair['common_pts_us'])
        rec['world'],rec['object_states']=scene_at(t)
        scenes.append(rec)
    samples=[]
    for row in fused:
        t=elapsed(row['external_index'])
        if t<=args.end+.1:
            world,states=scene_at(t,collision=True)
            samples.append(dict(index=row['external_index'],elapsed_s=t,world=world,object_states=states,
                                environment_complete=all(v['status']!='unknown' for v in states.values())))
    complete=all(r['environment_complete'] for r in samples)
    audit_data=dict(take=raw.name,full_episode_frames=len(frames),paired_frames=len(scenes),
        missing_policy='hold_last_valid_pose',before_first_observation='unknown',
        box_half_window_s=box['half_window_s'],box_instances=groups,
        box_observed_frames={key:len(v) for key,v in tracks.items()},
        box_max_observation_gap_s={key:max((b[0]-a[0] for a,b in zip(v,v[1:])),default=0.) for key,v in tracks.items()},
        rack_observations=len(rack_observations),rack_inliers=int(sum(keep)),
        complete_collision_environment_for_prepared_range=complete,
        missing_initial_object_samples=sum(not r['environment_complete'] for r in samples),
        pose_acceptance_gates_unchanged=True,robot_commands_sent=0)
    (root/'data/frame-scenes.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in scenes))
    (root/'data/full-episode-object-observations.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in frames))
    (root/'data/object-audit.json').write_text(json.dumps(audit_data,indent=2)+'\n')
    # Unknown initial geometry must not silently become a collision-free scene.
    if not complete:
        raise ValueError(('initial_box_pose_unknown_for_prepared_range',audit_data['missing_initial_object_samples']))
    (root/'data/world.json').write_text(json.dumps(dict(samples=samples)))
    print(json.dumps(audit_data,indent=2))


if __name__=='__main__':
    main()
