from pathlib import Path
import json,sys,hashlib,collections
import numpy as np,cv2
from scipy.spatial.transform import Rotation,Slerp
from box_shell import box_shell
from multi_tag_pose import estimate_tag_bundle
import argparse
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--fused-root',type=Path,required=True);p.add_argument('--reference-root',type=Path,required=True);p.add_argument('--end',type=float,default=20.);args=p.parse_args()
c=args.root.resolve();b=c.parent;raw=args.raw.resolve();take=raw.name;(c/'data').mkdir(exist_ok=True)

POLICY_ROOT=Path(__file__).resolve().parent.parent
object_policy=json.loads((POLICY_ROOT/'object_overlay_config.json').read_text())
box_policy=object_policy['box'];rack_policy=object_policy['rack']
rack_offsets={int(k):np.asarray(v,dtype=float) for k,v in rack_policy['tag_offsets_in_geometry_tag_axes_m'].items()}
rack_anchor_id=int(rack_policy['tag_id']);rack_anchor_offset=rack_offsets[rack_anchor_id]
half_window_s=float(box_policy['half_window_s'])

read=lambda p:[json.loads(x) for x in p.open()]
manifest=json.loads((args.manifest).read_text());A=np.array(manifest['world_alignment']['position_basis']);offset=np.array(manifest['world_alignment']['translation_m'])
cal=json.loads((raw/'calibration.json').read_text());fx,fy,cx,cy,skew=cal['intrinsics'];K=np.array([[fx,skew,cx],[0,fy,cy],[0,0,1.]])
lens_R=Rotation.from_quat(cal['lens_pose_rotation']).as_matrix();lens_t=np.array(cal['lens_pose_translation']);flip=np.diag([1.,1.,-1.])
heads=sorted([x for x in read(raw/'tracking.jsonl') if x['kind']=='head_and_tracking_status'],key=lambda h:h['timestamp_ticks']);ht=np.array([h['timestamp_ticks'] for h in heads],dtype=np.int64);sensors=read(raw/'camera_frames.jsonl')
xyz=lambda x:np.array([x[k] for k in 'xyz'])
def camera(pair):
 sensor=sensors[pair['quest_sensor_index']];target=621355968000000000+int(sensor['callback_unix_ms'])*10000;i=int(np.searchsorted(ht,target))
 if not 0<i<len(heads):raise ValueError('outside_head_tracking')
 a,z=heads[i-1:i+1];dt=z['timestamp_ticks']-a['timestamp_ticks']
 if not (a['valid'] and z['valid'] and dt<=500000 and a['tracking_origin_pose']==z['tracking_origin_pose']):raise ValueError('head_tracking_gap')
 u=(target-a['timestamp_ticks'])/dt;rh=Slerp([0,1],Rotation.from_quat([[h['head_pose']['rotation'][k] for k in 'xyzw'] for h in [a,z]]))(u).as_matrix();p=(1-u)*xyz(a['head_pose']['position'])+u*xyz(z['head_pose']['position'])
 return A@rh@flip@lens_R.T,A@(p+rh@flip@lens_t)+offset
refs={x['external_index']:x['reference'] for x in read(args.reference_root/take/'meta_frame_reference.jsonl')};fused=read(args.fused_root/take/'fused_both.jsonl')
def tick(i):
 r=refs[i]['body'];return r['sample']['timestamp_ticks']-round(r['delta_ms']*10000)
first=tick(fused[0]['external_index']);elapsed=lambda i:(tick(i)-first)*1e-7
pairs=[x for x in read(raw/'processed/paired_frames.jsonl') if x['external_index'] in refs and elapsed(x['external_index'])<=args.end+.1]
detections=collections.defaultdict(dict)
for r in read(raw/'processed/apriltag.jsonl'):detections[r['decoded_index']][r['tag_id']]=r
L,W,H=box_policy['size_m'];tag=box_policy['tag_size_m'];white=box_policy['paper_margin_m'];h=tag/2
def bundle_observation(ds,centers,size,Rwc,twc):
 settings=object_policy['multi_tag_pose'];selection=object_policy['pose_selection']
 return estimate_tag_bundle(ds,centers,size,K,Rwc,twc,
  max_rms_px=selection['maximum_reprojection_rms_px'],min_up=selection['minimum_tag_y_up_component'],
  ransac_error_px=settings['ransac_reprojection_error_px'],ransac_iterations=settings['ransac_iterations'],
  ransac_confidence=settings['ransac_confidence'])
def observation_audit(q):
 return {k:q[k] for k in ['method','tag_ids','observed_tag_ids','rejected_tag_ids','corner_count','per_tag_rms_px','joint_failure_reason'] if k in q}
rack_centers={tid:off-rack_anchor_offset for tid,off in rack_offsets.items()}
box_centers={int(tid):np.asarray(off,dtype=float) for tid,off in box_policy['tag_offsets_in_front_face_axes_m'].items()}

frames=[];rack_obs=[];boxes=[]
for pair in pairs:
 t=elapsed(pair['external_index']);rec=dict(external_index=pair['external_index'],elapsed_s=t,pts_us=pair['common_pts_us'])
 try:Rwc,twc=camera(pair)
 except ValueError as e:
  rec['camera_error']=str(e);frames.append(rec);continue
 rec.update(camera_rotation=Rwc.tolist(),camera_position_m=twc.tolist());ds=detections[pair['quest_decoded_index']]
 rack_q=bundle_observation(ds,rack_centers,rack_policy['tag_size_m'],Rwc,twc)
 if rack_q is not None:
  rack_obs.append(rack_q);rec['rack_observation']=dict(observation_audit(rack_q),rms_px=rack_q['rms'])
 box_q=bundle_observation(ds,box_centers,tag,Rwc,twc)
 if box_q is not None:
  q=box_q;front=q['rotation'][:,2].copy();front/=np.linalg.norm(front);up=np.array([0.,0.,1.])-front*front[2];up/=np.linalg.norm(up);along=np.cross(up,front);R=np.column_stack((along,-front,up));center=q['position']-front*W/2
  rec['box_raw']=dict(id='box_100_A13',position=center.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist(),dimensions=[L,W,H],**observation_audit(q),rms_px=q['rms']);boxes.append((t,rec['box_raw']))
 frames.append(rec)
assert len(rack_obs)>=10,('rack_observations',len(rack_obs));ps=np.array([x['position'] for x in rack_obs]);med=np.median(ps,axis=0);dist=np.linalg.norm(ps-med,axis=1);keep=dist<max(.025,3*np.median(dist));rp=np.median(ps[keep],axis=0);rr=Rotation.from_matrix([x['rotation'] for x,k in zip(rack_obs,keep) if k]).mean()
sys.path.insert(0,str(POLICY_ROOT/'object_assets'));from rack_geometry import load_rack_geometry,build_rack_cuboids,RackPose
# Adopted layout: outer paper edges at inner pillars; center tags 4/5 between 0/1 and 2/3.
rp=rp-rr.as_matrix()@rack_anchor_offset
(c/'data/rack-preview-anchor.json').write_text(json.dumps(dict(position=rp.tolist(),rotation=rr.as_matrix().tolist(),status='user_adopted_inner_pillar_shelf_front_midpoint',tag_offsets=rack_policy['tag_offsets_in_geometry_tag_axes_m'],white_margin_m=rack_policy['paper_margin_m'])))
q=rr.as_quat();rack=build_rack_cuboids(load_rack_geometry(POLICY_ROOT/'object_assets/rack.json'),RackPose(tuple(rp),tuple(q[[3,0,1,2]])))
fixed=[]
for name,g in rack.items():
 q=np.array(g['pose'][3:]);fixed.append(dict(id='rack_'+name,position=g['pose'][:3],quaternion_xyzw=q[[1,2,3,0]].tolist(),dimensions=g['dims']))
fixed.append(dict(id='ground',position=[0,0,-.05],quaternion_xyzw=[0,0,0,1],dimensions=[100,100,.1]))
bt=np.array([t for t,x in boxes]);smooth=[]
for t,x in boxes:
 chosen=[z for tt,z in boxes if abs(tt-t)<=half_window_s+1e-8];z=dict(x);z['position']=np.mean([v['position'] for v in chosen],axis=0).tolist();z['quaternion_xyzw']=Rotation.from_quat([v['quaternion_xyzw'] for v in chosen]).mean().as_quat().tolist();smooth.append(z)
bp=np.array([x['position'] for x in smooth]);rots=Slerp(bt,Rotation.from_quat([x['quaternion_xyzw'] for x in smooth]));gap=float(max(np.diff(bt)));assert gap<=.15,('box_pose_gap_s',gap)
def at(t):
 u=float(np.clip(t,bt[0],bt[-1]));return dict(id='box_100_A13',position=[float(np.interp(u,bt,bp[:,j])) for j in range(3)],quaternion_xyzw=rots(u).as_quat().tolist(),dimensions=[L,W,H])
for r in frames:r['world']=fixed+[at(r['elapsed_s'])]
samples=[dict(index=r['external_index'],elapsed_s=elapsed(r['external_index']),world=fixed+box_shell(at(elapsed(r['external_index'])),.001)) for r in fused if elapsed(r['external_index'])<=args.end+.1]
(c/'data/world.json').write_text(json.dumps(dict(samples=samples)));(c/'data/frame-scenes.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in frames))
audit=dict(box_half_window_s=half_window_s,box_full_window_s=2*half_window_s,take=take,source_window_s=[0,args.end],paired_frames=len(frames),box_observed_frames=len(boxes),box_max_observation_gap_s=gap,box_pose_methods=dict(collections.Counter(x['method'] for t,x in boxes)),box_reprojection_rms_px=dict(median=float(np.median([x['rms_px'] for t,x in boxes])),maximum=float(max(x['rms_px'] for t,x in boxes))),rack_observations=len(rack_obs),rack_inliers=int(sum(keep)),box_tag_black_side_m=tag,white_margin_m=white,white_margin_is_approximate=False,paper_side_m=tag+2*white,tag_center_from_top_and_outer_side_m=h+white,old_box_tag7_disabled=True)
audit.update(rack_tag_ids=sorted(rack_offsets),rack_observations_by_tag=dict(collections.Counter(tid for x in rack_obs for tid in x['tag_ids'])),rack_pose_methods=dict(collections.Counter(x['method'] for x in rack_obs)),rack_joint_failure_frames=sum(x['method']=='single_tag_fallback' for x in rack_obs),box_joint_failure_frames=sum(x['method']=='single_tag_fallback' for _,x in boxes),rack_tag_offsets_in_geometry_tag_axes_m=rack_policy['tag_offsets_in_geometry_tag_axes_m'])
(c/'data/object-audit.json').write_text(json.dumps(audit,indent=2));print(json.dumps(audit,indent=2))
