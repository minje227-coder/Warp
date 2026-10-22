import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,sys,time,hashlib
import numpy as np,av,mujoco
from scipy.spatial.transform import Rotation,Slerp
from PIL import Image,ImageDraw,ImageFont
from fractions import Fraction
c=Path(__file__).resolve().parent;policy=json.load(open(c/'POLICY.json'));src=Path(policy['source_candidate']);take=policy['take'];out=src/'results'/take
sys.path.insert(0,str(src));sys.path.insert(0,str(src/'source'))
from retarget.model import JOINT_NAMES
from retarget.preview import BODY_EDGES
from retarget.gripper_signal import linkage_angles
from render_four_views_head_down import project_edges
# Reuse the adopted model, overview camera and object-projection function.
saved_argv=sys.argv;sys.argv=[str(src/'render_objects.py'),'results/'+take]
ns={'__file__':str(src/'render_objects.py'),'__name__':'adopted_render_helpers'}
exec((src/'render_objects.py').read_text().split("target=DEST/")[0],ns)
sys.argv=saved_argv
model,data,cam,overlay=ns['model'],ns['data'],ns['cam'],ns['overlay'];position,rotation=ns['position'],ns['rot']
model.geom_rgba[:,3]=1.;model.vis.quality.offsamples=2
side_camera=mujoco.MjvCamera();side_camera.elevation=-8;side_camera.distance=2.65
renderer=mujoco.Renderer(model,height=540,width=960);option=mujoco.MjvOption();option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=False
adr=[model.joint(n).qposadr[0] for n in JOINT_NAMES];ba=model.joint('preview_base').qposadr[0];camera_id=model.camera('native_head_view').id
rows=[json.loads(l) for l in (c/'data/source-trajectory.jsonl').open() if json.loads(l)['valid']];ts=np.array([r['elapsed_s'] for r in rows])
retimed=json.load(open(c/'results/retimed.json'));samples=retimed['samples'];et=np.array([s['execution_s'] for s in samples]);phase=np.array([s['source_s'] for s in samples]);xs=np.array([s['x'] for s in samples]);duration=retimed['summary']['duration_s']
mapping=[json.loads(l) for l in (out/'object-overlay/frame_mapping.jsonl').open()];sourcefile=out/'object-overlay/four-views-window30-wholebody-base-0-20.mp4'
scenes={r['external_index']:r for r in ns['read'](src/'data/frame-scenes.jsonl')}
coordinate_preparation=json.load(open(c/'data/source-config.json')).get('coordinate_preparation')
if coordinate_preparation:
 T=np.array(coordinate_preparation['T_base_recording']);R,t=T[:3,:3],T[:3,3]
 for scene in scenes.values():
  for obj in scene['world']:
   if obj['id']=='ground':continue
   obj['position']=(R@np.array(obj['position'])+t).tolist()
   obj['quaternion_xyzw']=Rotation.from_matrix(R@Rotation.from_quat(obj['quaternion_xyzw']).as_matrix()).as_quat().tolist()
 # Recenter the same overview camera on the calibrated robot base. Derive its
 # projection from the actual MuJoCo camera, keeping overlays in exact parity.
 original_base=np.array(next(r['base_pose'] for r in ns['rows'] if r.get('valid') and r.get('base_pose') is not None))
 cam.lookat[:2]+=np.array(rows[0]['base_pose'][:2])-original_base[:2]
 renderer.update_scene(data,camera=cam,scene_option=option)
 sc=renderer.scene.camera;position=np.mean([v.pos for v in sc],axis=0)
 forward=np.mean([v.forward for v in sc],axis=0);forward/=np.linalg.norm(forward)
 up=np.mean([v.up for v in sc],axis=0);up/=np.linalg.norm(up)
 rotation=np.column_stack((np.cross(forward,up),up,-forward))
hands=[[1,2,3,4,5],[1,7,8,9,10],[1,12,13,14,15],[1,17,18,19,20],[1,22,23,24,25],[7,12,17,22]];hand_edges=[(a,b) for chain in hands for a,b in zip(chain,chain[1:])]
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
def interp_source(t):
 hi=min(int(np.searchsorted(ts,t)),len(ts)-1);lo=max(0,hi-1);u=0. if hi==lo else np.clip((t-ts[lo])/(ts[hi]-ts[lo]),0,1);a,b=rows[lo],rows[hi]
 hp={kind:{int(k):(1-u)*np.array(v)+u*np.array(b['human_points'][kind][k]) for k,v in a['human_points'][kind].items() if k in b['human_points'][kind]} for kind in ['body','XRHandLeft','XRHandRight']}
 grippers=[(1-u)*a['grippers'][j]['open_fraction']+u*b['grippers'][j]['open_fraction'] for j in range(2)]
 grippers=[0. if g<=.40 else g for g in grippers]
 # Source scene rows keep the adopted fixed rack and filled open-top box view.
 sa,sb=scenes.get(a['external_index']),scenes.get(b['external_index'])
 if sa is None:sa=sb
 if sb is None:sb=sa
 objects=[]
 for aa,bb in zip(sa['world'],sb['world']):
  z=dict(aa);z['position']=((1-u)*np.array(aa['position'])+u*np.array(bb['position'])).tolist();z['quaternion_xyzw']=Slerp([0,1],Rotation.from_quat([aa['quaternion_xyzw'],bb['quaternion_xyzw']]))(u).as_quat().tolist();objects.append(z)
 return hp,grippers,objects
finger_geoms={side:[i for i in range(model.ngeom) if
 (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[i])) or '') in (side[0]+'_f_finger',side[0]+'_b_finger')]
 for side in ('left','right')}
assert all(len(v)==2 for v in finger_geoms.values()),finger_geoms
finger_original_rgba=model.geom_rgba.copy();finger_original_matid=model.geom_matid.copy()
def set_pose(x,grippers):
 b=x[:3];data.qpos[:]=model.qpos0;data.qpos[ba:ba+7]=[b[0],b[1],0,np.cos(b[2]/2),0,0,np.sin(b[2]/2)];data.qpos[adr]=x[3:]
 for side,g in zip(['left','right'],grippers):
  ids=finger_geoms[side]
  model.geom_rgba[ids]=finger_original_rgba[ids];model.geom_matid[ids]=finger_original_matid[ids]
  if g==0.:
   model.geom_rgba[ids]=[1.,.015,.01,1.];model.geom_matid[ids]=-1
  for name,a in linkage_angles(side,True).items():data.qpos[model.joint(name).qposadr[0]]=a*g
 mujoco.mj_forward(model,data)
def show_closed_tips(composite,robot_image):
 raw=np.asarray(robot_image).astype(float)
 red=(raw[:,:,0]>25)&(raw[:,:,0]>raw[:,:,1]*2)&(raw[:,:,0]>raw[:,:,2]*2)
 if np.any(red):
  pixels=np.array(composite);pixels[red]=[255,25,20];composite=Image.fromarray(pixels)
 return composite
target=c/'results/four-views-parameter-winner.mp4';assert not target.exists();record=[];start=time.perf_counter()
with av.open(str(sourcefile)) as reader,av.open(str(target.with_suffix('.partial.mp4')),'w',options={'movflags':'+faststart'}) as writer:
 decoder=iter(reader.decode(video=0));current=next(decoder);ci=0;next_frame=next(decoder,None)
 stream=writer.add_stream('libx264',rate=30);stream.width=1920;stream.height=1200;stream.pix_fmt='yuv420p';stream.options={'crf':'18','preset':'veryfast','bf':'0','threads':'2'};stream.time_base=stream.codec_context.time_base=Fraction(1,1000000)
 for i in range(int(np.ceil(duration*30))+1):
  t=min(i/30,duration);p=float(np.interp(t,et,phase));x=np.array([np.interp(t,et,xs[:,j]) for j in range(21)])
  while next_frame is not None and ci+1<len(mapping) and mapping[ci+1]['source_s']<=p:
   current=next_frame;ci+=1;next_frame=next(decoder,None)
  image=current.to_image();hp,grippers,objects=interp_source(p);set_pose(x,grippers)
  renderer.update_scene(data,camera=cam,scene_option=option);left_robot=Image.fromarray(renderer.render());left=left_robot.copy()
  project_edges(left,hp['body'],BODY_EDGES,position,rotation,(255,65,65,255),7,45)
  for key in ['XRHandLeft','XRHandRight']:project_edges(left,hp[key],hand_edges,position,rotation,(35,145,255,255),3,45)
  image.paste(show_closed_tips(overlay(left,objects,position,rotation,45),left_robot),(0,642))
  side_camera.azimuth=-90+np.rad2deg(x[2])
  side_camera.lookat=np.array([x[0]+.4*np.cos(x[2]),x[1]+.4*np.sin(x[2]),.95])
  renderer.update_scene(data,camera=side_camera,scene_option=option)
  side_cameras=renderer.scene.camera;side_position=np.mean([v.pos for v in side_cameras],axis=0)
  side_forward=np.mean([v.forward for v in side_cameras],axis=0);side_forward/=np.linalg.norm(side_forward)
  side_up=np.mean([v.up for v in side_cameras],axis=0);side_up/=np.linalg.norm(side_up)
  side_rotation=np.column_stack((np.cross(side_forward,side_up),side_up,-side_forward))
  right_robot=Image.fromarray(renderer.render())
  right=show_closed_tips(overlay(right_robot,objects,side_position,side_rotation,45),right_robot)
  side_draw=ImageDraw.Draw(right);side_draw.rectangle((0,0,960,28),fill=(16,25,36))
  side_draw.text((10,4),'SIDE VIEW | human skeleton OFF',font=font,fill='white')
  image.paste(right,(960,642))
  draw=ImageDraw.Draw(image);draw.rectangle((0,600,1920,641),fill=(16,25,36));draw.text((12,608),f'{"OBSERVED BASE_LINK" if coordinate_preparation else "ORIGINAL LAZY BASE"} | NO TIME HOLD | TCP 10/20mm | 1 INPUT FRAME BRIDGED | execution {t:6.2f}/{duration:.2f}s | source {p:6.3f}s | jerk OFF | Kalman OFF',font=font,fill=(220,240,225))
  draw.rectangle((0,1182,1920,1200),fill=(15,20,29));draw.text((10,1181),'Native Drake IK + TOPPRA | base + torso + both arms share one time axis | gripper collision OFF | processed aperture + recorded box phase | adopted standard preview',font=font,fill='white')
  frame=av.VideoFrame.from_image(image);frame.pts=round(t*1e6);frame.time_base=Fraction(1,1000000)
  for packet in stream.encode(frame):writer.mux(packet)
  record.append(dict(display_index=i,execution_s=t,source_s=p,source_display_index=ci))
  if i==round(8.773/19.88*duration*30):image.save(c/'results/retimed-preview.jpg')
  if i%150==0:print(json.dumps(dict(stage='render',frame=i,total=int(np.ceil(duration*30))+1)),flush=True)
 for packet in stream.encode():writer.mux(packet)
renderer.close();target.with_suffix('.partial.mp4').rename(target)
(c/'results/video-mapping.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in record))
(c/'results/render-summary.json').write_text(json.dumps(dict(lower_right='adopted side view: azimuth -90 + base yaw, elevation -8, distance 2.65',human_skeleton_lower_right=False,gripper_close_indicator='red finger contact meshes only',gripper_policy=dict(close_at_open_fraction=.40,target_open_fraction=0.,applied_after_interpolation=True),coordinate_frame='observed_base_link' if coordinate_preparation else 'recording',frames=len(record),duration_s=duration,render_wall_s=time.perf_counter()-start,video_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),top_panels='source RGB from matching original frame; input camera cadence retained',bottom_panels='robot rendered at 30Hz from 100Hz native trajectory samples, source skeleton interpolated at the common source phase',jerk_applied=False,kalman_applied=False),indent=2))
print('COMPLETE',flush=True)
