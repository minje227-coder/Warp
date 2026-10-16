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
renderer=mujoco.Renderer(model,height=540,width=960);option=mujoco.MjvOption();option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=False
adr=[model.joint(n).qposadr[0] for n in JOINT_NAMES];ba=model.joint('preview_base').qposadr[0];camera_id=model.camera('native_head_view').id
rows=[json.loads(l) for l in (c/'data/source-trajectory.jsonl').open() if json.loads(l)['valid']];ts=np.array([r['elapsed_s'] for r in rows])
retimed=json.load(open(c/'results/retimed.json'));samples=retimed['samples'];et=np.array([s['execution_s'] for s in samples]);phase=np.array([s['source_s'] for s in samples]);xs=np.array([s['x'] for s in samples]);duration=retimed['summary']['duration_s']
mapping=[json.loads(l) for l in (out/'object-overlay/frame_mapping.jsonl').open()];sourcefile=out/'object-overlay/four-views-window30-wholebody-base-0-20.mp4'
scenes={r['external_index']:r for r in ns['read'](src/'data/frame-scenes.jsonl')}
hands=[[1,2,3,4,5],[1,7,8,9,10],[1,12,13,14,15],[1,17,18,19,20],[1,22,23,24,25],[7,12,17,22]];hand_edges=[(a,b) for chain in hands for a,b in zip(chain,chain[1:])]
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
def interp_source(t):
 hi=min(int(np.searchsorted(ts,t)),len(ts)-1);lo=max(0,hi-1);u=0. if hi==lo else np.clip((t-ts[lo])/(ts[hi]-ts[lo]),0,1);a,b=rows[lo],rows[hi]
 hp={kind:{int(k):(1-u)*np.array(v)+u*np.array(b['human_points'][kind][k]) for k,v in a['human_points'][kind].items() if k in b['human_points'][kind]} for kind in ['body','XRHandLeft','XRHandRight']}
 grippers=[(1-u)*a['grippers'][j]['open_fraction']+u*b['grippers'][j]['open_fraction'] for j in range(2)]
 # Source scene rows keep the adopted fixed rack and filled open-top box view.
 sa,sb=scenes.get(a['external_index']),scenes.get(b['external_index'])
 if sa is None:sa=sb
 if sb is None:sb=sa
 objects=[]
 for aa,bb in zip(sa['world'],sb['world']):
  z=dict(aa);z['position']=((1-u)*np.array(aa['position'])+u*np.array(bb['position'])).tolist();z['quaternion_xyzw']=Slerp([0,1],Rotation.from_quat([aa['quaternion_xyzw'],bb['quaternion_xyzw']]))(u).as_quat().tolist();objects.append(z)
 return hp,grippers,objects
def set_pose(x,grippers):
 b=x[:3];data.qpos[:]=model.qpos0;data.qpos[ba:ba+7]=[b[0],b[1],0,np.cos(b[2]/2),0,0,np.sin(b[2]/2)];data.qpos[adr]=x[3:]
 for side,g in zip(['left','right'],grippers):
  for name,a in linkage_angles(side,True).items():data.qpos[model.joint(name).qposadr[0]]=a*g
 mujoco.mj_forward(model,data)
target=c/'results/four-views-parameter-winner.mp4';assert not target.exists();record=[];start=time.perf_counter()
with av.open(str(sourcefile)) as reader,av.open(str(target.with_suffix('.partial.mp4')),'w',options={'movflags':'+faststart'}) as writer:
 decoder=iter(reader.decode(video=0));current=next(decoder);ci=0;next_frame=next(decoder,None)
 stream=writer.add_stream('libx264',rate=30);stream.width=1920;stream.height=1200;stream.pix_fmt='yuv420p';stream.options={'crf':'18','preset':'veryfast','bf':'0','threads':'2'};stream.time_base=stream.codec_context.time_base=Fraction(1,1000000)
 for i in range(int(np.ceil(duration*30))+1):
  t=min(i/30,duration);p=float(np.interp(t,et,phase));x=np.array([np.interp(t,et,xs[:,j]) for j in range(21)])
  while next_frame is not None and ci+1<len(mapping) and mapping[ci+1]['source_s']<=p:
   current=next_frame;ci+=1;next_frame=next(decoder,None)
  image=current.to_image();hp,grippers,objects=interp_source(p);set_pose(x,grippers)
  renderer.update_scene(data,camera=cam,scene_option=option);left=Image.fromarray(renderer.render())
  project_edges(left,hp['body'],BODY_EDGES,position,rotation,(255,65,65,255),7,45)
  for key in ['XRHandLeft','XRHandRight']:project_edges(left,hp[key],hand_edges,position,rotation,(35,145,255,255),3,45)
  image.paste(overlay(left,objects,position,rotation,45),(0,642))
  renderer.update_scene(data,camera='native_head_view',scene_option=option);right=Image.fromarray(renderer.render());hr=data.cam_xmat[camera_id].reshape(3,3);hp_camera=data.cam_xpos[camera_id].copy()
  project_edges(right,hp['body'],BODY_EDGES,hp_camera,hr,(255,65,65,255),4,55)
  for key in ['XRHandLeft','XRHandRight']:project_edges(right,hp[key],hand_edges,hp_camera,hr,(35,145,255,255),3,55)
  image.paste(overlay(right,objects,hp_camera,hr,55),(960,642))
  draw=ImageDraw.Draw(image);draw.rectangle((0,600,1920,641),fill=(16,25,36));draw.text((12,608),f'ORIGINAL LAZY BASE | NO TIME HOLD | TCP 10/20mm | 1 INPUT FRAME BRIDGED | execution {t:6.2f}/{duration:.2f}s | source {p:6.3f}s | jerk OFF | Kalman OFF',font=font,fill=(220,240,225))
  draw.rectangle((0,1182,1920,1200),fill=(15,20,29));draw.text((10,1181),'Native Drake IK + TOPPRA | base + torso + both arms share one time axis | gripper collision OFF | processed aperture + recorded box phase | adopted standard preview',font=font,fill='white')
  frame=av.VideoFrame.from_image(image);frame.pts=round(t*1e6);frame.time_base=Fraction(1,1000000)
  for packet in stream.encode(frame):writer.mux(packet)
  record.append(dict(display_index=i,execution_s=t,source_s=p,source_display_index=ci))
  if i==round(8.773/19.88*duration*30):image.save(c/'results/retimed-preview.jpg')
  if i%150==0:print(json.dumps(dict(stage='render',frame=i,total=int(np.ceil(duration*30))+1)),flush=True)
 for packet in stream.encode():writer.mux(packet)
renderer.close();target.with_suffix('.partial.mp4').rename(target)
(c/'results/video-mapping.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in record))
(c/'results/render-summary.json').write_text(json.dumps(dict(frames=len(record),duration_s=duration,render_wall_s=time.perf_counter()-start,video_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),top_panels='source RGB from matching original frame; input camera cadence retained',bottom_panels='robot rendered at 30Hz from 100Hz native trajectory samples, source skeleton interpolated at the common source phase',jerk_applied=False,kalman_applied=False),indent=2))
print('COMPLETE',flush=True)
