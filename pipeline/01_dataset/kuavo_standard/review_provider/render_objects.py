import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
from fractions import Fraction
import json,sys,hashlib
import numpy as np,av,mujoco
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
C=Path(__file__).resolve().parent;OUT=C/sys.argv[1];DEST=OUT/'object-overlay';DEST.mkdir(exist_ok=True)
read=lambda p:[json.loads(x) for x in p.open()]
rows=read(OUT/'trajectory.jsonl');by={x['external_index']:x for x in rows};scenes={r['external_index']:r for r in read(C/'data/frame-scenes.jsonl')};mapping=read(OUT/'head-down-20-frame-mapping.jsonl')
sys.path.insert(0,str(C/'source'));sys.path.insert(0,str(C))
from retarget.model import JOINT_NAMES
from retarget.gripper_signal import linkage_angles
from display_state import make_display_states
states=make_display_states(rows)
model=mujoco.MjModel.from_xml_path(str(OUT/'overlay-model/four-view-head-down-20.xml'));data=mujoco.MjData(model);model.vis.global_.fovy=45
adr=[model.joint(n).qposadr[0] for n in JOINT_NAMES];ba=model.joint('preview_base').qposadr[0];shoulders=[model.body('zarm_'+s+'1_link').id for s in ['l','r']]
def setpose(r):
 data.qpos[:]=model.qpos0;s=states[r['external_index']]
 if s['q'] is not None:
  b=s['base'];data.qpos[ba:ba+7]=[b[0],b[1],0,np.cos(b[2]/2),0,0,np.sin(b[2]/2)];data.qpos[adr]=s['q']
  for side,g in zip(['left','right'],s['grippers']):
   for name,a in linkage_angles(side,True).items():data.qpos[model.joint(name).qposadr[0]]=a*g['open_fraction']
 mujoco.mj_forward(model,data)
centers=[]
for r in rows:
 if r['valid']:setpose(r);centers.append(np.mean(data.xpos[shoulders],axis=0))
cam=mujoco.MjvCamera();cam.azimuth=145;cam.elevation=-12;cam.distance=3.35;cam.lookat=np.r_[np.median(np.array(centers)[:,:2],axis=0),.85]
renderer=mujoco.Renderer(model,height=540,width=960);renderer.update_scene(data,camera=cam);sc=renderer.scene.camera;position=np.mean([x.pos for x in sc],axis=0);forward=np.mean([x.forward for x in sc],axis=0);forward/=np.linalg.norm(forward);up=np.mean([x.up for x in sc],axis=0);up/=np.linalg.norm(up);rot=np.column_stack((np.cross(forward,up),up,-forward));renderer.close()
manifest=json.loads((OUT/'manifest.json').read_text());raw=Path(manifest['source_episode']);fx,fy,cx,cy,skew=json.loads((raw/'calibration.json').read_text())['intrinsics'];K=np.array([[fx,skew,cx],[0,fy,cy],[0,0,1.]])
signs=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]]);faces=[[0,2,3,1],[4,5,7,6],[0,1,5,4],[2,6,7,3],[0,4,6,2]];edges=[(a,b) for a in range(8) for b in range(a+1,8) if bin(a^b).count('1')==1]
def clip(poly):
 out=[]
 for a,b in zip(poly,poly[1:]+poly[:1]):
  ia=a[2]>=.025;ib=b[2]>=.025
  if ia:out.append(a)
  if ia!=ib:out.append(a+(.025-a[2])/(b[2]-a[2])*(b-a))
 return out
def overlay(panel,objects,p,R,fov=None):
 image=panel.convert('RGBA');layer=Image.new('RGBA',image.size);fill=ImageDraw.Draw(layer);lines=[]
 def project(q):
  if fov is None:
   uv=q@K.T;return np.clip(uv[:,:2]/uv[:,2:]*540/1280,-100000,100000)
  focal=image.height/2/np.tan(np.deg2rad(fov/2));return np.clip(q[:,:2]/q[:,2:]*focal+np.array([image.width/2,image.height/2]),-100000,100000)
 for obj in objects:
  if obj['id']=='ground':continue
  points=signs*np.array(obj['dimensions'])/2@Rotation.from_quat(obj['quaternion_xyzw']).as_matrix().T+np.array(obj['position']);q=(points-p)@R
  if fov is not None:q[:,1:]*=-1
  box=obj['id'].startswith('box_');color=(255,190,30,255) if box else (30,240,155,230)
  if box:
   for face in sorted(faces,key=lambda ids:-float(np.mean(q[ids,2]))):
    poly=clip(list(q[face]))
    if len(poly)>=3:fill.polygon([tuple(x) for x in project(np.array(poly))],fill=(255,190,30,140))
  for a,b in edges:
   aa,bb=q[[a,b]].copy()
   if max(aa[2],bb[2])<.025:continue
   if aa[2]<.025:aa+=(.025-aa[2])/(bb[2]-aa[2])*(bb-aa)
   if bb[2]<.025:bb+=(.025-bb[2])/(aa[2]-bb[2])*(aa-bb)
   lines.append((project(np.array([aa,bb])),color,3 if box else 2))
 image=Image.alpha_composite(image,layer);d=ImageDraw.Draw(image)
 for uv,color,width in lines:d.line([tuple(x) for x in uv],fill=color,width=width)
 return image.convert('RGB')
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
target=DEST/'four-views-window30-wholebody-base-0-20.mp4';assert not target.exists();written=[];saved=set();source=OUT/'four-views-common-frame.mp4'
with av.open(str(source)) as video,av.open(str(target.with_suffix('.partial.mp4')),'w',options={'movflags':'+faststart'}) as writer:
 stream=writer.add_stream('libx264',rate=30);stream.width=1920;stream.height=1200;stream.pix_fmt='yuv420p';stream.time_base=stream.codec_context.time_base=Fraction(1,1000000);stream.options={'crf':'18','preset':'veryfast','bf':'0','threads':'2'}
 for i,f in enumerate(video.decode(video=0)):
  m=mapping[i];r=by[m['external_index']];scene=scenes[m['external_index']];image=f.to_image();original=np.array(image)[:600,:960].copy()
  if 'camera_rotation' in scene:image.paste(overlay(image.crop((1170,42,1710,582)),scene['world'],np.array(scene['camera_position_m']),np.array(scene['camera_rotation'])),(1170,42))
  image.paste(overlay(image.crop((0,642,960,1182)),scene['world'],position,rot,45),(0,642))
  pose=m.get('camera_pose')
  if pose:image.paste(overlay(image.crop((960,642,1920,1182)),scene['world'],np.array(pose['position_m']),np.array(pose['opengl_camera_rotation']),55),(960,642))
  if r.get('input_gap_interpolation') and 'camera_rotation' in scene and 'human_points' in r:
   draw_interp=ImageDraw.Draw(image)
   for item in r['input_gap_interpolation']:
    kind=item['stream']
    if kind not in ['XRHandLeft','XRHandRight']:continue
    points={int(k):np.asarray(v) for k,v in r['human_points'][kind].items()}
    chains=[[1,2,3,4,5],[1,7,8,9,10],[1,12,13,14,15],[1,17,18,19,20],[1,22,23,24,25],[7,12,17,22]]
    for chain in chains:
     for aa,bb in zip(chain,chain[1:]):
      if aa not in points or bb not in points:continue
      q=(np.array([points[aa],points[bb]])-np.asarray(scene['camera_position_m']))@np.asarray(scene['camera_rotation'])
      if min(q[:,2])<=.025:continue
      uv=q@K.T;uv=uv[:,:2]/uv[:,2:]*540/1280+np.array([1170,42]);uv=np.clip(uv,-10000,10000)
      # Clip to the My view panel so other source panels are untouched.
      hand_panel=image.crop((1170,42,1710,582));pd=ImageDraw.Draw(hand_panel);pd.line([tuple(uv[0]-[1170,42]),tuple(uv[1]-[1170,42])],fill=(255,130,40),width=3);image.paste(hand_panel,(1170,42))
  d=ImageDraw.Draw(image);d.rectangle((0,600,1920,640),fill=(16,25,36));status=('ACCEPTED | INPUT INTERPOLATED' if r.get('input_gap_interpolation') else 'ACCEPTED') if r['valid'] else 'FAILED - RED POSE IS DISPLAY ONLY';status += ' | JOINT WINDOW' if r.get('window_optimized') else '';d.text((14,608),f"0-20s CANDIDATE | 30-FRAME RECOVERY | BOX MEAN +/-0.3s | source {r['elapsed_s']:.3f}s | {status}",font=font,fill=(220,240,225) if r['valid'] else (255,105,105));d.rectangle((0,1182,1920,1200),fill=(15,20,29));d.text((10,1181),'Green rack | Gold box sides + bottom | Tag 100/101: 60mm black + 5mm white margin | 1mm box walls + bottom ON; top open; gripper OFF',font=font,fill='white')
  assert np.array_equal(np.array(image)[:600,:960],original)
  pts=round(f.pts*f.time_base*1e6);v=av.VideoFrame.from_image(image);v.pts=pts;v.time_base=Fraction(1,1000000)
  for packet in stream.encode(v):writer.mux(packet)
  written.append(dict(pts_us=pts,external_index=r['external_index'],source_s=r['elapsed_s'],valid=r['valid']))
  for t in [1,7,14,20]:
   if t not in saved and r['elapsed_s']>=t:image.save(DEST/f'four-views-{t:02d}s.jpg');saved.add(t)
  if i%100==0:print('FRAME',i,flush=True)
 for packet in stream.encode():writer.mux(packet)
target.with_suffix('.partial.mp4').rename(target)
with av.open(str(target)) as v:pts=[round(f.pts*f.time_base*1e6) for f in v.decode(video=0)]
assert pts==[x['pts_us'] for x in written]
(DEST/'frame_mapping.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in written));(DEST/'verification.json').write_text(json.dumps(dict(frames=len(written),video_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),source_window_s=[0,20],all_candidate_frames=True,external_panel_unchanged_before_encoding=True,box_faces='four sides and bottom; top open; filled gold alpha140 projection without occlusion masking',tag_policy_sha256=hashlib.sha256((C/'tag_policy.json').read_bytes()).hexdigest()),indent=2));print('DONE',target,flush=True)
