"""2x2 paired-source review with the unmodified URDF head-camera transform."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
from fractions import Fraction
from collections import Counter
import sys,json,xml.etree.ElementTree as ET
import numpy as np
import mujoco,av
from PIL import Image,ImageDraw,ImageFont,ImageOps
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'source'))
from retarget.input import sha256
from display_state import make_display_states, display_metadata
from retarget.model import JOINT_NAMES,TCP_NAMES
from retarget.gripper_signal import linkage_angles
from run_retarget import readl,save,IDS

FOV=55
HEAD_DOWN_DEG=20
HEAD_DOWN=np.deg2rad(HEAD_DOWN_DEG)
LOCAL_CAMERA_ROTATION=np.array([[0,np.sin(HEAD_DOWN),-np.cos(HEAD_DOWN)],[-1,0,0],[0,np.cos(HEAD_DOWN),np.sin(HEAD_DOWN)]])
BODY_EDGES=[(3,4),(4,5),(5,6),(6,7),(5,8),(8,9),(9,10),(10,11),(11,12),(12,19),
            (5,13),(13,14),(14,15),(15,16),(16,17),(17,45)]
CHAINS=[[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20]]
HAND_EDGES=[(a,b) for c in CHAINS for a,b in zip(c,c[1:])]+[(5,9),(9,13),(13,17)]

def project_edges(image,points,edges,position,rotation,color,width,fov=FOV):
    """Same virtual camera, near-plane-clipped bones drawn above robot surfaces."""
    focal=image.height/2/np.tan(np.deg2rad(fov/2))
    draw=ImageDraw.Draw(image,'RGBA');segments=0
    for a,b in edges:
        if a not in points or b not in points:continue
        q=(np.asarray([points[a],points[b]])-position)@rotation
        q[:,1:]*=-1  # MuJoCo +Y-up/-Z-forward to image +Y-down/+Z-forward.
        if not np.isfinite(q).all() or max(q[:,2])<=.025:continue
        if q[0,2]<.025:q[0]+=(.025-q[0,2])/(q[1,2]-q[0,2])*(q[1]-q[0])
        if q[1,2]<.025:q[1]+=(.025-q[1,2])/(q[0,2]-q[1,2])*(q[0]-q[1])
        uv=q[:,:2]/q[:,2:]*focal+np.array([image.width/2,image.height/2])
        # Clip in image space to count actually visible segments.
        p0,p1=uv;delta=p1-p0;lo,hi=0.,1.
        for axis,bound in [(0,image.width-1),(1,image.height-1)]:
            if abs(delta[axis])<1e-12:
                if not 0<=p0[axis]<=bound:lo,hi=1.,0.;break
            else:
                t0,t1=sorted([(0-p0[axis])/delta[axis],(bound-p0[axis])/delta[axis]])
                lo=max(lo,t0);hi=min(hi,t1)
        if lo>hi:continue
        draw.line([tuple(p0+lo*delta),tuple(p0+hi*delta)],fill=color,width=width);segments+=1
    return segments

def render(take,samples=False):
    out=ROOT/take;manifest=json.loads((out/'manifest.json').read_text())
    take=Path(manifest['source_episode']).name
    tolerance=np.degrees(json.loads((out/'config.json').read_text())['ik']['tcp_orientation_tolerance_rad'])
    assert sha256(out/'trajectory.jsonl')==manifest['output_checksums']['trajectory.jsonl']
    rows={r['external_index']:r for r in readl(out/'trajectory.jsonl')};display_states=make_display_states(list(rows.values()))
    refs_path=ROOT.parent/'external-skeleton-20261016-newtake'/take/'meta_frame_reference.jsonl'
    fused_path=ROOT.parent/'wilor-both-hands-20261016-newtake'/take/'fused_both.jsonl'
    refs={r['external_index']:r['reference'] for r in readl(refs_path)}
    fused={r['external_index']:r for r in readl(fused_path)}
    basis=np.array(manifest['world_alignment']['position_basis']);offset=np.array(manifest['world_alignment']['translation_m'])
    def aligned(points):return {k:basis@np.asarray(p)+offset for k,p in points.items()}
    pairs=readl(Path(manifest['source_episode'])/'processed/paired_frames.jsonl');pairs=[p for p in pairs if p['external_index'] in rows]
    source=out/'quest-external-human-kuavo-overlay.mp4'
    previous=json.loads((out/'overlay-verification.json').read_text())
    assert sha256(source)==previous['video_sha256']
    tree=ET.parse(out/'overlay-model/visual.xml');root=tree.getroot()
    body=root.find(".//body[@name='head_camera_depth']")
    if body is None:raise ValueError('Native head camera body missing')
    # URDF camera body uses X-forward/Y-left/Z-up. MuJoCo cameras use -Z-forward/Y-up.
    # The camera follows the recorded robot head with a constant local downward pitch.
    # This changes the virtual view direction only; trajectory/head joints stay unchanged.
    ET.SubElement(body,'camera',name='native_head_view',pos='0 0 0',
                  xyaxes=f'0 -1 0 {np.sin(HEAD_DOWN)} 0 {np.cos(HEAD_DOWN)}',fovy=str(FOV))
    root.find('visual/global').attrib.update(offwidth='960',offheight='540')
    xml=out/'overlay-model/four-view-head-down-20.xml';tree.write(xml,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
    original_rgba=model.geom_rgba.copy()
    model.vis.quality.offsamples=2
    renderer=mujoco.Renderer(model,height=540,width=960)
    # Recover the original overview camera basis; draw the human above translucent meshes.
    overview=mujoco.MjvCamera();overview.azimuth=145;overview.elevation=-12;overview.distance=3.35
    center=np.median([r['human_points']['body']['1'] for r in rows.values() if 'human_points' in r],axis=0)
    overview.lookat=[center[0],center[1],.85]
    mujoco.mj_forward(model,data);renderer.update_scene(data,camera=overview)
    overview_position=np.mean([c.pos for c in renderer.scene.camera],axis=0)
    forward=np.mean([c.forward for c in renderer.scene.camera],axis=0);forward/=np.linalg.norm(forward)
    up=np.mean([c.up for c in renderer.scene.camera],axis=0);up/=np.linalg.norm(up)
    overview_rotation=np.column_stack((np.cross(forward,up),up,-forward))
    camera_id=model.camera('native_head_view').id
    indices=[model.joint(n).qposadr[0] for n in JOINT_NAMES]
    head_joints=[j for j in range(model.njnt) if (model.joint(j).name or '').startswith('zhead')]
    assert not head_joints  # Source visual model welded the requested fixed head joints.
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
    target=out/'four-views-head-follow-down-20.mp4'
    if not samples and (target.exists() or target.with_suffix('.partial.mp4').exists()):raise FileExistsError(target)
    writer=None;mapping=[];saved=set();counts=Counter()
    if not samples:
        writer=av.open(str(target.with_suffix('.partial.mp4')),'w',options={'movflags':'+faststart'})
        stream=writer.add_stream('libx264',rate=30);stream.width=1920;stream.height=1200;stream.pix_fmt='yuv420p'
        stream.time_base=stream.codec_context.time_base=Fraction(1,1000000)
        stream.options={'crf':'20','preset':'veryfast','bf':'0','threads':'2'}
    with av.open(str(source)) as video:
        for i,(frame,pair) in enumerate(zip(video.decode(video=0),pairs)):
            assert round(frame.pts*frame.time_base*1e6)==pair['common_pts_us']
            row=rows[pair['external_index']]
            desired=next((s for s in [1,3,7,12,14] if s not in saved and pair['common_pts_us']>=s*1e6),None)
            if samples and desired is None:continue
            image=frame.to_image();canvas=Image.new('RGB',(1920,1200),(15,20,29));draw=ImageDraw.Draw(canvas)
            rr=refs[pair['external_index']];br=rr['body'];bp={}
            if br and br['usable'] and abs(br['delta_ms'])<=20:
                bp=aligned({b['id']:[b['position'][k] for k in 'xyz'] for b in br['sample']['bones']})
            overview_image=Image.new('RGBA',(640,600),(0,0,0,0))
            overview_segments=project_edges(overview_image,bp,BODY_EDGES,overview_position,overview_rotation,(255,65,65,255),4,45)
            for h in fused[pair['external_index']]['hands'].values():
                if h['meta_reference'] is not None:
                    project_edges(overview_image,aligned(dict(enumerate(h['meta_reference']))),HAND_EDGES,overview_position,overview_rotation,(255,65,65,70),3,45)
                if h['joints'] is not None:
                    project_edges(overview_image,aligned(dict(enumerate(h['joints']))),HAND_EDGES,overview_position,overview_rotation,(35,145,255,255),3,45)
            overview_base=image.crop((1280,55,1920,594)).convert('RGBA')
            overview_base=Image.alpha_composite(overview_base,overview_image.crop((0,0,640,539))).convert('RGB')
            panels=[('External view | Meta + WiLoR',(0,0),image.crop((640,120,1280,600))),
                    ('My view | Meta + WiLoR',(960,0),image.crop((20,65,620,665))),
                    ('Human + KUAVO | red upper-body skeleton',(0,600),overview_base)]
            for title,(x,y),picture in panels:
                fitted=ImageOps.contain(picture,(952,540),Image.Resampling.LANCZOS)
                canvas.paste(fitted,(x+(960-fitted.width)//2,y+42+(540-fitted.height)//2))
                draw.text((x+18,y+10),title,font=font,fill='white')
            data.qpos[:]=model.qpos0
            state=display_states[row['external_index']];mode=state['mode']
            q=state['q']
            base=state['base']
            visibility=None;pose=None;segments={'upper_body':0,'meta_fingers':0,'wilor_fingers':0}
            if q is not None:
                address=model.joint('preview_base').qposadr[0]
                data.qpos[address:address+7]=[base[0],base[1],0,np.cos(base[2]/2),0,0,np.sin(base[2]/2)]
                data.qpos[indices]=q
                for side,g in zip(('left','right'),state['grippers']):
                    for name,angle in linkage_angles(side,True).items():
                        data.qpos[model.joint(name).qposadr[0]]=angle*g['open_fraction']
                mujoco.mj_forward(model,data)
                model.geom_rgba[:]=original_rgba
                if state['failed']:
                    for g in range(model.ngeom):
                        if model.geom(g).name!='preview_floor':model.geom_rgba[g]=[.95,.03,.03,1.]
                renderer.update_scene(data,camera='native_head_view')
                # Keep the source visual materials opaque in this first-person panel.
                head_image=Image.fromarray(renderer.render())
                rotation=data.cam_xmat[camera_id].reshape(3,3);position=data.cam_xpos[camera_id].copy()
                expected=data.xpos[model.body('head_camera_depth').id]
                assert np.allclose(position,expected,atol=1e-10)
                # Verify a rigid head-follow transform and fixed downward pitch, with no hand auto-aim.
                native=data.xmat[model.body('head_camera_depth').id].reshape(3,3)
                assert np.allclose(rotation,native@LOCAL_CAMERA_ROTATION,atol=1e-10)
                assert np.allclose(-rotation[:,2],native@np.array([np.cos(HEAD_DOWN),0,-np.sin(HEAD_DOWN)]),atol=1e-10)
                pose={'position_m':position.tolist(),'opengl_camera_rotation':rotation.tolist()}
                if bp:
                    segments['upper_body']=project_edges(head_image,bp,BODY_EDGES,position,rotation,(255,65,65,255),4)
                for side,h in fused[pair['external_index']]['hands'].items():
                    if h['meta_reference'] is not None:
                        segments['meta_fingers']+=project_edges(head_image,aligned(dict(enumerate(h['meta_reference']))),HAND_EDGES,position,rotation,(255,65,65,70),4)
                    if h['joints'] is not None:
                        segments['wilor_fingers']+=project_edges(head_image,aligned(dict(enumerate(h['joints']))),HAND_EDGES,position,rotation,(35,145,255,255),3)
                for tcp_goal in row.get('tcp_targets',[]):
                    if 'raw_position' in tcp_goal:
                        project_edges(head_image,{0:np.asarray(tcp_goal['raw_position']),1:np.asarray(tcp_goal['position'])},[(0,1)],position,rotation,(255,100,220,255),3)
                canvas.paste(head_image,(960,642))
                counts['head_frames_with_upper_body']+=segments['upper_body']>0
                counts['head_frames_with_wilor_fingers']+=segments['wilor_fingers']>0
                visibility={}
                half_height=np.tan(np.deg2rad(FOV/2));aspect=960/540
                for side,name in zip(('left','right'),TCP_NAMES):
                    camera=(data.xpos[model.body(name).id]-position)@rotation
                    depth=-camera[2]
                    visibility[side]=bool(depth>0 and abs(camera[1])<depth*half_height and abs(camera[0])<depth*half_height*aspect)
                counts['pose_available']+=1
                counts['both_TCP_in_frustum']+=all(visibility.values())
            else:
                draw.text((1170,895),'Head view unavailable: no current IK pose',font=font,fill='#eab879')
                counts['pose_unavailable']+=1
            draw.text((978,610),'KUAVO head view | 20 deg down',font=font,fill='white')
            draw.text((978,1180),(mode if state['failed'] else 'Head-follow view | fixed 20 deg down | FOV 55 deg'),font=small,fill='#b7c6d7')
            draw.text((18,580),f'{take[:8]} | {pair["common_pts_us"]/1e6:.2f}s | {i+1}/{len(pairs)}',font=small,fill='#b7c6d7')
            draw.text((978,580),'Red Meta | Blue WiLoR | approximate camera alignment',font=small,fill='#b7c6d7')
            draw.text((18,1180),mode+' | offline geometry; no robot commands',font=small,fill='#a7dfbf' if row['valid'] else '#eab879')
            draw.line((960,0,960,1200),fill='#334155',width=2);draw.line((0,600,1920,600),fill='#334155',width=2)
            if desired is not None:canvas.save(out/f'head-down-20-{desired:02d}s.jpg');saved.add(desired)
            mapping.append(dict(display_index=i,pts_us=pair['common_pts_us'],external_index=pair['external_index'],
                trajectory_index=row['index'],robot_display=mode,camera_pose=pose,tcp_in_nominal_frustum=visibility,
                camera_direction_adjusted=True,head_relative_down_pitch_deg=HEAD_DOWN_DEG,**display_metadata(state),head_skeleton_visible_segments=segments,overview_upper_body_segments=overview_segments))
            if writer:
                v=av.VideoFrame.from_image(canvas);v.pts=pair['common_pts_us'];v.time_base=Fraction(1,1000000)
                for packet in stream.encode(v):writer.mux(packet)
            if i%150==0:print('FOUR-VIEW',take,i,len(pairs),flush=True)
    renderer.close()
    if writer:
        for packet in stream.encode():writer.mux(packet)
        writer.close();target.with_suffix('.partial.mp4').rename(target)
        with av.open(str(target)) as v:pts=[round(f.pts*f.time_base*1e6) for f in v.decode(video=0)]
        assert pts==[p['common_pts_us'] for p in pairs]
        assert len(mapping)==len(pairs)
        (out/'head-down-20-frame-mapping.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in mapping))
        save(out/'head-down-20-verification.json',dict(frames=len(mapping),all_paired_pts_verified=True,
            counts=dict(counts),camera_body='head_camera_depth',local_position_m=[0,0,0],native_forward_axis='+X',head_relative_rotation=LOCAL_CAMERA_ROTATION.tolist(),head_relative_down_pitch_deg=HEAD_DOWN_DEG,head_follow_verified_every_pose=True,
            camera_direction_adjusted=True,head_joints_fixed=True,auto_aim=False,nominal_vertical_fov_deg=FOV,
            native_camera_mount_position_preserved=True,virtual_camera_rotation_adjusted=True,upper_body_overlay=True,
            skeleton_overlay_depth_occlusion=False,points_behind_camera_not_displayed=True,
            hardware_intrinsics_verified=False,frustum_check_is_not_occlusion_check=True,
            video_sha256=sha256(target),source_video_sha256=sha256(source),
            implementation_sha256=sha256(Path(__file__)),trajectory_sha256=sha256(out/'trajectory.jsonl'),robot_commands_sent=0))
        print('DONE',take,dict(counts),flush=True)

if __name__=='__main__':render(sys.argv[1],'--samples' in sys.argv)
