"""Exact paired-source frames, human skeleton and translucent real S63 meshes."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
from fractions import Fraction
from collections import Counter
import json, sys
import av, mujoco, numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'source'))
from retarget.preview import render_model, skeleton, BODY_EDGES
from display_state import make_display_states, display_metadata
from retarget.model import JOINT_NAMES, TCP_NAMES, WRIST_CAMERA_NAMES
from retarget.input import sha256
from retarget.gripper_signal import linkage_angles
from run_retarget import readl,save,IDS

CHAINS=[[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20]]
EDGES=[(str(IDS[a]),str(IDS[b])) for c in CHAINS for a,b in zip(c,c[1:])]
EDGES += [(str(IDS[a]),str(IDS[b])) for a,b in [(5,9),(9,13),(13,17)]]

def render(take,samples=False):
    out=ROOT/take;manifest=json.loads((out/'manifest.json').read_text())
    take=Path(manifest['source_episode']).name
    cfg=json.loads((out/'config.json').read_text())
    assert sha256(out/'trajectory.jsonl')==manifest['output_checksums']['trajectory.jsonl']
    rows=readl(out/'trajectory.jsonl');by_index={r['external_index']:r for r in rows};display_states=make_display_states(rows)
    raw=Path(manifest['source_episode']);pairs=readl(raw/'processed/paired_frames.jsonl');pairs=[p for p in pairs if p['external_index'] in by_index]
    fused_path=ROOT.parent/'wilor-both-hands-20261016-newtake'/take/'fused_both.jsonl'
    refs_path=ROOT.parent/'external-skeleton-20261016-newtake'/take/'meta_frame_reference.jsonl'
    refs={r['external_index']:r['reference'] for r in readl(refs_path)}
    fused={r['external_index']:r for r in readl(fused_path)}
    basis=np.array(manifest['world_alignment']['position_basis']);offset=np.array(manifest['world_alignment']['translation_m'])
    def aligned(points):return {str(k):basis@np.asarray(p)+offset for k,p in points.items()}
    urdf=Path(manifest['model']['source_urdf'])
    assert sha256(urdf)==manifest['model']['source_urdf_sha256']
    model=render_model(urdf,out/'overlay-model',binary_gripper=True)
    for i in range(model.ngeom):
        if model.geom(i).name!='preview_floor':model.geom_rgba[i]=[.82,.86,.90,.40]
    model.vis.quality.offsamples=2
    data=mujoco.MjData(model);renderer=mujoco.Renderer(model,height=600,width=640)
    indices=[model.joint(n).qposadr[0] for n in JOINT_NAMES]
    centers=[r['human_points']['body']['1'] for r in rows if 'human_points' in r]
    center=np.median(centers,axis=0)
    camera=mujoco.MjvCamera();camera.azimuth=145;camera.elevation=-12;camera.distance=3.35
    camera.lookat=[center[0],center[1],.85]
    option=mujoco.MjvOption();option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=False
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    source=ROOT.parent/'wilor-both-hands-20261016-newtake'/take/'quest-meta-wilor-red-blue.mp4'
    target=out/'quest-external-human-kuavo-overlay.mp4'
    if not samples and target.exists():raise FileExistsError(target)
    writer=None;mapping=[];saved=set();fk_positions=[];fk_rotations=[];counts=Counter()
    if not samples:
        writer=av.open(str(target.with_suffix('.partial.mp4')),'w',options={'movflags':'+faststart'})
        stream=writer.add_stream('libx264',rate=30);stream.width=1920;stream.height=1080;stream.pix_fmt='yuv420p'
        stream.time_base=stream.codec_context.time_base=Fraction(1,1000000)
        stream.options={'crf':'20','preset':'veryfast','bf':'0','threads':'2'}
    with av.open(str(source)) as video:
        for i,(frame,pair) in enumerate(zip(video.decode(video=0),pairs)):
            assert round(frame.pts*frame.time_base*1e6)==pair['common_pts_us']
            idx=pair['external_index'];row=by_index[idx];rr=refs[idx]
            desired=next((v for v in [1,3,7,12,14] if v not in saved and pair['common_pts_us']>=v*1e6),None)
            if samples and desired is None:continue
            image=frame.to_image();draw=ImageDraw.Draw(image)
            draw.rectangle((1280,0,1919,659),fill=(15,20,29))
            data.qpos[:]=model.qpos0
            state=display_states[row['external_index']];mode=state['mode']
            q=state['q']
            base=state['base']
            if q is not None:
                address=model.joint('preview_base').qposadr[0]
                data.qpos[address:address+7]=[base[0],base[1],0,np.cos(base[2]/2),0,0,np.sin(base[2]/2)]
                data.qpos[indices]=q
                # Preview-only gripper aperture from WiLoR pinch, no hardware calibration.
                for side,g in zip(('left','right'),state['grippers']):
                    for name,angle in linkage_angles(side,True).items():
                        data.qpos[model.joint(name).qposadr[0]]=angle*g['open_fraction']
            mujoco.mj_forward(model,data)
            if row['valid']:
                for name,camera_name,goal in zip(TCP_NAMES,WRIST_CAMERA_NAMES,row['tcp_targets']):
                    body=model.body(name);actual=data.xpos[body.id];rot=data.xmat[body.id].reshape(3,3)
                    pe=float(np.linalg.norm(actual-goal['position']));re=float(Rotation.from_matrix(rot.T@goal['rotation']).magnitude())
                    assert pe<cfg['ik']['tcp_position_tolerance_m']+1e-4,(idx,name,pe)
                    if cfg['ik'].get('orientation_mode') == 'body_downward':
                        approach=-rot[:,2];closing=np.asarray(goal['rotation'])[:,0]
                        down_angle=np.arccos(np.clip(np.dot(approach,goal['body_down_direction']),-1,1))
                        closing_angle=np.arccos(np.clip(np.dot(rot[:,0],closing),-1,1))
                        assert down_angle<np.deg2rad(cfg['ik']['body_down_max_deg'])+1e-4
                        if cfg['ik'].get('wrist_camera_flip_only'):
                            offset=data.xpos[model.body(camera_name).id]-actual
                            radial=offset-approach*np.dot(offset,approach)
                            radial/=np.linalg.norm(radial)
                            assert np.dot(radial,-np.asarray(goal['body_down_direction']))>=-1e-4
                        else:
                            assert closing_angle<cfg['ik']['tcp_orientation_tolerance_rad']+1e-4
                        assert np.dot(approach,goal['body_forward_direction'])>-1e-4
                    else:
                        assert re<cfg['ik']['tcp_orientation_tolerance_rad']+1e-4,(idx,name,re)
                    fk_positions.append(pe);fk_rotations.append(re)
            renderer.update_scene(data,camera=camera,scene_option=option)
            # Failure keeps the last accepted display pose, with an explicit red tint.
            if q is None:
                for g in renderer.scene.geoms[:renderer.scene.ngeom]:
                    if g.objtype==mujoco.mjtObj.mjOBJ_GEOM and g.objid>=0 and model.geom(g.objid).name!='preview_floor':g.rgba[3]=0
            elif not row['valid']:
                for g in renderer.scene.geoms[:renderer.scene.ngeom]:
                    if g.objtype==mujoco.mjtObj.mjOBJ_GEOM and g.objid>=0 and model.geom(g.objid).name!='preview_floor':g.rgba[:]=[.95,.03,.03,.8]
            b=rr['body'];body={}
            if b and b['usable'] and abs(b['delta_ms'])<=20:
                body=aligned({p['id']:[p['position'][k] for k in 'xyz'] for p in b['sample']['bones']})
                display_ids={x for e in BODY_EDGES for x in e}
                skeleton(renderer.scene,{k:v for k,v in body.items() if int(k) in display_ids},BODY_EDGES,[1,.18,.18,1],.006)
            for side,kind in [('left','XRHandLeft'),('right','XRHandRight')]:
                h=fused[idx]['hands'][side]
                if h['meta_reference'] is not None:
                    skeleton(renderer.scene,aligned(dict(zip(IDS,h['meta_reference']))),EDGES,[1,.22,.22,.25],.0024)
                if h['joints'] is not None:
                    skeleton(renderer.scene,aligned(dict(zip(IDS,h['joints']))),EDGES,[.08,.45,1,1],.003)
            image.paste(Image.fromarray(renderer.render()),(1280,55));draw=ImageDraw.Draw(image)
            draw.text((1293,16),'Human + KUAVO | opacity 40%',font=font,fill='white')
            draw.rectangle((1280,594,1919,659),fill=(15,20,29))
            draw.text((1293,600),mode+('' if row['valid'] else ': '+row['reason']),font=small,fill='#b3edce' if row['valid'] else '#ffc577')
            draw.text((1293,625),'Gray: accepted | Red robot: failed, display only',font=small,fill='white')
            draw.rectangle((1290,674,1919,715),fill=(15,20,29))
            draw.text((1300,682),'Head fixed | base + torso + both arms',font=small,fill='#b5c1d2')
            draw.rectangle((0,1038,1919,1079),fill=(15,20,29))
            draw.text((20,1045),'OFFLINE GEOMETRY | timing / camera / TCP / gripper calibration pending | no robot commands',font=small,fill='#eac777')
            if desired is not None:image.save(out/f'overlay-{desired:02d}s.jpg');saved.add(desired)
            if writer:
                v=av.VideoFrame.from_image(image);v.pts=pair['common_pts_us'];v.time_base=Fraction(1,1000000)
                for packet in stream.encode(v):writer.mux(packet)
            counts[mode]+=1;mapping.append(dict(display_index=i,pts_us=pair['common_pts_us'],external_index=idx,
                trajectory_index=row['index'],trajectory_timestamp_ticks=row['timestamp_ticks'],robot_display=mode,
                valid_geometric_frame=row['valid'],**display_metadata(state)))
            if i%150==0:print('OVERLAY',take,i,len(pairs),flush=True)
    renderer.close()
    if writer:
        for packet in stream.encode():writer.mux(packet)
        writer.close();target.with_suffix('.partial.mp4').rename(target)
        with av.open(str(target)) as v:pts=[round(f.pts*f.time_base*1e6) for f in v.decode(video=0)]
        assert pts==[p['common_pts_us'] for p in pairs]
        assert len(mapping)==len(pairs)
        (out/'overlay-frame-mapping.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in mapping))
        save(out/'overlay-verification.json',dict(frames=len(mapping),all_paired_pts_verified=True,counts=dict(counts),
            max_independent_mujoco_tcp_position_m=max(fk_positions,default=None),
            max_independent_mujoco_tcp_rotation_rad=max(fk_rotations,default=None),
            robot_opacity=.4,failure_opacity=.8,display_only_held_failure_frames=sum(x['robot_pose_held'] for x in mapping),failure_display_policy='last_accepted_red_display_only',video_sha256=sha256(target),
            inputs={str(p):sha256(p) for p in [source,fused_path,refs_path,out/'trajectory.jsonl',raw/'processed/paired_frames.jsonl',Path(__file__)]},
            dataset_ready=False,trajectory_ready_for_robot=False,robot_commands_sent=0))
        print('DONE',take,dict(counts),flush=True)

if __name__=='__main__':render(sys.argv[1],'--samples' in sys.argv)
