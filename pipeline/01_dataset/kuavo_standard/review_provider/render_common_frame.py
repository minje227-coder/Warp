"""Overview in the actual IK world frame, without a display-only body transform.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
from fractions import Fraction
from collections import Counter
import sys,json
import numpy as np
import mujoco,av
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'source'))
from display_state import make_display_states, display_metadata
from retarget.model import JOINT_NAMES
from retarget.preview import BODY_EDGES
from retarget.gripper_signal import linkage_angles
from retarget.input import sha256
from run_retarget import readl,save
from render_four_views_head_down import project_edges

CHAINS=[[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20]]
HAND_EDGES=[(a,b) for c in CHAINS for a,b in zip(c,c[1:])]+[(5,9),(9,13),(13,17)]

def render(result,samples=False):
    out=ROOT/result
    manifest=json.loads((out/'manifest.json').read_text());take=Path(manifest['source_episode']).name
    rows=readl(out/'trajectory.jsonl');by_index={r['external_index']:r for r in rows};display_states=make_display_states(rows)
    source=out/'four-views-head-follow-down-20.mp4';old=json.loads((out/'head-down-20-verification.json').read_text())
    assert sha256(source)==old['video_sha256']
    trajectory_sha=sha256(out/'trajectory.jsonl')
    model=mujoco.MjModel.from_xml_path(str(out/'overlay-model/four-view-head-down-20.xml'))
    model.vis.quality.offsamples=2
    model.vis.global_.fovy=45
    # Opaque surfaces prevent the many internal mesh faces from showing through.
    model.geom_rgba[:,3]=1.
    original_rgba=model.geom_rgba.copy()
    data=mujoco.MjData(model)
    indices=[model.joint(n).qposadr[0] for n in JOINT_NAMES]
    base_address=model.joint('preview_base').qposadr[0]
    shoulders=[model.body(f'zarm_{s}1_link').id for s in ('l','r')]
    def set_pose(row):
        data.qpos[:]=model.qpos0
        state=display_states[row['external_index']]
        q=state['q']
        base=state['base']
        if q is not None:
            data.qpos[base_address:base_address+7]=[base[0],base[1],0,np.cos(base[2]/2),0,0,np.sin(base[2]/2)]
            data.qpos[indices]=q
            for side,g in zip(('left','right'),state['grippers']):
                for name,angle in linkage_angles(side,True).items():
                    data.qpos[model.joint(name).qposadr[0]]=angle*g['open_fraction']
        mujoco.mj_forward(model,data)
        return q
    scale=1.
    translation=np.zeros(3)
    # Every human point and robot link uses the same stored IK world frame.
    def normalized(points):return {k:np.asarray(v) for k,v in points.items()}
    robot_centers=[]
    for row in rows:
        if row['valid']:
            set_pose(row);robot_centers.append(np.mean(data.xpos[shoulders],axis=0))
    renderer=mujoco.Renderer(model,height=540,width=960)
    option=mujoco.MjvOption();option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=False
    camera=mujoco.MjvCamera();camera.azimuth=145;camera.elevation=-12;camera.distance=3.35
    camera.lookat=np.r_[np.median(np.asarray(robot_centers)[:,:2],axis=0),.85]
    refs_path=ROOT.parent/'external-skeleton-20261016-newtake'/take/'meta_frame_reference.jsonl'
    fused_path=ROOT.parent/'wilor-both-hands-20261016-newtake'/take/'fused_both.jsonl'
    refs={r['external_index']:r['reference'] for r in readl(refs_path)}
    fused={r['external_index']:r for r in readl(fused_path)}
    basis=np.asarray(manifest['world_alignment']['position_basis']);offset=np.asarray(manifest['world_alignment']['translation_m'])
    def aligned(points):return {k:basis@np.asarray(p)+offset for k,p in points.items()}
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    target=out/'four-views-common-frame.mp4'
    if not samples and target.exists():raise FileExistsError(target)
    writer=None;mapping=[];saved=set();counts=Counter();pinch_errors=[];tcp_errors=[];goal_errors=[];chest_offsets=[]
    if not samples:
        writer=av.open(str(target.with_suffix('.partial.mp4')),'w',options={'movflags':'+faststart'})
        stream=writer.add_stream('libx264',rate=30);stream.width=1920;stream.height=1200;stream.pix_fmt='yuv420p'
        stream.time_base=stream.codec_context.time_base=Fraction(1,1000000)
        stream.options={'crf':'18','preset':'veryfast','bf':'0','threads':'2'}
    with av.open(str(source)) as video:
        for i,frame in enumerate(video.decode(video=0)):
            pts_us=round(frame.pts*frame.time_base*1e6)
            oldmap=read_mapping[i];row=by_index[oldmap['external_index']]
            desired=next((s for s in (1,3,7,12,14) if s not in saved and pts_us>=s*1e6),None)
            if samples and desired is None:continue
            image=frame.to_image();original=np.asarray(image)
            state=display_states[row['external_index']]
            q=set_pose(row);model.geom_rgba[:]=original_rgba
            if q is not None and not row['valid']:
                for g in range(model.ngeom):
                    if model.geom(g).name!='preview_floor':model.geom_rgba[g]=[.95,.03,.03,1.]
            renderer.update_scene(data,camera=camera,scene_option=option)
            if q is None:
                for g in renderer.scene.geoms[:renderer.scene.ngeom]:
                    if g.objtype==mujoco.mjtObj.mjOBJ_GEOM and g.objid>=0 and model.geom(g.objid).name!='preview_floor':g.rgba[3]=0
            panel=Image.fromarray(renderer.render())
            position=np.mean([c.pos for c in renderer.scene.camera],axis=0)
            forward=np.mean([c.forward for c in renderer.scene.camera],axis=0);forward/=np.linalg.norm(forward)
            up=np.mean([c.up for c in renderer.scene.camera],axis=0);up/=np.linalg.norm(up)
            rotation=np.column_stack((np.cross(forward,up),up,-forward))
            br=refs[oldmap['external_index']]['body'];bp={}
            if br and br['usable'] and abs(br['delta_ms'])<=20:
                bp=normalized(aligned({b['id']:[b['position'][k] for k in 'xyz'] for b in br['sample']['bones']}))
            body_segments=project_edges(panel,bp,BODY_EDGES,position,rotation,(255,65,65,255),7,45)
            hand_segments=0
            if row['valid']:
                expected_body={int(k):np.asarray(v) for k,v in row['human_points']['body'].items()}
                for bone in (10,15):np.testing.assert_allclose(bp[bone],expected_body[bone],atol=1e-10)
                chest_offsets.append((np.mean(data.xpos[shoulders],axis=0)-(bp[10]+bp[15])/2).tolist())
            for side,h in fused[oldmap['external_index']]['hands'].items():
                # One WiLoR hand per side; no second Meta hand skeleton layer.
                if h['joints'] is not None:
                    hand=normalized(aligned(dict(enumerate(h['joints']))))
                    hand_segments+=project_edges(panel,hand,HAND_EDGES,position,rotation,(35,145,255,255),3,45)
                    if row['valid']:
                        index=0 if side=='left' else 1
                        goal=np.asarray(row['tcp_targets'][index]['position'])
                        pinch=(hand[4]+hand[8])/2
                        pinch_error=float(np.linalg.norm(pinch-goal))
                        raw_goal=np.asarray(row['tcp_targets'][index].get('raw_position',goal))
                        assert np.linalg.norm(pinch-raw_goal)<1e-10,(i,side)
                        assert np.linalg.norm(data.xpos[model.body('zarm_l7_endeffector_center' if side=='left' else 'zarm_r7_endeffector_center').id]-goal)<.0051
                        tcp_name='zarm_l7_endeffector_center' if side=='left' else 'zarm_r7_endeffector_center'
                        tcp_error=float(np.linalg.norm(data.xpos[model.body(tcp_name).id]-pinch))
                        # Raw hand remains visible; target-filter displacement is measured, never hidden.
                        pinch_errors.append(pinch_error);tcp_errors.append(tcp_error)
                        goal_errors.append(float(np.linalg.norm(data.xpos[model.body(tcp_name).id]-goal)))
                        project_edges(panel,{0:pinch,1:goal},[(0,1)],position,rotation,(255,100,220,255),3,45)
            draw=ImageDraw.Draw(image)
            draw.rectangle((0,601,959,1199),fill=(15,20,29))
            image.paste(panel,(0,642))
            draw.text((18,610),'Human input + KUAVO | common coordinates',font=font,fill='white')
            draw.text((18,1181),'Original input coordinates | '+oldmap['robot_display'],font=small,fill='#a7dfbf' if row['valid'] else '#ff6666')
            # Other three panels are copied unchanged before video encoding.
            actual=np.asarray(image)
            assert np.array_equal(actual[:600],original[:600])
            assert np.array_equal(actual[:,960:],original[:,960:])
            counts[oldmap['robot_display']]+=1
            mapping.append(dict(display_index=i,pts_us=pts_us,external_index=oldmap['external_index'],
                body_segments=body_segments,hand_segments=hand_segments,body_skeleton_layers=1,
                upper_body_duplicate_removed=True,**display_metadata(state),
                other_panels_exactly_unchanged_before_encoding=True))
            if desired is not None:
                image.save(out/f'common-frame-{desired:02d}s.jpg');saved.add(desired)
            if writer:
                v=av.VideoFrame.from_image(image);v.pts=pts_us;v.time_base=Fraction(1,1000000)
                for packet in stream.encode(v):writer.mux(packet)
            if i%150==0:print('COMMON-FRAME',i,flush=True)
    renderer.close()
    assert sha256(out/'trajectory.jsonl')==trajectory_sha
    if writer:
        for packet in stream.encode():writer.mux(packet)
        writer.close();target.with_suffix('.partial.mp4').rename(target)
        with av.open(str(target)) as video:actual_pts=[round(f.pts*f.time_base*1e6) for f in video.decode(video=0)]
        assert actual_pts==[r['pts_us'] for r in read_mapping]
        save(out/'common-frame-verification.json',dict(frames=len(mapping),source_pts_identical=True,
            robot_trajectory_unchanged=True,failure_display_policy='last_accepted_red_display_only',display_state_implementation_sha256=sha256(ROOT/'display_state.py'),display_only_held_failure_frames=sum(x['robot_pose_held'] for x in mapping),trajectory_sha256=trajectory_sha,source_video_sha256=sha256(source),video_sha256=sha256(target),
            other_three_panels_unchanged_before_encoding=True,whole_body_layers=1,additional_upper_body_layer=False,
            body_line_width_px=7,hand_line_width_px=3,robot_opacity=1.,
            display_only_uniform_scale=scale,display_only_translation_m=translation.tolist(),
            ground_plane_preserved_by_display_alignment=True,raw_targets_or_inputs_modified=False,ik_targets_filtered=False,
            alignment='stored IK world frame; no extra scale or translation',
            accepted_hand_comparisons=len(pinch_errors),max_displayed_pinch_to_ik_goal_error_m=max(pinch_errors),
            max_independent_tcp_to_displayed_pinch_error_m=max(tcp_errors),max_independent_tcp_to_offset_goal_error_m=max(goal_errors),tcp_target_offset_world_m=manifest['tcp_target_offset_world_m'],
            median_robot_minus_human_shoulder_center_m=np.median(chest_offsets,axis=0).tolist(),
            display_only_hand_transform_removed=True,
            body_proportion_or_ik_mismatch_may_remain=True,counts=dict(counts),implementation_sha256=sha256(Path(__file__)),robot_commands_sent=0))
        (out/'common-frame-mapping.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in mapping))
        print('DONE',dict(counts),flush=True)

if __name__=='__main__':
    read_mapping=readl(ROOT/sys.argv[1]/'head-down-20-frame-mapping.jsonl')
    render(sys.argv[1],'--samples' in sys.argv)
