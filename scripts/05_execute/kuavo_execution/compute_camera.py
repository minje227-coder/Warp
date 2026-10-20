"""Regenerate only camera-source imagery; keep the solved motion and gates intact."""
from pathlib import Path
from fractions import Fraction
import bisect
import hashlib
import importlib.util
import json
import sys

import av
import numpy as np
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation,Slerp

TAKE=sys.argv[1]
BASE=Path('/home/lab6/Warp-review')
C=Path(sys.argv[2])
OUT=C/'results'/TAKE
WI=BASE/'standard-input-seven-native-20261019/wilor-both-hands-20261016-newtake'
RAW=Path('/home/lab6/Warp_data/2026-10-19')/TAKE
read=lambda p:[json.loads(s) for s in p.open() if s.strip()]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0,str(C));sys.path.insert(0,str(C/'source'));sys.path.insert(0,str(WI))
spec=importlib.util.spec_from_file_location('recorded_projection',WI/'render_red_blue_native.py')
review=importlib.util.module_from_spec(spec);spec.loader.exec_module(review)
projection=review.QuestProjection(RAW)
alignment=json.loads((OUT/'manifest.json').read_text())['world_alignment']
A=np.array(alignment['position_basis']);offset=np.array(alignment['translation_m']);inverse=np.linalg.inv(A)
refs={r['external_index']:r for r in read(C/'data/reference.jsonl')}
fused={r['external_index']:r for r in read(WI/TAKE/'fused_both.jsonl')}
pairs=[p for p in read(RAW/'processed/paired_frames.jsonl') if p['external_index'] in refs]
scenes={r['external_index']:r for r in read(C/'data/frame-scenes.jsonl')}
saved=sys.argv;sys.argv=[str(C/'render_objects.py'),'results/'+TAKE]
ns={'__file__':str(C/'render_objects.py'),'__name__':'adopted_projection'}
exec((C/'render_objects.py').read_text().split('target=DEST/')[0],ns);sys.argv=saved
cal=projection.cal;fx,fy,cx,cy,skew=cal['intrinsics'];K=np.array([[fx,skew,cx],[0,fy,cy],[0,0,1.]])
xyz=lambda v:np.array([v[k] for k in 'xyz'])

def current_camera(pair):
    sensor=projection.sensors[pair['quest_sensor_index']]
    target=621355968000000000+int(sensor['callback_unix_ms'])*10000
    i=bisect.bisect_left(projection.ticks,target)
    if not 0<i<len(projection.heads):return None,'outside_head_tracking',None
    left,right=projection.heads[i-1:i+1];dt=right['timestamp_ticks']-left['timestamp_ticks']
    if not left['valid'] or not right['valid'] or left['tracking_origin_pose']!=right['tracking_origin_pose'] or dt<=0:
        return None,'invalid_or_changed_tracking',dt/10000.
    u=(target-left['timestamp_ticks'])/dt
    R=Slerp([0.,1.],Rotation.from_quat([
        [h['head_pose']['rotation'][k] for k in 'xyzw'] for h in [left,right]
    ]))(u).as_matrix()
    position=(1-u)*xyz(left['head_pose']['position'])+u*xyz(right['head_pose']['position'])
    # Display-only interpolation between actual valid head samples. Camera motion
    # is never held; object observations retain the original 50ms acceptance gate.
    return (R@projection.flip@projection.lens_r.T,
            position+R@projection.flip@projection.lens_t),'interpolated',dt/10000.

source=OUT/'object-overlay/four-views-window30-wholebody-base-0-20.mp4'
target=source.with_name(source.stem+'-no-flicker.mp4')
assert not target.exists()
records=[]
with av.open(str(source)) as old,av.open(str(RAW/'processed/paired_quest.mp4')) as quest,av.open(str(target),'w',options={'movflags':'+faststart'}) as writer:
    stream=writer.add_stream('libx264',rate=30);stream.width,stream.height,stream.pix_fmt=1920,1200,'yuv420p'
    stream.options={'crf':'18','preset':'ultrafast','bf':'0','threads':'2'}
    stream.time_base=stream.codec_context.time_base=Fraction(1,1000000)
    for of,qf,pair in zip(old.decode(video=0),quest.decode(video=0),pairs):
        pts=pair['common_pts_us'];assert round(of.pts*of.time_base*1e6)==round(qf.pts*qf.time_base*1e6)==pts
        index=pair['external_index'];row=refs[index];f=fused[index]
        image=qf.to_image();camera,status,bracket=current_camera(pair)
        if camera is not None:
            body={};hands={side:h['joints'] for side,h in f['hands'].items()}
            if row.get('human_points'):
                body={int(k):inverse@(np.array(v)-offset) for k,v in row['human_points']['body'].items()}
                for side,kind in [('left','XRHandLeft'),('right','XRHandRight')]:
                    hands[side]=[inverse@(np.array(row['human_points'][kind][str(j)])-offset) for j in [1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]]
            review.draw_world_edges(ImageDraw.Draw(image),body,review.CAMERA_BODY,
                lambda p:(p-camera[1])@camera[0],K,review.META,5,image.size)
            for side,h in hands.items():
                native_pose=(camera[0],camera[1],{})
                review.draw_lines(ImageDraw.Draw(image),projection.project(h,native_pose),review.EDGES,review.WILOR,5)
        panel=image.resize((540,540),Image.Resampling.LANCZOS)
        if camera is not None:
            panel=ns['overlay'](panel,scenes[index]['world'],A@camera[1]+offset,A@camera[0])
        canvas=of.to_image()
        canvas.paste(panel,(1170,42))
        output=av.VideoFrame.from_image(canvas);output.pts=pts;output.time_base=Fraction(1,1000000)
        for packet in stream.encode(output):writer.mux(packet)
        records.append(dict(external_index=index,pts_us=pts,camera_projection=status,head_bracket_ms=bracket,
                            old_camera_projection_available='camera_rotation' in scenes[index]))
    for packet in stream.encode():writer.mux(packet)
assert len(records)==len(pairs)
with av.open(str(target)) as result:
    assert [round(f.pts*f.time_base*1e6) for f in result.decode(video=0)]==[p['common_pts_us'] for p in pairs]
original=source.with_name(source.stem+'-before-flicker-fix.mp4')
assert not original.exists();source.rename(original);target.rename(source)
receipt=dict(frames=len(records),before_camera_projection_frames=sum(r['old_camera_projection_available'] for r in records),
    after_camera_projection_frames=sum(r['camera_projection']=='interpolated' for r in records),
    display_interpolation_only=True,pose_acceptance_gate_unchanged=True,
    original_pair_pts_preserved=True,object_world_and_ik_unchanged=True,
    video_sha256=sha(source),records=records)
(OUT/'upper-right-flicker-fix.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({k:v for k,v in receipt.items() if k!='records'}),flush=True)
