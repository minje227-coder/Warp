"""Label rejected previews and bind them to the exact rendered replay trajectory."""
from pathlib import Path
from fractions import Fraction
import hashlib
import json
import sys
import av
from PIL import ImageDraw, ImageFont


def publish(result, output, cutoff=0.):
    result,output=Path(result),Path(output)
    report=json.loads((result/'REPORT.json').read_text())
    rejected=report['status']!='verified'
    source=result/'results/four-views-parameter-winner.mp4'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    rendered=json.loads((result/'results/render-summary.json').read_text())
    assert sha(source)==rendered['video_sha256']
    target=output/'replay-preview.mp4'
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23)
    pts=[]
    with av.open(str(source)) as reader,av.open(str(target),'w',options={'movflags':'+faststart'}) as writer:
        s=writer.add_stream('libx264',rate=30);s.width,s.height,s.pix_fmt=1920,1200,'yuv420p'
        s.options={'crf':'18','preset':'veryfast','bf':'0','threads':'2'}
        s.time_base=s.codec_context.time_base=Fraction(1,1000000)
        for f in reader.decode(video=0):
            t=float(f.pts*f.time_base)
            if cutoff and t>=cutoff:break
            image=f.to_image()
            if rejected:
                d=ImageDraw.Draw(image);d.rectangle((0,600,1919,641),fill=(112,24,30))
                d.text((15,608),'REVIEW ONLY | STANDARD VALIDATION NOT PASSED | NOMINAL ARM INITIALIZATION RESTORED',font=font,fill='white')
            p=round(t*1e6);frame=av.VideoFrame.from_image(image);frame.pts=p;frame.time_base=Fraction(1,1000000)
            for packet in s.encode(frame):writer.mux(packet)
            pts.append(p)
        for packet in s.encode():writer.mux(packet)
    with av.open(str(target)) as reader:
        actual=[round(f.pts*f.time_base*1e6) for f in reader.decode(video=0)]
        duration=float(reader.streams.video[0].duration*reader.streams.video[0].time_base)
    assert actual==pts and pts
    import numpy as np
    with np.load(result/'results/execution-trajectory-100hz.npz',allow_pickle=False) as z:
        mapping=[json.loads(l) for l in (result/'results/video-mapping.jsonl').open()]
        mapping=[m for m in mapping if not cutoff or m['execution_s']<cutoff]
        assert len(mapping)==len(pts)
        assert max(abs(m['source_s']-float(np.interp(m['execution_s'],z['execution_s'],z['source_s']))) for m in mapping)<1e-9
    manifest=dict(kind='replay_trajectory_preview',result=str(result),video=str(target),video_sha256=sha(target),
        trajectory_sha256=sha(result/'results/execution-trajectory-100hz.npz'),frames=len(pts),duration_s=duration,
        arm_initialization_fixed=True,original_render_pts_preserved=True,validation_status=report['status'],
        lower_right=rendered.get('lower_right'),human_skeleton_lower_right=rendered.get('human_skeleton_lower_right'),
        gripper_policy=rendered['gripper_policy'],
        gripper_policy_manifest=str(result/'results/gripper-policy.json'),
        gripper_policy_trace=str(result/'results/gripper-policy-trace.jsonl'),
        physical_replay_recording=False,trajectory_verified=not rejected,executable=False,robot_commands_sent=0)
    (output/'MEDIA.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':publish(sys.argv[1],sys.argv[2],float(sys.argv[3]))
