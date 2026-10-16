#!/usr/bin/env python3
"""Adopted offline retargeting from a prepared, take-specific reference and world."""
from pathlib import Path
import argparse, hashlib, json, os, shutil, subprocess, sys, time
ROOT=Path(__file__).resolve().parent

def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 p=Path(p);tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(x,indent=2)+'\n');tmp.replace(p)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(out):
 r=read(out/'REPORT.json');v=r['validation'];m=r['moveit']
 assert v['source_knot_invalid']==v['sampled_pose_failures']==v['clearance_below10mm_minus1um_states']==0
 assert m['accepted_invalid']==m['interframe_invalid']==0
 assert v['max_sampled_velocity_ratio']<=1.00001 and v['max_sampled_acceleration_ratio']<=1.00001
 for f,h in read(out/'artifact-sha256.json').items():assert sha(out/f)==h,f
 print(json.dumps(dict(result=str(out),checked=m['checked_states'],duration_s=v['summary']['duration_s'],verified=True)))
 return r

def stage(out,name):
 env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',MUJOCO_GL='egl')
 with (out/'logs'/f'{name}.log').open('w') as log:
  p=subprocess.run([sys.executable,str(out/name)],cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT)
 if p.returncode:raise RuntimeError(f'{name} failed: '+(out/'logs'/f'{name}.log').read_text()[-3000:])

def run(args):
 if not all([args.reference,args.world,args.reference_config]):raise ValueError('run requires --reference, --world and --reference-config for this take; raw hand/tag preprocessing remains a separate stage')
 out=ROOT/'results'/args.take
 if out.exists():raise FileExistsError(out)
 for sub in ['data','results','logs']:(out/sub).mkdir(parents=True,exist_ok=True)
 cfg=read(args.reference_config)
 cfg['ik'].update(read(ROOT/'standard_config.json')['ik']);cfg['ik']['environment_world_json']=str(out/'data/world.json')
 # standard_config stores effective costs; solve.py applies the profile's scale once.
 scale=read(ROOT/'STANDARD.json')['solver_profile'].get('body_scale',1.)
 for key in ['chest_position_weight','chest_orientation_weight','elbow_weight']:cfg['ik'][key]/=scale
 rows=[json.loads(l) for l in Path(args.reference).open()]
 # Prepared references must explicitly preserve available-input and seed semantics.
 usable=[i for i,r in enumerate(rows) if r.get('input_available',r.get('valid',False))]
 if len(usable)<3:raise ValueError('Need at least three prepared input frames')
 for i in usable:
  r=rows[i]
  if r.get('q') is None or r.get('base_pose') is None:raise ValueError(f'Frame {i} requires take-specific seed q and base reference')
  r['valid']=True
 (out/'data/reference.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
 shutil.copy2(args.world,out/'data/world.json');write(out/'data/reference-config.json',cfg)
 profile=read(ROOT/'STANDARD.json')['solver_profile'];write(out/'PROFILE.json',profile)
 policy=read(ROOT/'motion_policy.json');policy.update(take=args.recording_take or args.take,source_candidate=str(Path(args.display_source).resolve()) if args.display_source else str(ROOT))
 write(out/'POLICY.json',policy)
 (out/'source').symlink_to(ROOT/'pipeline/source',target_is_directory=True)
 scripts={'solve.py':'solve.py','retime.py':'retime.py','check.py':'check.py','render.py':'render.py'}
 for src,dst in scripts.items():shutil.copy2(ROOT/'pipeline'/src,out/dst)
 inputs={str(Path(p).resolve()):sha(p) for p in [args.reference,args.world,args.reference_config]}
 write(out/'input-sha256.json',inputs)
 stage(out,'solve.py');result=out/'profiles'/profile['name'];search=read(result/'summary.json')
 if search['failures']:
  write(out/'REPORT.json',dict(status='IK failures; no execution trajectory accepted',search=search,input_sha256=inputs))
  print(json.dumps(dict(accepted=search['metrics']['accepted'],failures=len(search['failures']),report=str(out/'REPORT.json'))));return 2
 shutil.copy2(result/'trajectory.jsonl',out/'data/source-trajectory.jsonl')
 cfg['ik']=read(result/'config.json')['ik'];write(out/'data/source-config.json',cfg)
 stage(out,'retime.py');stage(out,'check.py')
 validator=read(ROOT/'STANDARD.json')['moveit_validator']
 if sha(validator['path'])!=validator['sha256']:raise RuntimeError('MoveIt validator changed')
 import shlex
 command='. /opt/ros/humble/setup.sh && ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1 '+shlex.quote(validator['path'])+' '+shlex.quote(str(out/'moveit_verify'))
 with (out/'logs/moveit.log').open('w') as log:subprocess.run(['bash','-c',command],stdout=log,stderr=subprocess.STDOUT,check=True)
 full=read(out/'results/drake-validation.json');validation=full['summary'];validation['sampled_state_invalid']=sum(not x['check']['valid'] for x in full['samples'])
 report=dict(status='verified',search=search,validation=validation,moveit=read(out/'moveit_verify/results/verification.json')['summary'],input_sha256=inputs,source_times_preserved=True)
 rejected=validation['sampled_state_invalid'] or validation['source_knot_invalid'] or report['moveit']['accepted_invalid'] or report['moveit']['interframe_invalid']
 if rejected:report['status']='rejected_by_dense_validation'
 write(out/'REPORT.json',report)
 if rejected:
  print(json.dumps(dict(status=report['status'],invalid=validation['sampled_state_invalid'],report=str(out/'REPORT.json'))));return 2
 if args.display_source:stage(out,'render.py')
 artifacts=['data/source-trajectory.jsonl','data/source-config.json','data/world.json','POLICY.json','PROFILE.json','REPORT.json','results/retimed.json','results/execution-trajectory-100hz.npz']
 video=out/'results/four-views-parameter-winner.mp4'
 if video.exists():artifacts+=['results/four-views-parameter-winner.mp4','results/render-summary.json']
 write(out/'artifact-sha256.json',{f:sha(out/f) for f in artifacts});verify(out)
 return 0

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['status','run','verify','render']);p.add_argument('take',nargs='?');p.add_argument('--reference');p.add_argument('--world');p.add_argument('--reference-config');p.add_argument('--display-source');p.add_argument('--recording-take');a=p.parse_args()
 if a.command=='status':print((ROOT/'STANDARD.json').read_text());print((ROOT/'CURRENT.json').read_text());return
 if not a.take or Path(a.take).name!=a.take:p.error('Provide a take name')
 out=ROOT/'results'/a.take
 if a.command=='run':raise SystemExit(run(a))
 verify(out)
 if a.command=='render':
  video=out/'results/four-views-parameter-winner.mp4'
  if not video.exists():stage(out,'render.py')
  print(str(video))
if __name__=='__main__':main()
