from pathlib import Path
import sys,json,time,hashlib,shutil
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.special import betainc
c=Path(__file__).resolve().parent;policy=json.load(open(c/'POLICY.json'));src=Path(policy['source_candidate'])
sys.path.insert(0,str(c/'source'))
from retarget.model import KuavoModel,JOINT_NAMES
rows=[json.loads(l) for l in (c/'data/source-trajectory.jsonl').open()];valid=[r for r in rows if r['valid']];ts=np.array([r['elapsed_s'] for r in valid])
profile=json.load(open(c/'PROFILE.json'));distances=np.array([r['box_surface_distance_m'] for r in valid])
def allowed(t):
 if not profile.get('adaptive'):return profile.get('tol',.01)
 d=float(np.interp(t,ts,distances));return float(.01+.01*betainc(3,3,np.clip((d-.1)/.1,0,1)))
data=json.load(open(c/'results/retimed.json'));samples=data['samples'];start=time.perf_counter()
# Audit the exact library curve at 240 Hz as well as all delivered 100 Hz samples.
if (c/'results/parameterization.npz').exists():
 import toppra as ta
 z=np.load(c/'results/parameterization.npz');pp=ta.SplineInterpolator(z['source_s'],z['knots'],bc_type='natural');tr=ta.ParametrizeConstAccel(pp,z['grid'],z['phase_velocity']);ph=ta.ParametrizeConstAccel(ta.SplineInterpolator([ts[0],ts[-1]],[[ts[0]],[ts[-1]]]),z['grid'],z['phase_velocity'])
 phase_grid_time=np.r_[0.,np.cumsum(2*np.diff(z['grid'])/(z['phase_velocity'][:-1]+z['phase_velocity'][1:]))]
 source_probe=np.unique(np.r_[ts,np.arange(ts[0],ts[-1],1/120),ts[-1]])
 extra_time=np.interp(source_probe,z['grid'],phase_grid_time)
 tt=np.unique(np.r_[[r['execution_s'] for r in samples],np.arange(0,tr.duration,1/240),extra_time,tr.duration]);xx=tr(tt);vv=tr(tt,1);aa=tr(tt,2);phase=ph(tt)[:,0]
 samples=[dict(execution_s=float(t),source_s=float(s),x=q.tolist(),v=v.tolist(),a=a.tolist()) for t,s,q,v,a in zip(tt,phase,xx,vv,aa)]
cfg=json.load(open(c/'data/source-config.json'));cfg['ik']['environment_world_json']=str(c/'data/world.json')
model=KuavoModel('/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf',c/'validation-model',cfg['ik'])
world_data=json.load(open(c/'data/world.json'));ws=world_data['samples'];wt=np.array([r['elapsed_s'] for r in ws])
def interpolate_world(t):
 hi=min(int(np.searchsorted(wt,t)),len(wt)-1);lo=max(0,hi-1);u=0. if hi==lo else float(np.clip((t-wt[lo])/(wt[hi]-wt[lo]),0,1));out=[]
 for a,b in zip(ws[lo]['world'],ws[hi]['world']):
  z=dict(a);z['position']=((1-u)*np.array(a['position'])+u*np.array(b['position'])).tolist();z['quaternion_xyzw']=Slerp([0,1],Rotation.from_quat([a['quaternion_xyzw'],b['quaternion_xyzw']]))(u).as_quat().tolist();out.append(z)
 return out
def source(t):
 hi=min(int(np.searchsorted(ts,t)),len(ts)-1);lo=max(0,hi-1);u=0 if lo==hi else float(np.clip((t-ts[lo])/(ts[hi]-ts[lo]),0,1));a,b=valid[lo],valid[hi];goals=[]
 for aa,bb in zip(a['tcp_targets'],b['tcp_targets']):
  z={}
  for k,v in aa.items():
   if k not in bb:continue
   if k in ['rotation','raw_rotation']:z[k]=Slerp([0,1],Rotation.from_matrix([aa[k],bb[k]]))(u).as_matrix()
   elif isinstance(v,list):z[k]=(1-u)*np.array(v)+u*np.array(bb[k])
   else:z[k]=v
  for key in ['body_down_direction','body_forward_direction']:z[key]/=np.linalg.norm(z[key])
  goals.append(z)
 x=(1-u)*np.r_[a['base_pose'],a['q']]+u*np.r_[b['base_pose'],b['q']]
 return x,goals,a,b,u
checks=[];moveit=[];max_tcp_change=0.;max_path_delta=np.zeros(21);clearance_violations=0;minimum=.02;pose_fail=0;baseline_pose_fail=0
for k,s in enumerate(samples):
 x=np.array(s['x']);t=s['source_s'];reference,goals,a,b,u=source(t);base,q=x[:3],x[3:]
 model.cfg['tcp_position_tolerance_m']=allowed(t);model.set_environment(t);check=model.validate(q,base,goals)
 candidate_poses=model.poses(q,base);model.update_environment(base);model.set_q(q)
 pairs=model.query().ComputeSignedDistancePairwiseClosestPoints(.02);distance=min((float(p.distance) for p in pairs),default=.02)
 minimum=min(minimum,distance);clearance_violations+=distance<.01-1e-6
 baseline_check=model.validate(reference[3:],reference[:3],goals);baseline_poses=model.poses(reference[3:],reference[:3])
 max_tcp_change=max(max_tcp_change,max(np.linalg.norm(p[0]-r[0]) for p,r in zip(candidate_poses,baseline_poses)))
 max_path_delta=np.maximum(max_path_delta,abs(x-reference))
 pose_ok=all(e['position_m']<=allowed(t)+1e-5 for e in check['tcp_errors']) and check['mutual_tcp_orientation_error_rad']<=np.deg2rad(10)+1e-5
 base_pose_ok=all(e['position_m']<=allowed(t)+1e-5 for e in baseline_check['tcp_errors']) and baseline_check['mutual_tcp_orientation_error_rad']<=np.deg2rad(10)+1e-5
 pose_fail+=not pose_ok;baseline_pose_fail+=not base_pose_ok
 checks.append(dict(index=k,execution_s=s['execution_s'],source_s=t,position_allowance_m=allowed(t),box_surface_distance_m=float(np.interp(t,ts,distances)),minimum_distance_m=distance,check=check,reference_path_check=baseline_check))
 moveit.append(dict(index=k,elapsed_s=t,execution_s=s['execution_s'],kind='accepted',q=q.tolist(),base=base.tolist(),world=interpolate_world(t),tcp_world_m=[p.tolist() for p,R in candidate_poses]))
 if k and k%500==0:print(json.dumps(dict(stage='validation',sample=k,total=len(samples))),flush=True)
# Original measured/source knots get separately checked; no new IK solution is used.
knot_checks=[]
for knot,r in zip(data['knots'],valid):
 x=np.array(knot['x']);goals=[{k:np.array(v) if isinstance(v,list) else v for k,v in g.items()} for g in r['tcp_targets']]
 model.cfg['tcp_position_tolerance_m']=allowed(r['elapsed_s']);model.set_environment(r['elapsed_s']);check=model.validate(x[3:],x[:3],goals);knot_checks.append(dict(index=r['index'],check=check))
v=np.array([s['v'] for s in samples]);acc=np.array([s['a'] for s in samples]);vl=np.array(policy['limits']['velocity']);al=np.array(policy['limits']['acceleration'])
phase=np.array([s['source_s'] for s in samples]);assert np.all(np.diff(phase)>=-1e-9)
assert np.max(abs(v)/vl)<=1+1e-5 and np.max(abs(acc)/al)<=1+1e-5
audit=dict(summary=data['summary'],source_knot_checks=len(knot_checks),source_knot_invalid=sum(not z['check']['valid'] for z in knot_checks),
 sampled_states=len(samples),minimum_sampled_clearance_m=minimum,clearance_below10mm_minus1um_states=int(clearance_violations),
 sampled_pose_failures=int(pose_fail),reference_path_pose_failures_at_same_phases=int(baseline_pose_fail),
 maximum_tcp_change_from_piecewise_linear_smoothed_knots_m=max_tcp_change,maximum_coordinate_change_from_piecewise_linear_smoothed_knots=max_path_delta.tolist(),
 max_sampled_velocity_ratio=float(np.max(abs(v)/vl)),max_sampled_acceleration_ratio=float(np.max(abs(acc)/al)),
 elapsed_s=time.perf_counter()-start,phase_monotone=True,jerk_enforced=False,kalman_applied=False)
(c/'results/drake-validation.json').write_text(json.dumps(dict(summary=audit,source_knots=knot_checks,samples=checks)))
vroot=c/'moveit_verify'
for name in ['model','config','data','results']:(vroot/name).mkdir(parents=True,exist_ok=True)
shutil.copy2(Path(policy['runtime_root'])/'pipeline/moveit_verify/model/s63.urdf',vroot/'model/s63.urdf');shutil.copy2(Path(policy['runtime_root'])/'pipeline/moveit_verify/config/s63.srdf',vroot/'config/s63.srdf')
(vroot/'data/checks.json').write_text(json.dumps(dict(joint_names=JOINT_NAMES,samples=moveit)))
policy.update(method='Existing Drake IK with arm-first posture costs, original lazy-base reference and isolated input bridge; library TOPPRA V/A retiming',geometry='Natural cubic spline through optimized path knots; independent dense verification required',jerk_limits_enabled=False)
(c/'POLICY.json').write_text(json.dumps(policy,indent=2));print(json.dumps(audit,indent=2))
