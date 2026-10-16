from pathlib import Path
import sys,json,time,copy,gc,hashlib
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation,Slerp
from scipy.special import betainc
import toppra as ta
import toppra.constraint as tc
import toppra.algorithm as talgo
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'source'))
from retarget.model import KuavoModel,JOINT_NAMES
from pydrake.solvers import SolverOptions
rows=[json.loads(l) for l in (ROOT/'data/reference.jsonl').open()]
cfg=json.load(open(ROOT/'data/reference-config.json'));policy=json.load(open(ROOT/'POLICY.json'))
world=json.load(open(ROOT/'data/world.json'))['samples'];wt=np.array([r['elapsed_s'] for r in world]);times=np.array([r['elapsed_s'] for r in rows]);input_indices=[i for i,r in enumerate(rows) if r.get('input_available',r.get('valid',False))];valid=[rows[i] for i in input_indices];ts=times[input_indices]
URDF='/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf'
def save(path,data):path.write_text(json.dumps(data,indent=2))
def goals(r):return [{k:np.array(v) if isinstance(v,list) else v for k,v in g.items()} for g in r['tcp_targets']]
def distance(t,gs):
 ws=world[int(np.argmin(abs(wt-t)))]['world'];ds=[]
 for g in gs:
  for ob in ws:
   if not ob['id'].startswith('box_'):continue
   x=Rotation.from_quat(ob['quaternion_xyzw']).inv().apply(g['position']-ob['position']);d=abs(x)-np.array(ob['dimensions'])/2
   ds.append(max(0.,float(np.linalg.norm(np.maximum(d,0))+min(max(d),0))))
 return min(ds)
distances=np.array([distance(r['elapsed_s'],goals(r)) if r['valid'] else np.nan for r in rows])
# Experimental, auditable isolated-spike mask. No raw recording is changed.
spikes=[]
for side in range(2):
 p=np.array([r['tcp_targets'][side]['position'] for r in valid])
 for k in range(1,len(p)-1):
  u=(ts[k]-ts[k-1])/(ts[k+1]-ts[k-1]);expected=(1-u)*p[k-1]+u*p[k+1]
  a=p[k]-p[k-1];b=p[k+1]-p[k]
  residual=float(np.linalg.norm(p[k]-expected));net=float(np.linalg.norm(p[k+1]-p[k-1]))
  opposite=float(a@b/max(np.linalg.norm(a)*np.linalg.norm(b),1e-12))
  if residual>.025 and net<.015 and opposite<-.8:
   spikes.append(dict(index=input_indices[k],side=side,elapsed_s=float(ts[k]),residual_m=residual,neighbor_net_m=net,position=expected.tolist(),interpolation_weight=float(u)))
save(ROOT/'input-spikes.json',spikes)
profiles=[json.load(open(ROOT/'PROFILE.json'))]
for p in profiles:p.update(solver_position_factor=.85,solver_collision_reserve_m=.001,solver_joint_reserve_rad=.001,solver_orientation_reserve_deg=.5)
save(ROOT/'profiles-originalbase.json',profiles)
def base(r,p):
 return np.array(r['base_pose'])
def allowance(t,p):
 if not p.get('adaptive'):return p.get('tol',.01)
 d=float(np.interp(t,times[input_indices],distances[input_indices]));u=np.clip((d-.1)/.1,0,1)
 return float(.01+.01*betainc(3,3,u))
def metrics(rs):
 good=[r for r in rs if r['valid']]
 if len(good)<3:return dict(accepted=len(good),maximum_joint_step_deg=None,joint_squared_steps=None,source_acceleration_rms=None,base_9_16_xy_path_m=None,base_9_16_range_m=None,base_9_16_yaw_range_deg=None)
 q=np.array([r['q'] for r in good]);b=np.array([r['base_pose'] for r in good]);tt=np.array([r['elapsed_s'] for r in good]);v=np.diff(q,axis=0)/np.diff(tt)[:,None];a=2*np.diff(v,axis=0)/(np.diff(tt)[:-1]+np.diff(tt)[1:])[:,None]
 roi=b[(tt>=9)&(tt<=16)];roi=roi if len(roi) else b
 if len(roi)==0:roi=b[:1]
 return dict(accepted=len(good),maximum_joint_step_deg=float(np.rad2deg(abs(np.diff(q,axis=0))).max()),joint_squared_steps=float(np.sum(np.diff(q,axis=0)**2)),source_acceleration_rms=float(np.sqrt(np.mean(a*a))),base_9_16_xy_path_m=float(np.linalg.norm(np.diff(roi[:,:2],axis=0),axis=1).sum()),base_9_16_range_m=np.ptp(roi[:,:2],axis=0).tolist(),base_9_16_yaw_range_deg=float(np.rad2deg(np.ptp(roi[:,2]))))
def interp_goal(a,b,u):
 result=[]
 for aa,bb in zip(a['tcp_targets'],b['tcp_targets']):
  z={}
  for key,v in aa.items():
   if key not in bb:continue
   if key in ['rotation','raw_rotation']:z[key]=Slerp([0,1],Rotation.from_matrix([aa[key],bb[key]]))(u).as_matrix()
   elif isinstance(v,list):z[key]=(1-u)*np.array(v)+u*np.array(bb[key])
   else:z[key]=v
  for key in ['body_down_direction','body_forward_direction']:z[key]/=np.linalg.norm(z[key])
  result.append(z)
 return result
def timing(path,vl,al):
 for sub in [8,32,64]:
  grid=np.unique(np.concatenate([np.linspace(a,b,sub+1) for a,b in zip(ts[:-1],ts[1:])]))
  inst=talgo.TOPPRA([tc.JointVelocityConstraint(vl*.99),tc.JointAccelerationConstraint(al*.99,discretization_scheme=tc.DiscretizationType.Interpolation)],path,gridpoints=grid,parametrizer='ParametrizeConstAccel');_,sd,_=inst.compute_parameterization(0,0)
  if sd is None:return None
  tr=ta.ParametrizeConstAccel(path,grid,sd);tt=np.r_[np.arange(0,tr.duration,.001),tr.duration]
  vr=float(np.max(abs(tr(tt,1))/vl));ar=float(np.max(abs(tr(tt,2))/al))
  if vr<=1.0001 and ar<=1.0001:return dict(duration_s=float(tr.duration),velocity_ratio=vr,acceleration_ratio=ar,gridpoints=len(grid))
 return dict(duration_s=float(tr.duration),velocity_ratio=vr,acceleration_ratio=ar,gridpoints=len(grid),audit_failed=True)
summary=[]
for profile in profiles:
 dest=ROOT/'profiles'/profile['name'];dest.mkdir(parents=True,exist_ok=True)
 if (dest/'summary.json').exists():summary.append(json.load(open(dest/'summary.json')));continue
 start=time.perf_counter();conf=copy.deepcopy(cfg['ik']);conf['max_iterations']=180;conf['tool_continuity_weight']=profile.get('tool_weight',100.);conf['arm_continuity_weight']=profile.get('arm_weight',100.)
 if 'parallel' in profile:conf['mutual_tcp_orientation_tolerance_rad']=np.deg2rad(profile['parallel'])
 for key in ['chest_position_weight','chest_orientation_weight','elbow_weight']:conf[key]*=profile.get('body_scale',1.)
 conf['enable_acceleration_cost']='acc_weight' in profile;conf['acceleration_weight']=profile.get('acc_weight',.01)
 if 'torso_tolerance_deg' in profile:
  conf['torso_adjust_orientation_rad']=np.deg2rad(profile['torso_tolerance_deg']);conf['torso_orientation_tolerance_rad']=conf['torso_adjust_orientation_rad']
 conf['torso_adjust_weight']=profile.get('body_anchor_weight',conf['torso_adjust_weight'])
 if 'body_cont_weight' in profile:
  for key in ['torso_motion_weight','torso_intent_motion_weight','torso_follow_motion_weight']:conf[key]=profile['body_cont_weight']
 m=KuavoModel(URDF,ROOT/'cache',conf);original_lower=m.lower.copy();original_upper=m.upper.copy();out=copy.deepcopy(rows);prev=None;prevbase=None;prevtime=None;velocity=None;velocity_dt=None;failures=[];attempts=0;status=[];ik_start=time.perf_counter()
 for i in input_indices:
  r=out[i];t=r['elapsed_s'];b=base(r,profile);gs=goals(r)
  if profile.get('bridge'):
   for spike in spikes:
    if spike['index']!=i:continue
    s=spike['side'];r['tcp_targets'][s]['observed_position']=copy.deepcopy(r['tcp_targets'][s]['position']);r['tcp_targets'][s]['position']=spike['position'];r['target_outlier_bridge']=spike
    u=spike['interpolation_weight'];r['grippers'][s]['observed_open_fraction']=r['grippers'][s]['open_fraction'];r['grippers'][s]['open_fraction']=(1-u)*rows[i-1]['grippers'][s]['open_fraction']+u*rows[i+1]['grippers'][s]['open_fraction'];gs=goals(r)
  conf['tcp_position_tolerance_m']=allowance(t,profile);r['position_allowance_m']=conf['tcp_position_tolerance_m'];r['box_surface_distance_m']=float(distances[i]);r['base_pose']=b.tolist()
  m.set_environment(t);m.environment_previous_base=prevbase;m.environment_previous_time=prevtime
  ref={k:np.array(v) for k,v in r['torso_placement_target'].items()};ref['intent_active']=r['torso_reference_active']
  ref['elbows']=m.elbow_targets({int(k):np.array(v) for k,v in r['human_points']['body'].items()},gs,np.array(r['q']),b)
  objective_ref=np.array(r['q']) if prev is None else prev
  ref['continuity_pose']=dict(q=objective_ref.tolist(),base=(b if prevbase is None else prevbase).tolist(),ticks=r['timestamp_ticks'])
  fixed=None if profile.get('free_torso') else np.array(r.get('torso_anchor_q',r['torso_q']))
  freeze_body=False
  conf['bounded_torso']=not freeze_body
  if freeze_body:fixed=np.array(rows[3 if profile.get('static_body') else int(np.argmin(abs(times-9)))]['q'][:4])
  dt=None if prevtime is None else t-prevtime;best=None;candidate_checks=[]
  seeds=[objective_ref,np.array(rows[i]['q'])]
  if np.max(abs(seeds[0]-seeds[1]))<1e-10:seeds=seeds[:1]
  for retry in [False,True]:
   m.path_collision_fractions=(.25,.5,.75) if retry else ()
   for seed in seeds:
    allowed=allowance(t,profile);conf['tcp_position_tolerance_m']=allowed*profile['solver_position_factor'];conf['collision_margin_m']=.01+profile['solver_collision_reserve_m'];conf['body_down_max_deg']=60-profile['solver_orientation_reserve_deg'];conf['mutual_tcp_orientation_tolerance_rad']=np.deg2rad(10-profile['solver_orientation_reserve_deg'])
    guard=np.full(18,profile['solver_joint_reserve_rad'])
    if freeze_body:guard[:4]=0.
    m.lower=original_lower+guard;m.upper=original_upper-guard
    prog,qv=m.solve(gs,b,ref,seed,previous=prev,velocity=velocity,dt=dt,velocity_dt=velocity_dt,build_only=True,fixed_torso=fixed)
    m.lower=original_lower;m.upper=original_upper;conf['tcp_position_tolerance_m']=allowed;conf['collision_margin_m']=.01;conf['body_down_max_deg']=60.;conf['mutual_tcp_orientation_tolerance_rad']=np.deg2rad(10.)
    options=SolverOptions();options.SetOption(m.solver.id(),'Major iterations limit',180);options.SetOption(m.solver.id(),'Major feasibility tolerance',1e-7);options.SetOption(m.solver.id(),'Major optimality tolerance',1e-4)
    res=m.solver.Solve(prog,None,options);attempts+=1;q=res.GetSolution(qv)
    check=m.validate(q,b,gs,prev,dt=dt)
    if fixed is not None:check['valid']=bool(check['valid'] and m.torso_adjust_valid(q,fixed,b,ref))
    check['snopt_info']=int(res.get_solver_details().info);check['optimality_reported']=bool(res.is_success());candidate_checks.append(check)
    if check['valid']:
     score=float(res.get_optimal_cost())
     if best is None or score<best[0]:best=(score,q.copy(),copy.deepcopy(check))
   if best is not None:break
  r['attempt_checks']=candidate_checks
  if best is None:
   r.update(valid=False,q=None,reason='search_no_verified_candidate');failures.append(dict(index=i,source_s=t))
   continue
  r.update(valid=True,q=best[1].tolist(),torso_q=best[1][:4].tolist(),verification=best[2],reason='native_ik_parameter_candidate')
  if prev is not None:velocity=(best[1]-prev)/dt;velocity_dt=dt
  prev=best[1];prevbase=b;prevtime=t;status.append(best[2]['snopt_info'])
  if i%200==0:print(json.dumps(dict(profile=profile['name'],stage='IK',frame=i,failures=len(failures))),flush=True)
 ik_s=time.perf_counter()-ik_start
 (dest/'trajectory.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in out));save(dest/'config.json',dict(profile=profile,ik=conf))
 report=dict(profile=profile,metrics=metrics(out),ik_wall_s=ik_s,attempts=attempts,failures=failures,solver_statuses={str(k):status.count(k) for k in set(status)},optimality_claimed=False)
 if not failures:
  xx=np.array([np.r_[r['base_pose'],r['q']] for r in [out[i] for i in input_indices]]);path=ta.SplineInterpolator(ts,xx,bc_type='natural');tt=np.unique(np.r_[ts,np.arange(ts[0],ts[-1],1/60),ts[-1]])
  checks=[];maxerr=0.;clearance=.02;invalid=0;position_fail=0;angle_fail=0;guard_fail=0;collfail=0
  for t in tt:
   hi=min(int(np.searchsorted(ts,t)),len(ts)-1);lo=max(0,hi-1);u=0 if lo==hi else (t-ts[lo])/(ts[hi]-ts[lo]);gs=interp_goal(out[input_indices[lo]],out[input_indices[hi]],u);z=path(float(t));conf['tcp_position_tolerance_m']=allowance(t,profile);m.set_environment(float(t));check=m.validate(z[3:],z[:3],gs)
   near=[float(p.distance) for p in m.query().ComputeSignedDistancePairwiseClosestPoints(.02)];d=min(near,default=.02);clearance=min(clearance,d);err=max(e['position_m'] for e in check['tcp_errors']);maxerr=max(maxerr,err)
   posbad=err>conf['tcp_position_tolerance_m']+1e-5;anglebad=check['mutual_tcp_orientation_error_rad']>conf['mutual_tcp_orientation_tolerance_rad']+1e-5
   invalid+=not check['valid'];position_fail+=posbad;angle_fail+=anglebad;collfail+=d<.01-1e-6;guard_fail+=bool(np.any(z[3:]<m.lower-1e-7) or np.any(z[3:]>m.upper+1e-7))
   if not check['valid']:checks.append(dict(source_s=float(t),minimum_distance_m=d,check=check))
  report['spline_screen']=dict(states=len(tt),invalid=int(invalid),position_failures=int(position_fail),parallel_failures=int(angle_fail),joint_guard_failures=int(guard_fail),collision_margin_failures=int(collfail),minimum_clearance_m=clearance,maximum_grasp_error_m=maxerr)
  save(dest/'spline_failures.json',checks)
  report['timing']=timing(path,np.array(policy['limits']['velocity']),np.array(policy['limits']['acceleration']))
 report['wall_s']=time.perf_counter()-start;save(dest/'summary.json',report);summary.append(report);save(ROOT/'results/sweep-originalbase.json',summary)
 print(json.dumps(report),flush=True);del m;gc.collect()
save(ROOT/'results/sweep-originalbase.json',summary)
