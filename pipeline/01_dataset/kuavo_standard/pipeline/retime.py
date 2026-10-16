from pathlib import Path
import json,time
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
ROOT=Path(__file__).resolve().parent
p=json.load(open(ROOT/'POLICY.json'));rows=[json.loads(l) for l in (ROOT/'data/source-trajectory.jsonl').open()];valid=[r for r in rows if r['valid']]
ss=np.array([r['elapsed_s'] for r in valid]);x=np.array([np.r_[r['base_pose'],r['q']] for r in valid]);vlim=np.array(p['limits']['velocity']);alim=np.array(p['limits']['acceleration'])
start=time.perf_counter();path=ta.SplineInterpolator(ss,x,bc_type='natural')
# Refine the standard TOPPRA discretization on its own constraint failures,
# rather than applying an additional whole-trajectory slowdown afterwards.
reports=[]
for subdivisions in [8,16,32,64]:
 grid=np.unique(np.concatenate([np.linspace(a,b,subdivisions+1) for a,b in zip(ss[:-1],ss[1:])]))
 inst=algo.TOPPRA([constraint.JointVelocityConstraint(vlim*.99),constraint.JointAccelerationConstraint(alim*.99,discretization_scheme=constraint.DiscretizationType.Interpolation)],path,gridpoints=grid,parametrizer='ParametrizeConstAccel')
 sdd,sd,_=inst.compute_parameterization(0,0)
 if sd is None:raise RuntimeError('TOPPRA failed')
 traj=ta.ParametrizeConstAccel(path,grid,sd)
 # The phase is output from the same public library parameterizer, as a
 # separate identity path. It does not alter the physical optimization.
 phase=ta.ParametrizeConstAccel(ta.SplineInterpolator([ss[0],ss[-1]],[[ss[0]],[ss[-1]]]),grid,sd)
 audit_t=np.unique(np.r_[np.arange(0,traj.duration,.0001),traj.duration])
 vv=traj(audit_t,1);aa=traj(audit_t,2);vr=float(np.max(abs(vv)/vlim));ar=float(np.max(abs(aa)/alim))
 reports.append(dict(subdivisions=subdivisions,gridpoints=len(grid),duration_s=float(traj.duration),velocity_ratio=vr,acceleration_ratio=ar))
 print(json.dumps(reports[-1]),flush=True)
 if vr<=1.0001 and ar<=1.0001:break
else:raise RuntimeError('TOPPRA refinement did not pass independent V/A audit')
np.savez_compressed(ROOT/'results/parameterization.npz',source_s=ss,knots=x,grid=grid,phase_velocity=sd)
tt=np.r_[np.arange(0,traj.duration,.01),traj.duration];xx=traj(tt);vv=traj(tt,1);aa=traj(tt,2);ph=phase(tt)[:,0]
samples=[dict(execution_s=float(t),source_s=float(s),x=q.tolist(),v=v.tolist(),a=a.tolist()) for t,s,q,v,a in zip(tt,ph,xx,vv,aa)]
knots=[]
for r in valid:
 lo,hi=0.,float(traj.duration)
 for _ in range(50):
  mid=(lo+hi)/2
  if phase(mid)[0]<r['elapsed_s']:lo=mid
  else:hi=mid
 knots.append(dict(index=r['index'],source_s=r['elapsed_s'],execution_s=(lo+hi)/2,x=np.r_[r['base_pose'],r['q']].tolist()))
vp=np.max(abs(traj(audit_t,1)),axis=0);ap=np.max(abs(traj(audit_t,2)),axis=0)
summary=dict(duration_s=float(traj.duration),source_valid_duration_s=float(ss[-1]-ss[0]),wall_s=time.perf_counter()-start,sample_count=len(samples),solver_limit_scale=.99,jerk_enforced=False,numerical_time_scale=1.,refinement=reports,joint_peaks=[dict(name=n,velocity=float(v),acceleration=float(a),velocity_limit=float(vl),acceleration_limit=float(al)) for n,v,a,vl,al in zip(p['limits']['names'],vp,ap,vlim,alim)])
(ROOT/'results/retimed.json').write_text(json.dumps(dict(summary=summary,names=p['limits']['names'],knots=knots,samples=samples)))
np.savez_compressed(ROOT/'results/execution-trajectory-100hz.npz',execution_s=tt,source_s=ph,position=xx,velocity=vv,acceleration=aa,names=np.array(p['limits']['names']))
print(json.dumps(summary),flush=True)
