"""Validate every adopted sample in stable order, with process-local models."""
from pathlib import Path
import json
import shutil
import time
import numpy as np
from path_tools import Reference, load_parameterized_path
from validation_fast import make_model, validate_samples, worker_count


def main():
    c = Path(__file__).resolve().parent
    policy = json.loads((c/'POLICY.json').read_text())
    ref = Reference(c); ts = ref.ts; valid = ref.rows
    data = json.loads((c/'results/retimed.json').read_text())
    samples = data['samples']; start = time.perf_counter()
    # EXACTLY the existing execution/source grids and delivered 100-Hz samples.
    if (c/'results/parameterization.npz').exists():
        import toppra as ta
        with np.load(c/'results/parameterization.npz') as z:
            pp = load_parameterized_path(z)
            tr = ta.ParametrizeConstAccel(pp, z['grid'], z['phase_velocity'])
            ph = ta.ParametrizeConstAccel(ta.SplineInterpolator([ts[0],ts[-1]], [[ts[0]],[ts[-1]]]), z['grid'], z['phase_velocity'])
            phase_grid_time = np.r_[0., np.cumsum(2*np.diff(z['grid'])/(z['phase_velocity'][:-1]+z['phase_velocity'][1:]))]
            source_probe = np.unique(np.r_[ts, np.arange(ts[0],ts[-1],1/120),ts[-1]])
            extra_time = np.interp(source_probe,z['grid'],phase_grid_time)
        tt = np.unique(np.r_[[r['execution_s'] for r in samples], np.arange(0,tr.duration,1/240),extra_time,tr.duration])
        xx = tr(tt); vv = tr(tt,1); aa = tr(tt,2); phase = ph(tt)[:,0]
        samples = [dict(execution_s=float(t),source_s=float(s),x=q.tolist(),v=v.tolist(),a=a.tolist()) for t,s,q,v,a in zip(tt,phase,xx,vv,aa)]
    checks = []; moveit = []; max_tcp_change = 0.; max_path_delta = np.zeros(21)
    clearance_violations = 0; minimum = .02; pose_fail = 0; baseline_pose_fail = 0
    n = worker_count()
    for k, (check, native, tcp_delta, path_delta, pose_ok, baseline_ok) in enumerate(validate_samples(c, samples, n)):
        if check['index'] != k or native['index'] != k:
            raise RuntimeError('Validation sample order/coverage mismatch')
        checks.append(check); moveit.append(native)
        distance = check['minimum_distance_m']; minimum = min(minimum,distance)
        clearance_violations += distance < .01-1e-6
        max_tcp_change = max(max_tcp_change, tcp_delta)
        max_path_delta = np.maximum(max_path_delta, path_delta)
        pose_fail += not pose_ok; baseline_pose_fail += not baseline_ok
        if k and k%500 == 0:
            print(json.dumps(dict(stage='validation',sample=k,total=len(samples),workers=n)),flush=True)
    if len(checks) != len(samples):
        raise RuntimeError('Missing validation samples')
    model = make_model(c, c/'validation-model')
    knot_checks = []
    for knot, r in zip(data['knots'], valid):
        x = np.array(knot['x'])
        goals = [{k:np.array(v) if isinstance(v,list) else v for k,v in g.items()} for g in r['tcp_targets']]
        model.cfg['tcp_position_tolerance_m'] = ref.allowance(r['elapsed_s'])
        model.set_environment(r['elapsed_s'])
        knot_checks.append(dict(index=r['index'],check=model.validate(x[3:],x[:3],goals)))
    v = np.array([s['v'] for s in samples]); acc = np.array([s['a'] for s in samples])
    vl = np.array(policy['limits']['velocity']); al = np.array(policy['limits']['acceleration'])
    phase = np.array([s['source_s'] for s in samples])
    assert np.all(np.diff(phase)>=-1e-9)
    assert np.max(abs(v)/vl)<=1+1e-5 and np.max(abs(acc)/al)<=1+1e-5
    audit = dict(summary=data['summary'],source_knot_checks=len(knot_checks),source_knot_invalid=sum(not z['check']['valid'] for z in knot_checks),
        sampled_states=len(samples),minimum_sampled_clearance_m=minimum,clearance_below10mm_minus1um_states=int(clearance_violations),
        sampled_pose_failures=int(pose_fail),reference_path_pose_failures_at_same_phases=int(baseline_pose_fail),
        maximum_tcp_change_from_piecewise_linear_smoothed_knots_m=max_tcp_change,maximum_coordinate_change_from_piecewise_linear_smoothed_knots=max_path_delta.tolist(),
        max_sampled_velocity_ratio=float(np.max(abs(v)/vl)),max_sampled_acceleration_ratio=float(np.max(abs(acc)/al)),
        elapsed_s=time.perf_counter()-start,phase_monotone=True,jerk_enforced=False,kalman_applied=False,validation_workers=n)
    (c/'results/drake-validation.json').write_text(json.dumps(dict(summary=audit,source_knots=knot_checks,samples=checks)))
    vroot = c/'moveit_verify'
    for name in ['model','config','data','results']:(vroot/name).mkdir(parents=True,exist_ok=True)
    shutil.copy2(Path(policy['runtime_root'])/'pipeline/moveit_verify/model/s63.urdf',vroot/'model/s63.urdf')
    shutil.copy2(Path(policy['runtime_root'])/'pipeline/moveit_verify/config/s63.srdf',vroot/'config/s63.srdf')
    from retarget.model import JOINT_NAMES
    (vroot/'data/checks.json').write_text(json.dumps(dict(joint_names=JOINT_NAMES,samples=moveit)))
    policy.update(method='Existing Drake IK with arm-first posture costs, original lazy-base reference and isolated input bridge; library TOPPRA V/A retiming',
        geometry='Constrained C2 B-spline with fixed lazy-base curve; independent dense verification required',jerk_limits_enabled=False)
    (c/'POLICY.json').write_text(json.dumps(policy,indent=2))
    print(json.dumps(audit,indent=2),flush=True)


if __name__ == '__main__':
    main()
