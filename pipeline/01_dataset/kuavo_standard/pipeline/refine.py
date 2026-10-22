"""Constrained C2 B-spline refinement of an existing Drake IK path.

Fix the existing base curve, retain the original IK/reference files and optimize
only locally affected body/arm control points. Bounds on ALL control points
enforce joint limits throughout the curve. FK/orientation/collision constraints
are imposed at samples, then independently checked at finer samples. Failed
samples become new constraints; finite refinement exhaustion is a hard failure.
"""
from pathlib import Path
import json
import sys
import time
import numpy as np
from scipy.interpolate import BSpline
from pydrake.math import BsplineBasis, RotationMatrix
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.trajectories import BsplineTrajectory
from pydrake.multibody.inverse_kinematics import InverseKinematics, PointToPointDistanceConstraint
from pydrake.solvers import SolverOptions
from path_tools import Reference, SplinePath, audit
from validation_fast import install_environment_cache

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'source'))
from retarget.model import KuavoModel, TCP_SEMANTIC_BASES
from retarget.geometry import yaw_rotation


def optimize(path, ref, model, failures):
    b = path.spline
    start, end = path.path_interval
    duration = end-start
    basis = BsplineBasis(b.k+1, ((b.t-start)/duration).tolist())
    # Drake's native constraints consume plant order, not the named joint order.
    raw = np.empty_like(b.c[:, 3:])
    raw[:, model.indices] = b.c[:, 3:]
    initial = BsplineTrajectory(basis, [q[:, None] for q in raw])
    opt = KinematicTrajectoryOptimization(initial)
    prog = opt.get_mutable_prog()
    controls = opt.control_points()
    count = len(b.c)

    active = set()
    for f in failures:
        weights = BSpline.design_matrix([f['source_s']], b.t, b.k).toarray()[0]
        for j in np.flatnonzero(abs(weights) > 1e-14):
            active.update(range(max(0, j-2), min(count, j+3)))
    lower = np.empty(18); upper = np.empty(18)
    # Small numerical reserve, keeping the adopted 0.02-rad interior margin.
    lower[model.indices] = model.lower + 1e-5
    upper[model.indices] = model.upper - 1e-5
    out = np.flatnonzero(np.any((raw < lower) | (raw > upper), axis=1))
    for j in out:
        active.update(range(max(0, j-2), min(count, j+3)))
    opt.AddPositionBounds(lower, upper)
    prog.AddBoundingBoxConstraint(initial.end_time()-initial.start_time(), initial.end_time()-initial.start_time(), opt.duration())
    for j in range(count):
        if j not in active:
            prog.AddBoundingBoxConstraint(raw[j], raw[j], controls[:, j])
        else:
            prog.AddQuadraticErrorCost(np.eye(18), raw[j], controls[:, j])
    opt.AddPathPositionConstraint(raw[0], raw[0], 0.)
    opt.AddPathPositionConstraint(raw[-1], raw[-1], 1.)
    opt.SetInitialGuess(initial)

    # B-spline changes have compact support. Constrain every original knot and
    # a 60-Hz source grid on that support; independent audit uses 240 Hz.
    probe = np.unique(np.r_[ref.ts, np.arange(start, end, 1/60), end,
                             [f['source_s'] for f in failures]])
    matrices = BSpline.design_matrix(probe, b.t, b.k)
    active_array = np.array(sorted(active), dtype=int)
    selected = np.asarray(abs(matrices[:, active_array]).sum(axis=1)).ravel() > 1e-14
    selected_times = probe[selected]
    # Own a root context for each time-dependent world: constraints must not
    # accidentally share the last sample's obstacle transforms.
    contexts = []
    for t in selected_times:
        x = path(float(t)); base = x[:3]
        rb = yaw_rotation(base[2]); origin = np.r_[base[:2], 0.]
        root = model.root_context.Clone()
        old = model.root_context, model.context, model.scene_context
        try:
            model.root_context = root
            model.context = model.plant.GetMyMutableContextFromRoot(root)
            model.scene_context = model.scene.GetMyMutableContextFromRoot(root)
            model.update_environment(base, float(t))
            model.set_q(x[3:])
            context = model.context
        finally:
            model.root_context, model.context, model.scene_context = old
        ik = InverseKinematics(model.plant, context, with_joint_limits=False)
        phase = float((t-start)/duration)
        for frame, goal, camera_axis in zip(model.frames, ref.goals(t), model.camera_mount_axes):
            point = rb.T @ (goal['position']-origin)
            distance = PointToPointDistanceConstraint(model.plant, frame, np.zeros(3),
                model.plant.world_frame(), point, 0., ref.allowance(t)*.98, context)
            opt.AddPathPositionConstraint(distance, phase)
            down = rb.T @ goal['body_down_direction']
            forward = rb.T @ goal['body_forward_direction']
            ik.AddAngleBetweenVectorsConstraint(frame, np.array([0.,0.,-1.]), model.plant.world_frame(), down,
                0., np.deg2rad(model.cfg['body_down_max_deg']-.05))
            ik.AddAngleBetweenVectorsConstraint(frame, np.array([0.,0.,-1.]), model.plant.world_frame(), forward,
                0., np.pi/2-1e-4)
            ik.AddAngleBetweenVectorsConstraint(frame, camera_axis, model.plant.world_frame(), -down,
                0., np.pi/2-1e-4)
        ik.AddOrientationConstraint(model.frames[0], RotationMatrix(TCP_SEMANTIC_BASES[0]),
            model.frames[1], RotationMatrix(TCP_SEMANTIC_BASES[1]),
            model.cfg['mutual_tcp_orientation_tolerance_rad']-np.deg2rad(.05))
        ik.AddMinimumDistanceLowerBoundConstraint(model.cfg['collision_margin_m']+1e-5, .02)
        for binding in ik.prog().GetAllConstraints():
            opt.AddPathPositionConstraint(binding.evaluator(), phase)
        contexts.append((root, ik))
    options = SolverOptions()
    options.SetOption(model.solver.id(), 'Major iterations limit', 200)
    options.SetOption(model.solver.id(), 'Major feasibility tolerance', 1e-8)
    options.SetOption(model.solver.id(), 'Major optimality tolerance', 1e-6)
    print(json.dumps(dict(stage='path_optimize', active_controls=len(active), constrained_states=len(selected_times))), flush=True)
    result = model.solver.Solve(prog, None, options)
    solution = result.GetSolution(controls).T
    if not np.isfinite(solution).all():
        raise RuntimeError('Nonfinite constrained path solution')
    # Check both the solver constraints and independent geometric audit below.
    feasible = prog.CheckSatisfied(prog.GetAllConstraints(), result.GetSolution(), 2e-7)
    details = dict(snopt_info=int(result.get_solver_details().info), success=bool(result.is_success()),
        constraints_satisfied=bool(feasible), active_controls=len(active), constrained_states=len(selected_times))
    if not feasible:
        raise RuntimeError('Constrained path optimization infeasible: '+json.dumps(details))
    coeff = b.c.copy()
    coeff[:, 3:] = solution[:, model.indices]
    # The fixed lazy-base curve is copied bit-for-bit.
    return SplinePath(b.t, coeff, b.k), details


def main():
    started = time.perf_counter()
    ref = Reference(ROOT)
    cfg = json.loads((ROOT/'data/source-config.json').read_text())['ik']
    cfg['environment_world_json'] = str(ROOT/'data/world.json')
    if cfg.get('orientation_mode') != 'body_downward' or not cfg.get('wrist_camera_flip_only'):
        raise ValueError('Constrained refinement requires the adopted body-downward camera-aware policy')
    model = KuavoModel('/home/lab6/humanoid/git/kuavo_ros_application/data_collection/custom_urdf/kuavo_s63_twofinger.urdf',
        ROOT/'path-model', cfg)
    install_environment_cache(model)
    destination = ROOT/'results/constrained-path.npz'
    if destination.exists():
        with np.load(destination) as z:
            path = SplinePath(z['path_knots'], z['path_coefficients'], z['path_degree'])
    else:
        path = SplinePath(ref.initial.t, ref.initial.c, ref.initial.k)
    times = np.unique(np.r_[ref.ts, np.arange(ref.ts[0], ref.ts[-1], 1/240), ref.ts[-1]])
    prior = ROOT/'results/drake-validation.json'
    extra = []
    if prior.exists():
        extra = [r['source_s'] for r in json.loads(prior.read_text())['samples'] if not r['check']['valid']]
        times = np.unique(np.r_[times, extra])
    reports = []
    audit_cache = {}
    audit_stats = dict(computed=0, reused=0)
    for iteration in range(6):
        failures = audit(path, ref, model, times, cache=audit_cache, statistics=audit_stats)
        # A spline's convex hull gives an all-time joint-bound guarantee.
        bad_controls = bool(np.any(path.spline.c[:, 3:] < model.lower) or np.any(path.spline.c[:, 3:] > model.upper))
        print(json.dumps(dict(stage='path_audit', iteration=iteration, checked=len(times), invalid=len(failures),
                              control_bounds_invalid=bad_controls)), flush=True)
        if not failures and not bad_controls:
            np.savez_compressed(destination, **path.fields())
            initial = SplinePath(ref.initial.t, ref.initial.c, ref.initial.k)
            delta = np.max(abs(path(times)-initial(times)), axis=0)
            previous = ROOT/'results/path-refinement.json'
            history = json.loads(previous.read_text()).get('optimization_history', []) if previous.exists() else []
            report = dict(status='verified_geometry', method='Drake SNOPT constrained C2 B-spline; adaptive sampled FK/collision refinement',
                checked_source_states=len(times), invalid=0, joint_bounds_all_time=True,
                original_source_rows_unchanged=True, base_curve_unchanged=bool(np.array_equal(path.spline.c[:, :3], ref.initial.c[:, :3])),
                maximum_change_from_initial_by_coordinate=delta.tolist(), optimization_history=history+reports,
                elapsed_s=time.perf_counter()-started, audit_cache=audit_stats)
            (ROOT/'results/path-refinement.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report), flush=True)
            return
        if iteration == 5:
            break
        path, detail = optimize(path, ref, model, failures)
        reports.append(dict(iteration=iteration, invalid_before=len(failures), **detail))
    raise RuntimeError('Constrained path failed independent geometric verification after five refinements')


if __name__ == '__main__':
    main()
