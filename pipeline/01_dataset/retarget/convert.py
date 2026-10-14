"""Generate auditable offline trajectories without touching raw capture files."""
import argparse
from collections import Counter
import copy
import importlib.metadata
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from .geometry import LazyBase, PlacementFilter, TorsoReference, adaptive_torso_target, chest_frame, check_rotation, pinch_frame, rotation_error, wrap
from .input import WorldAlignment, interpolate, read_episode, sha256
from .model import JOINT_NAMES, KuavoModel
from .hand_mapping import tcp_frame, calibrate_palm_axes, HandOrientationFilter


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(type(value).__name__)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=serial, allow_nan=False) + '\n')


def base_valid(candidate, last, velocity, dt, previous_dt, config):
    if last is None:
        return True
    delta = candidate - last
    delta[2] = wrap(delta[2])
    speed = delta / dt
    if np.linalg.norm(speed[:2]) > config['speed_m_s'] + 1e-5 or abs(speed[2]) > config['yaw_speed_rad_s'] + 1e-5:
        return False
    if velocity is not None:
        accel = (speed - velocity) / ((dt + previous_dt) / 2)
        if np.linalg.norm(accel[:2]) > config['accel_m_s2'] + 1e-4 or abs(accel[2]) > config['yaw_accel_rad_s2'] + 1e-4:
            return False
    return True


def retry_base(preferred, desired, previous, velocity, dt, previous_dt, config):
    """A reachability retry, always bounded relative to the last accepted pose."""
    if previous is None:
        return np.asarray(desired).copy()
    error = desired - previous
    error[2] = wrap(error[2])
    target_velocity = error / dt
    n = np.linalg.norm(target_velocity[:2])
    target_velocity[:2] *= min(1., config['speed_m_s'] / max(n, 1e-12))
    target_velocity[2] = np.clip(target_velocity[2], -config['yaw_speed_rad_s'], config['yaw_speed_rad_s'])
    if velocity is not None:
        allowed = np.array([config['accel_m_s2']] * 2 + [config['yaw_accel_rad_s2']]) * (dt + previous_dt) / 2
        delta = np.clip(target_velocity - velocity, -allowed, allowed)
        n = np.linalg.norm(delta[:2])
        delta[:2] *= min(1., allowed[0] / max(n, 1e-12))
        target_velocity = velocity + delta
    return previous + target_velocity * dt


def validate_config(cfg):
    if cfg.get('schema_version') != 1:
        raise ValueError('Unknown retarget configuration version')
    for correction in cfg['hand_to_tcp_rotation'].values():
        check_rotation(correction)
    if not 0 < cfg['ik']['velocity_scale'] <= 1:
        raise ValueError('Velocity scale must be in (0,1]')
    positive = [('input', 'output_hz'), ('input', 'maximum_bracket_s'), ('input', 'wrist_agreement_m'),
        ('ik', 'tcp_position_tolerance_m'), ('ik', 'tcp_orientation_tolerance_rad'), ('ik', 'collision_margin_m'),
        ('ik', 'torso_accel_rad_s2'), ('ik', 'arm_accel_rad_s2')]
    if cfg.get('hand_mapping') == 'mcp_filtered':
        positive.extend(('hand_orientation_filter',key) for key in ('tau_s','maximum_lag_rad'))
    if cfg['ik'].get('bounded_torso',False):
        positive.extend(('ik',key) for key in ('torso_adjust_joint_rad','torso_adjust_position_m','torso_adjust_orientation_rad','torso_adjust_weight'))
    for section, key in positive:
        if not np.isfinite(cfg[section][key]) or cfg[section][key] <= 0:
            raise ValueError('Invalid config value: ' + section + '.' + key)
    if not 0 < cfg['input']['maximum_bracket_s'] <= .05:
        raise ValueError('Tracking brackets may not exceed 50ms')
    if cfg['gripper']['closed_pinch_m'] >= cfg['gripper']['open_pinch_m']:
        raise ValueError('Invalid pinch normalization')
    for axis in ('position', 'angle'):
        if not 0 <= cfg['torso_reference'][axis + '_exit_' + ('m' if axis == 'position' else 'rad')] < cfg['torso_reference'][axis + '_enter_' + ('m' if axis == 'position' else 'rad')]:
            raise ValueError('Torso hysteresis exit must be below entry')


def convert(source, output, urdf, config_path, mode='lazy', maximum_frames=None, start_seconds=0., time_scale=1., allocation='geometry', hand_method=None):
    source, output, urdf = Path(source).resolve(), Path(output).resolve(), Path(urdf).resolve()
    if output == source or source in output.parents or output in source.parents:
        raise ValueError('Retarget output must be outside the raw episode tree')
    if output.exists() or output.with_name(output.name + '.partial').exists():
        raise FileExistsError('Use a fresh output directory')
    cfg = json.loads(Path(config_path).read_text())
    validate_config(cfg)
    hand_method = hand_method or cfg.get('hand_mapping', 'legacy_pinch')
    if hand_method not in ('mcp_pinch', 'legacy_pinch', 'palm_calibrated', 'index_palm', 'mcp_filtered'):
        raise ValueError('unknown_hand_mapping')
    if not np.isfinite(time_scale) or time_scale < 1:
        raise ValueError('Time scale must be finite and >=1')
    raw_manifest, streams, discontinuities = read_episode(source)
    if not all(streams.values()):
        raise ValueError('Missing body/XRHand streams')
    # Use source body timestamps, retaining at most one sample per nominal slot.
    first_tick = streams['body'][0]['ticks']
    selected, last_slot = [], -1
    for sample in streams['body']:
        elapsed = (sample['ticks'] - first_tick) * 1e-7
        slot = int(np.floor(elapsed * cfg['input']['output_hz']))
        if elapsed >= start_seconds and slot > last_slot:
            selected.append(sample)
            last_slot = slot
    if maximum_frames is not None:
        selected = selected[:maximum_frames]
    staging = output.with_name(output.name + '.partial')
    staging.mkdir(parents=True)
    dump(staging / 'config.json', cfg)
    model = KuavoModel(urdf, staging / 'model', cfg['ik'])
    provenance = {'schema_version': 1, 'state': 'processing', 'torso_mode': mode, 'allocation': allocation, 'hand_mapping': hand_method, 'source_episode': str(source),
        'source_manifest_sha256': sha256(source / 'manifest.json'), 'source_checksums': {k: v['sha256'] for k, v in raw_manifest['files'].items()},
        'config_sha256': sha256(config_path), 'model': model.metadata, 'packages': {n: importlib.metadata.version(n) for n in ('drake', 'numpy', 'scipy', 'trimesh')},
        'implementation_sha256': {p.name: sha256(p) for p in Path(__file__).parent.glob('*.py')},
        'dataset_ready': False, 'physical_motion': False, 'robot_commands_sent': 0, 'camera_tracking_clock_verified': False,
        'execution_time_scale': time_scale, 'note': 'Source timestamps never change; execution time scale is explicit.'}
    dump(staging / 'manifest.json', provenance)
    alignment = None
    hand_corrections = cfg['hand_to_tcp_rotation']
    if hand_method == 'palm_calibrated':
        hand_corrections = {}; calibration = {}
        for side,kind in (('left','XRHandLeft'),('right','XRHandRight')):
            samples=[]
            for sample in streams['body']:
                if (sample['ticks']-first_tick)*1e-7 > 1.:
                    break
                try:
                    body,_=interpolate(streams['body'],sample['ticks'],cfg['input']['maximum_bracket_s'],discontinuities)
                    hand,_=interpolate(streams[kind],sample['ticks'],cfg['input']['maximum_bracket_s'],discontinuities)
                    if np.linalg.norm(body[19 if side=='left' else 45]-hand[1]) > cfg['input']['wrist_agreement_m']:
                        continue
                    align=WorldAlignment(body,cfg['world'])
                    samples.append(align.points(hand))
                except ValueError:
                    continue
            matrix,meta=calibrate_palm_axes(samples,cfg['hand_to_tcp_rotation'][side])
            hand_corrections[side]=matrix
            calibration[side]=dict(meta,palm_to_tcp_rotation=matrix.tolist())
        provenance['hand_axis_reference']=calibration
        dump(staging/'manifest.json',provenance)
    filtered_rotations={}
    if hand_method == 'mcp_filtered':
        fixed_alignment=None;last_tick=None;last_raw=None
        filters=[HandOrientationFilter(cfg['hand_orientation_filter']['tau_s'],cfg['hand_orientation_filter']['maximum_lag_rad']) for _ in range(2)]
        for sample in selected:
            t=sample['ticks']
            try:
                data={k:interpolate(stream,t,cfg['input']['maximum_bracket_s'],discontinuities)[0] for k,stream in streams.items()}
                if max(np.linalg.norm(data['body'][i]-data[k][1]) for i,k in ((19,'XRHandLeft'),(45,'XRHandRight'))) > cfg['input']['wrist_agreement_m']:
                    raise ValueError('native_xr_coordinate_disagreement')
                if fixed_alignment is None:fixed_alignment=WorldAlignment(data['body'],cfg['world'])
                raw=[tcp_frame(fixed_alignment.points(data[k]),hand_corrections[side],'mcp_pinch')[1] for side,k in (('left','XRHandLeft'),('right','XRHandRight'))]
                interval=None if last_tick is None else (t-last_tick)*1e-7
                if interval is not None and (interval>cfg['base']['max_gap_s'] or any(last_tick<d<=t for d in discontinuities)):
                    interval=None
                if interval is not None and last_raw is not None and any(rotation_error(a,b)/interval>cfg['input']['maximum_orientation_speed_rad_s'] for a,b in zip(last_raw,raw)):
                    raise ValueError('hand_orientation_jump')
                filtered_rotations[t]=[f.update(rot,interval) for f,rot in zip(filters,raw)]
                last_tick,last_raw=t,raw
            except ValueError:
                last_tick=last_raw=None
                for f in filters:f.rotation=None
        provenance['hand_orientation_filter']=dict(cfg['hand_orientation_filter'],method='causal_SO3_before_IK; capped_lag; no post_IK_filter; resets_on_invalid_tracking')
        dump(staging/'manifest.json',provenance)

    def corrected_rotation(rotation,tick,side):
        if hand_method != 'mcp_filtered':return rotation
        if tick not in filtered_rotations:raise ValueError('invalid_hand_orientation_source')
        return filtered_rotations[tick][0 if side=='left' else 1]
    filter_config = dict(cfg['base'], max_gap_s=cfg['base']['max_gap_s'] * time_scale)
    filter_base = LazyBase(filter_config, cfg['world']['initial_base_pose'])
    torso_reference = None
    placement_filter = None
    body_previous = body_velocity = body_dt = body_tick = None
    physics_base = physics_base_velocity = physics_base_dt = None
    initial_body_base_set = False
    body_base_offset = np.zeros(3)
    initial_geometry_center = initial_geometry_base = None
    seed = np.array(cfg['ik']['initial_q'], dtype=float)
    previous = velocity = previous_base = base_velocity = previous_dt = previous_tick = None
    previous_goals = None
    warm_plan = None
    previous_filter_tick = None
    segment = 0
    rows, wall_start = [], time.monotonic()
    for number, sample in enumerate(selected):
        ticks = sample['ticks']
        row = {'index': number, 'timestamp_ticks': ticks, 'utc_ns': sample['utc_ns'], 'elapsed_s': (ticks - first_tick) * 1e-7,
               'execution_elapsed_s': (ticks - first_tick) * 1e-7 * time_scale,
               'valid': False, 'segment': segment, 'q': None, 'base_pose': None}
        try:
            data, origins = {}, {}
            for kind, stream in streams.items():
                data[kind], origins[kind] = interpolate(stream, ticks, cfg['input']['maximum_bracket_s'], discontinuities)
            errors = [float(np.linalg.norm(data['body'][i] - data[kind][1])) for i, kind in ((19, 'XRHandLeft'), (45, 'XRHandRight'))]
            row.update(source=origins, wrist_agreement_m=errors)
            if max(errors) > cfg['input']['wrist_agreement_m']:
                raise ValueError('native_xr_coordinate_disagreement')
            if alignment is None:
                alignment = WorldAlignment(data['body'], cfg['world'])
            points = {kind: alignment.points(value) for kind, value in data.items()}
            chest, chest_rotation = chest_frame(points['body'])
            goals, grippers = [], []
            for side, kind in (('left', 'XRHandLeft'), ('right', 'XRHandRight')):
                p, r, width = tcp_frame(points[kind], hand_corrections[side], hand_method)
                goals.append({'position': p, 'rotation': corrected_rotation(r,ticks,side), 'raw_rotation':r})
                grip = cfg['gripper']
                grippers.append({'human_pinch_m': width, 'open_fraction': float(np.clip((width - grip['closed_pinch_m']) / (grip['open_pinch_m'] - grip['closed_pinch_m']), 0, 1)), 'calibration_verified': False})
            row.update(tcp_targets=goals, grippers=grippers, human_points={kind: {str(k): v for k, v in value.items()} for kind, value in points.items()},
                       chest_target={'position': chest, 'rotation': chest_rotation})
            source_dt = None if previous_tick is None else (ticks - previous_tick) * 1e-7
            dt = None if source_dt is None else source_dt * time_scale
            if previous_tick is not None and (source_dt > cfg['base']['max_gap_s'] or any(previous_tick < d <= ticks for d in discontinuities)):
                raise ValueError('trajectory_time_discontinuity')
            if previous_goals is not None and any(rotation_error(a['rotation'], b['rotation']) / source_dt > cfg['input']['maximum_orientation_speed_rad_s'] for a, b in zip(previous_goals, goals)):
                raise ValueError('hand_orientation_jump')
            desired = np.r_[points['body'][1][:2], np.arctan2(chest_rotation[1, 0], chest_rotation[0, 0])]
            if allocation == 'geometry':
                placement_raw, placement_rotation_raw, offset_raw = adaptive_torso_target(points['body'], goals, model.arm_geometry)
                if initial_geometry_center is not None:
                    # WARP's desired base comes from the implied upper-body target,
                    # not pelvis translation alone. KUAVO has less torso freedom.
                    desired[:2] = placement_raw[:2] - initial_geometry_center[:2] + initial_geometry_base[:2]
            if previous_filter_tick is None:
                preferred = filter_base.pose.copy()
            else:
                interval = (ticks - previous_filter_tick) * 1e-7 * time_scale
                preferred = filter_base.update(desired, min(interval, filter_config['max_gap_s']))
            previous_filter_tick = ticks
            if torso_reference is None:
                torso_reference = TorsoReference(chest, chest_rotation, cfg['torso_reference'])
            if mode == 'lazy':
                held_chest, held_rotation, active = torso_reference.update(chest, chest_rotation)
            else:
                held_chest, held_rotation, active = chest, chest_rotation, True
            reference = {'position': held_chest, 'rotation': held_rotation, 'elbows': [points['body'][11], points['body'][16]], 'intent_active': active}
            fixed_body = None
            if allocation == 'geometry':
                placement, placement_rotation, offset = placement_raw, placement_rotation_raw, offset_raw
                if placement_filter is None:
                    placement_filter = PlacementFilter(placement, placement_rotation, cfg['placement'])
                physical_dt = None if body_tick is None else (ticks - body_tick) * 1e-7 * time_scale
                if physical_dt is not None:
                    placement, placement_rotation = placement_filter.update(placement, placement_rotation, min(physical_dt, filter_config['max_gap_s']))
                body_seed = seed.copy()
                if body_previous is not None:
                    body_seed[:4] = body_previous
                fixed_body, body_check = model.place_torso(placement, placement_rotation, preferred, body_seed,
                    body_previous, body_velocity, physical_dt, body_dt)
                if fixed_body is not None and not initial_body_base_set:
                    # No robot start state has been recorded. Establish its first
                    # placement from torso geometry, before any accepted trajectory.
                    # TCP/world targets stay unchanged; this is not a motion command.
                    probe = body_seed.copy()
                    probe[:4] = fixed_body
                    actual_center, _ = model.shoulder_pose(probe, preferred)
                    correction = placement[:2] - actual_center[:2]
                    body_base_offset[:2] = correction
                    preferred[:2] += correction
                    filter_base.pose = preferred.copy()
                    provenance['initial_geometry_base_offset_m'] = correction.tolist()
                    fixed_body, body_check = model.place_torso(placement, placement_rotation, preferred, probe)
                    initial_body_base_set = True
                    initial_geometry_center, initial_geometry_base = placement_raw.copy(), preferred.copy()
                row.update(adaptive_torso_offset_m=offset, torso_placement_target={'position': placement, 'rotation': placement_rotation}, torso_projection=body_check)
                if fixed_body is None:
                    raise ValueError(body_check['reason'])
                row['torso_q'] = fixed_body
                new_body_velocity = None if body_previous is None else (fixed_body - body_previous) / physical_dt
                body_previous, body_velocity, body_dt, body_tick = fixed_body.copy(), new_body_velocity, physical_dt, ticks
                seed[:4] = fixed_body
                reference.update(position=placement, rotation=placement_rotation)
            row['torso_reference_active'] = active
            candidates = [preferred]
            if allocation == 'geometry':
                retry_dt = None if physics_base is None else (ticks - physics_base_tick) * 1e-7 * time_scale
                retry = preferred if physics_base is None else retry_base(preferred, desired, physics_base, physics_base_velocity, retry_dt, physics_base_dt, cfg['base'])
            else:
                retry = retry_base(preferred, desired, previous_base, base_velocity, dt, previous_dt, cfg['base'])
            if not np.allclose(retry, preferred, atol=1e-6):
                candidates.append(retry)
            checks, solution, accepted_base = [], None, None
            seeds = [seed]
            if allocation == 'geometry' and cfg['ik'].get('arm_seed_candidates', 1) > 1:
                predicted = seed.copy()
                if previous is not None and velocity is not None:
                    predicted = np.clip(previous + velocity * dt, model.lower, model.upper)
                alternate = np.asarray(cfg['ik']['initial_q'], dtype=float).copy()
                for extra in (predicted, alternate):
                    extra[:4] = fixed_body
                    if not any(np.allclose(extra, old) for old in seeds):
                        seeds.append(extra)
                seeds = seeds[:cfg['ik']['arm_seed_candidates']]
            if previous is None and allocation == 'joint':
                for knee, leg, pitch in ((.4, -.8, .4), (.8, -1.5, .7)):
                    extra = seed.copy()
                    extra[:3] = [knee, leg, pitch]
                    seeds.append(extra)
            for candidate in candidates:
                if not base_valid(candidate, previous_base, base_velocity, dt, previous_dt, cfg['base']):
                    checks.append({'valid': False, 'reason': 'base_continuity_bounds'})
                    continue
                if allocation == 'geometry' and cfg['ik'].get('reuse_verified_plan', False) and not cfg['ik'].get('bounded_torso',False) and warm_plan and len(warm_plan) >= 3:
                    planned = np.asarray(warm_plan[0]).copy()
                    planned[:4] = fixed_body
                    started = time.monotonic()
                    reuse = model.validate(planned, candidate, goals, previous, velocity, dt, previous_dt)
                    reuse.update(solve_s=time.monotonic()-started, plan_reuse_attempt=True, planned_remaining_frames=len(warm_plan))
                    checks.append(reuse)
                    if reuse['valid']:
                        solution, accepted_base = planned, candidate
                        reuse.update(plan_reused=True,horizon_frames=1,horizon_plan_q=[planned.tolist()]+warm_plan[1:])
                        break
                horizon_cache = None
                for initial in seeds:
                    if allocation == 'geometry':
                        ref = dict(reference, elbows=model.elbow_targets(points['body'], goals, initial, candidate))
                        if horizon_cache is None:
                            horizon = [{'goals': goals, 'base': candidate, 'reference': ref, 'ticks': ticks, 'fixed_torso': fixed_body}]
                            hfilter, pfilter = copy.deepcopy(filter_base), copy.deepcopy(placement_filter)
                            hfilter.pose = candidate.copy()
                            hq, hv, hdt, htick = fixed_body.copy(), body_velocity, body_dt, ticks
                            future_seed = initial.copy()
                            for future in selected[number + 1:number + cfg['ik']['horizon_frames']]:
                                try:
                                    ft = future['ticks']
                                    interval = (ft-htick)*1e-7*time_scale
                                    if interval > filter_config['max_gap_s'] or any(htick < d <= ft for d in discontinuities):
                                        break
                                    fp = {kind: alignment.points(interpolate(stream, ft, cfg['input']['maximum_bracket_s'], discontinuities)[0]) for kind, stream in streams.items()}
                                    if max(np.linalg.norm(fp['body'][i]-fp[kind][1]) for i,kind in ((19,'XRHandLeft'),(45,'XRHandRight'))) > cfg['input']['wrist_agreement_m']:
                                        break
                                    fg = []
                                    for side, kind in (('left','XRHandLeft'),('right','XRHandRight')):
                                        pos, rot, _ = tcp_frame(fp[kind], hand_corrections[side], hand_method)
                                        fg.append({'position':pos,'rotation':corrected_rotation(rot,ft,side)})
                                    if any(rotation_error(a['rotation'],b['rotation'])/((ft-htick)*1e-7) > cfg['input']['maximum_orientation_speed_rad_s'] for a,b in zip(horizon[-1]['goals'],fg)):
                                        break
                                    pos,rot,_ = adaptive_torso_target(fp['body'],fg,model.arm_geometry)
                                    _, cr = chest_frame(fp['body'])
                                    desired_future = np.r_[pos[:2]-initial_geometry_center[:2]+initial_geometry_base[:2], np.arctan2(cr[1,0],cr[0,0])]
                                    fb = hfilter.update(desired_future,interval)
                                    pos,rot = pfilter.update(pos,rot,interval)
                                    future_seed[:4] = hq
                                    body, bc = model.place_torso(pos,rot,fb,future_seed,hq,hv,interval,hdt)
                                    if body is None:
                                        break
                                    future_seed[:4] = body
                                    fr = {'position':pos,'rotation':rot,'elbows':model.elbow_targets(fp['body'],fg,future_seed,fb),'intent_active':True}
                                    horizon.append({'goals':fg,'base':fb,'reference':fr,'ticks':ft,'fixed_torso':body})
                                    hv,hq,hdt,htick = (body-hq)/interval,body,interval,ft
                                except ValueError:
                                    break
                            horizon_cache = horizon
                        else:
                            horizon = list(horizon_cache)
                            horizon[0] = dict(horizon[0],reference=ref)
                        lengths = sorted(set([len(horizon), min(3,len(horizon)), min(2,len(horizon)), 1]), reverse=True)
                        for length in lengths:
                            solution, check = model.solve_horizon(horizon[:length], initial, previous, velocity, dt, previous_dt, mode, time_scale=time_scale, warm_plan=warm_plan if np.array_equal(initial,seeds[0]) else None)
                            check['seed_attempt'] = next(i for i,x in enumerate(seeds) if x is initial)
                            check['requested_horizon_frames'] = len(horizon)
                            checks.append(check)
                            if solution is not None:
                                break
                        if solution is not None:
                            accepted_base = candidate
                            break
                        continue
                    horizon = [{'goals': goals, 'base': candidate, 'reference': reference, 'ticks': ticks}]
                    horizon_base = copy.deepcopy(filter_base)
                    horizon_base.pose = candidate.copy()
                    horizon_torso = copy.deepcopy(torso_reference)
                    horizon_tick = ticks
                    for future in selected[number + 1:number + cfg['ik']['horizon_frames']]:
                        try:
                            future_points = {kind: alignment.points(interpolate(stream, future['ticks'], cfg['input']['maximum_bracket_s'], discontinuities)[0]) for kind, stream in streams.items()}
                            fp = future_points['body']
                            if max(np.linalg.norm(fp[i] - future_points[kind][1]) for i, kind in ((19, 'XRHandLeft'), (45, 'XRHandRight'))) > cfg['input']['wrist_agreement_m']:
                                break
                            fc, fr = chest_frame(fp)
                            future_goals = []
                            for side, kind in (('left', 'XRHandLeft'), ('right', 'XRHandRight')):
                                hp, hr, _ = tcp_frame(future_points[kind], hand_corrections[side], hand_method)
                                future_goals.append({'position': hp, 'rotation': corrected_rotation(hr,future['ticks'],side)})
                            if any(rotation_error(a['rotation'], b['rotation']) / ((future['ticks'] - horizon_tick) * 1e-7) > cfg['input']['maximum_orientation_speed_rad_s'] for a, b in zip(horizon[-1]['goals'], future_goals)):
                                break
                            next_base = horizon_base.update(np.r_[fp[1][:2], np.arctan2(fr[1, 0], fr[0, 0])], (future['ticks'] - horizon_tick) * 1e-7)
                            if mode == 'lazy':
                                fc, fr, _ = horizon_torso.update(fc, fr)
                            horizon.append({'goals': future_goals, 'base': next_base, 'reference': {'position': fc, 'rotation': fr, 'elbows': [fp[11], fp[16]], 'intent_active': active}, 'ticks': future['ticks']})
                            horizon_tick = future['ticks']
                        except ValueError:
                            break
                    if time_scale == 1:
                        solution, check = model.solve_horizon(horizon, initial, previous, velocity, dt, previous_dt, mode)
                    else:
                        solution, check = model.solve(goals, candidate, reference, initial, previous, velocity, dt, previous_dt, mode)
                    checks.append(check)
                    if solution is not None:
                        accepted_base = candidate
                        break
                if solution is not None:
                    break
            row['attempts'] = checks
            if solution is None:
                # Separate geometry-only diagnostic for the video; never a valid command.
                for initial in seeds:
                    diagnostic, diagnostic_check = model.solve(goals, preferred, reference, initial, mode=mode, fixed_torso=fixed_body)
                    if diagnostic is not None:
                        row.update(diagnostic_q=diagnostic, diagnostic_base_pose=preferred,
                                   diagnostic_verification=diagnostic_check, diagnostic_only=True)
                        seed = diagnostic.copy()
                        break
                raise ValueError('ik_failed')
            if previous is not None:
                new_velocity = (solution - previous) / dt
                delta_base = accepted_base - previous_base
                delta_base[2] = wrap(delta_base[2])
                base_velocity = delta_base / dt
            else:
                new_velocity, base_velocity = None, None
            row.update(valid=True, reason='verified_kinematic_frame', q=solution, base_pose=accepted_base,
                verification=checks[-1], first_frame_of_segment=previous is None, base_reachability_retry=not np.allclose(accepted_base, preferred))
            warm_plan = checks[-1].pop('horizon_plan_q', None)
            if warm_plan is not None:
                warm_plan = warm_plan[1:]
            for attempt in checks:
                attempt.pop('horizon_plan_q', None)
            if allocation == 'geometry':
                row['torso_anchor_q'] = row['torso_q']
                row['torso_q'] = solution[:4].copy()
            seed, previous = solution.copy(), solution.copy()
            velocity, previous_dt, previous_tick = new_velocity, dt, ticks
            previous_base, previous_goals = accepted_base.copy(), goals
            filter_base.pose = accepted_base.copy()
            if base_velocity is not None:
                filter_base.velocity = base_velocity.copy()
        except ValueError as exc:
            row['reason'] = str(exc)
            warm_plan = None
            # Never silently connect trajectories across rejected frames.
            previous = velocity = previous_base = base_velocity = previous_tick = previous_dt = previous_goals = None
            if allocation == 'joint':
                previous_filter_tick = None
                filter_base.velocity[:] = 0.
                torso_reference = None
            segment += 1
        if allocation == 'geometry' and alignment is not None and previous_filter_tick == ticks:
            current_base = filter_base.pose.copy()
            if physics_base is not None:
                interval = (ticks - physics_base_tick) * 1e-7 * time_scale
                delta = current_base - physics_base
                delta[2] = wrap(delta[2])
                physics_base_velocity = delta / interval
                physics_base_dt = interval
            physics_base, physics_base_tick = current_base, ticks
        rows.append(row)
        if number % 30 == 0:
            print(json.dumps({'frame': number, 'valid': sum(r['valid'] for r in rows), 'total': len(selected), 'wall_s': round(time.monotonic() - wall_start, 2)}), flush=True)
    with (staging / 'trajectory.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row, default=serial, allow_nan=False) + '\n')
    valid = [r for r in rows if r['valid']]
    contiguous = [[a, b] for a, b in zip(valid, valid[1:]) if a['segment'] == b['segment']]
    total_torso = sum(float(np.linalg.norm(np.asarray(b['q'])[:4] - np.asarray(a['q'])[:4], ord=1)) for a, b in contiguous)
    errors = [e for r in valid for e in r['verification']['tcp_errors']]
    durations = [a['solve_s'] for r in rows for a in r.get('attempts', []) if 'solve_s' in a]
    summary = {'frames': len(rows), 'valid_frames': len(valid), 'valid_fraction': len(valid) / max(1, len(rows)), 'failure_reasons': dict(Counter(r['reason'] for r in rows if not r['valid'])),
        'continuous_segments': len(set(r['segment'] for r in valid)), 'torso_total_variation_rad': total_torso,
        'maximum_tcp_position_error_m': max((e['position_m'] for e in errors), default=None),
        'maximum_tcp_orientation_error_rad': max((e['orientation_rad'] for e in errors), default=None),
        'solver_mean_s': float(np.mean(durations)) if durations else None, 'solver_p95_s': float(np.percentile(durations, 95)) if durations else None,
        'wall_s': time.monotonic() - wall_start, 'dataset_ready': False, 'trajectory_ready_for_robot': False,
        'clock_alignment': 'tracking timestamps only; camera alignment unverified', 'execution_time_scale': time_scale,
        'diagnostic_geometry_only_frames': sum(r.get('diagnostic_only', False) for r in rows),
        'native_body_basis_verified': False, 'hand_tcp_axis_calibration_verified': False,
        'gripper_calibration_verified': False, 'environment_collision_verified': False, 'dynamics_verified': False,
        'head_fixed': True, 'physical_motion': False, 'robot_commands_sent': 0, 'allocation': allocation}
    body_rows = [r for r in rows if r.get('torso_q') is not None]
    body_deltas = [np.asarray(b.get('torso_anchor_q',b['torso_q']))-a.get('torso_anchor_q',a['torso_q']) for a,b in zip(body_rows,body_rows[1:])]
    summary.update(hand_mapping=hand_method,
        plan_reused_frames=sum(r.get('verification',{}).get('plan_reused',False) for r in rows),
        certified_pruned_attempts=sum(a['reason'].startswith('certified_') for r in rows for a in r.get('attempts',[])),
        prescribed_torso_total_variation_all_rows_rad=sum(float(np.linalg.norm(d,ord=1)) for d in body_deltas),
        prescribed_torso_max_sample_step_rad=max((float(np.max(np.abs(d))) for d in body_deltas),default=None),
        prescribed_torso_samples=len(body_rows),
        torso_metric_note='All prescribed torso samples including invalid arm frames; sample gaps may span tracking dropouts.')
    dump(staging / 'quality.json', summary)
    if alignment is not None:
        provenance['world_alignment'] = alignment.metadata()
    provenance.update(state='complete', output_checksums={n: sha256(staging / n) for n in ('trajectory.jsonl', 'quality.json', 'config.json')})
    dump(staging / 'manifest.json', provenance)
    staging.rename(output)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode')
    parser.add_argument('output')
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--torso-mode', choices=('lazy', 'follow', 'free'), default='lazy')
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('--start-seconds', type=float, default=0.)
    parser.add_argument('--time-scale', type=float, default=1., help='Explicit execution time stretch >=1; source timestamps are unchanged')
    parser.add_argument('--allocation', choices=('geometry', 'joint'), default='geometry', help='Prescribe torso before arms; joint retains the previous baseline')
    parser.add_argument('--hand-mapping', choices=('mcp_pinch', 'legacy_pinch', 'palm_calibrated', 'index_palm', 'mcp_filtered'))
    args = parser.parse_args()
    if args.max_frames is not None and args.max_frames <= 0 or args.start_seconds < 0:
        parser.error('Frame count must be positive and start time nonnegative')
    print(json.dumps(convert(args.episode, args.output, args.urdf, args.config, args.torso_mode, args.max_frames, args.start_seconds, args.time_scale, args.allocation, args.hand_mapping), indent=2))


if __name__ == '__main__':
    main()
