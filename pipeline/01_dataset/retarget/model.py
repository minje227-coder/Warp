"""URDF-derived 18-DoF Drake IK and separate final FK/collision checks."""
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from pydrake.geometry import Box, CollisionFilterDeclaration, GeometrySet
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.solvers import SnoptSolver, SolverOptions
from pydrake.systems.framework import DiagramBuilder
from .geometry import rotation_error, yaw_rotation
from .input import sha256

TORSO = ['knee_joint', 'leg_joint', 'waist_pitch_joint', 'waist_yaw_joint']
ARMS = [f'zarm_{side}{i}_joint' for side in ('l', 'r') for i in range(1, 8)]
JOINT_NAMES = TORSO + ARMS
TCP_NAMES = ['zarm_l7_endeffector_center', 'zarm_r7_endeffector_center']
# Physical segment junctions represented by overlapping simplified URDF cylinders.
# All other non-welded pairs remain candidates; this is not a blanket arm exemption.
ADJACENT_COLLISIONS = [
    (f'zarm_{s}{a}_link', f'zarm_{s}{b}_link') for s in ('l', 'r') for a, b in ((2, 4), (4, 7))]


def prepare_urdf(source, destination):
    """Freeze wheels/head/gripper links at zero; retain all collision geometry.

    Draco's mesh reader needs OBJ. STL is converted without scaling/translation;
    Drake uses convex hulls for mesh distance queries. Never edit the source URDF.
    """
    import trimesh
    source, destination = Path(source).resolve(), Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source)
    root = tree.getroot()
    declared = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
    if not set(JOINT_NAMES) <= declared:
        raise ValueError('URDF missing required KUAVO joints')
    meshes, frozen, missing_collision = [], [], []
    for link in root.findall('link'):
        for visual in list(link.findall('visual')):
            link.remove(visual)
        if link.find('collision') is None:
            missing_collision.append(link.get('name'))
        for mesh in link.iter('mesh'):
            filename = mesh.get('filename')
            if filename.startswith('package://'):
                raise ValueError('Resolve ROS package mesh paths explicitly: ' + filename)
            path = (source.parent / filename).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            checksum = sha256(path)
            target = destination / (checksum + '.obj')
            if not target.exists():
                trimesh.load_mesh(str(path)).export(str(target), file_type='obj')
            mesh.set('filename', str(target))
            meshes.append({'source': str(path), 'sha256': checksum, 'converted_sha256': sha256(target)})
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed' and joint.get('name') not in JOINT_NAMES:
            frozen.append(joint.get('name'))
            joint.set('type', 'fixed')
            for child in list(joint):
                if child.tag not in ('origin', 'parent', 'child'):
                    joint.remove(child)
    path = destination / 'kuavo_s63_retarg.urdf'
    tree.write(path, encoding='unicode')
    return path, {'source_urdf': str(source), 'source_urdf_sha256': sha256(source),
        'derived_urdf_sha256': sha256(path), 'frozen_joints_at_zero': frozen, 'collision_meshes': meshes,
        'links_without_collision': missing_collision, 'collision_model': 'URDF primitives and mesh convex hulls; static gripper at zero',
        'gripper_swept_collision_verified': False, 'environment_obstacles_verified': False,
        'head_joint_values': {'zhead_1_joint': 0., 'zhead_2_joint': 0.}}


class KuavoModel:
    def __init__(self, urdf, cache, config):
        self.cfg = config
        derived, self.metadata = prepare_urdf(urdf, cache)
        builder = DiagramBuilder()
        self.plant, self.scene = AddMultibodyPlantSceneGraph(builder, time_step=0.)
        Parser(self.plant).AddModels(str(derived))
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName('base_link'))
        # Cylinder-HalfSpace signed-distance queries are unsupported by Drake.
        # A broad solid slab has the same z=0 surface throughout arm workspace.
        ground = self.plant.RegisterCollisionGeometry(self.plant.world_body(), RigidTransform([0., 0., -.05]), Box(100., 100., .1), 'ground', CoulombFriction(.7, .7))
        self.plant.Finalize()
        if self.plant.num_positions() != 18:
            raise ValueError('Expected exactly 18 body/arm positions')
        self.diagram = builder.Build()
        self.root_context = self.diagram.CreateDefaultContext()
        self.context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        self.scene_context = self.scene.GetMyMutableContextFromRoot(self.root_context)
        manager = self.scene.collision_filter_manager(self.scene_context)
        allowed = []
        for left, right in ADJACENT_COLLISIONS:
            a = self.plant.GetCollisionGeometriesForBody(self.plant.GetBodyByName(left))
            b = self.plant.GetCollisionGeometriesForBody(self.plant.GetBodyByName(right))
            if not a or not b:
                raise ValueError('Collision adjacency refers to unmodeled body')
            manager.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet(a), GeometrySet(b)))
            allowed.append([left, right])
        # Ground contact is intentional only for the base and wheels.
        for name in ('base_link', 'wheel_left_front', 'wheel_right_front', 'wheel_left_behind', 'wheel_right_behind'):
            ids = self.plant.GetCollisionGeometriesForBody(self.plant.GetBodyByName(name))
            manager.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet(ids), GeometrySet([ground])))
        self.metadata['allowed_structural_pairs'] = allowed
        self.indices = np.array([self.plant.GetJointByName(name).position_start() for name in JOINT_NAMES])
        self.frames = [self.plant.GetFrameByName(name) for name in TCP_NAMES]
        self.chest = self.plant.GetFrameByName('waist_yaw_link')
        self.shoulder_local = np.array([0., 0., .227])
        self.arm_geometry = {}
        for side, key in (('l', 'left'), ('r', 'right')):
            origins = {i: self.plant.GetJointByName(f'zarm_{side}{i}_joint').frame_on_parent().GetFixedPoseInBodyFrame().translation() for i in range(1, 8)}
            tcp = self.plant.CalcRelativeTransform(self.context, self.plant.GetFrameByName(f'zarm_{side}7_link'), self.frames[0 if side == 'l' else 1]).translation()
            self.arm_geometry[key] = {'shoulder_offset': (origins[1] - self.shoulder_local).tolist(),
                'upper_length_m': float(np.linalg.norm(origins[4])), 'lower_length_m': float(np.linalg.norm(sum(origins[i] for i in (5, 6, 7)))),
                'wrist_to_tcp': tcp.tolist(), 'nominal_geometry_only': True}
        self.metadata['adaptive_offset_geometry'] = self.arm_geometry
        self.elbows = [self.plant.GetFrameByName(f'zarm_{s}4_link') for s in ('l', 'r')]
        self.lower = self.plant.GetPositionLowerLimits()[self.indices] + config['joint_margin_rad']
        self.upper = self.plant.GetPositionUpperLimits()[self.indices] - config['joint_margin_rad']
        hardware = self.plant.GetVelocityUpperLimits()[self.indices]
        self.speed = hardware * config['velocity_scale']
        self.accel = np.array([config['torso_accel_rad_s2']] * 4 + [config['arm_accel_rad_s2']] * 14)
        self.solver = SnoptSolver()
        if not self.solver.available():
            raise RuntimeError('This Drake build has no SNOPT solver')
        self.metadata.update(joint_names=JOINT_NAMES, lower_limits=self.lower.tolist(), upper_limits=self.upper.tolist(),
            speed_limits_rad_s=self.speed.tolist(), acceleration_limits_rad_s2=self.accel.tolist(), solver='Drake SNOPT',
            dynamics_verified=False, collision_candidate_pairs=len(self.query().inspector().GetCollisionCandidates()))

    def set_q(self, q):
        internal = np.empty(18)
        internal[self.indices] = q
        self.plant.SetPositions(self.context, internal)

    def query(self):
        return self.scene.get_query_output_port().Eval(self.scene_context)

    def poses(self, q, base):
        self.set_q(q)
        rb = yaw_rotation(base[2])
        origin = np.r_[base[:2], 0.]
        result = []
        for frame in self.frames:
            pose = self.plant.CalcRelativeTransform(self.context, self.plant.world_frame(), frame)
            result.append((rb @ pose.translation() + origin, rb @ pose.rotation().matrix()))
        return result

    def shoulder_pose(self, q, base):
        self.set_q(q)
        pose = self.plant.CalcRelativeTransform(self.context, self.plant.world_frame(), self.chest)
        rb = yaw_rotation(base[2])
        return rb @ (pose.translation() + pose.rotation().matrix() @ self.shoulder_local) + np.r_[base[:2], 0.], rb @ pose.rotation().matrix()

    def validate(self, q, base, goals, previous=None, velocity=None, dt=None, velocity_dt=None):
        if not np.isfinite(q).all():
            return {'valid': False, 'reason': 'nonfinite_solution'}
        self.set_q(q)
        errors = [{'position_m': float(np.linalg.norm(p - goal['position'])), 'orientation_rad': rotation_error(r, goal['rotation'])}
                  for (p, r), goal in zip(self.poses(q, base), goals)]
        pairs = self.query().ComputeSignedDistancePairwiseClosestPoints(self.cfg['collision_margin_m'] + 1e-6)
        near = [float(p.distance) for p in pairs if p.distance < self.cfg['collision_margin_m'] - 1e-6]
        valid = not near and np.all(q >= self.lower - 1e-7) and np.all(q <= self.upper + 1e-7)
        valid = bool(valid and all(e['position_m'] <= self.cfg['tcp_position_tolerance_m'] + 1e-5 and e['orientation_rad'] <= self.cfg['tcp_orientation_tolerance_rad'] + 1e-5 for e in errors))
        speed, acceleration = None, None
        if previous is not None:
            speed = (q - previous) / dt
            valid = bool(valid and np.all(np.abs(speed) <= self.speed + 1e-5))
            if velocity is not None:
                acceleration = (speed - velocity) / ((dt + velocity_dt) / 2)
                valid = bool(valid and np.all(np.abs(acceleration) <= self.accel + 1e-4))
            # Check the linear joint interpolation between accepted samples too.
            # As in the existing GraspSystem, use <=0.02rad per check interval.
            subdivisions = max(1, int(np.ceil(np.max(np.abs(q - previous)) / self.cfg['collision_validation_step_rad'])))
            interframe = 0
            for fraction in np.linspace(0., 1., subdivisions + 1)[1:-1]:
                self.set_q(previous + fraction * (q - previous))
                interframe += sum(p.distance < self.cfg['collision_margin_m'] - 1e-6 for p in self.query().ComputeSignedDistancePairwiseClosestPoints(self.cfg['collision_margin_m'] + 1e-6))
            valid = bool(valid and interframe == 0)
            self.set_q(q)
        else:
            subdivisions, interframe = 0, 0
        return {'valid': valid, 'reason': 'verified' if valid else 'fk_limits_collision_or_continuity_failure', 'tcp_errors': errors,
            'collision_violations': len(near), 'interframe_collision_violations': int(interframe), 'interframe_subdivisions': subdivisions,
            'minimum_near_distance_m': min(near, default=None),
            'speed_rad_s': None if speed is None else speed.tolist(), 'acceleration_rad_s2': None if acceleration is None else acceleration.tolist()}

    def solve(self, goals, base, reference, seed, previous=None, velocity=None, dt=None, velocity_dt=None, mode='lazy', build_only=False, fixed_torso=None):
        start = time.monotonic()
        self.set_q(seed)
        ik = InverseKinematics(self.plant, self.context, with_joint_limits=True)
        prog, raw = ik.prog(), ik.q()
        q = raw[self.indices]
        lo, hi = self.lower.copy(), self.upper.copy()
        if previous is not None:
            lo = np.maximum(lo, previous - self.speed * dt)
            hi = np.minimum(hi, previous + self.speed * dt)
            if velocity is not None:
                step = (dt + velocity_dt) / 2
                lo = np.maximum(lo, previous + (velocity - self.accel * step) * dt)
                hi = np.minimum(hi, previous + (velocity + self.accel * step) * dt)
        if np.any(lo > hi):
            return None, {'valid': False, 'reason': 'empty_continuity_bounds', 'solve_s': time.monotonic() - start}
        prog.AddBoundingBoxConstraint(lo, hi, q)
        if fixed_torso is not None:
            prog.AddBoundingBoxConstraint(fixed_torso, fixed_torso, q[:4])
        rb = yaw_rotation(base[2])
        origin = np.r_[base[:2], 0.]
        tolerance = self.cfg['tcp_position_tolerance_m'] / np.sqrt(3)
        for frame, goal in zip(self.frames, goals):
            position = rb.T @ (goal['position'] - origin)
            rotation = rb.T @ goal['rotation']
            ik.AddPositionConstraint(frame, np.zeros(3), self.plant.world_frame(), position - tolerance, position + tolerance)
            ik.AddOrientationConstraint(self.plant.world_frame(), RotationMatrix(rotation), frame, RotationMatrix(), self.cfg['tcp_orientation_tolerance_rad'])
        ik.AddMinimumDistanceLowerBoundConstraint(self.cfg['collision_margin_m'], .02)
        position = rb.T @ (reference['position'] - origin)
        rotation = rb.T @ reference['rotation']
        ik.AddPositionCost(self.plant.world_frame(), position, self.chest, self.shoulder_local,
                           np.eye(3) * self.cfg['chest_position_weight'])
        ik.AddOrientationCost(self.plant.world_frame(), RotationMatrix(rotation), self.chest, RotationMatrix(), self.cfg['chest_orientation_weight'])
        for frame, elbow in zip(self.elbows, reference['elbows']):
            ik.AddPositionCost(self.plant.world_frame(), rb.T @ (elbow - origin), frame, np.zeros(3), np.eye(3) * self.cfg['elbow_weight'])
        # Normalize body motion by conservative rates: large-joint motion is costly.
        weights = np.ones(18) * self.cfg['arm_continuity_weight']
        if mode == 'lazy':
            weights[:4] = self.cfg['torso_intent_motion_weight'] if reference.get('intent_active', True) else self.cfg['torso_motion_weight']
            if previous is None:
                weights[:4] = self.cfg['torso_follow_motion_weight']
        elif mode == 'follow':
            weights[:4] = self.cfg['torso_follow_motion_weight']
        elif mode != 'free':
            raise ValueError('Unknown torso mode')
        else:
            weights[:4] = 0.
        prog.AddQuadraticErrorCost(np.diag(weights), seed, q)
        if velocity is not None:
            target = previous + velocity * dt
            prog.AddQuadraticErrorCost(np.eye(18) * self.cfg['acceleration_weight'], target, q)
        internal = np.empty(18)
        internal[self.indices] = np.clip(seed, lo, hi)
        prog.SetInitialGuess(raw, internal)
        if build_only:
            return prog, q
        options = SolverOptions()
        options.SetOption(self.solver.id(), 'Major iterations limit', self.cfg['max_iterations'])
        options.SetOption(self.solver.id(), 'Major feasibility tolerance', 1e-7)
        options.SetOption(self.solver.id(), 'Major optimality tolerance', 1e-4)
        result = self.solver.Solve(prog, None, options)
        if not result.is_success():
            return None, {'valid': False, 'reason': 'ik_infeasible_or_not_converged', 'solver_result': str(result.get_solution_result()), 'solve_s': time.monotonic() - start}
        solution = result.GetSolution(q)
        check = self.validate(solution, base, goals, previous, velocity, dt, velocity_dt)
        check['solve_s'] = time.monotonic() - start
        return (solution if check['valid'] else None), check

    def place_torso(self, position, rotation, base, seed, previous=None, velocity=None, dt=None, velocity_dt=None):
        """Project the prescribed human/adaptive pose onto the 4-DoF torso.

        Arms are frozen while establishing placement. EEF goals do not participate
        and cannot buy torso motion by trading off a soft chest weight.
        """
        start = time.monotonic()
        self.set_q(seed)
        ik = InverseKinematics(self.plant, self.context, with_joint_limits=True)
        q, prog = ik.q()[self.indices], ik.prog()
        lo, hi = self.lower[:4].copy(), self.upper[:4].copy()
        if previous is not None:
            lo = np.maximum(lo, previous - self.speed[:4] * dt)
            hi = np.minimum(hi, previous + self.speed[:4] * dt)
            # Retain enough distance to brake before mechanical position limits.
            a = self.accel[:4]
            vhi = np.sqrt((a * dt)**2 + 2 * a * np.maximum(0., self.upper[:4] - previous)) - a * dt
            vlo = -np.sqrt((a * dt)**2 + 2 * a * np.maximum(0., previous - self.lower[:4])) + a * dt
            lo = np.maximum(lo, previous + vlo * dt)
            hi = np.minimum(hi, previous + vhi * dt)
            if velocity is not None:
                step = (dt + velocity_dt) / 2
                lo = np.maximum(lo, previous + (velocity - self.accel[:4] * step) * dt)
                hi = np.minimum(hi, previous + (velocity + self.accel[:4] * step) * dt)
        if np.any(lo > hi):
            return None, {'valid': False, 'reason': 'body_empty_continuity_bounds'}
        prog.AddBoundingBoxConstraint(lo, hi, q[:4])
        prog.AddBoundingBoxConstraint(seed[4:], seed[4:], q[4:])
        from scipy.spatial.transform import Rotation
        rb = yaw_rotation(base[2])
        pitch, yaw, unavailable_roll = Rotation.from_matrix(rb.T @ rotation).as_euler('YZX')
        projected_rotation = Rotation.from_euler('YZ', [pitch, yaw]).as_matrix()
        # KUAVO torso is R_y(pitch) R_z(yaw); roll is not an independent DOF.
        # Preserve the represented human orientation as a constraint, not a cost
        # that can be sacrificed to improve EEF/centroid tracking.
        ik.AddOrientationConstraint(self.plant.world_frame(), RotationMatrix(projected_rotation), self.chest, RotationMatrix(), self.cfg['torso_orientation_tolerance_rad'])
        ik.AddPositionCost(self.plant.world_frame(), rb.T @ (position - np.r_[base[:2], 0.]), self.chest, self.shoulder_local, np.eye(3) * self.cfg['placement_position_weight'])
        ik.AddOrientationCost(self.plant.world_frame(), RotationMatrix(projected_rotation), self.chest, RotationMatrix(), self.cfg['placement_orientation_weight'])
        prog.AddQuadraticErrorCost(np.eye(4) * self.cfg['placement_continuity_weight'], seed[:4], q[:4])
        guess = seed.copy()
        guess[:4] = np.clip(guess[:4], lo, hi)
        internal = np.empty(18)
        internal[self.indices] = guess
        prog.SetInitialGuess(ik.q(), internal)
        options = SolverOptions()
        options.SetOption(self.solver.id(), 'Major iterations limit', self.cfg['max_iterations'])
        options.SetOption(self.solver.id(), 'Major feasibility tolerance', 1e-7)
        result = self.solver.Solve(prog, None, options)
        if not result.is_success():
            return None, {'valid': False, 'reason': 'body_projection_failed', 'solver_result': str(result.get_solution_result())}
        body = result.GetSolution(q[:4])
        actual_q = seed.copy()
        actual_q[:4] = body
        actual, actual_rotation = self.shoulder_pose(actual_q, base)
        represented_error = rotation_error(actual_rotation, rb @ projected_rotation)
        if represented_error > self.cfg['torso_orientation_tolerance_rad'] + 1e-5:
            return None, {'valid': False, 'reason': 'body_orientation_verification_failed'}
        return body, {'valid': True, 'reason': 'body_pose_projected', 'position_residual_m': float(np.linalg.norm(actual - position)),
            'orientation_residual_rad': rotation_error(actual_rotation, rotation), 'represented_orientation_error_rad': represented_error,
            'unrepresented_human_roll_rad': float(unavailable_roll), 'solve_s': time.monotonic() - start}

    def elbow_targets(self, body, goals, q, base):
        from .geometry import elbow_reference
        self.set_q(q)
        rb = yaw_rotation(base[2])
        result = []
        for side, key, goal in zip(('l', 'r'), ('left', 'right'), goals):
            pose = self.plant.CalcRelativeTransform(self.context, self.plant.world_frame(), self.plant.GetFrameByName(f'zarm_{side}1_link'))
            shoulder = rb @ pose.translation() + np.r_[base[:2], 0.]
            result.append(elbow_reference(body, shoulder, goal, self.arm_geometry[key], key))
        return result

    def solve_horizon(self, targets, seed, previous=None, velocity=None, dt=None, velocity_dt=None, mode='lazy', time_scale=1., warm_plan=None):
        """Receding-horizon IK anticipates braking before hitting joint boundaries.

        All horizon frames retain pose/collision/position constraints. Shared
        finite-difference bounds constrain velocity and acceleration; only the
        independently checked first frame is committed. No post-IK smoothing.
        """
        from pydrake.solvers import MathematicalProgram
        start = time.monotonic()
        if len(targets) < 2:
            t = targets[0]
            return self.solve(t['goals'], t['base'], t['reference'], seed, previous, velocity, dt, velocity_dt, mode, fixed_torso=t.get('fixed_torso'))
        combined = MathematicalProgram()
        variables = []
        for index, target in enumerate(targets):
            guess = seed.copy() if warm_plan is None or index >= len(warm_plan) else np.asarray(warm_plan[index]).copy()
            if target.get('fixed_torso') is not None:
                guess[:4] = target['fixed_torso']
            local, q = self.solve(target['goals'], target['base'], target['reference'], guess, mode=mode, build_only=True, fixed_torso=target.get('fixed_torso'))
            combined.AddDecisionVariables(local.decision_variables())
            for binding in local.GetAllConstraints():
                combined.AddConstraint(binding)
            for binding in local.GetAllCosts():
                combined.AddCost(binding)
            variables.append(q)
            combined.SetInitialGuess(q, guess)
        last_q, last_v, last_dt = previous, velocity, velocity_dt
        for index, q in enumerate(variables):
            interval = dt if index == 0 else (targets[index]['ticks'] - targets[index - 1]['ticks']) * 1e-7 * time_scale
            if last_q is not None:
                v = [(q[j] - last_q[j]) / interval for j in range(18)]
                for j in range(18):
                    combined.AddLinearConstraint(v[j], -self.speed[j], self.speed[j])
                if last_v is not None:
                    accel = [(v[j] - last_v[j]) / ((interval + last_dt) / 2) for j in range(18)]
                    for j in range(18):
                        combined.AddLinearConstraint(accel[j], -self.accel[j], self.accel[j])
                    combined.AddQuadraticCost(sum(a * a for a in accel) * self.cfg['horizon_acceleration_weight'])
                combined.AddQuadraticCost(sum(x * x for x in v) * self.cfg['horizon_velocity_weight'])
                last_v, last_dt = v, interval
            last_q = q
        options = SolverOptions()
        options.SetOption(self.solver.id(), 'Major iterations limit', self.cfg['max_iterations'])
        options.SetOption(self.solver.id(), 'Major feasibility tolerance', 1e-7)
        options.SetOption(self.solver.id(), 'Major optimality tolerance', 1e-4)
        result = self.solver.Solve(combined, None, options)
        if not result.is_success():
            return None, {'valid': False, 'reason': 'horizon_infeasible_or_not_converged', 'solver_result': str(result.get_solution_result()), 'horizon_frames': len(targets), 'solve_s': time.monotonic() - start}
        plan = [result.GetSolution(q) for q in variables]
        last_q, last_v, last_dt = previous, velocity, velocity_dt
        checks = []
        for index, (target, q) in enumerate(zip(targets, plan)):
            interval = dt if index == 0 else (target['ticks'] - targets[index - 1]['ticks']) * 1e-7 * time_scale
            item = self.validate(q, target['base'], target['goals'], last_q, last_v, interval, last_dt)
            if target.get('fixed_torso') is not None and np.max(np.abs(q[:4] - target['fixed_torso'])) > 1e-7:
                item.update(valid=False, reason='horizon_prescribed_torso_changed')
            checks.append(item)
            if not item['valid']:
                return None, dict(item, horizon_frames=len(targets), horizon_failed_index=index, solve_s=time.monotonic()-start)
            last_v = None if last_q is None else (q-last_q)/interval
            last_q, last_dt = q, interval
        solution, check = plan[0], checks[0]
        check.update(solve_s=time.monotonic() - start, horizon_frames=len(targets), horizon_plan_q=[q.tolist() for q in plan], horizon_all_frames_verified=True)
        return (solution if check['valid'] else None), check
