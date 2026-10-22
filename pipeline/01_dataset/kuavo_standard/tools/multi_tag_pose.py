"""Estimate a rigid tag bundle from all consistent visible tag corners.

Input corners use pupil-apriltags order (BL, BR, TR, TL). Tag centers and
coplanar tag faces are expressed in one object coordinate system. Per-tag
in-plane rotations map printed tag axes into that object coordinate system.
"""
import cv2
import numpy as np


def square(size):
    h = float(size) / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)


def _residuals(obj, uv, rv, tv, camera_matrix):
    projected = cv2.projectPoints(obj, rv, tv, camera_matrix, None)[0].reshape(-1, 2)
    return np.linalg.norm(projected - uv, axis=1)


def _world_pose(obj, rv, tv, camera_matrix, camera_rotation, camera_position, min_up):
    rotation = cv2.Rodrigues(rv)[0]
    translation = np.asarray(tv).reshape(3)
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        return None
    if np.any((obj @ rotation.T + translation)[:, 2] <= 0):
        return None
    if rotation[:, 2] @ (-translation) <= 0:
        return None
    world_rotation = camera_rotation @ rotation
    if world_rotation[2, 1] < min_up:
        return None
    return dict(rotation=world_rotation, position=camera_rotation @ translation + camera_position)


def estimate_tag_bundle(detections, tag_centers, tag_size, camera_matrix,
                        camera_rotation, camera_position, *, max_rms_px=2.0,
                        min_up=0.65, ransac_error_px=3.0,
                        ransac_iterations=200, ransac_confidence=0.999,
                        tag_inplane_rotations_rad=None):
    """Joint planar PnP + LM, with whole-tag rejection and explicit fallback.

    A joint result must explain at least two complete tags. A failed joint
    estimate may return the previous best-single estimate, marked as fallback;
    it is never reported as multi-tag evidence. Distortion is None because the
    existing pipeline supplies rectified image corners and calibrated K.
    """
    ids = sorted(set(detections) & set(tag_centers))
    if not ids:
        return None
    local = square(tag_size)
    angles = tag_inplane_rotations_rad or {}
    rotations = {tid: cv2.Rodrigues(np.array([0., 0., float(angles.get(tid, 0.))]))[0] for tid in ids}
    objects = np.array([local @ rotations[tid].T + tag_centers[tid] for tid in ids], dtype=float)
    images = np.array([np.asarray(detections[tid]['corners'], dtype=float)[[3, 2, 1, 0]] for tid in ids])
    if not np.isfinite(objects).all() or not np.isfinite(images).all():
        return None
    singles = []
    single_seeds = []
    for tid, obj, uv in zip(ids, objects, images):
        ok, rvs, tvs, _ = cv2.solvePnPGeneric(local, uv, camera_matrix, None,
                                             flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        candidates = []
        for rv, tv in zip(rvs, tvs):
            pose = _world_pose(local, rv, tv, camera_matrix, camera_rotation,
                               camera_position, -1.)
            if pose is None:
                continue
            pose["rotation"] = pose["rotation"] @ rotations[tid].T
            if pose["rotation"][2, 1] < min_up:
                continue
            rms = float(np.sqrt(np.mean(_residuals(local, uv, rv, tv, camera_matrix)**2)))
            pose['position'] = pose['position'] - pose['rotation'] @ tag_centers[tid]
            pose.update(rms=rms, tag_ids=[tid], corner_count=4,
                        per_tag_rms_px={str(tid): rms})
            candidates.append(pose)
            if rms <= max_rms_px:
                rotation = cv2.Rodrigues(rv)[0] @ rotations[tid].T
                object_rv = cv2.Rodrigues(rotation)[0]
                single_seeds.append((object_rv, np.asarray(tv).reshape(3) - rotation @ tag_centers[tid]))
        if candidates:
            best = min(candidates, key=lambda q: q['rms'])
            if best['rms'] <= max_rms_px:
                singles.append(best)
    if len(ids) == 1:
        if not singles:
            return None
        result = singles[0]
        result.update(method='single_tag', observed_tag_ids=ids, rejected_tag_ids=[])
        return result

    # Center the board plane at Z=0 for IPPE and numerical conditioning.
    origin = objects.mean(axis=(0, 1))
    objects = objects - origin
    obj = objects.reshape(-1, 3)
    uv = images.reshape(-1, 2)
    seeds = []
    ok, rvs, tvs, _ = cv2.solvePnPGeneric(obj, uv, camera_matrix, None,
                                         flags=cv2.SOLVEPNP_IPPE)
    if ok:
        seeds.extend(zip(rvs, tvs))
    # Single-tag poses are only initial guesses here; LM still fits all
    # consistent tags. These also rescue planar RANSAC/IPPE degeneracies.
    for rv, tv in single_seeds:
        seeds.append((rv, (tv + cv2.Rodrigues(rv)[0] @ origin).reshape(3, 1)))
    # RANSAC complements the all-corners solution when one tag is mistracked.
    cv2.setRNGSeed(0)
    ok, rv, tv, _ = cv2.solvePnPRansac(
        obj, uv, camera_matrix, None, flags=cv2.SOLVEPNP_EPNP,
        iterationsCount=ransac_iterations, reprojectionError=ransac_error_px,
        confidence=ransac_confidence)
    if ok:
        seeds.append((rv, tv))
    joint = []
    for rv, tv in seeds:
        rv, tv = rv.copy(), tv.copy()
        # First refine all corners: IPPE's approximate solution alone may not
        # meet the final pixel threshold even for a consistent board.
        errors = _residuals(obj, uv, rv, tv, camera_matrix).reshape(-1, 4)
        initial_keep = np.max(errors, axis=1) <= ransac_error_px
        if np.count_nonzero(initial_keep) < 2:
            initial_keep = np.ones(len(ids), dtype=bool)
        keep = initial_keep
        for _ in range(len(ids) + 1):
            if np.count_nonzero(keep) < 2:
                break
            rv, tv = cv2.solvePnPRefineLM(objects[keep].reshape(-1, 3),
                                         images[keep].reshape(-1, 2),
                                         camera_matrix, None, rv, tv)
            errors = _residuals(obj, uv, rv, tv, camera_matrix).reshape(-1, 4)
            tag_rms = np.sqrt(np.mean(errors**2, axis=1))
            new_keep = (tag_rms <= max_rms_px) & (np.max(errors, axis=1) <= ransac_error_px)
            if np.array_equal(keep, new_keep):
                break
            keep = new_keep
        if np.count_nonzero(keep) < 2:
            continue
        errors = _residuals(obj, uv, rv, tv, camera_matrix).reshape(-1, 4)
        tag_rms = np.sqrt(np.mean(errors**2, axis=1))
        # Never accept a set changed by the last refinement without checking it.
        if np.any(tag_rms[keep] > max_rms_px) or np.any(errors[keep] > ransac_error_px):
            continue
        pose = _world_pose(objects[keep].reshape(-1, 3), rv, tv, camera_matrix,
                           camera_rotation, camera_position, min_up)
        if pose is None:
            continue
        pose['position'] = pose['position'] - pose['rotation'] @ origin
        pose.update(method='joint_multi_tag', rms=float(np.sqrt(np.mean(errors[keep]**2))),
                    tag_ids=[tid for tid, good in zip(ids, keep) if good],
                    rejected_tag_ids=[tid for tid, good in zip(ids, keep) if not good],
                    observed_tag_ids=ids, corner_count=int(np.count_nonzero(keep))*4,
                    per_tag_rms_px={str(tid): float(err) for tid, err in zip(ids, tag_rms)})
        joint.append(pose)
    if joint:
        return min(joint, key=lambda q: (-len(q['tag_ids']), q['rms']))
    if not singles:
        return None
    result = min(singles, key=lambda q: q['rms'])
    result.update(method='single_tag_fallback', observed_tag_ids=ids,
                  rejected_tag_ids=[tid for tid in ids if tid not in result['tag_ids']],
                  joint_failure_reason='no_consistent_two_tag_solution_within_pixel_limits')
    return result
