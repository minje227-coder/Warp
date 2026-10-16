"""Measured fixed-rack JSON expanded into cuRobo cuboids."""
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


def _finite_tuple(values, size, label):
    result = tuple(float(value) for value in values)
    if len(result) != size or not np.isfinite(result).all():
        raise ValueError("%s must contain %d finite values" % (label, size))
    return result


def _normalize_quaternion(values, label):
    result = np.asarray(_finite_tuple(values, 4, label), dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if norm < 1e-9:
        raise ValueError(label + " must be nonzero")
    return tuple((result / norm).tolist())


@dataclass(frozen=True)
class RackPose:
    position_b: tuple
    quaternion_wxyz: tuple

    def __post_init__(self):
        object.__setattr__(self, "position_b", _finite_tuple(self.position_b, 3, "position_b"))
        object.__setattr__(self, "quaternion_wxyz", _normalize_quaternion(self.quaternion_wxyz, "quaternion_wxyz"))


@dataclass(frozen=True)
class ShelfSpec:
    index: int
    floor_front: float
    floor_back: float
    clear_height: float


@dataclass(frozen=True)
class RackGeometry:
    tag_id: int
    outer_width: float
    inner_width: float
    depth: float
    front_offset: float
    height: float
    tag_center_height: float
    shelf_thickness: float
    lip_height: float
    lip_depth: float
    axes: dict
    shelves: tuple

    @property
    def tag_to_rack_basis(self):
        return np.column_stack([
            self.axes["x_inside_in_tag"],
            self.axes["y_left_in_tag"],
            self.axes["z_up_in_tag"],
        ])


def _positive(value, label):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(label + " must be finite and positive")
    return value


def load_rack_geometry(path):
    with Path(path).open("r") as stream:
        raw = json.load(stream)
    if not isinstance(raw, dict):
        raise ValueError("rack geometry must be a mapping")
    tag_id = raw.get("tag_id")
    if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
        raise ValueError("tag_id must be a nonnegative integer")
    outer = _positive(raw.get("outer_width"), "outer_width")
    inner = _positive(raw.get("inner_width"), "inner_width")
    if inner >= outer:
        raise ValueError("inner_width must be smaller than outer_width")
    front_offset = float(raw.get("front_offset"))
    if not math.isfinite(front_offset):
        raise ValueError("front_offset must be finite")
    axes_raw = raw.get("tag_axes_to_rack")
    names = ("x_inside_in_tag", "y_left_in_tag", "z_up_in_tag")
    if not isinstance(axes_raw, dict) or set(axes_raw) != set(names):
        raise ValueError("tag_axes_to_rack must define the three rack axes")
    axes = {name: _finite_tuple(axes_raw[name], 3, name) for name in names}
    basis = np.column_stack([axes[name] for name in names])
    if not np.allclose(basis.T @ basis, np.eye(3), atol=1e-6) or np.linalg.det(basis) <= 0:
        raise ValueError("tag_axes_to_rack must be right-handed orthonormal")
    shelves_raw = raw.get("shelves")
    if not isinstance(shelves_raw, dict) or not shelves_raw:
        raise ValueError("shelves must be a nonempty mapping")
    shelves = []
    for key in sorted(shelves_raw, key=lambda value: int(value)):
        index = int(key)
        if index <= 0 or str(index) != str(key):
            raise ValueError("shelf names must be positive integer strings")
        item = shelves_raw[key]
        front = _positive(item.get("floor_front"), "floor_front")
        back = _positive(item.get("floor_back"), "floor_back")
        if back <= front:
            raise ValueError("floor_back must exceed floor_front")
        shelves.append(ShelfSpec(index, front, back, _positive(item.get("clear_height"), "clear_height")))
    return RackGeometry(
        tag_id, outer, inner,
        _positive(raw.get("depth"), "depth"), front_offset,
        _positive(raw.get("height"), "height"),
        _positive(raw.get("tag_center_height"), "tag_center_height"),
        _positive(raw.get("shelf_thickness"), "shelf_thickness"),
        _positive(raw.get("lip_height"), "lip_height"),
        _positive(raw.get("lip_depth"), "lip_depth"),
        axes, tuple(shelves),
    )


def rotation_matrix_wxyz(quaternion):
    w, x, y, z = _normalize_quaternion(quaternion, "quaternion")
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def quaternion_from_matrix_wxyz(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation matrix must be finite 3x3")
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = [0.25*scale, (matrix[2,1]-matrix[1,2])/scale,
                  (matrix[0,2]-matrix[2,0])/scale, (matrix[1,0]-matrix[0,1])/scale]
    elif matrix[0,0] > matrix[1,1] and matrix[0,0] > matrix[2,2]:
        scale = math.sqrt(1 + matrix[0,0] - matrix[1,1] - matrix[2,2]) * 2
        result = [(matrix[2,1]-matrix[1,2])/scale, 0.25*scale,
                  (matrix[0,1]+matrix[1,0])/scale, (matrix[0,2]+matrix[2,0])/scale]
    elif matrix[1,1] > matrix[2,2]:
        scale = math.sqrt(1 + matrix[1,1] - matrix[0,0] - matrix[2,2]) * 2
        result = [(matrix[0,2]-matrix[2,0])/scale, (matrix[0,1]+matrix[1,0])/scale,
                  0.25*scale, (matrix[1,2]+matrix[2,1])/scale]
    else:
        scale = math.sqrt(1 + matrix[2,2] - matrix[0,0] - matrix[1,1]) * 2
        result = [(matrix[1,0]-matrix[0,1])/scale, (matrix[0,2]+matrix[2,0])/scale,
                  (matrix[1,2]+matrix[2,1])/scale, 0.25*scale]
    return _normalize_quaternion(result, "rotation quaternion")


def _rotation_y(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]], dtype=float)


def _cuboid(geometry, pose, center_rack, dims, rotation_rack):
    tag_to_base = rotation_matrix_wxyz(pose.quaternion_wxyz)
    rack_to_base = tag_to_base @ geometry.tag_to_rack_basis
    center = np.asarray(pose.position_b) + rack_to_base @ np.asarray(center_rack)
    rotation = rack_to_base @ np.asarray(rotation_rack)
    return {"pose": list(center) + list(quaternion_from_matrix_wxyz(rotation)), "dims": list(dims)}


def build_rack_cuboids(geometry, pose):
    x_center = geometry.front_offset + geometry.depth / 2.0
    pillar = (geometry.outer_width - geometry.inner_width) / 2.0
    identity = np.eye(3)
    result = {
        "rack_pillar_left": _cuboid(geometry, pose,
            (x_center, geometry.inner_width/2 + pillar/2, geometry.height/2 - geometry.tag_center_height),
            (geometry.depth, pillar, geometry.height), identity),
        "rack_pillar_right": _cuboid(geometry, pose,
            (x_center, -geometry.inner_width/2 - pillar/2, geometry.height/2 - geometry.tag_center_height),
            (geometry.depth, pillar, geometry.height), identity),
        "rack_top_beam": _cuboid(geometry, pose,
            (x_center, 0, geometry.height - geometry.shelf_thickness/2 - geometry.tag_center_height),
            (geometry.depth, geometry.outer_width, geometry.shelf_thickness), identity),
    }
    for shelf in geometry.shelves:
        rise = shelf.floor_back - shelf.floor_front
        rotation = _rotation_y(-math.atan2(rise, geometry.depth))
        center = np.array([x_center, 0, (shelf.floor_front + shelf.floor_back)/2], dtype=float)
        center -= rotation[:, 2] * geometry.shelf_thickness / 2
        center[2] -= geometry.tag_center_height
        length = math.hypot(geometry.depth, rise)
        result["rack_shelf_%d" % shelf.index] = _cuboid(
            geometry, pose, center, (length, geometry.inner_width, geometry.shelf_thickness), rotation)
        result["rack_lip_%d" % shelf.index] = _cuboid(
            geometry, pose,
            (geometry.front_offset + geometry.lip_depth/2, 0,
             shelf.floor_front + geometry.lip_height/2 - geometry.tag_center_height),
            (geometry.lip_depth, geometry.inner_width, geometry.lip_height), identity)
    return result
