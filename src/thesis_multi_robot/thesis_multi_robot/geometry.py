"""Conservative capsule coverage of every URDF collision element.

Meshes are enclosed in a capsule around their longest AABB axis; checking all
vertices encloses the mesh faces too, since a capsule is convex.  The resulting
geometry is intentionally conservative and serves the experimental simulation
supervisor, not a manufacturer's certified safety model.
"""

from __future__ import annotations

import math
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


GEOMETRY_MARGIN_M = 0.005


def _numbers(value: str, count: int) -> np.ndarray:
    result = np.asarray([float(number) for number in value.split()], dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f'Expected {count} finite geometry values: {value!r}')
    return result


def _origin_transform(origin: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, 3] = _numbers(origin.get('xyz', '0 0 0'), 3)
    roll, pitch, yaw = _numbers(origin.get('rpy', '0 0 0'), 3)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    transform[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    return transform


def _point(transform: np.ndarray, value) -> list[float]:
    return (transform[:3, :3] @ np.asarray(value) + transform[:3, 3]).tolist()


def _mesh_vertices(mesh: ET.Element) -> np.ndarray:
    filename = mesh.get('filename', '')
    if filename.startswith('file://'):
        uri = urlparse(filename)
        if uri.netloc not in ('', 'localhost'):
            raise ValueError(f'Non-local collision mesh: {filename}')
        filename = unquote(uri.path)
    if not Path(filename).is_absolute():
        raise ValueError(f'Collision mesh must resolve to an absolute path: {filename}')
    loaded = trimesh.load(filename, force='mesh', process=False)
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f'Collision resource did not contain a mesh: {filename}')
    vertices = np.asarray(loaded.vertices, dtype=float)
    scale = _numbers(mesh.get('scale', '1 1 1'), 3)
    if np.any(scale == 0.0):
        raise ValueError(f'Collision mesh has zero scale: {filename}')
    vertices = vertices * scale
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
            or not np.isfinite(vertices).all()):
        raise ValueError(f'Collision mesh has no finite vertices: {filename}')
    return vertices


def _bound_vertices(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    axis = int(np.argmax(high - low))
    start = (low + high) * 0.5
    end = start.copy()
    start[axis], end[axis] = low[axis], high[axis]
    other = [index for index in range(3) if index != axis]
    radius = float(np.linalg.norm(vertices[:, other] - start[other], axis=1).max())
    return start, end, radius


def collision_capsules(root: ET.Element) -> list[dict]:
    """Enclose every link collision, expressed in that link's own frame.

    Package mesh URIs must be resolved by the builder before calling this
    function.  Invalid or unsupported geometry raises rather than silently
    dropping part of the robot's collision envelope.
    """
    result = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError(f'Invalid collision geometry on {link.get("name")}')
            shape = geometry[0]
            if shape.tag == 'mesh':
                start, end, radius = _bound_vertices(_mesh_vertices(shape))
            elif shape.tag == 'box':
                size = _numbers(shape.get('size', ''), 3)
                if np.any(size <= 0.0):
                    raise ValueError('Collision box dimensions must be positive')
                vertices = np.array([
                    [x, y, z] for x in (-size[0] / 2, size[0] / 2)
                    for y in (-size[1] / 2, size[1] / 2)
                    for z in (-size[2] / 2, size[2] / 2)
                ])
                start, end, radius = _bound_vertices(vertices)
            elif shape.tag == 'cylinder':
                radius = float(shape.get('radius', 'nan'))
                length = float(shape.get('length', 'nan'))
                if not math.isfinite(length) or length <= 0.0:
                    raise ValueError('Collision cylinder length must be positive')
                start, end = np.array([0., 0., -length / 2]), np.array([0., 0., length / 2])
            elif shape.tag == 'sphere':
                radius = float(shape.get('radius', 'nan'))
                start = end = np.zeros(3)
            else:
                raise ValueError(f'Unsupported collision geometry: {shape.tag}')
            if not math.isfinite(radius) or radius < 0.0:
                raise ValueError('Collision radius must be finite and non-negative')
            origin = _origin_transform(collision.find('origin'))
            result.append({
                'link': link.get('name'),
                'start': _point(origin, start),
                'end': _point(origin, end),
                'radius': radius + GEOMETRY_MARGIN_M,
            })
    return result


def world_capsules(capsules: list[dict], transforms: dict) -> list[dict]:
    """Transform each local capsule with the corresponding world-from-link TF."""
    result = []
    for capsule in capsules:
        transform = np.asarray(transforms[capsule['link']], dtype=float)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError(f'Invalid transform for {capsule["link"]}')
        result.append({
            **capsule,
            'start': _point(transform, capsule['start']),
            'end': _point(transform, capsule['end']),
        })
    return result


def _segment_distance(p1, q1, p2, q2) -> float:
    """Shortest distance of closed segments, including points and parallels."""
    p1, q1, p2, q2 = (np.asarray(point, dtype=float) for point in (p1, q1, p2, q2))
    if any(point.shape != (3,) or not np.isfinite(point).all()
           for point in (p1, q1, p2, q2)):
        raise ValueError('Segment endpoints must be finite 3-vectors')
    d1, d2, relative = q1 - p1, q2 - p2, p1 - p2
    a, e = float(d1 @ d1), float(d2 @ d2)
    epsilon = 1e-20
    if a <= epsilon and e <= epsilon:
        return float(np.linalg.norm(p1 - p2))
    f = float(d2 @ relative)
    if a <= epsilon:
        s, t = 0.0, float(np.clip(f / e, 0., 1.))
    else:
        c = float(d1 @ relative)
        if e <= epsilon:
            s, t = float(np.clip(-c / a, 0., 1.)), 0.0
        else:
            b = float(d1 @ d2)
            denominator = a * e - b * b
            s = float(np.clip((b * f - c * e) / denominator, 0., 1.)) \
                if denominator > epsilon * max(1.0, a * e) else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, float(np.clip(-c / a, 0., 1.))
            elif t > 1.0:
                t, s = 1.0, float(np.clip((b - c) / a, 0., 1.))
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def capsule_clearance(a: dict, b: dict) -> float:
    """Signed capsule separation; negative means overlapping envelopes."""
    return (_segment_distance(a['start'], a['end'], b['start'], b['end'])
            - float(a['radius']) - float(b['radius']))


def capsule_sets_clearance(first: list[dict], second: list[dict], cutoff=.25) -> float:
    """Return exact nearby clearance or a conservative distant lower bound.

    Each set's AABB encloses its full capsules, including spherical end caps.
    The distance between disjoint AABBs cannot exceed the actual set distance.
    Once that lower bound exceeds ``cutoff``, exact pair testing cannot change
    any proximity classification below the cutoff and is skipped.  Reported
    distant values are lower bounds, not geometric witness distances.

    The default .25 m cutoff accounts for the supervisor's .05 m additional
    margin and its .20 m ALLOW threshold.  Overlapping AABBs always use exact
    signed pair distances so penetration is never hidden by a zero box gap.
    """
    if not math.isfinite(cutoff) or cutoff < 0.0:
        raise ValueError('Broad-phase cutoff must be finite and non-negative')

    def bounds(capsules):
        if not capsules:
            raise ValueError('Cannot evaluate an empty collision envelope')
        starts = np.asarray([capsule['start'] for capsule in capsules], dtype=float)
        ends = np.asarray([capsule['end'] for capsule in capsules], dtype=float)
        radii = np.asarray([capsule['radius'] for capsule in capsules], dtype=float)
        if (starts.shape != (len(capsules), 3) or ends.shape != starts.shape
                or not np.isfinite(starts).all() or not np.isfinite(ends).all()
                or not np.isfinite(radii).all() or np.any(radii < 0.0)):
            raise ValueError('Capsule set must contain finite endpoints and radii')
        return (np.min(np.minimum(starts, ends) - radii[:, None], axis=0),
                np.max(np.maximum(starts, ends) + radii[:, None], axis=0),
                starts, ends, radii)

    low_a, high_a, p1, q1, radius_a = bounds(first)
    low_b, high_b, p2, q2, radius_b = bounds(second)
    gap = np.maximum(np.maximum(low_a - high_b, low_b - high_a), 0.0)
    lower_bound = float(np.linalg.norm(gap))
    if lower_bound > cutoff:
        return lower_bound
    # Vectorized equivalent of _segment_distance for every pair.  Nearby
    # robots need exact checks without blocking the ROS executor for hundreds
    # of milliseconds as nested Python/Numpy scalar calls would do.
    p1, q1 = p1[:, None, :], q1[:, None, :]
    p2, q2 = p2[None, :, :], q2[None, :, :]
    d1, d2, relative = q1 - p1, q2 - p2, p1 - p2
    a, e = np.sum(d1 * d1, axis=2), np.sum(d2 * d2, axis=2)
    b = np.sum(d1 * d2, axis=2)
    c, f = np.sum(d1 * relative, axis=2), np.sum(d2 * relative, axis=2)
    epsilon = 1e-20
    denominator = a * e - b * b
    s = np.clip(np.divide(
        b * f - c * e, denominator, out=np.zeros_like(b),
        where=denominator > epsilon * np.maximum(1., a * e)), 0., 1.)
    t = np.divide(b * s + f, e, out=np.zeros_like(b), where=e > epsilon)
    low_s = np.clip(np.divide(-c, a, out=np.zeros_like(b), where=a > epsilon), 0., 1.)
    high_s = np.clip(np.divide(b - c, a, out=np.zeros_like(b), where=a > epsilon), 0., 1.)
    s = np.where(t < 0., low_s, np.where(t > 1., high_s, s))
    t = np.clip(t, 0., 1.)
    # Point/segment, segment/point and point/point pairs.
    s = np.where(a <= epsilon, 0., np.where(e <= epsilon, low_s, s))
    point_t = np.clip(np.divide(f, e, out=np.zeros_like(b), where=e > epsilon), 0., 1.)
    t = np.where(a <= epsilon, point_t, np.where(e <= epsilon, 0., t))
    distances = np.linalg.norm(
        (p1 + d1 * s[:, :, None]) - (p2 + d2 * t[:, :, None]), axis=2)
    clearances = distances - radius_a[:, None] - radius_b[None, :]
    return float(clearances.min())


def sphere_clearance(capsule: dict, center, radius: float) -> float:
    """Signed capsule-to-sphere separation in a common coordinate frame."""
    return (_segment_distance(capsule['start'], capsule['end'], center, center)
            - float(capsule['radius']) - float(radius))
