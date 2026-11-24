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


def _mesh_data(mesh: ET.Element) -> tuple[np.ndarray, np.ndarray]:
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
    return vertices, np.asarray(loaded.faces, dtype=int)


def _mesh_vertices(mesh: ET.Element) -> np.ndarray:
    return _mesh_data(mesh)[0]


def _bound_vertices(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    axis = int(np.argmax(high - low))
    start = (low + high) * 0.5
    end = start.copy()
    start[axis], end[axis] = low[axis], high[axis]
    other = [index for index in range(3) if index != axis]
    radius = float(np.linalg.norm(vertices[:, other] - start[other], axis=1).max())
    return start, end, radius


def _split_mesh_bounds(mesh: ET.Element, count: int):
    """Bound whole triangles in axial groups, retaining conservative coverage."""
    vertices, faces = _mesh_data(mesh)
    axis = int(np.argmax(np.ptp(vertices, axis=0)))
    triangles = vertices[faces]
    centers = triangles[:, :, axis].mean(axis=1)
    edges = np.linspace(vertices[:, axis].min(), vertices[:, axis].max(), count + 1)
    groups = np.clip(np.searchsorted(edges, centers, side='right') - 1, 0, count - 1)
    return [_bound_vertices(triangles[groups == index].reshape(-1, 3))
            for index in range(count) if np.any(groups == index)]


def gripper_mesh_capsules(link_name: str, mesh: ET.Element) -> list[dict]:
    """Bound CAD triangles in small spatial groups in the gripper link frame.

    Each triangle belongs wholly to one convex capsule. The grouping preserves
    disconnected parts and gaps better than one capsule around the CAD box.
    The static mesh does not represent finger motion.
    """
    vertices, faces = _mesh_data(mesh)
    triangles = vertices[faces]
    centers = triangles.mean(axis=1)
    low, span = vertices.min(axis=0), np.ptp(vertices, axis=0)
    if np.any(span <= 0):
        raise ValueError('Gripper CAD must have nonzero extent in every axis')
    # Longer axial groups create continuous capsule bodies rather than a
    # succession of mostly spherical end caps, while keeping the two sides
    # separate where the static CAD has gaps.
    bins = np.array((2, 3, 2))
    cells = np.minimum(((centers - low) / span * bins).astype(int), bins - 1)
    result = []
    for cell in np.unique(cells, axis=0):
        group = triangles[np.all(cells == cell, axis=1)].reshape(-1, 3)
        start, end, radius = _bound_vertices(group)
        result.append({'link': link_name, 'start': start.tolist(), 'end': end.tolist(),
                       'radius': radius + GEOMETRY_MARGIN_M})
    return result


def collision_capsules(root: ET.Element, split_links: frozenset[str] = frozenset()) -> list[dict]:
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
                bounds = (_split_mesh_bounds(shape, 3) if link.get('name') in split_links
                          else [_bound_vertices(_mesh_vertices(shape))])
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
                bounds = [(start, end, radius)]
            elif shape.tag == 'cylinder':
                radius = float(shape.get('radius', 'nan'))
                length = float(shape.get('length', 'nan'))
                if not math.isfinite(length) or length <= 0.0:
                    raise ValueError('Collision cylinder length must be positive')
                start, end = np.array([0., 0., -length / 2]), np.array([0., 0., length / 2])
                bounds = [(start, end, radius)]
            elif shape.tag == 'sphere':
                radius = float(shape.get('radius', 'nan'))
                start = end = np.zeros(3)
                bounds = [(start, end, radius)]
            else:
                raise ValueError(f'Unsupported collision geometry: {shape.tag}')
            origin = _origin_transform(collision.find('origin'))
            for start, end, radius in bounds:
                if not math.isfinite(radius) or radius < 0.0:
                    raise ValueError('Collision radius must be finite and non-negative')
                result.append({
                    'link': link.get('name'),
                    'start': _point(origin, start),
                    'end': _point(origin, end),
                    'radius': radius + GEOMETRY_MARGIN_M,
                })
    return result


def link_mesh_capsules(root: ET.Element, sections: dict[str, int],
                       compact_ends: bool = False) -> list[dict]:
    """Enclose link collision meshes in axial groups of complete triangles.

    Triangles are transformed into the link frame before grouping. Each
    triangle belongs to one convex capsule, preserving coverage at boundaries.
    """
    result = []
    for link in root.findall('link'):
        name = link.get('name')
        if name not in sections:
            continue
        parts = []
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1 or geometry[0].tag != 'mesh':
                raise ValueError(f'Expected collision meshes on {name}')
            vertices, faces = _mesh_data(geometry[0])
            transform = _origin_transform(collision.find('origin'))
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            parts.append(vertices[faces])
        if not parts:
            raise ValueError(f'No collision meshes on {name}')
        triangles = np.concatenate(parts)
        vertices = triangles.reshape(-1, 3)
        axis = int(np.argmax(np.ptp(vertices, axis=0)))
        count = sections[name]
        if count < 1:
            raise ValueError(f'Invalid number of sections for {name}')
        if count == 3:
            low, high = vertices[:, axis].min(), vertices[:, axis].max()
            edges = low + (high - low) * np.array((0., .2, .8, 1.))
        else:
            edges = np.linspace(vertices[:, axis].min(), vertices[:, axis].max(), count + 1)
        centers = triangles[:, :, axis].mean(axis=1)
        groups = np.clip(np.searchsorted(edges, centers, side='right') - 1, 0, count - 1)
        for index in range(count):
            group = triangles[groups == index]
            if not len(group):
                continue
            points = group.reshape(-1, 3)
            if count == 3 and index in (0, 2):
                start = end = (points.min(axis=0) + points.max(axis=0)) * 0.5
                radius = float(np.linalg.norm(points - start, axis=1).max())
            else:
                start, end, radius = _bound_vertices(points)
                if compact_ends:
                    axis = int(np.argmax(np.abs(end - start)))
                    other = [i for i in range(3) if i != axis]
                    radial = np.linalg.norm(points[:, other] - start[other], axis=1)
                    reach = np.sqrt(np.maximum(0., (radius + GEOMETRY_MARGIN_M) ** 2 - radial ** 2))
                    low = float(np.min(points[:, axis] + reach))
                    high = float(np.max(points[:, axis] - reach))
                    if low > high:
                        low = high = (low + high) / 2.
                    start[axis], end[axis] = low, high
            result.append({'link': name, 'start': start.tolist(), 'end': end.tolist(),
                           'radius': radius + GEOMETRY_MARGIN_M})
    if {item['link'] for item in result} != set(sections):
        raise ValueError('Some selected collision links are missing')
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
