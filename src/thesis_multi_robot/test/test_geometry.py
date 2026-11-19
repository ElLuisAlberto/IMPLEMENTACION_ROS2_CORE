"""Coverage and distance contracts used by the simulation supervisor."""

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

from thesis_multi_robot.geometry import (
    GEOMETRY_MARGIN_M,
    capsule_clearance,
    capsule_sets_clearance,
    collision_capsules,
    sphere_clearance,
    world_capsules,
)


def _capsule(start, end, radius=0.):
    return {'link': 'link', 'start': start, 'end': end, 'radius': radius}


def test_distance_intersecting_parallel_and_degenerate_segments():
    horizontal = _capsule([0, 0, 0], [2, 0, 0], .1)
    crossing = _capsule([1, -1, 0], [1, 1, 0], .2)
    parallel = _capsule([0, 1, 0], [2, 1, 0], .2)
    point = _capsule([3, 0, 0], [3, 0, 0])
    assert capsule_clearance(horizontal, crossing) == pytest.approx(-.3)
    assert capsule_clearance(horizontal, parallel) == pytest.approx(.7)
    assert capsule_clearance(horizontal, point) == pytest.approx(.9)
    assert capsule_clearance(point, point) == pytest.approx(0.)


def test_distance_symmetry_and_sphere():
    a = _capsule([0, 0, 0], [1, 2, 3], .1)
    b = _capsule([3, 0, 1], [0, -1, 2], .4)
    assert capsule_clearance(a, b) == pytest.approx(capsule_clearance(b, a))
    assert sphere_clearance(a, [0, 0, 0], .2) == pytest.approx(-.3)


def test_distant_set_broad_phase_is_a_conservative_lower_bound():
    first = [_capsule([0, 0, 0], [1, 1, 0], .1),
             _capsule([-1, 0, 0], [-1, 2, 0], .2)]
    second = [_capsule([4, 3, 1], [5, 4, 2], .15)]
    exact = min(capsule_clearance(a, b) for a in first for b in second)
    bound = capsule_sets_clearance(first, second)
    assert .25 < bound <= exact
    assert bound == pytest.approx(capsule_sets_clearance(second, first))


@pytest.mark.parametrize('clearance', [-.1, 0., .05, .15, .25, .30])
def test_set_clearance_keeps_nearby_thresholds_and_overlap(clearance):
    first = [_capsule([0, 0, 0], [1, 0, 0], .1)]
    second = [_capsule([0, .2 + clearance, 0], [1, .2 + clearance, 0], .1)]
    assert capsule_sets_clearance(first, second) == pytest.approx(clearance)


def test_invalid_broad_phase_inputs_fail_closed():
    valid = [_capsule([0, 0, 0], [1, 0, 0], .1)]
    with pytest.raises(ValueError, match='empty'):
        capsule_sets_clearance([], valid)
    with pytest.raises(ValueError, match='finite'):
        capsule_sets_clearance(valid, [_capsule([math.nan, 0, 0], [0, 0, 0])])
    with pytest.raises(ValueError, match='cutoff'):
        capsule_sets_clearance(valid, valid, math.nan)


def test_vectorized_exact_pair_check_matches_scalar_including_points():
    random = np.random.default_rng(42)
    first = [_capsule(random.normal(size=3), random.normal(size=3), .1)
             for _ in range(13)]
    second = [_capsule(random.normal(size=3), random.normal(size=3), .2)
              for _ in range(22)]
    first.append(_capsule([1, 1, 1], [1, 1, 1], .1))
    second.append(_capsule([0, 0, 0], [0, 0, 0], .1))
    expected = min(capsule_clearance(a, b) for a in first for b in second)
    assert capsule_sets_clearance(first, second, cutoff=1e6) == pytest.approx(expected)
    for a in first:
        for b in second:
            assert capsule_sets_clearance([a], [b], cutoff=1e6) == pytest.approx(
                capsule_clearance(a, b), abs=1e-12)


def test_all_collisions_are_included_and_origins_respected():
    root = ET.fromstring('''<robot name="test"><link name="link">
      <collision><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
        <geometry><box size="4 2 1"/></geometry></collision>
      <collision><geometry><sphere radius="0.2"/></geometry></collision>
      <collision><geometry><cylinder radius="0.4" length="2"/></geometry></collision>
    </link></robot>''')
    capsules = collision_capsules(root)
    assert len(capsules) == 3
    assert capsules[0]['start'] == pytest.approx([1, 0, 3])
    assert capsules[0]['end'] == pytest.approx([1, 4, 3])
    assert capsules[0]['radius'] == pytest.approx(math.sqrt(1.25) + GEOMETRY_MARGIN_M)
    assert capsules[1]['radius'] == pytest.approx(.2 + GEOMETRY_MARGIN_M)
    assert capsules[2]['start'] == pytest.approx([0, 0, -1])


def test_mesh_vertices_are_enclosed_with_scale_and_collision_origin(tmp_path):
    mesh = trimesh.creation.box(extents=[1, 2, 4])
    path = tmp_path / 'box.stl'
    mesh.export(path)
    root = ET.fromstring(f'''<robot><link name="link"><collision>
      <origin xyz="1 0 0"/>
      <geometry><mesh filename="{path.as_uri()}" scale="2 1 0.5"/></geometry>
    </collision></link></robot>''')
    capsule = collision_capsules(root)[0]
    for vertex in np.asarray(mesh.vertices) * np.array([2, 1, .5]) + [1, 0, 0]:
        assert sphere_clearance(capsule, vertex, 0.) <= 0.


def test_world_transform_keeps_radius_and_identifies_missing_link():
    capsule = _capsule([0, 0, 0], [1, 0, 0], .2)
    transform = np.eye(4)
    transform[:3, 3] = [2, 3, 4]
    world = world_capsules([capsule], {'link': transform})[0]
    assert world['start'] == pytest.approx([2, 3, 4])
    assert world['end'] == pytest.approx([3, 3, 4])
    assert world['radius'] == .2
    with pytest.raises(KeyError):
        world_capsules([capsule], {})


def test_invalid_or_unresolved_geometry_fails_closed():
    root = ET.fromstring('''<robot><link name="link"><collision>
      <geometry><mesh filename="package://unknown/file.stl"/></geometry>
    </collision></link></robot>''')
    with pytest.raises(ValueError, match='absolute path'):
        collision_capsules(root)
    root = ET.fromstring('''<robot><link name="link"><collision>
      <geometry><box size="1 0 2"/></geometry>
    </collision></link></robot>''')
    with pytest.raises(ValueError, match='positive'):
        collision_capsules(root)
