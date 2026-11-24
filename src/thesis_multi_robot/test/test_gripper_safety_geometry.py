"""Checks on the experimental fixed CAD tool and its safety envelope."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

from thesis_multi_robot.geometry import sphere_clearance, world_capsules
from thesis_multi_robot.kinematics import RobotKinematics
from thesis_multi_robot.profiles import (
    GRIPPER_COLLISION_MESH, GRIPPER_VISUAL, build_robot, resource_path,
)
from thesis_multi_robot.supervisor import GEOMETRIC_MARGIN, classify_clearance


def test_coloured_cad_preserves_materials_and_stl_bounds():
    """The RViz/Gazebo visual matches the static CAD geometry in metres."""
    import collada
    visual_path = resource_path(GRIPPER_VISUAL)
    model = collada.Collada(visual_path)
    assert len(model.materials) > 10
    colors = {tuple(effect.diffuse) for effect in model.effects
              if isinstance(effect.diffuse, tuple)}
    assert len(colors) > 10
    visual = trimesh.load(visual_path, force='scene', process=False)
    collision = trimesh.load(resource_path(GRIPPER_COLLISION_MESH), force='mesh', process=False)
    np.testing.assert_allclose(visual.bounds, collision.bounds * .001, atol=1e-6)


@pytest.mark.parametrize('name', ('ur17g', 'p3020', 'cs625'))
def test_tool_capsule_covers_cad_and_stops_at_contact(tmp_path, name):
    """The generated tool envelope must enclose the actual exported STL."""
    model = build_robot(name, name, tmp_path, provisional_gripper=True)
    root = ET.parse(model['urdf']).getroot()
    link_name = name + '_provisional_gripper'
    link = root.find(f"link[@name='{link_name}']")
    assert link is not None
    cad = trimesh.load(resource_path(GRIPPER_COLLISION_MESH), force='mesh', process=False)
    vertices = np.asarray(cad.vertices) * 0.001
    capsules = [c for c in model['capsules'] if c['link'] == link_name]
    assert len(capsules) > 1
    covered = np.zeros(len(cad.faces), dtype=bool)
    for capsule in capsules:
        a, b = np.asarray(capsule['start']), np.asarray(capsule['end'])
        axis = b - a
        length2 = axis @ axis
        ratio = np.clip((vertices - a) @ axis / length2, 0., 1.) if length2 else 0.
        distance = np.linalg.norm(vertices - (a + np.asarray(ratio)[..., None] * axis), axis=1)
        covered |= (distance <= capsule['radius'] + 1e-9)[cad.faces].all(axis=1)
    assert covered.all()

    fk = RobotKinematics(root, model['joint_names'])
    frames = fk.forward(model['initial_positions'])
    world_tool = world_capsules(capsules, frames)[0]
    obstacle = (np.asarray(world_tool['start']) + world_tool['end']) / 2
    clearance = sphere_clearance(world_tool, obstacle, 0.01) - GEOMETRIC_MARGIN
    assert classify_clearance(clearance).state == 'STOP'
