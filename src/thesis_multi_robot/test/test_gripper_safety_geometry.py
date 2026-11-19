"""Checks on the experimental fixed CAD tool and its safety envelope."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

from thesis_multi_robot.geometry import sphere_clearance, world_capsules
from thesis_multi_robot.kinematics import RobotKinematics
from thesis_multi_robot.profiles import build_robot, resource_path
from thesis_multi_robot.supervisor import GEOMETRIC_MARGIN, classify_clearance


@pytest.mark.parametrize('name', ('ur17g', 'p3020', 'cs625'))
def test_tool_capsule_covers_cad_and_stops_at_contact(tmp_path, name):
    """The generated tool envelope must enclose the actual exported STL."""
    model = build_robot(name, name, tmp_path, provisional_gripper=True)
    root = ET.parse(model['urdf']).getroot()
    link_name = name + '_provisional_gripper'
    link = root.find(f"link[@name='{link_name}']")
    assert link is not None
    mesh = link.find('visual/geometry/mesh')
    vertices = np.asarray(trimesh.load(
        resource_path(mesh.get('filename')), force='mesh', process=False
    ).vertices) * np.asarray([float(v) for v in mesh.get('scale').split()])
    capsules = [c for c in model['capsules'] if c['link'] == link_name]
    assert len(capsules) == 1
    capsule = capsules[0]
    a, b = np.asarray(capsule['start']), np.asarray(capsule['end'])
    axis = b - a
    ratio = np.clip((vertices - a) @ axis / (axis @ axis), 0., 1.)
    distance = np.linalg.norm(vertices - (a + ratio[:, None] * axis), axis=1)
    assert np.max(distance) <= capsule['radius'] + 1e-9

    fk = RobotKinematics(root, model['joint_names'])
    frames = fk.forward(model['initial_positions'])
    world_tool = world_capsules(capsules, frames)[0]
    obstacle = (np.asarray(world_tool['start']) + world_tool['end']) / 2
    clearance = sphere_clearance(world_tool, obstacle, 0.01) - GEOMETRIC_MARGIN
    assert classify_clearance(clearance).state == 'STOP'
