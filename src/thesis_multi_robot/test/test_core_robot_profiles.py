"""Offline contracts between core profiles and the three active models."""

import math
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory

import numpy as np
import pytest

from thesis_core.joint_model import JOINT_VELOCITY_LIMITS
from thesis_core.robot_profile import load_robot_profile, load_robot_profiles
from thesis_multi_robot.profiles import build_cell


@pytest.fixture(scope='module')
def profiles(tmp_path_factory):
    """Build and load all official profiles without starting ROS nodes."""
    manifest = build_cell(tmp_path_factory.mktemp('core_profile_cell'))
    return load_robot_profiles(manifest)


@pytest.mark.parametrize(
    ('name', 'expected_dof'),
    [('jaco2', 6), ('ur17g', 6), ('p3020', 5), ('cs625', 6)],
)
def test_official_profile_is_ready_for_generic_safety(
    profiles,
    name,
    expected_dof,
):
    profile = profiles[name]
    assert profile.name == name
    assert profile.profile_id == name
    assert profile.joint_count == expected_dof
    assert len(profile.joint_position_limits) == expected_dof
    assert len(profile.joint_velocity_limits) == expected_dof
    assert profile.world_frame == 'world'
    assert profile.controller_action == (
        f'/{name}/arm_controller/follow_joint_trajectory'
    )
    assert profile.joint_states_topic == f'/{name}/joint_states'
    assert profile.controller_config.is_file()

    poses = profile.forward(profile.initial_positions)
    assert profile.base_link in poses
    assert profile.tip_link in poses
    segments = profile.capsule_segments(profile.initial_positions)
    assert len(segments) == len(profile.collision_capsules)
    assert all(segment.radius > 0.0 for segment in segments)
    assert all(np.isfinite(segment.start).all() for segment in segments)
    assert all(np.isfinite(segment.end).all() for segment in segments)


@pytest.mark.parametrize(('name', 'positions'), [
    ('ur17g', (0., -1.57, 0., -1.57, 0., 0.)),
    ('ur17g', (0.7, -0.8, 1.0, -1.2, 0.5, -0.4)),
    ('ur17g', (-0.6, -2.0, -0.8, 0.3, -1.0, 0.9)),
    ('p3020', (0., 0., 0., 0., 0.)),
    ('p3020', (0.3, -0.4, 0.5, 0.2, -0.3)),
    ('cs625', (0., -1.57, 0., -1.57, 0., 0.)),
    ('cs625', (0.4, -0.9, 0.7, -1.2, 0.3, -0.2)),
])
def test_capsules_cover_collision_meshes_and_tool(profiles, name, positions):
    """The actual monitor geometry follows every collision in different poses."""
    from thesis_multi_robot.geometry import _mesh_data, _origin_transform
    from thesis_multi_robot.profiles import GRIPPER_COLLISION_MESH, resource_path

    profile = profiles[name]
    poses = profile.forward(positions)
    segments = profile.capsule_segments(positions)
    collisions = [(link, collision) for link in profile.urdf_root.findall('link')
                  for collision in link.findall('collision')]
    assert any(link.get('name') == name + '_provisional_gripper'
               for link, _ in collisions)
    for link, collision in collisions:
        link_name = link.get('name')
        link_segments = [segment for capsule, segment in zip(profile.capsules, segments)
                         if capsule.link == link_name]
        assert link_segments
        shape = (link.find('visual/geometry/mesh')
                 if link_name == name + '_provisional_gripper'
                 else collision.find('geometry')[0])
        if shape.tag == 'mesh':
            import copy
            mesh = copy.deepcopy(shape)
            if link_name == name + '_provisional_gripper':
                mesh.set('filename', resource_path(GRIPPER_COLLISION_MESH))
                mesh.set('scale', '0.001 0.001 0.001')
            else:
                mesh.set('filename', resource_path(mesh.get('filename')))
            vertices, faces = _mesh_data(mesh)
        elif shape.tag == 'box':
            size = np.array([float(v) for v in shape.get('size').split()])
            vertices = np.array([[x, y, z] for x in (-size[0]/2, size[0]/2)
                                 for y in (-size[1]/2, size[1]/2)
                                 for z in (-size[2]/2, size[2]/2)])
            faces = np.arange(len(vertices)).reshape(1, -1)
        else:
            pytest.fail(f'unexpected collision shape: {shape.tag}')
        origin = (link.find('visual/origin') if link_name == name + '_provisional_gripper'
                  else collision.find('origin'))
        transform = poses[link_name] @ _origin_transform(origin)
        vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        face_covered = np.zeros(len(faces), dtype=bool)
        for segment in link_segments:
            start, end = np.array(segment.start), np.array(segment.end)
            axis = end - start
            length2 = axis @ axis
            ratio = np.clip((vertices - start) @ axis / length2, 0., 1.) if length2 else 0.
            closest = start + np.asarray(ratio)[..., None] * axis
            inside = np.linalg.norm(vertices - closest, axis=1) <= segment.radius + 1e-8
            face_covered |= inside[faces].all(axis=1)
        assert face_covered.all(), link_name


@pytest.mark.parametrize('name', ('jaco2', 'ur17g', 'p3020', 'cs625'))
def test_mounted_arm_envelopes_clear_the_board_at_start(profiles, name):
    from thesis_core.work_table import fixed_mount_links, WorkTable, SIM_TABLE_SIZE, TABLE_CENTER

    profile = profiles[name]
    fixed = fixed_mount_links(profile.urdf_root)
    table = WorkTable((*profile.base_xyz[:2], TABLE_CENTER[2]), SIM_TABLE_SIZE,
                      frozenset(
                          c.name for c in profile.capsules
                          if c.link in fixed
                          or c.start_frame in fixed
                          or c.end_frame in fixed
                      ))
    capsules = profile.capsule_segments(profile.initial_positions)
    hit = table.closest([(c.start, c.end) for c in capsules],
                        [c.name for c in capsules], [c.radius for c in capsules])
    # The UR17g shoulder sits close to the table when mounted directly on it;
    # keep it monitored and above the 5 cm STOP threshold.
    assert hit.clearance > (.05 if name in ('ur17g', 'cs625') else .15)


def test_cell_jaco_keeps_historical_operational_limits(profiles):
    """Keep legacy GUI/safety bounds despite namespaced URDF joints."""
    profile = profiles['jaco2']
    assert profile.continuous_flags == (
        True, False, False, True, True, True
    )
    for actual_name, canonical_name in zip(
        profile.joint_names,
        JOINT_VELOCITY_LIMITS,
    ):
        assert profile.joint_velocity_limits[actual_name] == pytest.approx(
            JOINT_VELOCITY_LIMITS[canonical_name]
        )
        lower, upper = profile.joint_position_limits[actual_name]
        assert math.isfinite(lower)
        assert math.isfinite(upper)


def test_no_manifest_preserves_legacy_jaco2_default():
    """Keep the original unnamespaced JACO2 profile as the default."""
    profile = load_robot_profile()
    assert profile.name == 'jaco2'
    assert profile.joint_names == tuple(JOINT_VELOCITY_LIMITS)
    # Compare with the simulated robot's actual default startup pose.
    model = Path(get_package_share_directory('thesis_simulation')) / 'urdf/jaco_gazebo.ros2_control.xacro'
    urdf = xacro.process_file(str(model))
    from xml.etree import ElementTree as ET
    control = ET.fromstring(urdf.toxml()).find('ros2_control')
    expected = [float(control.findtext(
        f"joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']"
    )) for name in profile.joint_names]
    assert profile.initial_positions == pytest.approx(expected)
    assert profile.world_frame == 'world'
    assert profile.controller_action == (
        '/arm_controller/follow_joint_trajectory'
    )
    assert len(profile.capsule_segments(profile.initial_positions)) == 6
