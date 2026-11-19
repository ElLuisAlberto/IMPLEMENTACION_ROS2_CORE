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


@pytest.mark.parametrize('name', ('jaco2', 'ur17g', 'p3020', 'cs625'))
def test_mounted_arm_envelopes_clear_the_board_at_start(profiles, name):
    from thesis_core.work_table import fixed_mount_links, WorkTable, TABLE_SIZE, TABLE_CENTER

    profile = profiles[name]
    fixed = fixed_mount_links(profile.urdf_root)
    table = WorkTable((*profile.base_xyz[:2], TABLE_CENTER[2]), TABLE_SIZE,
                      frozenset(c.name for c in profile.capsules if c.link in fixed))
    capsules = profile.capsule_segments(profile.initial_positions)
    hit = table.closest([(c.start, c.end) for c in capsules],
                        [c.name for c in capsules], [c.radius for c in capsules])
    # Includes both nominal cell geometric margin (.05) and reduction band (.10).
    assert hit.clearance > .15


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
