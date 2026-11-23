"""Compare configured world geometry against independently expanded URDF chains."""

from functools import lru_cache
import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ElementTree

import pytest
import yaml

from thesis_core.geometry_profile import legacy_jaco_geometry_profile
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII, SEGMENT_NAMES, capsule_segments, end_effector_transform,
    identity_matrix, multiply, rotation_z, transform_matrix, translation,
)
from thesis_core.multiarm_coordination import (
    CapsuleOccupancy, RobotOccupancySnapshot, evaluate_four_arm_safety,
)
from thesis_simulation.hybrid_scenario import build_hybrid_three_sim_scenario


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
INITIAL_JOINTS = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)


def scenario():
    with (PACKAGE_ROOT / 'config/hybrid_three_sim.yaml').open() as stream:
        return build_hybrid_three_sim_scenario(yaml.safe_load(stream))


@lru_cache(maxsize=4)
def expanded_joints(robot_id):
    """Expand each actual Xacro rather than reusing FK joint constants."""
    xacro = shutil.which('xacro')
    if xacro is None:
        pytest.skip('xacro is required for URDF/FK integration checks')
    instance = scenario().instance(robot_id)
    model = (
        'hybrid_jaco.synthetic.xacro'
        if instance.source == 'synthetic_state' else
        'hybrid_jaco.ros2_control.xacro'
    )
    command = [
        xacro, str(PACKAGE_ROOT / 'urdf' / model),
        f'prefix:={instance.tf_prefix}',
        f'root_name:={instance.tf_prefix}_root',
        'base_xyz:=' + ' '.join(map(str, instance.base_pose[:3])),
        'base_rpy:=' + ' '.join(map(str, instance.base_pose[3:])),
        f'tool_mode:={instance.tool_mode}',
    ]
    robot = ElementTree.fromstring(subprocess.check_output(command, text=True))
    return {
        joint.find('child').attrib['link']: joint
        for joint in robot.findall('joint')
    }


def test_jaco_visuals_use_muted_body_and_contrasting_joint_colors():
    xacro = shutil.which('xacro')
    if xacro is None:
        pytest.skip('xacro is required for JACO visual material checks')
    model = PACKAGE_ROOT / 'urdf' / 'jaco_gazebo.ros2_control.xacro'
    robot = ElementTree.fromstring(
        subprocess.check_output([xacro, str(model)], text=True)
    )
    links = {
        link.attrib['name']: link
        for link in robot.findall('link')
    }

    def color(link_name, visual_index):
        rgba = links[link_name].findall('visual')[visual_index].find(
            'material/color'
        )
        return tuple(map(float, rgba.attrib['rgba'].split()))

    body = color('j2n6s300_link_2', 0)
    joint = color('j2n6s300_link_2', 1)
    assert body == pytest.approx((0.66, 0.67, 0.68, 1.0))
    assert joint == pytest.approx((0.28, 0.29, 0.30, 1.0))
    assert max(body[:3]) - min(body[:3]) < 0.03
    assert max(joint[:3]) < min(body[:3]) - 0.25


def urdf_transform(joints, child_frame, positions):
    """Walk URDF parent links to world and apply its declared Z-axis joints."""
    chain = []
    while child_frame != 'world':
        joint = joints[child_frame]
        chain.append(joint)
        child_frame = joint.find('parent').attrib['link']
    transform = identity_matrix()
    for joint in reversed(chain):
        origin = joint.find('origin')
        attributes = {} if origin is None else origin.attrib
        xyz = tuple(map(float, attributes.get('xyz', '0 0 0').split()))
        rpy = tuple(map(float, attributes.get('rpy', '0 0 0').split()))
        transform = multiply(transform, transform_matrix(xyz, rpy))
        if joint.attrib['type'] != 'fixed':
            assert joint.find('axis').attrib['xyz'] == '0 0 1'
            index = int(joint.attrib['name'].rsplit('_', 1)[-1]) - 1
            transform = multiply(transform, rotation_z(positions[index]))
    return transform


@pytest.mark.parametrize('robot_id', ('arm_01', 'arm_02', 'arm_03', 'arm_04'))
@pytest.mark.parametrize('positions', (
    INITIAL_JOINTS, (-0.2, 2.7, 3.4, 0.2, -0.1, 0.3),
))
def test_each_instance_capsules_and_tool_match_configured_urdf(robot_id, positions):
    instance = scenario().instance(robot_id)
    joints = expanded_joints(robot_id)
    base = transform_matrix(instance.base_pose[:3], instance.base_pose[3:])
    axes = capsule_segments(positions, base)
    points = [axes[0][0]] + [segment[1] for segment in axes]
    frames = [f'{instance.tf_prefix}_link_base'] + [
        f'{instance.tf_prefix}_link_{index}' for index in range(1, 6)
    ] + [f'{instance.tf_prefix}_end_effector']
    for frame, expected in zip(frames, points):
        actual = translation(urdf_transform(joints, frame, positions))
        assert actual == pytest.approx(expected, abs=1.0e-10)
    actual_tool = urdf_transform(joints, frames[-1], positions)
    expected_tool = end_effector_transform(positions, base)
    for actual, expected in zip(actual_tool, expected_tool):
        assert actual == pytest.approx(expected, abs=1.0e-10)


def test_default_scenario_all_six_robot_pairs_are_well_separated():
    snapshots = []
    for instance in scenario().instances:
        profile = legacy_jaco_geometry_profile(
            instance.robot_id, frame_prefix=instance.tf_prefix
        )
        axes = capsule_segments(
            INITIAL_JOINTS,
            transform_matrix(instance.base_pose[:3], instance.base_pose[3:]),
        )
        snapshots.append(RobotOccupancySnapshot(
            robot_id=instance.robot_id,
            profile_id=profile.profile_id,
            geometry_version=profile.geometry_version,
            asset_hash=profile.asset_hash,
            reference_frame='world', stamp_sec=10.0, ttl_sec=0.25,
            capsules=tuple(
                CapsuleOccupancy(name, *axis, radius)
                for name, axis, radius in zip(SEGMENT_NAMES, axes, CAPSULE_RADII)
            ),
        ))
    decision = evaluate_four_arm_safety(snapshots, 10.1, 0.05)
    assert decision.state == 'ALLOW'
    assert len(decision.evaluated_robot_pairs) == 6
    assert all(pair.minimum.clearance > 1.0 for pair in decision.pair_clearances)
