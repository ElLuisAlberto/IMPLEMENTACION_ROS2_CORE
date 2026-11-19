"""Small independent URDF fixtures exercise FK and bounded IK contracts."""

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from thesis_multi_robot.kinematics import RobotKinematics


def planar_arm():
    # The last fixed link makes the endpoint dependent on both joint angles.
    return ET.fromstring('''<robot name="planar">
      <link name="world"/><link name="base"/><link name="a"/>
      <link name="b"/><link name="tip"/>
      <joint name="mount" type="fixed"><parent link="world"/>
        <child link="base"/><origin xyz="1 2 0" rpy="0 0 0"/></joint>
      <joint name="q1" type="revolute"><parent link="base"/>
        <child link="a"/><axis xyz="0 0 1"/>
        <limit lower="-2" upper="2" velocity="1" effort="10"/></joint>
      <joint name="q2" type="revolute"><parent link="a"/>
        <child link="b"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-2" upper="2" velocity="1" effort="10"/></joint>
      <joint name="tool" type="fixed"><parent link="b"/>
        <child link="tip"/><origin xyz="1 0 0"/></joint>
    </robot>''')


def test_forward_fixed_mount_and_explicit_joint_order():
    robot = RobotKinematics(planar_arm(), ('q2', 'q1'))
    frames = robot.forward((0.0, math.pi / 2))
    np.testing.assert_allclose(frames['world'], np.eye(4), atol=1e-12)
    np.testing.assert_allclose(frames['tip'][:3, 3], (1, 4, 0), atol=1e-12)
    np.testing.assert_allclose(
        frames['tip'][:3, :3], Rotation.from_euler('z', math.pi / 2).as_matrix(),
        atol=1e-12,
    )


@pytest.mark.parametrize('positions', [(0,), (0, 0, 0), (math.nan, 0), (3, 0)])
def test_invalid_vectors_and_limits_are_rejected(positions):
    with pytest.raises(ValueError):
        RobotKinematics(planar_arm(), ('q1', 'q2')).forward(positions)


def test_inverse_recovers_a_reachable_pose():
    robot = RobotKinematics(planar_arm(), ('q1', 'q2'))
    target = robot.forward((0.8, -0.6))['tip']
    result = robot.inverse(target, (0.2, -0.1), 'tip')
    np.testing.assert_allclose(robot.forward(result)['tip'], target, atol=1e-5)


def test_inverse_position_only():
    robot = RobotKinematics(planar_arm(), ('q1', 'q2'))
    target = robot.forward((0.8, -0.6))['tip']
    target[:3, :3] = Rotation.from_euler('x', 0.8).as_matrix()
    result = robot.inverse(target, (0.2, -0.1), 'tip', position_only=True)
    np.testing.assert_allclose(
        robot.forward(result)['tip'][:3, 3], target[:3, 3], atol=1e-5,
    )


def test_underactuated_orientation_and_unreachable_position_fail():
    robot = RobotKinematics(planar_arm(), ('q1', 'q2'))
    target = robot.forward((0.5, 0.2))['tip']
    target[:3, :3] = Rotation.from_euler('x', math.pi / 2).as_matrix()
    with pytest.raises(ValueError, match='unreachable'):
        robot.inverse(target, (0.2, 0.1), 'tip')
    target = np.eye(4)
    target[0, 3] = 20
    with pytest.raises(ValueError, match='unreachable'):
        robot.inverse(target, (0.2, 0.1), 'tip', position_only=True)


def test_prismatic_origin_rotation_and_mimic():
    root = ET.fromstring('''<robot name="slider"><link name="world"/>
      <link name="slide"/><link name="follower"/>
      <joint name="q" type="prismatic"><parent link="world"/>
        <child link="slide"/><origin rpy="0 0 1.5707963267948966"/>
        <axis xyz="1 0 0"/><limit lower="0" upper="1" velocity="1" effort="1"/></joint>
      <joint name="mirror" type="prismatic"><parent link="slide"/>
        <child link="follower"/><axis xyz="1 0 0"/>
        <limit lower="-1" upper="0" velocity="1" effort="1"/>
        <mimic joint="q" multiplier="-1"/></joint></robot>''')
    robot = RobotKinematics(root, ('q',))
    frames = robot.forward((0.5,))
    np.testing.assert_allclose(frames['slide'][:3, 3], (0, 0.5, 0), atol=1e-12)
    np.testing.assert_allclose(frames['follower'][:3, 3], (0, 0, 0), atol=1e-12)


def test_invalid_model_and_target_rejected():
    with pytest.raises(ValueError):
        RobotKinematics(planar_arm(), ('q1', 'q1'))
    with pytest.raises(ValueError):
        RobotKinematics(planar_arm(), ('missing',))
    root = planar_arm()
    ET.SubElement(root, 'link', name='island')
    with pytest.raises(ValueError):
        RobotKinematics(root, ('q1', 'q2'))
    robot = RobotKinematics(planar_arm(), ('q1', 'q2'))
    target = np.eye(4)
    target[0, 0] = 2
    with pytest.raises(ValueError, match='rigid'):
        robot.inverse(target, (0, 0), 'tip')
