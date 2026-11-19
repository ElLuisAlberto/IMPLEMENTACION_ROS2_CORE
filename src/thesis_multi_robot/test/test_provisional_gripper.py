"""Static preview includes the exact exported CAD bounds, never JACO2."""
import xml.etree.ElementTree as ET

import pytest

from thesis_multi_robot.profiles import _add_provisional_gripper, build_robot


def test_collision_box_covers_exported_bounds():
    root = ET.fromstring('<robot><link name="flange"/></robot>')
    _add_provisional_gripper(root, 'flange')
    box = root.find("link[@name='provisional_gripper']/collision")
    center = [float(x) for x in box.find('origin').get('xyz').split()]
    size = [float(x) for x in box.find('geometry/box').get('size').split()]
    lower = [-0.0575, -0.063, -0.0575]
    upper = [0.06302875, 0.13539823, 0.0575]
    assert all(c - s / 2 <= x for c, s, x in zip(center, size, lower))
    assert all(c + s / 2 >= x for c, s, x in zip(center, size, upper))
    assert root.find("joint[@name='provisional_gripper_mount']/parent").get('link') == 'flange'
    mesh = root.find("link[@name='provisional_gripper']/visual/geometry/mesh")
    assert 'sistema_gripper_instrumentado.stl' in mesh.get('filename')
    assert mesh.get('scale') == '0.001 0.001 0.001'


def test_jaco2_does_not_accept_provisional_tool(tmp_path):
    with pytest.raises(ValueError, match='unavailable'):
        build_robot('jaco2', 'jaco2', tmp_path, provisional_gripper=True)
