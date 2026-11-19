"""CS625 preview and normal profile contracts."""
import xml.etree.ElementTree as ET

from thesis_multi_robot.profiles import PROFILES, build_robot


def test_cs625_preview_has_six_joints_and_full_tool(tmp_path):
    model = build_robot('cs625', 'cs625', tmp_path, provisional_gripper=True)
    robot = ET.parse(model['urdf']).getroot()
    assert len(model['joint_names']) == 6
    assert robot.find("link[@name='cs625_provisional_gripper']") is not None
    assert model['tool_safety_validated'] is False
    assert 'cs625' in PROFILES
    arm = robot.find("link[@name='cs625_upperarm_link']/visual")
    assert arm.find('material') is None
    assert arm.find('geometry/mesh').get('filename').endswith('upperarm.dae')


def test_cs625_can_enter_normal_cell(tmp_path):
    model = build_robot('cs625', 'cs625', tmp_path)
    assert len(model['joint_names']) == 6
    assert model['profile'] == 'cs625'
