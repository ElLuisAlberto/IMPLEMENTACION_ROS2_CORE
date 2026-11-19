"""CS625 nominal manufacturer description for simulation."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro

JOINT_NAMES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
BASE_LINK = 'base_link'
TIP_LINK = 'tool0'
INITIAL_POSITIONS = dict(zip(JOINT_NAMES, (0.0, -1.57, 0.0, -1.57, 0.0, 0.0)))
SOURCE_PACKAGES = ('elite_robots_description',)


def load_model():
    path = Path(get_package_share_directory('elite_robots_description'))
    robot = ET.fromstring(xacro.process_file(
        str(path / 'urdf' / 'cs.urdf.xacro'),
        mappings={'name': 'cs625', 'cs_type': 'cs625',
                  'use_fake_hardware': 'true', 'sim_gazebo': 'false',
                  'sim_ignition': 'false'},
    ).toxml())
    for element in list(robot):
        if element.tag in ('ros2_control', 'transmission', 'gazebo'):
            robot.remove(element)
        elif element.tag == 'link' and element.get('name') == 'world':
            robot.remove(element)
        elif element.tag == 'joint':
            parent = element.find('parent')
            if parent is not None and parent.get('link') == 'world':
                robot.remove(element)
    # The vendor Xacro overrides every multi-material DAE with LightGrey.
    # Remove that override so RViz and Gazebo can use the embedded materials.
    for visual in robot.findall('link/visual'):
        material = visual.find('material')
        if material is not None:
            visual.remove(material)
    return robot
