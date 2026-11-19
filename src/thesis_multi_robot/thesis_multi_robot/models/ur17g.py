"""Official UR17g-1300 geometry, isolated from the manufacturer's driver.

The Humble description includes hardware tags by default.  They are removed
here so the common simulation builder is the sole owner of ros2_control and
the Gazebo plugin.  No network-connected hardware backend is instantiated.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


JOINT_NAMES = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)
BASE_LINK = 'base_link'
TIP_LINK = 'tool0'
INITIAL_POSITIONS = dict(zip(JOINT_NAMES, (0.0, -1.57, 0.0, -1.57, 0.0, 0.0)))
SOURCE_PACKAGES = ('ur_description',)


def load_model() -> ET.Element:
    """Return manufacturer geometry with an unprefixed ``base_link`` root."""
    description = Path(get_package_share_directory('ur_description'))
    document = xacro.process_file(
        str(description / 'urdf' / 'ur.urdf.xacro'),
        mappings={
            'name': 'ur17g',
            'ur_type': 'ur17g-1300',
            'tf_prefix': '',
            'use_fake_hardware': 'true',
            'sim_gazebo': 'false',
            'sim_ignition': 'false',
        },
    )
    robot = ET.fromstring(document.toxml())
    for element in list(robot):
        if element.tag in ('ros2_control', 'transmission', 'gazebo'):
            robot.remove(element)
        elif element.tag == 'link' and element.get('name') == 'world':
            robot.remove(element)
        elif element.tag == 'joint':
            parent = element.find('parent')
            if parent is not None and parent.get('link') == 'world':
                robot.remove(element)
    return robot
