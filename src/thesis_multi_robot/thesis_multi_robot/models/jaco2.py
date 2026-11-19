"""Existing Kinova JACO2 geometry in the common simulation profile."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro

JOINT_NAMES = tuple(f'j2n6s300_joint_{i}' for i in range(1, 7))
BASE_LINK = 'root'
TIP_LINK = 'j2n6s300_end_effector'
INITIAL_POSITIONS = dict(zip(JOINT_NAMES, (0., math.pi, math.pi, 0., 0., 0.)))
SOURCE_PACKAGES = ('thesis_description',)


def load_model():
    path = Path(get_package_share_directory('thesis_description')) / 'urdf/j2n6s300_standalone.xacro'
    root = ET.fromstring(xacro.process_file(str(path)).toxml())
    for child in list(root):
        if child.tag in ('gazebo', 'transmission', 'ros2_control'):
            root.remove(child)
        elif child.tag == 'link' and child.get('name') == 'world':
            root.remove(child)
        elif child.tag == 'joint' and child.find('parent').get('link') == 'world':
            root.remove(child)
    # This cell controls the arm only. Fix each uncommanded finger at zero so
    # Gazebo cannot let it fall while the geometric model assumes a zero pose.
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed' and joint.get('name') not in JOINT_NAMES:
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'mimic'):
                element = joint.find(tag)
                if element is not None:
                    joint.remove(element)
    return root
