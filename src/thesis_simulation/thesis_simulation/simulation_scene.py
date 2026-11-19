"""Build the visible simulation obstacle from its configured geometry."""

import math
import xml.etree.ElementTree as ET


def spherical_obstacle_sdf(template, radius):
    """Use one radius for both collision and visual geometry, in memory."""
    value = float(radius)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError('obstacle_radius must be finite and positive')
    root = ET.fromstring(template)
    spheres = root.findall('.//sphere/radius')
    if len(spheres) != 2:
        raise ValueError('obstacle template must have visual and collision spheres')
    for element in spheres:
        element.text = str(value)
    model = root.find('model')
    if model is None:
        raise ValueError('obstacle template must contain a model')
    # SceneBroadcaster dates Pose_V only at the vector level, which the Humble
    # TFMessage bridge drops. PosePublisher dates each individual pose at its
    # actual simulation acquisition time, so exact-stamp TF remains possible.
    plugin = ET.SubElement(model, 'plugin', {
        'filename': 'libignition-gazebo-pose-publisher-system.so',
        'name': 'ignition::gazebo::systems::PosePublisher',
    })
    for name, text in {
        'publish_link_pose': 'false',
        'publish_visual_pose': 'false',
        'publish_collision_pose': 'false',
        'publish_sensor_pose': 'false',
        'publish_nested_model_pose': 'true',
        'publish_model_pose': 'true',
        'use_pose_vector_msg': 'true',
        'static_publisher': 'false',
        'update_frequency': '30',
    }.items():
        ET.SubElement(plugin, name).text = text
    return ET.tostring(root, encoding='unicode')
