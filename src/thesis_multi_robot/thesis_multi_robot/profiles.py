"""Build isolated, reproducible ros2_control descriptions from official assets."""
import copy
from importlib import import_module
import json
import math
from pathlib import Path
import re
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml
from thesis_core.work_table import MOUNT_HEIGHT, TABLE_CENTER

PROFILES = ('jaco2', 'ur17g', 'p3020', 'cs625')
# Mount adapters provide clearance for conservative shoulder envelopes.
RISER_HEIGHT = {'jaco2': 0.06, 'ur17g': 0.20, 'p3020': 0.03, 'cs625': 0.20}
CELL_POSES = {name: (x, y, MOUNT_HEIGHT + RISER_HEIGHT[name])
              for name, x, y in [('jaco2', -4., 0.), ('ur17g', 0., 0.),
                                 ('p3020', 0., 4.), ('cs625', 4., 0.)]}

GRIPPER_VISUAL = ('package://sistema_gripper_instrumentado_description/'
                  'models/sistema_gripper_instrumentado/meshes/'
                  'sistema_gripper_instrumentado.stl')


def _add_provisional_gripper(root, parent):
    """Attach static CAD and a bounding box; mounting is uncalibrated.

    The box encloses the exported STL AABB with 1 mm margin on each side.
    It does not cover finger motion or validate any physical tool geometry.
    """
    if root.find(f"link[@name='{parent}']") is None:
        raise ValueError(f'Missing tool parent {parent}')
    link = ET.SubElement(root, 'link', name='provisional_gripper')
    visual = ET.SubElement(link, 'visual')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                  filename=GRIPPER_VISUAL, scale='0.001 0.001 0.001')
    material = ET.SubElement(visual, 'material', name='system_cad_neutral_gray')
    ET.SubElement(material, 'color', rgba='0.68 0.72 0.75 1')
    collision = ET.SubElement(link, 'collision')
    ET.SubElement(collision, 'origin', xyz='0.0027644 0.0361992 0', rpy='0 0 0')
    ET.SubElement(ET.SubElement(collision, 'geometry'), 'box',
                  size='0.122529 0.200399 0.117')
    joint = ET.SubElement(root, 'joint', name='provisional_gripper_mount', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link='provisional_gripper')
    # Visual estimate: CAD +Y is the tool direction and its minimum Y is
    # approximately -63 mm. Rotate around the tool axis to put the camera
    # above the fingers in the preview pose; not a measured adapter.
    ET.SubElement(joint, 'origin', xyz='0 0 0.063', rpy='1.5707963267948966 0 3.141592653589793')


def source_module(profile):
    if profile not in PROFILES:
        raise ValueError(f'Unknown robot profile: {profile}')
    return import_module(f'thesis_multi_robot.models.{profile}')


def resource_path(uri):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
        path = Path(get_package_share_directory(package)) / relative
    elif uri.startswith('file://'):
        parsed = urlparse(uri)
        if parsed.netloc not in ('', 'localhost'):
            raise ValueError(f'Non-local model resource: {uri}')
        path = Path(unquote(parsed.path))
    else:
        path = Path(uri)
    if not path.is_file():
        raise FileNotFoundError(f'Missing model resource: {uri}')
    return str(path.resolve())


def _public_resource_uri(uri):
    """Validate a mesh URI while keeping ROS package resources portable."""
    resolved = Path(resource_path(uri))
    if uri.startswith('package://'):
        return uri
    return resolved.as_uri()


def _resource_search_paths(root):
    """Return Gazebo search roots for every package-backed mesh in ``root``."""
    paths = set()
    for mesh in root.findall('.//mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            package = uri[10:].split('/', 1)[0]
            paths.add(str(Path(get_package_share_directory(package)).parent))
    return sorted(paths)


def build_robot(
    profile, name, output_dir, xyz=(0., 0., 0.), visual_cache_dir=None,
    provisional_gripper=False,
):
    if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
        raise ValueError('Robot name must match [a-z][a-z0-9_]*')
    if len(xyz) != 3 or not all(math.isfinite(float(v)) for v in xyz):
        raise ValueError('Base pose must contain three finite coordinates')
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    module = source_module(profile)
    # The manufacturer blue is nearly black in RViz. Apply a brighter,
    # explicitly approximate palette only to the provisional P3020 preview.
    root = (module.load_model('steel_preview')
            if profile == 'p3020' and provisional_gripper
            else module.load_model())
    if provisional_gripper and profile == 'jaco2':
        raise ValueError('Provisional gripper is unavailable for JACO2')
    resource_aliases = {}
    prepare_visuals = getattr(module, 'prepare_visual_resources', None)
    if prepare_visuals is not None:
        visual_output = (
            Path(visual_cache_dir) / profile
            if visual_cache_dir is not None
            else output_dir / f'{name}_visual_resources'
        )
        resource_aliases = prepare_visuals(
            root, visual_output
        )
    if provisional_gripper:
        _add_provisional_gripper(root, module.TIP_LINK)
    root.set('name', name)
    prefix = name + '_'
    links = {link.get('name') for link in root.findall('link')}
    joints = {joint.get('name') for joint in root.findall('joint')}
    if 'world' in links:
        raise ValueError('Source adapter must remove its original world anchor')
    for element in root.iter():
        if element.tag in ('link', 'joint') and element.get('name') in links | joints:
            element.set('name', prefix + element.get('name'))
        if element.tag in ('parent', 'child') and element.get('link') in links:
            element.set('link', prefix + element.get('link'))
        if element.tag == 'mimic' and element.get('joint') in joints:
            element.set('joint', prefix + element.get('joint'))
        if element.tag == 'mesh':
            element.set(
                'filename', _public_resource_uri(element.get('filename'))
            )
    ET.SubElement(root, 'link', name='world')
    anchor = ET.SubElement(root, 'joint', name=f'{name}_world_mount', type='fixed')
    ET.SubElement(anchor, 'parent', link='world')
    ET.SubElement(anchor, 'child', link=prefix + module.BASE_LINK)
    ET.SubElement(anchor, 'origin', xyz=' '.join(map(str, xyz)), rpy='0 0 0')
    joint_names = [prefix + joint for joint in module.JOINT_NAMES]
    initial = [float(module.INITIAL_POSITIONS[joint]) for joint in module.JOINT_NAMES]
    controls = ET.SubElement(root, 'ros2_control', name=name + '_system', type='system')
    ET.SubElement(ET.SubElement(controls, 'hardware'), 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for joint_name, value in zip(joint_names, initial):
        joint = ET.SubElement(controls, 'joint', name=joint_name)
        ET.SubElement(joint, 'command_interface', name='position')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(value)
        ET.SubElement(joint, 'state_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='effort')
    controller_path = output_dir / f'{name}_controllers.yaml'
    controller_path.write_text(yaml.safe_dump({
        f'/{name}/controller_manager': {'ros__parameters': {
            'update_rate': 100,
            'use_sim_time': True,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'arm_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        f'/{name}/arm_controller': {'ros__parameters': {
            'joints': joint_names, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'state_publish_rate': 50., 'action_monitor_rate': 20.,
            'allow_partial_joints_goal': False,
            'constraints': {'goal_time': 3., 'stopped_velocity_tolerance': 0.1}}},
    }, sort_keys=False))
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin',
                           filename='gz_ros2_control-system',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/' + name
    ET.SubElement(plugin, 'robot_param').text = 'robot_description'
    # Fortress's plugin prepends its ROS namespace to this node name.
    ET.SubElement(plugin, 'robot_param_node').text = 'robot_state_publisher'
    ET.SubElement(plugin, 'parameters').text = str(controller_path)
    ET.SubElement(plugin, 'controller_manager_name').text = 'controller_manager'
    # Fixed fingers and ancillary joints remain part of geometry, not arm commands.
    # Geometry is derived from every collision element, including P3020's 22 parts.
    from .geometry import collision_capsules
    from .kinematics import RobotKinematics
    RobotKinematics(root, joint_names).forward(initial)
    # RViz and Gazebo need portable package:// (or explicit file://) URIs in
    # robot_description. Trimesh, used only to derive the safety envelope,
    # needs local filesystem paths. Resolve those paths in an internal copy so
    # the published URDF never receives a scheme-less absolute filename.
    collision_root = copy.deepcopy(root)
    for mesh in collision_root.findall('.//collision/geometry/mesh'):
        mesh.set('filename', resource_path(mesh.get('filename')))
    capsules = collision_capsules(collision_root)
    if not capsules:
        raise ValueError(f'{profile} has no collision geometry')
    urdf = ET.tostring(root, encoding='unicode')
    urdf_path = output_dir / f'{name}.urdf'
    urdf_path.write_text(urdf)
    return {'profile': profile, 'urdf': str(urdf_path), 'joint_names': joint_names,
            'initial_positions': initial, 'base_link': prefix + module.BASE_LINK,
            'tip_link': prefix + module.TIP_LINK, 'capsules': capsules,
            'controller_config': str(controller_path), 'base_xyz': list(xyz),
            'resource_search_paths': _resource_search_paths(root),
            'resource_aliases': resource_aliases,
            'provisional_gripper': provisional_gripper,
            'tool_safety_validated': False if provisional_gripper else None}


def build_cell(
    output_dir, profiles=PROFILES, single=False, visual_cache_dir=None,
):
    robots = {}
    for profile in profiles:
        x, y, height = CELL_POSES[profile]
        robots[profile] = build_robot(
            profile,
            profile,
            output_dir,
            (0., 0., height) if single else (x, y, height),
            visual_cache_dir=visual_cache_dir,
            provisional_gripper=profile != 'jaco2',
        )
    manifest = {'schema_version': 1, 'world': 'thesis_cell', 'robots': robots,
                'scope': 'nominal_simulation', 'supervision': 'experimental'}
    manifest['table_centers'] = {
        name: [*profile['base_xyz'][:2], TABLE_CENTER[2]] for name, profile in robots.items()
    }
    manifest_path = Path(output_dir) / 'cell.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    return manifest_path
