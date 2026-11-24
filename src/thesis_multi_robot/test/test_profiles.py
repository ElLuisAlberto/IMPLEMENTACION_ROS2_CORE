"""Integration contracts for the installed official descriptions and wrappers."""

import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import yaml

from thesis_multi_robot.kinematics import RobotKinematics
from thesis_multi_robot.profiles import (
    CELL_POSES, PROFILES, build_cell, build_robot, resource_path,
    source_module,
)


@pytest.fixture(scope='module')
def cell(tmp_path_factory):
    """Generate collision envelopes once for the four full official models."""
    directory = tmp_path_factory.mktemp('four_robot_profiles')
    manifest = json.loads(build_cell(directory).read_text())
    return {
        name: {
            'profile': profile,
            'root': ET.parse(profile['urdf']).getroot(),
            'source': (source_module(name).load_model('steel_preview')
                       if name == 'p3020' else source_module(name).load_model()),
            'module': source_module(name),
        }
        for name, profile in manifest['robots'].items()
    }


def _canonical_geometry(link, aliases=None):
    """Compare original geometry after resource resolution, ignoring names."""
    aliases = aliases or {}
    result = []
    for child in link:
        if child.tag not in {'visual', 'collision', 'inertial'}:
            continue
        element = copy.deepcopy(child)
        for mesh in element.findall('.//mesh'):
            uri = mesh.attrib['filename']
            mesh.set('filename', resource_path(aliases.get(uri, uri)))
        result.append(ET.tostring(element, encoding='unicode'))
    return result


@pytest.mark.parametrize('name', PROFILES)
def test_published_mesh_uris_are_portable_and_resolvable(cell, name):
    model = cell[name]
    meshes = model['root'].findall('.//mesh')
    assert meshes
    for mesh in meshes:
        uri = mesh.attrib['filename']
        assert uri.startswith(('package://', 'file://'))
        assert Path(resource_path(uri)).is_file()
    expected_search_paths = {
        str(Path(get_package_share_directory(
            mesh.attrib['filename'][10:].split('/', 1)[0]
        )).parent)
        for mesh in model['source'].findall('.//mesh')
        if mesh.attrib['filename'].startswith('package://')
    }
    if name != 'jaco2':
        expected_search_paths.add(str(Path(get_package_share_directory(
            'sistema_gripper_instrumentado_description'
        )).parent))
    assert set(model['profile']['resource_search_paths']) == (
        expected_search_paths
    )
    assert all(Path(path).is_dir()
               for path in model['profile']['resource_search_paths'])


def test_capsule_extraction_resolves_only_an_internal_copy(
        tmp_path, monkeypatch):
    from thesis_multi_robot import geometry

    original = geometry.collision_capsules
    observed = []

    def inspect_internal_root(root, **kwargs):
        filenames = [
            mesh.attrib['filename']
            for mesh in root.findall('.//collision/geometry/mesh')
        ]
        assert filenames
        assert all(Path(filename).is_absolute() for filename in filenames)
        observed.extend(filenames)
        return original(root, **kwargs)

    monkeypatch.setattr(geometry, 'collision_capsules', inspect_internal_root)
    profile = build_robot('ur17g', 'uri_probe', tmp_path)
    published = ET.parse(profile['urdf']).getroot()
    assert observed
    assert all(
        mesh.attrib['filename'].startswith('package://')
        for mesh in published.findall('.//mesh')
    )


def test_file_resource_uri_decodes_spaces(tmp_path):
    resource = tmp_path / 'directory with spaces' / 'mesh.obj'
    resource.parent.mkdir()
    resource.write_text('o test\n')
    assert resource_path(resource.resolve().as_uri()) == str(
        resource.resolve()
    )


def test_file_resource_uri_rejects_remote_host():
    with pytest.raises(ValueError, match='Non-local model resource'):
        resource_path('file://remote-host/tmp/mesh.obj')


@pytest.mark.parametrize('name', PROFILES)
def test_source_geometry_limits_and_joint_types_are_preserved(cell, name):
    model = cell[name]
    root, source, module = model['root'], model['source'], model['module']
    prefix = name + '_'
    links = {link.attrib['name']: link for link in root.findall('link')}
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    expected_links = {prefix + link.attrib['name'] for link in source.findall('link')} | {'world'}
    if name != 'jaco2':
        expected_links.add(prefix + 'provisional_gripper')
        assert model['profile']['provisional_gripper']
        assert model['profile']['tool_safety_validated'] is False
    assert set(links) == expected_links
    for original in source.findall('link'):
        assert _canonical_geometry(original) == _canonical_geometry(
            links[prefix + original.attrib['name']],
            model['profile'].get('resource_aliases'),
        )
    for original in source.findall('joint'):
        wrapped = joints[prefix + original.attrib['name']]
        assert wrapped.attrib['type'] == original.attrib['type']
        for tag in ('origin', 'axis', 'limit', 'dynamics'):
            before, after = original.find(tag), wrapped.find(tag)
            assert (None if before is None else before.attrib) == (None if after is None else after.attrib)
    for joint in module.JOINT_NAMES:
        original = source.find(f"joint[@name='{joint}']")
        assert original is not None
        assert original.attrib['type'] != 'fixed'
    assert len(model['profile']['joint_names']) == (5 if name == 'p3020' else 6)
    if name == 'p3020':
        assert joints['p3020_joint_4'].attrib['type'] == 'fixed'
        assert len(root.findall('.//collision')) >= 22
    if name == 'p3020':
        arm = [c for c in model['profile']['capsules']
               if c['link'] != 'p3020_provisional_gripper']
        assert len(arm) == 10
        assert len({c['link'] for c in arm}) == 6
        for link in {c['link'] for c in arm}:
            assert sum(c['start'] != c['end'] for c in arm if c['link'] == link) == 1
    if name == 'ur17g':
        for link in ('ur17g_upper_arm_link', 'ur17g_forearm_link'):
            segments = [c for c in model['profile']['capsules'] if c['link'] == link]
            assert len(segments) == 3
            assert sum(c['start'] != c['end'] for c in segments) == 1
    if name == 'cs625':
        for link in ('cs625_upperarm_link', 'cs625_forearm_link'):
            segments = [c for c in model['profile']['capsules'] if c['link'] == link]
            assert len(segments) == 3
            assert sum(c['start'] != c['end'] for c in segments) == 1
    assert model['profile']['capsules']
    assert all(capsule['radius'] > 0 for capsule in model['profile']['capsules'])


@pytest.mark.parametrize('name', ('ur17g', 'cs625'))
def test_base_is_on_table(cell, name):
    from thesis_core.work_table import MOUNT_HEIGHT
    assert cell[name]['profile']['base_xyz'][2] == pytest.approx(MOUNT_HEIGHT)


def test_cell_frame_and_controller_names_are_isolated(cell):
    seen_links, seen_joints, seen_namespaces = set(), set(), set()
    for name, model in cell.items():
        root, profile = model['root'], model['profile']
        links = {link.attrib['name'] for link in root.findall('link')} - {'world'}
        joints = {joint.attrib['name'] for joint in root.findall('joint')}
        assert not seen_links.intersection(links)
        assert not seen_joints.intersection(joints)
        assert all(link.startswith(name + '_') for link in links)
        assert all(joint.startswith(name + '_') for joint in joints)
        seen_links.update(links)
        seen_joints.update(joints)
        control = root.findall('ros2_control')
        assert len(control) == 1
        assert control[0].find('hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
        assert [joint.attrib['name'] for joint in control[0].findall('joint')] == profile['joint_names']
        plugins = root.findall('gazebo/plugin')
        assert len(plugins) == 1
        plugin = plugins[0]
        namespace = plugin.find('ros/namespace').text
        assert namespace == '/' + name
        assert namespace not in seen_namespaces
        seen_namespaces.add(namespace)
        node_name = plugin.find('robot_param_node').text
        resolved_node = node_name if node_name.startswith('/') else namespace + '/' + node_name
        assert resolved_node == f'/{name}/robot_state_publisher'
        assert Path(plugin.find('parameters').text).resolve() == Path(profile['controller_config']).resolve()
        config = yaml.safe_load(Path(profile['controller_config']).read_text())
        assert set(config) == {f'/{name}/controller_manager', f'/{name}/arm_controller'}
        arm = config[f'/{name}/arm_controller']['ros__parameters']
        assert arm['joints'] == profile['joint_names']
        assert arm['command_interfaces'] == ['position']
        assert not arm['allow_partial_joints_goal']


@pytest.mark.parametrize('name', PROFILES)
def test_prefixed_fk_matches_official_model_and_world_mount(cell, name):
    model = cell[name]
    profile, module = model['profile'], model['module']
    source_model = RobotKinematics(model['source'], module.JOINT_NAMES)
    wrapped_model = RobotKinematics(model['root'], profile['joint_names'])
    source_frames = source_model.forward(profile['initial_positions'])
    wrapped_frames = wrapped_model.forward(profile['initial_positions'])
    mount = np.eye(4)
    mount[:3, 3] = CELL_POSES[name]
    for link, expected in source_frames.items():
        np.testing.assert_allclose(wrapped_frames[name + '_' + link], mount @ expected, atol=1e-10)
    np.testing.assert_allclose(wrapped_frames['world'], np.eye(4), atol=1e-12)


@pytest.mark.parametrize('name', PROFILES)
def test_real_profile_small_pose_ik_respects_actual_dof_and_limits(cell, name):
    model = cell[name]
    profile = model['profile']
    robot = RobotKinematics(model['root'], profile['joint_names'])
    seed = np.asarray(profile['initial_positions'])
    delta = np.linspace(0.01, 0.02, len(seed))
    desired = seed + delta
    for index, joint in enumerate(profile['joint_names']):
        lower, upper = robot.joint_limits[joint]
        desired[index] = np.clip(desired[index], lower, upper)
    target = robot.forward(desired)[profile['tip_link']]
    solution = robot.inverse(target, seed, profile['tip_link'])
    actual = robot.forward(solution)[profile['tip_link']]
    assert len(solution) == (5 if name == 'p3020' else 6)
    assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) <= 0.003
    assert np.linalg.norm(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()) <= np.deg2rad(1)
    for joint, position in zip(profile['joint_names'], solution):
        lower, upper = robot.joint_limits[joint]
        assert lower <= position <= upper


def test_overlapping_real_robot_envelopes_are_stopped(cell):
    from thesis_multi_robot.geometry import capsule_sets_clearance, world_capsules
    from thesis_multi_robot.supervisor import classify_clearance, GEOMETRIC_MARGIN
    envelopes = []
    for name in ('ur17g', 'p3020'):
        model = cell[name]
        profile = model['profile']
        frames = RobotKinematics(model['root'], profile['joint_names']).forward(profile['initial_positions'])
        # Place the models at the same base for a deterministic inter-robot
        # collision test, without launching interpenetrating bodies in physics.
        for frame in frames.values():
            frame[:3, 3] -= np.asarray(CELL_POSES[name])
        envelopes.append(world_capsules(profile['capsules'], frames))
    clearance = capsule_sets_clearance(*envelopes)
    assert clearance < 0
    assert classify_clearance(clearance-GEOMETRIC_MARGIN).state == 'STOP'
