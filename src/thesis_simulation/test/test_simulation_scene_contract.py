"""The simulated visual obstacle must represent the configured geometry."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest

from thesis_simulation.simulation_scene import spherical_obstacle_sdf


ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / 'models/safety_obstacle/model.sdf'


def load_launch(filename):
    path = ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location('scene_' + filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('radius', (0.04, 0.12, 0.25))
def test_visible_and_collision_radii_follow_the_monitor_configuration(radius):
    template = TEMPLATE.read_text(encoding='utf-8')
    root = ET.fromstring(spherical_obstacle_sdf(template, radius))
    assert float(root.findtext('.//collision/geometry/sphere/radius')) == radius
    assert float(root.findtext('.//visual/geometry/sphere/radius')) == radius
    assert TEMPLATE.read_text(encoding='utf-8') == template


@pytest.mark.parametrize('radius', (0, -0.1, float('nan'), float('inf')))
def test_invalid_radius_never_creates_a_different_visible_geometry(radius):
    with pytest.raises(ValueError):
        spherical_obstacle_sdf(TEMPLATE.read_text(encoding='utf-8'), radius)


def test_obstacle_model_publishes_individually_dated_model_poses():
    root = ET.fromstring(spherical_obstacle_sdf(TEMPLATE.read_text(encoding='utf-8'), 0.12))
    plugin = root.find('.//plugin')
    assert plugin is not None
    assert plugin.attrib['name'] == 'ignition::gazebo::systems::PosePublisher'
    assert plugin.findtext('publish_model_pose') == 'true'
    assert plugin.findtext('publish_nested_model_pose') == 'true'
    assert plugin.findtext('use_pose_vector_msg') == 'true'
    assert plugin.findtext('static_publisher') == 'false'


def test_topic_source_does_not_create_an_unrelated_gazebo_sphere():
    module = load_launch('jaco_gazebo.launch.py')
    context = LaunchContext()
    context.launch_configurations['obstacle_source_mode'] = 'topic'
    # No Gazebo resource access is needed: the source has no scene obstacle.
    assert module._spawn_scene_obstacle(context, model_file='unused') == []


@pytest.mark.parametrize('filename', ('jaco_gazebo.launch.py', 'thesis_system.launch.py'))
def test_launch_defaults_follow_the_visible_gazebo_obstacle(filename):
    module = load_launch(filename)
    declaration = next(
        action for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument) and action.name == 'obstacle_source_mode'
    )
    context = LaunchContext()
    assert perform_substitutions(context, declaration.default_value) == 'gazebo'
    assert set(declaration.choices) == {'gazebo', 'topic'}
    context.launch_configurations['obstacle_source_mode'] = 'unrecognized'
    with pytest.raises(RuntimeError, match='is not valid'):
        declaration.execute(context)


@pytest.mark.parametrize(
    'filename', ('jaco_gazebo.launch.py', 'thesis_system.launch.py')
)
def test_jaco_launch_defaults_to_cartesian_ready_pose(filename):
    module = load_launch(filename)
    declaration = next(
        action for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'initial_pose'
    )
    context = LaunchContext()
    assert perform_substitutions(context, declaration.default_value) == 'z_ready'


@pytest.mark.parametrize('source, enabled, expected', (
    ('gazebo', 'true', False),
    ('gazebo', 'false', False),
    ('topic', 'false', False),
    ('topic', 'true', True),
    ('topic', 'TRUE', True),
    ('topic', '1', True),
    ('topic', '0', False),
))
def test_fixed_demo_never_competes_with_visible_gazebo_source(source, enabled, expected):
    module = load_launch('thesis_system.launch.py')
    demo = next(
        action for action in module.generate_launch_description().entities
        if isinstance(action, Node) and action.node_executable == 'obstacle_demo_source'
    )
    context = LaunchContext()
    context.launch_configurations.update({
        'obstacle_source_mode': source, 'use_demo_obstacle': enabled,
    })
    assert demo.condition.evaluate(context) is expected


def test_gazebo_spawn_uses_configured_radius_and_world_coordinates():
    module = load_launch('jaco_gazebo.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'obstacle_source_mode': 'gazebo', 'obstacle_radius': '0.25',
        'obstacle_x': '1.7', 'obstacle_y': '-0.4', 'obstacle_z': '0.8',
    })
    actions = module._spawn_scene_obstacle(context, model_file=str(TEMPLATE))
    assert len(actions) == 1
    arguments = []
    # Resolve launch substitutions only. The create process is never started.
    for token in actions[0].cmd[1:]:
        value = perform_substitutions(context, token)
        if value == '--ros-args':
            break
        arguments.append(value)
    sdf = ET.fromstring(arguments[arguments.index('-string') + 1])
    assert float(sdf.findtext('.//collision/geometry/sphere/radius')) == 0.25
    assert float(sdf.findtext('.//visual/geometry/sphere/radius')) == 0.25
    assert arguments[arguments.index('-x') + 1] == '1.7'
    assert arguments[arguments.index('-y') + 1] == '-0.4'
    assert arguments[arguments.index('-z') + 1] == '0.8'


def test_world_mounts_jaco_on_a_240_by_160_cm_table():
    world = ET.parse(ROOT / 'worlds' / 'jaco2_workstation.sdf').getroot()
    tabletop = world.find(".//model[@name='jaco_work_table']//collision[@name='tabletop_collision']")
    assert tabletop is not None
    assert [float(value) for value in tabletop.findtext('pose').split()[:3]] == [0.0, 0.0, 0.73]
    assert tabletop.findtext('geometry/box/size') == '2.40 1.60 0.04'
    table = world.find(".//model[@name='jaco_work_table']")
    assert len([c for c in table.findall('.//collision') if c.get('name').startswith('leg_')]) == 4
    assert len([v for v in table.findall('.//visual') if v.get('name').startswith('leg_')]) == 4


def test_workstation_academic_lab_decor_is_visual_only():
    world = ET.parse(ROOT / 'worlds' / 'jaco2_workstation.sdf').getroot()
    room = world.find(".//model[@name='academic_lab_decor']")
    assert room is not None
    assert room.find('static').text == 'true'
    assert room.findall('.//collision') == []
    visuals = {visual.get('name') for visual in room.findall('.//visual')}
    assert {
        'back_wall',
        'left_wall',
        'right_wall',
        'whiteboard_panel',
        'lab_bench_top',
        'lab_cabinet_left',
        'window_landscape_sky',
        'window_landscape_sun',
        'window_landscape_hill_left',
        'window_landscape_field',
        'window_tree_trunk_left',
        'window_tree_crown_middle',
        'window_tree_trunk_right',
    } <= visuals
    assert not any('ceiling' in name for name in visuals)
    assert not any('sofa' in name for name in visuals)
    window_sky = next(
        visual for visual in room.findall('.//visual')
        if visual.get('name') == 'window_landscape_sky'
    )
    sky_color = [
        float(value)
        for value in window_sky.findtext('material/diffuse').split()
    ]
    assert sky_color[0] > 0.5 and sky_color[1] > sky_color[0]
    trees = {name for name in visuals if name.startswith('window_tree_')}
    assert len(trees) >= 9
    for cabinet_name in ('lab_cabinet_left', 'lab_cabinet_right', 'lab_bench_cabinet'):
        cabinet = next(
            visual for visual in room.findall('.//visual')
            if visual.get('name') == cabinet_name
        )
        wood_color = [
            float(value)
            for value in cabinet.findtext('material/diffuse').split()
        ]
        assert wood_color[0] > wood_color[2]
    ground_link = world.find(".//model[@name='ground_plane']/link[@name='ground_link']")
    tile_grout = [
        visual for visual in ground_link.findall('visual')
        if visual.get('name', '').startswith('tile_grout_')
    ]
    assert len(tile_grout) == 20
    assert len(ground_link.findall('collision')) == 1
    scene = world.find('.//world/scene')
    background = [float(value) for value in scene.findtext('background').split()]
    floor = world.find(
        ".//model[@name='ground_plane']//visual[@name='ground_visual']"
    )
    floor_color = [
        float(value)
        for value in floor.findtext('material/diffuse').split()
    ]
    back_wall = next(
        visual for visual in room.findall('.//visual')
        if visual.get('name') == 'back_wall'
    )
    wall_color = [
        float(value)
        for value in back_wall.findtext('material/diffuse').split()
    ]
    assert background[2] > background[0]
    assert floor_color[0] > floor_color[2] > 0.6
    assert max(floor_color[:3]) - min(floor_color[:3]) < 0.06
    assert wall_color[2] > wall_color[1] > wall_color[0]


def test_jaco_launch_enables_the_matching_preventive_table_geometry():
    module = load_launch('jaco_gazebo.launch.py')
    monitor = next(
        action for action in module.generate_launch_description().entities
        if isinstance(action, Node) and action.node_executable == 'proximity_monitor'
    )
    # Humble stores normalized parameters on the Node action.
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    context.launch_configurations.update(obstacle_source_mode='gazebo', obstacle_x='0.6',
                                         obstacle_y='0.0', obstacle_z='1.27', obstacle_radius='0.12')
    parameters, = evaluate_parameters(context, monitor._Node__parameters[-1:])
    assert parameters['table_enabled'] is True
    assert list(parameters['table_center']) == [0.0, 0.0, 0.73]
    assert list(parameters['table_size']) == [2.40, 1.60, 0.04]
    assert list(parameters['table_excluded_segments']) == [
        'base_to_shoulder',
        'shoulder_to_upper_arm',
    ]
