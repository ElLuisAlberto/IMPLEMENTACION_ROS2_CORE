"""Tests for the offline three-Gazebo hybrid scenario contract."""

from pathlib import Path
import xml.etree.ElementTree as ElementTree

import pytest
import yaml

from thesis_simulation.hybrid_scenario import (
    GAZEBO_ROBOT_IDS,
    build_hybrid_three_sim_scenario,
)


def scenario_mapping():
    path = Path(__file__).parents[1] / 'config' / 'hybrid_three_sim.yaml'
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def raw_instances():
    return scenario_mapping()['hybrid_three_sim']['instances']


def test_repository_scenario_has_three_gazebo_backends_and_one_synthetic_source():
    scenario = build_hybrid_three_sim_scenario(scenario_mapping())
    assert scenario.reference_frame == 'world'
    assert tuple(item.robot_id for item in scenario.instances) == (
        'arm_01', 'arm_02', 'arm_03', 'arm_04',
    )
    assert scenario.instance('arm_01').source == 'synthetic_state'
    assert scenario.instance('arm_01').output == 'none'
    assert scenario.instance('arm_01').entity_name is None
    assert tuple(item.robot_id for item in scenario.instances
                 if item.source == 'gazebo') == GAZEBO_ROBOT_IDS
    assert scenario.instance('arm_01').tool_mode == 'interface'
    assert all(scenario.instance(robot_id).tool_mode == 'synthetic_gripper'
               for robot_id in GAZEBO_ROBOT_IDS)


@pytest.mark.parametrize('field', (
    'entity_name', 'controller_manager_name', 'arm_controller_name',
))
def test_synthetic_identity_rejects_gazebo_resources(field):
    data = scenario_mapping()
    data['hybrid_three_sim']['instances'][0][field] = 'forbidden_resource'
    with pytest.raises(ValueError, match='synthetic_state'):
        build_hybrid_three_sim_scenario(data)


@pytest.mark.parametrize('field', ('source', 'output'))
def test_hardware_source_or_output_is_rejected(field):
    data = scenario_mapping()
    data['hybrid_three_sim']['instances'][1][field] = 'hardware'
    with pytest.raises(ValueError, match='arm_02'):
        build_hybrid_three_sim_scenario(data)


def test_duplicate_runtime_resource_is_rejected():
    data = scenario_mapping()
    instances = data['hybrid_three_sim']['instances']
    instances[2]['entity_name'] = instances[1]['entity_name']
    with pytest.raises(ValueError, match='entity_name'):
        build_hybrid_three_sim_scenario(data)


@pytest.mark.parametrize('field, invalid', (
    ('base_pose', [0.0] * 5),
    ('world_from_base', [0.0, 0.0, 0.0, 0.0, 0.0, float('nan')]),
))
def test_invalid_pose_is_rejected(field, invalid):
    data = scenario_mapping()
    data['hybrid_three_sim']['instances'][1][field] = invalid
    with pytest.raises(ValueError, match=field):
        build_hybrid_three_sim_scenario(data)


def test_base_pose_and_world_transform_must_match():
    data = scenario_mapping()
    data['hybrid_three_sim']['instances'][1]['world_from_base'][0] = 1.0
    with pytest.raises(ValueError, match='must match'):
        build_hybrid_three_sim_scenario(data)


def test_missing_logical_identity_is_rejected():
    data = scenario_mapping()
    data['hybrid_three_sim']['instances'].pop()
    with pytest.raises(ValueError, match='exactly four'):
        build_hybrid_three_sim_scenario(data)


def test_hybrid_world_reduces_clock_load_without_changing_shared_world():
    worlds = Path(__file__).parents[1] / 'worlds'

    hybrid = ElementTree.parse(
        worlds / 'hybrid_three_jaco_empty.sdf'
    ).getroot()
    shared = ElementTree.parse(worlds / 'jaco_empty.sdf').getroot()

    assert hybrid.findtext('.//max_step_size') == '0.005'
    assert hybrid.findtext('.//real_time_factor') == '1.0'
    assert shared.findtext('.//max_step_size') == '0.001'
