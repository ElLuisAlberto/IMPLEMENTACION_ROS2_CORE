"""Tests for the static four-instance simulation configuration contract."""

from pathlib import Path

import pytest
import yaml

from thesis_simulation.multiarm_config import (
    CANONICAL_ROBOT_IDS,
    build_multiarm_config,
)


def valid_mapping():
    return {
        'reference_frame': 'world',
        'minimum_base_separation_m': 2.5,
        'instances': [
            {
                'robot_id': f'arm_0{index}',
                'namespace': f'arm_0{index}',
                'tf_prefix': f'arm_0{index}',
                'entity_name': f'arm_0{index}_jaco',
                'controller_manager_name': f'arm_0{index}_controller_manager',
                'arm_controller_name': f'arm_0{index}_arm_controller',
                'base_pose': [
                    -1.5 if index < 3 else 1.5,
                    -1.5 if index in (1, 3) else 1.5,
                    0.0, 0.0, 0.0, 0.0,
                ],
            }
            for index in range(1, 5)
        ],
    }


def test_valid_contract_has_four_sorted_instances_and_one_world():
    result = build_multiarm_config(valid_mapping())
    assert result.reference_frame == 'world'
    assert tuple(
        item.robot_id for item in result.instances
    ) == CANONICAL_ROBOT_IDS
    assert result.instance('arm_03').namespace == 'arm_03'


def test_top_level_yaml_key_is_accepted():
    assert len(build_multiarm_config({
        'multiarm_simulation': valid_mapping(),
    }).instances) == 4


@pytest.mark.parametrize('field', (
    'namespace', 'tf_prefix', 'entity_name', 'controller_manager_name',
    'arm_controller_name',
))
def test_duplicate_instance_resource_is_rejected(field):
    data = valid_mapping()
    data['instances'][1][field] = data['instances'][0][field]
    with pytest.raises(ValueError, match=field):
        build_multiarm_config(data)


def test_non_world_reference_is_rejected():
    data = valid_mapping()
    data['reference_frame'] = 'arm_01_world'
    with pytest.raises(ValueError, match='reference_frame'):
        build_multiarm_config(data)


def test_missing_or_invalid_robot_id_set_is_rejected():
    data = valid_mapping()
    data['instances'] = data['instances'][:-1]
    with pytest.raises(ValueError, match='exactly four'):
        build_multiarm_config(data)

    data = valid_mapping()
    data['instances'][3]['robot_id'] = 'arm_05'
    with pytest.raises(ValueError, match='robot_id'):
        build_multiarm_config(data)


def test_overlapping_base_poses_are_rejected():
    data = valid_mapping()
    data['instances'][1]['base_pose'] = data['instances'][0]['base_pose']
    with pytest.raises(ValueError, match='base poses'):
        build_multiarm_config(data)


def test_repository_yaml_satisfies_the_contract():
    path = Path(__file__).parents[1] / 'config' / 'multiarm_simulation.yaml'
    with path.open(encoding='utf-8') as stream:
        result = build_multiarm_config(yaml.safe_load(stream))
    assert result.minimum_base_separation_m == pytest.approx(2.5)
    assert [item.entity_name for item in result.instances] == [
        'arm_01_jaco', 'arm_02_jaco', 'arm_03_jaco', 'arm_04_jaco',
    ]
