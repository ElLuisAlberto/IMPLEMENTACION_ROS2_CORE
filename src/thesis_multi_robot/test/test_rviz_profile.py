"""Static contracts for the shared RViz profile."""

from pathlib import Path

import yaml


RVIZ_CONFIG = Path(__file__).parents[1] / 'rviz' / 'profile.rviz'


def _displays():
    config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding='utf-8'))
    return {
        display['Name']: display
        for display in config['Visualization Manager']['Displays']
    }


def test_robot_description_is_latched_for_deterministic_startup():
    """RViz must receive robot_description even when it starts late."""
    robot_model = _displays()['RobotModel']
    assert robot_model['Description Source'] == 'Topic'
    assert robot_model['Description Topic']['Value'] == 'robot_description'
    assert (
        robot_model['Description Topic']['Durability Policy']
        == 'Transient Local'
    )
    assert robot_model['Visual Enabled'] is True
    assert robot_model['Collision Enabled'] is False


def test_current_capsules_are_not_rendered_twice():
    """Keep current geometry in one display while retaining all predictions."""
    displays = _displays()
    assert displays['SafetyCapsules']['Namespaces']['current_capsules'] is True
    horizon = displays['HorizonVolume']['Namespaces']
    assert horizon['current_capsules'] is False
    assert horizon['predicted_volume'] is True
    assert horizon['horizon_endpoint_traces'] is True
    assert horizon['horizon_time_poses'] is True
    # Diagnostic text overlaps the robot in the default 3-D view; operators
    # can enable it explicitly in RViz when diagnosing prediction internals.
    assert horizon['horizon_labels'] is False
