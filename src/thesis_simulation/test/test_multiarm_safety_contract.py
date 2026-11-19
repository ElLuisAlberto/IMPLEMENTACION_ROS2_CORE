"""Tests binding four instance names to their safety profile provenance."""

from pathlib import Path

import pytest
import yaml

from thesis_simulation.multiarm_config import build_multiarm_config
from thesis_simulation.multiarm_safety_contract import (
    geometry_expectations,
    legacy_geometry_profiles,
)


def configuration():
    path = Path(__file__).parents[1] / 'config' / 'multiarm_simulation.yaml'
    with path.open(encoding='utf-8') as stream:
        return build_multiarm_config(yaml.safe_load(stream))


def test_every_instance_gets_one_prefixed_legacy_profile():
    profiles = legacy_geometry_profiles(configuration())

    assert tuple(profiles) == ('arm_01', 'arm_02', 'arm_03', 'arm_04')
    assert profiles['arm_03'].mount_frame == 'arm_03_jaco_link_6'
    assert profiles['arm_03'].tcp_frame == 'arm_03_jaco_end_effector'
    assert profiles['arm_03'].profile_id == 'jaco2_legacy'


def test_expectations_bind_identity_and_version_for_each_robot():
    profiles = legacy_geometry_profiles(configuration())
    expectations = geometry_expectations(profiles)

    assert set(expectations) == set(profiles)
    assert expectations['arm_04'].profile_id == 'jaco2_legacy'
    assert len(expectations['arm_04'].asset_hash) == 64


def test_incomplete_or_wrong_identity_profile_map_is_rejected():
    profiles = dict(legacy_geometry_profiles(configuration()))
    profiles.pop('arm_04')
    with pytest.raises(ValueError, match='exactly'):
        geometry_expectations(profiles)

    profiles = dict(legacy_geometry_profiles(configuration()))
    profiles['arm_01'] = profiles['arm_02']
    with pytest.raises(ValueError, match='invalid identity'):
        geometry_expectations(profiles)
