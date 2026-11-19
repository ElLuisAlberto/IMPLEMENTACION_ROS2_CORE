"""
Bind four isolated simulation instances to approved safety geometry.

This is a launch-independent contract. It prevents a future four-arm launch
from starting the coordinator with anonymous or mixed geometry revisions.
"""

from __future__ import annotations

from typing import Dict, Mapping

from thesis_core.geometry_profile import (
    GeometryProfile,
    legacy_jaco_geometry_profile,
)
from thesis_core.multiarm_coordination import GeometryExpectation
from thesis_simulation.multiarm_config import (
    CANONICAL_ROBOT_IDS,
    MultiArmSimulationConfig,
)


def legacy_geometry_profiles(
    configuration: MultiArmSimulationConfig,
) -> Mapping[str, GeometryProfile]:
    """
    Build one explicit legacy geometry profile for every simulation arm.

    Each profile receives the instance entity name as its TF prefix. The
    future gripper profile will replace this factory entry per arm without
    changing coordinator identity checks.
    """
    if not isinstance(configuration, MultiArmSimulationConfig):
        raise ValueError('configuration must be a MultiArmSimulationConfig')
    profiles: Dict[str, GeometryProfile] = {}
    for instance in configuration.instances:
        profile = legacy_jaco_geometry_profile(
            robot_id=instance.robot_id,
            frame_prefix=instance.entity_name,
        )
        profiles[instance.robot_id] = profile
    if tuple(sorted(profiles)) != CANONICAL_ROBOT_IDS:
        raise ValueError('configuration must define the canonical four robots')
    return profiles


def geometry_expectations(
    profiles: Mapping[str, GeometryProfile],
) -> Mapping[str, GeometryExpectation]:
    """Return the provenance map required by the global coordinator."""
    if set(profiles) != set(CANONICAL_ROBOT_IDS):
        raise ValueError('profiles must define exactly arm_01 through arm_04')
    expectations: Dict[str, GeometryExpectation] = {}
    for robot_id in CANONICAL_ROBOT_IDS:
        profile = profiles[robot_id]
        if not isinstance(profile, GeometryProfile) or profile.robot_id != robot_id:
            raise ValueError(f'profile for {robot_id} has invalid identity')
        expectations[robot_id] = GeometryExpectation(
            profile.profile_id,
            profile.geometry_version,
            profile.asset_hash,
        )
    return expectations
