"""Pure configuration contract for four isolated simulated manipulators."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any, Mapping, Sequence, Tuple


CANONICAL_ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')
CANONICAL_REFERENCE_FRAME = 'world'
_IDENTIFIER = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


@dataclass(frozen=True)
class RobotInstanceConfig:
    """Names and fixed base pose for one independent robot instance."""

    robot_id: str
    namespace: str
    tf_prefix: str
    entity_name: str
    controller_manager_name: str
    arm_controller_name: str
    base_pose: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class MultiArmSimulationConfig:
    """Validated four-robot configuration sharing one world frame."""

    reference_frame: str
    minimum_base_separation_m: float
    instances: Tuple[RobotInstanceConfig, ...]

    def instance(self, robot_id: str) -> RobotInstanceConfig:
        """Return an instance by its stable logical robot identifier."""
        for item in self.instances:
            if item.robot_id == robot_id:
                return item
        raise KeyError(f'unknown robot_id: {robot_id!r}')


def _nonempty_identifier(value: Any, field_name: str) -> str:
    """Return a ROS-safe relative identifier without a leading slash."""
    result = str(value)
    if not _IDENTIFIER.fullmatch(result):
        raise ValueError(
            f'{field_name} must be a non-empty relative identifier: {result!r}'
        )
    return result


def _finite_pose(value: Any, robot_id: str) -> Tuple[float, ...]:
    """Validate a fixed XYZ/RPY base pose expressed in the common world."""
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(
            f'{robot_id}.base_pose must contain six numeric values'
        )
    if len(value) != 6:
        raise ValueError(f'{robot_id}.base_pose must contain six values')
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f'{robot_id}.base_pose must contain finite values')
    return result


def _instance_from_mapping(value: Any) -> RobotInstanceConfig:
    """Build one immutable instance record from YAML-compatible input."""
    if not isinstance(value, Mapping):
        raise ValueError('every multiarm instance must be a mapping')
    robot_id = _nonempty_identifier(value.get('robot_id'), 'robot_id')
    return RobotInstanceConfig(
        robot_id=robot_id,
        namespace=_nonempty_identifier(value.get('namespace'), 'namespace'),
        tf_prefix=_nonempty_identifier(value.get('tf_prefix'), 'tf_prefix'),
        entity_name=_nonempty_identifier(
            value.get('entity_name'), 'entity_name'
        ),
        controller_manager_name=_nonempty_identifier(
            value.get('controller_manager_name'), 'controller_manager_name'
        ),
        arm_controller_name=_nonempty_identifier(
            value.get('arm_controller_name'), 'arm_controller_name'
        ),
        base_pose=_finite_pose(value.get('base_pose'), robot_id),
    )


def _unique(values: Sequence[str], label: str) -> None:
    """Reject aliases that would cause ROS, TF, or Gazebo crosstalk."""
    if len(set(values)) != len(values):
        raise ValueError(f'{label} must be unique across four robot instances')


def _validate_separation(
    instances: Sequence[RobotInstanceConfig], minimum_separation: float
) -> None:
    """Ensure bases are not co-located in the shared horizontal workspace."""
    for index, first in enumerate(instances):
        for second in instances[index + 1:]:
            separation = math.dist(first.base_pose[:2], second.base_pose[:2])
            if separation < minimum_separation:
                raise ValueError(
                    f'base poses {first.robot_id!r} and {second.robot_id!r} '
                    f'are {separation:.6f} m apart; minimum is '
                    f'{minimum_separation:.6f} m'
                )


def build_multiarm_config(
    value: Mapping[str, Any],
) -> MultiArmSimulationConfig:
    """
    Validate and build the exact four-instance simulation contract.

    ``value`` may be the contents below the ``multiarm_simulation`` YAML key or
    a mapping containing that single top-level key.  The function deliberately
    accepts no implicit defaults: an omitted name could silently merge two ROS
    entities and invalidate isolation tests.
    """
    if not isinstance(value, Mapping):
        raise ValueError('multiarm configuration must be a mapping')
    raw = value.get('multiarm_simulation', value)
    if not isinstance(raw, Mapping):
        raise ValueError('multiarm_simulation must be a mapping')

    reference_frame = str(raw.get('reference_frame'))
    if reference_frame != CANONICAL_REFERENCE_FRAME:
        raise ValueError(
            f'reference_frame must be {CANONICAL_REFERENCE_FRAME!r}; '
            'all four robots must share the same world frame'
        )

    try:
        minimum_separation = float(raw.get('minimum_base_separation_m'))
    except (TypeError, ValueError) as exception:
        raise ValueError(
            'minimum_base_separation_m must be numeric'
        ) from exception
    if not math.isfinite(minimum_separation) or minimum_separation <= 0.0:
        raise ValueError(
            'minimum_base_separation_m must be finite and positive'
        )

    raw_instances = raw.get('instances')
    if isinstance(raw_instances, (str, bytes)) or not isinstance(
            raw_instances, Sequence):
        raise ValueError('instances must be a sequence of four mappings')
    instances = tuple(_instance_from_mapping(item) for item in raw_instances)
    if len(instances) != len(CANONICAL_ROBOT_IDS):
        raise ValueError('exactly four robot instances are required')

    robot_ids = tuple(item.robot_id for item in instances)
    if set(robot_ids) != set(CANONICAL_ROBOT_IDS):
        raise ValueError(
            'robot_id values must be exactly arm_01, arm_02, arm_03, arm_04'
        )
    _unique(robot_ids, 'robot_id')
    _unique(tuple(item.namespace for item in instances), 'namespace')
    _unique(tuple(item.tf_prefix for item in instances), 'tf_prefix')
    _unique(tuple(item.entity_name for item in instances), 'entity_name')
    _unique(
        tuple(item.controller_manager_name for item in instances),
        'controller_manager_name',
    )
    _unique(
        tuple(item.arm_controller_name for item in instances),
        'arm_controller_name',
    )
    _validate_separation(instances, minimum_separation)

    return MultiArmSimulationConfig(
        reference_frame=reference_frame,
        minimum_base_separation_m=minimum_separation,
        instances=tuple(sorted(instances, key=lambda item: item.robot_id)),
    )
