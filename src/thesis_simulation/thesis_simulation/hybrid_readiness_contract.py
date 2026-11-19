"""Pure readiness rules for the offline four-identity campaign."""

from __future__ import annotations

import math
from typing import Mapping, Tuple


CANONICAL_ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')
GAZEBO_ROBOT_IDS = CANONICAL_ROBOT_IDS[1:]


def evaluate_hybrid_readiness(
    received_monotonic: Mapping[str, float],
    controller_states: Mapping[str, Mapping[str, str]],
    *,
    monotonic_now_sec: float,
    freshness_timeout_sec: float,
    global_status_valid: bool,
    simulation_output_enabled: bool,
) -> Tuple[bool, Tuple[str, ...]]:
    """Return readiness and stable machine-readable failure reasons."""
    now = float(monotonic_now_sec)
    timeout = float(freshness_timeout_sec)
    if (
        not math.isfinite(now)
        or now < 0.0
        or not math.isfinite(timeout)
        or timeout <= 0.0
    ):
        raise ValueError('readiness clocks must be finite and bounded')
    reasons = []
    required_samples = ('clock', 'global_status') + tuple(
        f'{robot_id}_joint_state' for robot_id in CANONICAL_ROBOT_IDS
    ) + tuple(
        f'{robot_id}_controller_state' for robot_id in GAZEBO_ROBOT_IDS
    )
    for sample in required_samples:
        received = received_monotonic.get(sample)
        if received is None:
            reasons.append(f'{sample.upper()}_MISSING')
            continue
        age = now - float(received)
        if not math.isfinite(age) or age < 0.0 or age > timeout:
            reasons.append(f'{sample.upper()}_STALE')
    if not global_status_valid:
        reasons.append('GLOBAL_STATUS_INVALID')

    desired_arm_state = (
        'active' if simulation_output_enabled else 'inactive'
    )
    for robot_id in GAZEBO_ROBOT_IDS:
        states = controller_states.get(robot_id)
        if states is None:
            reasons.append(f'{robot_id.upper()}_CONTROLLERS_MISSING')
            continue
        if states.get(f'{robot_id}_joint_state_broadcaster') != 'active':
            reasons.append(f'{robot_id.upper()}_BROADCASTER_NOT_ACTIVE')
        if states.get(f'{robot_id}_arm_controller') != desired_arm_state:
            reasons.append(
                f'{robot_id.upper()}_ARM_CONTROLLER_NOT_{desired_arm_state.upper()}'
            )
    unique_reasons = tuple(dict.fromkeys(reasons))
    return not unique_reasons, unique_reasons
