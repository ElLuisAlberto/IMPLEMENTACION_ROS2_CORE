"""Tests for aggregate readiness of the offline hybrid topology."""

from thesis_simulation.hybrid_readiness_contract import (
    evaluate_hybrid_readiness,
)


def _samples(now=10.0):
    values = {'clock': now, 'global_status': now}
    values.update({
        f'arm_0{index}_joint_state': now for index in range(1, 5)
    })
    values.update({
        f'arm_0{index}_controller_state': now for index in range(2, 5)
    })
    return values


def _controllers(arm_state='inactive'):
    return {
        f'arm_0{index}': {
            f'arm_0{index}_joint_state_broadcaster': 'active',
            f'arm_0{index}_arm_controller': arm_state,
        }
        for index in range(2, 5)
    }


def test_observation_mode_ready_with_loaded_inactive_arm_controllers():
    ready, reasons = evaluate_hybrid_readiness(
        _samples(),
        _controllers(),
        monotonic_now_sec=10.1,
        freshness_timeout_sec=0.75,
        global_status_valid=True,
        simulation_output_enabled=False,
    )

    assert ready
    assert reasons == ()


def test_output_mode_requires_all_three_arm_controllers_active():
    ready, reasons = evaluate_hybrid_readiness(
        _samples(),
        _controllers(),
        monotonic_now_sec=10.1,
        freshness_timeout_sec=0.75,
        global_status_valid=True,
        simulation_output_enabled=True,
    )

    assert not ready
    assert len([reason for reason in reasons if 'NOT_ACTIVE' in reason]) == 3


def test_missing_stale_invalid_or_partial_evidence_fails_closed():
    samples = _samples()
    samples.pop('arm_04_joint_state')
    samples['arm_02_joint_state'] = 8.0
    controllers = _controllers()
    controllers.pop('arm_03')

    ready, reasons = evaluate_hybrid_readiness(
        samples,
        controllers,
        monotonic_now_sec=10.1,
        freshness_timeout_sec=0.75,
        global_status_valid=False,
        simulation_output_enabled=False,
    )

    assert not ready
    assert 'ARM_02_JOINT_STATE_STALE' in reasons
    assert 'ARM_04_JOINT_STATE_MISSING' in reasons
    assert 'GLOBAL_STATUS_INVALID' in reasons
    assert 'ARM_03_CONTROLLERS_MISSING' in reasons
