"""Tests for the adapter's independent global-safety watchdog boundary."""

from types import SimpleNamespace

import pytest

from thesis_simulation.global_safety_gate import GlobalSafetyGate
from thesis_simulation.simulation_command_adapter import (
    global_gate_decision,
    global_safety_evidence_from_message,
)


def _time(sec, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_status_message_is_converted_without_dropping_epoch_or_ttl():
    message = SimpleNamespace(
        session_id='hybrid-three-sim',
        decision_sequence=12,
        state='ALLOW',
        valid=True,
        stamp=_time(10, 250_000_000),
        valid_for=_time(0, 120_000_000),
    )

    evidence = global_safety_evidence_from_message(message)

    assert evidence.session_id == 'hybrid-three-sim'
    assert evidence.sequence == 12
    assert evidence.stamp_sec == 10.25
    assert evidence.ttl_sec == pytest.approx(0.12)


def test_legacy_mode_bypasses_watchdog_but_required_mode_starts_stopped():
    legacy = global_gate_decision(False, None, 10.0, 20.0)
    required = global_gate_decision(True, None, 10.0, 20.0)

    assert legacy.allowed
    assert legacy.reason_code == 'GLOBAL_SAFETY_NOT_REQUIRED'
    assert not required.allowed
    assert required.reason_code == 'GLOBAL_SAFETY_WATCHDOG_MISSING'


def test_required_mode_expires_on_ros_ttl_and_monotonic_receive_age():
    message = SimpleNamespace(
        session_id='hybrid-three-sim',
        decision_sequence=1,
        state='ALLOW',
        valid=True,
        stamp=_time(10),
        valid_for=_time(0, 120_000_000),
    )
    gate = GlobalSafetyGate('hybrid-three-sim', 0.20)
    gate.observe(global_safety_evidence_from_message(message), 20.0)

    assert global_gate_decision(True, gate, 10.1, 20.1).allowed
    assert not global_gate_decision(True, gate, 10.13, 20.1).allowed
    assert not global_gate_decision(True, gate, 10.1, 20.21).allowed
