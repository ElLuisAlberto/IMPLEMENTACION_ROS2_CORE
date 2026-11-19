"""Fail-closed tests for the single-authority multi-arm command gate."""

from thesis_simulation.global_safety_gate import (
    GlobalSafetyEvidence,
    GlobalSafetyGate,
)


def evidence(
    *, sequence=1, state='ALLOW', valid=True, session='hybrid',
    stamp=10.0, ttl=0.2,
):
    return GlobalSafetyEvidence(
        session, sequence, state, valid, stamp, ttl
    )


def test_gate_starts_stopped_and_allows_only_fresh_valid_allow():
    gate = GlobalSafetyGate('hybrid', 0.3)
    assert gate.decide(10.0, 20.0).reason_code == 'GLOBAL_STATUS_MISSING'

    assert gate.observe(evidence(), 20.0)
    decision = gate.decide(10.1, 20.1)

    assert decision.allowed
    assert decision.reason_code == 'GLOBAL_SAFETY_ALLOW'


def test_stop_invalid_expired_future_and_receive_stale_never_relay():
    cases = (
        (evidence(state='STOP'), 10.1, 20.1, 'GLOBAL_SAFETY_STOP'),
        (evidence(valid=False), 10.1, 20.1, 'GLOBAL_STATUS_INVALID'),
        (evidence(ttl=0.05), 10.1, 20.1, 'GLOBAL_STATUS_EXPIRED'),
        (evidence(stamp=10.2), 10.1, 20.1,
         'GLOBAL_STATUS_TIMESTAMP_FUTURE'),
        (evidence(), 10.1, 20.4, 'GLOBAL_STATUS_RECEIVE_STALE'),
    )
    for status, now, monotonic_now, reason in cases:
        gate = GlobalSafetyGate('hybrid', 0.3)
        gate.observe(status, 20.0)
        decision = gate.decide(now, monotonic_now)
        assert not decision.allowed
        assert decision.reason_code == reason


def test_wrong_session_and_replayed_sequence_poison_until_new_status():
    gate = GlobalSafetyGate('hybrid', 0.3)
    assert not gate.observe(evidence(session='other'), 20.0)
    assert gate.decide(10.1, 20.1).reason_code == 'SESSION_MISMATCH'

    assert gate.observe(evidence(sequence=4), 20.1)
    assert gate.decide(10.1, 20.1).allowed
    assert not gate.observe(evidence(sequence=4), 20.2)
    assert gate.decide(10.1, 20.2).reason_code == 'STATUS_SEQUENCE_REPLAY'
    assert gate.observe(evidence(sequence=5), 20.21)
    assert gate.decide(10.1, 20.22).allowed


def test_monotonic_receive_clock_cannot_go_backwards():
    gate = GlobalSafetyGate('hybrid', 0.3)
    gate.observe(evidence(), 20.0)
    decision = gate.decide(10.1, 19.9)
    assert not decision.allowed
    assert decision.reason_code == 'STATUS_RECEIVE_TIMESTAMP_FUTURE'
