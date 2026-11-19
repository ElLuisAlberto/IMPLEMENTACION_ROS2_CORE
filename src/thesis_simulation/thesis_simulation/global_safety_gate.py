"""Pure watchdog contract for the global multi-arm simulation gate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class GlobalSafetyEvidence:
    """Minimal immutable view of one global coordinator publication."""

    session_id: str
    sequence: int
    state: str
    valid: bool
    stamp_sec: float
    ttl_sec: float

    def __post_init__(self):
        session = str(self.session_id)
        if not session or session != session.strip():
            raise ValueError('session_id must be a non-empty trimmed string')
        sequence = int(self.sequence)
        if sequence < 0:
            raise ValueError('sequence must be non-negative')
        if self.state not in ('ALLOW', 'STOP'):
            raise ValueError('state must be ALLOW or STOP')
        stamp = float(self.stamp_sec)
        ttl = float(self.ttl_sec)
        if not math.isfinite(stamp) or stamp < 0.0:
            raise ValueError('stamp_sec must be finite and non-negative')
        if not math.isfinite(ttl) or ttl <= 0.0:
            raise ValueError('ttl_sec must be finite and positive')
        object.__setattr__(self, 'sequence', sequence)
        object.__setattr__(self, 'valid', bool(self.valid))
        object.__setattr__(self, 'stamp_sec', stamp)
        object.__setattr__(self, 'ttl_sec', ttl)


@dataclass(frozen=True)
class GateDecision:
    """Whether a candidate command may cross the sole output boundary."""

    allowed: bool
    reason_code: str


class GlobalSafetyGate:
    """Authorize relay only while fresh global ALLOW evidence exists."""

    def __init__(self, expected_session_id, maximum_receive_age_sec):
        session = str(expected_session_id)
        receive_limit = float(maximum_receive_age_sec)
        if not session or session != session.strip():
            raise ValueError(
                'expected_session_id must be a non-empty trimmed string'
            )
        if not math.isfinite(receive_limit) or receive_limit <= 0.0:
            raise ValueError(
                'maximum_receive_age_sec must be finite and positive'
            )
        self.expected_session_id = session
        self.maximum_receive_age_sec = receive_limit
        self._evidence: Optional[GlobalSafetyEvidence] = None
        self._received_monotonic_sec: Optional[float] = None
        self._last_sequence = -1
        self._rejected_reason: Optional[str] = None

    def reject(self, reason_code):
        """Poison authorization after malformed or replayed transport data."""
        reason = str(reason_code)
        if not reason or reason != reason.strip():
            raise ValueError('reason_code must be a non-empty trimmed string')
        self._rejected_reason = reason

    def observe(self, evidence, received_monotonic_sec):
        """Accept newer status from the configured simulation epoch."""
        if not isinstance(evidence, GlobalSafetyEvidence):
            raise ValueError('evidence must be GlobalSafetyEvidence')
        received = float(received_monotonic_sec)
        if not math.isfinite(received) or received < 0.0:
            raise ValueError(
                'received_monotonic_sec must be finite and non-negative'
            )
        if evidence.session_id != self.expected_session_id:
            self.reject('SESSION_MISMATCH')
            return False
        if evidence.sequence <= self._last_sequence:
            self.reject('STATUS_SEQUENCE_REPLAY')
            return False
        self._evidence = evidence
        self._received_monotonic_sec = received
        self._last_sequence = evidence.sequence
        self._rejected_reason = None
        return True

    def decide(self, now_sec, monotonic_now_sec):
        """Return fail-closed relay authorization for the current clocks."""
        now = float(now_sec)
        monotonic_now = float(monotonic_now_sec)
        if not all(math.isfinite(value) and value >= 0.0
                   for value in (now, monotonic_now)):
            raise ValueError('gate clocks must be finite and non-negative')
        if self._rejected_reason is not None:
            return GateDecision(False, self._rejected_reason)
        if self._evidence is None or self._received_monotonic_sec is None:
            return GateDecision(False, 'GLOBAL_STATUS_MISSING')
        receive_age = monotonic_now - self._received_monotonic_sec
        if receive_age < 0.0:
            return GateDecision(False, 'STATUS_RECEIVE_TIMESTAMP_FUTURE')
        if receive_age > self.maximum_receive_age_sec:
            return GateDecision(False, 'GLOBAL_STATUS_RECEIVE_STALE')
        if self._evidence.stamp_sec > now:
            return GateDecision(False, 'GLOBAL_STATUS_TIMESTAMP_FUTURE')
        if now > self._evidence.stamp_sec + self._evidence.ttl_sec:
            return GateDecision(False, 'GLOBAL_STATUS_EXPIRED')
        if not self._evidence.valid:
            return GateDecision(False, 'GLOBAL_STATUS_INVALID')
        if self._evidence.state != 'ALLOW':
            return GateDecision(False, 'GLOBAL_SAFETY_STOP')
        return GateDecision(True, 'GLOBAL_SAFETY_ALLOW')
