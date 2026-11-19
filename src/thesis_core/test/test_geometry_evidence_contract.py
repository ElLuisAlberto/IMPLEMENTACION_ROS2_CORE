"""Fail-closed geometry-evidence tests at the proximity/supervisor boundary."""

from types import SimpleNamespace

from thesis_interfaces.msg import ProximityStatus

from thesis_core.proximity_monitor_node import geometry_status_is_valid
from thesis_core.safety_supervisor_node import proximity_geometry_is_valid


def test_explicit_invalid_geometry_is_rejected_by_both_boundaries():
    message = SimpleNamespace(geometry_valid=False)

    assert not geometry_status_is_valid(message)
    assert not proximity_geometry_is_valid(message)


def test_explicit_valid_geometry_is_accepted_by_both_boundaries():
    message = SimpleNamespace(geometry_valid=True)

    assert geometry_status_is_valid(message)
    assert proximity_geometry_is_valid(message)


def test_legacy_test_double_is_supported_without_weakening_ros_messages():
    message = SimpleNamespace()

    assert geometry_status_is_valid(message)
    assert proximity_geometry_is_valid(message)


def test_generated_message_defaults_to_fail_closed_until_marked_valid():
    message = ProximityStatus()

    assert not geometry_status_is_valid(message)
    assert not proximity_geometry_is_valid(message)
    message.geometry_valid = True
    assert geometry_status_is_valid(message)
    assert proximity_geometry_is_valid(message)
