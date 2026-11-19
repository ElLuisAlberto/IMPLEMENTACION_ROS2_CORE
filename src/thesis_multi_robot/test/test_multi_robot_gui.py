"""Headless Qt contracts for the unified profile selector."""

import math
import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from thesis_multi_robot.multi_robot_gui import MultiRobotWindow  # noqa: E402


class _Backend:
    def __init__(self):
        self.robot_names = ('ur17g', 'p3020')
        self.calls = []
        self._profiles = {
            'ur17g': SimpleNamespace(
                display_name='UR17g-1300',
                joint_names=tuple(f'ur17g_joint_{index}' for index in range(6)),
                joint_limits=tuple((-3.0, 3.0, 1.0) for _ in range(6)),
            ),
            'p3020': SimpleNamespace(
                display_name='P3020',
                joint_names=tuple(f'p3020_joint_{index}' for index in range(5)),
                joint_limits=tuple((-3.0, 3.0, 1.0) for _ in range(5)),
            ),
        }

    def profile(self, name):
        return self._profiles[name]

    def snapshot(self, name):
        count = len(self._profiles[name].joint_names)
        return SimpleNamespace(
            positions=(0.0,) * count,
            state_fresh=True,
            proximity_fresh=True,
            proximity_state='ALLOW',
            clearance=1.0,
            decision_state='IDLE',
            decision_reason='Sin comando.',
            command_status='IDLE',
            maximum_target_error=None,
            tool_xyz=(0.5, 0.0, 0.8),
            tool_rpy=(0.0, 0.0, 0.0),
        )

    def send_joint_delta(self, name, index, delta, duration):
        self.calls.append((name, index, delta, duration))
        return 'gui-test'

    def send_joint_target(self, name, positions, duration):
        self.calls.append((name, tuple(positions), duration))
        return 'gui-test'

    def send_cartesian_target(self, *arguments):
        self.calls.append(arguments)
        return 'gui-test'


class _FreshTransitionBackend(_Backend):
    def __init__(self):
        super().__init__()
        self.robot_names = ('ur17g',)
        self.fresh = False

    def snapshot(self, name):
        count = len(self._profiles[name].joint_names)
        return SimpleNamespace(
            positions=tuple(0.1 * (index + 1) for index in range(count)),
            state_fresh=self.fresh,
            proximity_fresh=self.fresh,
            proximity_state='ALLOW' if self.fresh else 'STALE',
            clearance=1.0 if self.fresh else None,
            decision_state='IDLE',
            decision_reason='Sin comando.',
            command_status='IDLE',
            maximum_target_error=None,
            tool_xyz=(0.5, -0.1, 0.8) if self.fresh else None,
            tool_rpy=(0.1, 0.2, 0.3) if self.fresh else None,
        )


def test_selector_changes_dof_and_routes_only_selected_robot():
    application = QApplication.instance() or QApplication([])
    backend = _Backend()
    window = MultiRobotWindow(backend)
    try:
        window.spin_timer.stop()
        window.refresh_timer.stop()
        assert window.joint_table.rowCount() == 6
        window.robot_selector.setCurrentIndex(1)
        application.processEvents()
        assert window.selected_name == 'p3020'
        assert window.joint_table.rowCount() == 5
        window._send_delta(4, 1.0)
        assert backend.calls[-1][0] == 'p3020'
        assert backend.calls[-1][1] == 4
    finally:
        window.close()


def test_targets_initialize_only_after_first_fresh_state():
    application = QApplication.instance() or QApplication([])
    backend = _FreshTransitionBackend()
    window = MultiRobotWindow(backend)
    try:
        window.spin_timer.stop()
        window.refresh_timer.stop()
        assert not window._joint_target_initialized
        assert not window._cartesian_target_initialized
        assert not window.send_joints_button.isEnabled()
        assert not window.send_pose_button.isEnabled()

        backend.fresh = True
        window._refresh()
        application.processEvents()

        assert window._joint_target_initialized
        assert window._cartesian_target_initialized
        assert window.send_joints_button.isEnabled()
        assert window.send_pose_button.isEnabled()
        assert math.isclose(
            window.joint_targets[0].value(), math.degrees(0.1), abs_tol=0.001
        )
        assert math.isclose(window.cartesian_fields['X'].value(), 0.5)
        assert math.isclose(
            window.cartesian_fields['Roll'].value(),
            math.degrees(0.1),
            abs_tol=0.01,
        )
    finally:
        window.close()
