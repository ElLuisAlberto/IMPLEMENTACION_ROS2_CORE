"""Distinguish missing geometry from measured proximity without changing STOP."""

import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from test_gui_state import FakeNode, execution  # noqa: E402
from thesis_ui.joint_control_gui import JointControlWindow, JointGuiNode  # noqa: E402


def proximity(clearance=0.0, segment='upper_arm_to_forearm', valid=True):
    return SimpleNamespace(
        minimum_clearance=clearance, limiting_segment=segment,
        geometry_valid=valid, state='STOP',
    )


def control(command_id, **changes):
    values = dict(
        command_id=command_id, state='STOP', speed_scale=0.0,
        minimum_clearance=0.0, minimum_time_from_now=0.0,
        minimum_sample_index=0, minimum_sample_count=21,
        limiting_segment='upper_arm_to_forearm',
    )
    values.update(changes)
    return SimpleNamespace(**values)


def test_display_freshness_rejects_paused_clock_and_old_source_stamp():
    node = SimpleNamespace(
        display_clock_ns=None, display_clock_changed_at=0.0,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
        get_parameter=lambda name: SimpleNamespace(value=True),
    )
    stamp = SimpleNamespace(sec=10, nanosec=0)
    with patch('thesis_ui.joint_control_gui.time.monotonic', return_value=1.0):
        assert JointGuiNode.display_sample_is_fresh(node, 1.0, stamp)
    with patch('thesis_ui.joint_control_gui.time.monotonic', return_value=2.1):
        assert not JointGuiNode.display_sample_is_fresh(node, 2.1, stamp)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=12_000_000_000))
    with patch('thesis_ui.joint_control_gui.time.monotonic', return_value=2.2):
        assert not JointGuiNode.display_sample_is_fresh(node, 2.2, stamp)
        assert JointGuiNode.display_sample_is_fresh(
            node, 2.2, SimpleNamespace(sec=12, nanosec=0)
        )


class ProximityDisplayTests(unittest.TestCase):
    """Use the actual rendering path and retain message/control identity."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.node = FakeNode()
        self.window = JointControlWindow(self.node)
        for timer in (
            self.window.ros_timer, self.window.ui_timer,
            self.window.preview_timer, self.window.jog_timer,
        ):
            timer.stop()

    def tearDown(self):
        self.window.deleteLater()

    def actual(self, message):
        self.node.last_proximity = message
        self.node.proximity_received_at = time.monotonic()
        self.window.refresh_runtime_metrics()
        return self.window.metric_labels['Distancia actual'].text()

    def test_tf_failure_is_not_displayed_as_measured_zero_or_changed_to_allow(self):
        message = proximity(segment='TF_UNAVAILABLE')
        self.assertEqual(self.actual(message), 'Sin geometría válida (TF)')
        self.assertIs(self.node.last_proximity, message)
        self.assertEqual(message.state, 'STOP')
        self.assertEqual(message.minimum_clearance, 0.0)

    def test_invalid_geometry_and_nonfinite_distances_are_not_measurements(self):
        for message in (
            proximity(valid=False), proximity(float('nan')),
            proximity(float('inf')), proximity(float('-inf')),
        ):
            with self.subTest(message=message):
                self.assertEqual(self.actual(message), 'Sin geometría válida (datos)')

    def test_missing_obstacle_stop_is_not_contact_in_workspace_or_legacy(self):
        for layout in ('workspace', 'legacy'):
            with self.subTest(layout=layout):
                node = FakeNode()
                node.ui_layout = layout
                message = proximity(segment='OBSTACLE_UNAVAILABLE', valid=False)
                message.reference_frame = 'world'
                message.obstacle_radius = 0.0
                node.last_proximity = message
                node.proximity_received_at = time.monotonic()
                window = JointControlWindow(node)
                for timer in (
                    window.ros_timer, window.ui_timer,
                    window.preview_timer, window.jog_timer,
                ):
                    timer.stop()
                window.refresh_runtime_metrics()
                self.assertEqual(
                    window.metric_labels['Distancia actual'].text(),
                    'Sin geometría válida (datos)',
                )
                window.is_hardware_mode = True
                self.assertFalse(window.protective_recovery_available())
                self.assertIs(node.last_proximity, message)
                self.assertEqual(message.state, 'STOP')
                self.assertEqual(node.candidate_publisher.messages, [])
                window.deleteLater()

    def test_real_contact_overlap_and_clearance_are_still_numeric(self):
        for value, expected in ((0.0, '0.000 m'), (-0.03, '-0.030 m'), (0.4, '0.400 m')):
            with self.subTest(value=value):
                self.assertEqual(self.actual(proximity(value)), expected)

    def test_stale_or_missing_reception_is_not_reported_as_current(self):
        self.actual(proximity(0.4))
        for stamp in (time.monotonic() - 2.0, None):
            self.node.proximity_received_at = stamp
            self.window.refresh_runtime_metrics()
            self.assertEqual(
                self.window.metric_labels['Distancia actual'].text(), 'Sin dato reciente'
            )
            self.assertIn('SIN DATOS', self.window.proximity_state_label.text())

    def test_shared_status_shows_live_and_stale_proximity_in_all_views(self):
        self.window.resize(1920, 1080)
        self.window.show()
        self.window.workspace_navigation.setCurrentRow(
            self.window.workspace_page_ids.index('joints'))
        self.node.last_proximity = proximity(0.4)
        self.node.proximity_received_at = time.monotonic()
        self.window.refresh_ui()
        summary = self.window.manual_metric_summary
        diagnostic = self.window.manual_diagnostic_summary
        for index in (0, 1, 2):
            self.window.manual_subviews.setCurrentIndex(index)
            self.app.processEvents()
            self.assertTrue(summary.isVisible())
            self.assertTrue(diagnostic.isVisible())
            self.assertIn('0.400 m', summary.text())
            self.assertIn('STOP', diagnostic.text())
            self.assertFalse(self.window.manual_subviews.isAncestorOf(summary))
            self.assertFalse(self.window.manual_subviews.isAncestorOf(diagnostic))
        self.node.proximity_received_at = time.monotonic() - 2.0
        self.window.refresh_ui()
        self.assertIn('Sin dato reciente', summary.text())
        self.assertIn('SIN DATOS', diagnostic.text())

    def test_shared_connection_indicators_follow_live_colors(self):
        name = next(iter(self.window.manual_connection_indicators))
        mirror = self.window.manual_connection_indicators[name]
        for connected, color, symbol in (
            (True, '#166534', '●'), (False, '#b91c1c', '○')
        ):
            self.node.connection_status = lambda: {name: connected}
            self.window.refresh_ui()
            self.assertIn(color, mirror.styleSheet())
            self.assertTrue(mirror.text().startswith(symbol))
            self.assertEqual(mirror.text(), self.window.connection_indicators[name].text())

    def test_stop_is_visible_even_when_distance_is_unknown(self):
        self.actual(proximity(segment='TF_UNAVAILABLE'))
        self.assertIn('STOP', self.window.proximity_state_label.text())
        self.assertIn('Sin geometría válida', self.window.proximity_state_label.text())

    def test_jog_reference_requires_current_command_and_fresh_arrival(self):
        self.window.jog_command_id = 'jog-test-stream'
        self.node.last_sent_jog = SimpleNamespace(
            command_id='jog-old', duration=SimpleNamespace(sec=0, nanosec=100000000),
            joint_names=[self.window.joint_names[0]], positions=[0.2],
        )
        self.node.sent_jog_received_at = time.monotonic()
        self.window.refresh_runtime_metrics()
        self.assertIn('sin dato reciente', self.window.motion_observation_label.text())
        self.node.last_sent_jog.command_id = 'jog-test-stream'
        self.window.refresh_runtime_metrics()
        self.assertIn('0.10 s', self.window.motion_observation_label.text())
        self.node.sent_jog_received_at -= 2.0
        self.window.refresh_runtime_metrics()
        self.assertIn('sin dato reciente', self.window.motion_observation_label.text())

    def test_projection_uses_current_joint_jog_stream_and_rejects_other_ids(self):
        self.window.jog_mode_button.blockSignals(True)
        self.window.jog_mode_button.setChecked(True)
        self.window.jog_mode_button.blockSignals(False)
        self.window.jog_command_id = 'jog-test-stream'
        self.node.last_execution_control = control('jog-test-stream', state='REDUCTION')
        self.node.control_received_at = time.monotonic()
        self.window.refresh_runtime_metrics()
        self.assertEqual(self.window.metric_labels['Mínimo proyectado'].text(), '0.000 m')
        self.assertIn('REDUCTION', self.window.metric_labels['Decisión preventiva'].text())
        for wrong_id in ('jog-old-stream', 'jog_continuous'):
            self.node.last_execution_control = control(wrong_id)
            self.window.refresh_runtime_metrics()
            self.assertEqual(
                self.window.metric_labels['Mínimo proyectado'].text(), 'Sin evaluación activa'
            )

    def test_projection_sentinels_are_distinguished_from_contact_and_keep_stop(self):
        self.node.execution_callback(execution(status='ACCEPTED'))
        for changes in (
            dict(minimum_sample_count=0),
            dict(limiting_segment='TF_UNAVAILABLE'),
            dict(minimum_clearance=float('nan')),
            dict(minimum_time_from_now=-1.0),
        ):
            with self.subTest(changes=changes):
                message = control('gui_test', **changes)
                self.node.last_execution_control = message
                self.node.control_received_at = time.monotonic()
                self.window.refresh_runtime_metrics()
                self.assertEqual(
                    self.window.metric_labels['Mínimo proyectado'].text(),
                    'Sin evaluación geométrica',
                )
                self.assertEqual(
                    self.window.metric_labels['Instante del mínimo'].text(), 'Sin muestra válida'
                )
                self.assertIn('STOP', self.window.metric_labels['Decisión preventiva'].text())
                self.assertEqual(message.state, 'STOP')

    def test_banner_identifies_visible_and_topic_obstacles_without_affecting_controls(self):
        for source, expected in (
            ('gazebo', 'Obstáculo: esfera visible de Gazebo'),
            ('topic', 'Obstáculo: estimación recibida por tópico (puede ser virtual)'),
        ):
            with self.subTest(source=source):
                node = FakeNode()
                node.obstacle_source_mode = source
                window = JointControlWindow(node)
                for timer in (
                    window.ros_timer, window.ui_timer,
                    window.preview_timer, window.jog_timer,
                ):
                    timer.stop()
                self.assertIn(expected, window.workspace_banner.text())
                self.assertEqual(node.candidate_publisher.messages, [])
                window.deleteLater()

    def test_missing_or_invalid_geometry_never_enables_protective_recovery(self):
        # This changes only a fake window flag, never contacts physical hardware.
        self.window.is_hardware_mode = True
        message = proximity(clearance=0.049)
        message.reference_frame = 'world'
        message.obstacle_radius = 0.12
        self.node.last_proximity = message
        self.node.proximity_received_at = time.monotonic()
        self.assertTrue(self.window.protective_recovery_available())
        for changes in (
            dict(geometry_valid=False),
            dict(limiting_segment='TF_UNAVAILABLE'),
            dict(limiting_segment='OBSTACLE_UNAVAILABLE'),
            dict(obstacle_radius=0.0),
            dict(obstacle_radius=-0.12),
        ):
            with self.subTest(changes=changes):
                previous = {key: getattr(message, key) for key in changes}
                for key, value in changes.items():
                    setattr(message, key, value)
                self.assertFalse(self.window.protective_recovery_available())
                for key, value in previous.items():
                    setattr(message, key, value)
        self.assertTrue(self.window.protective_recovery_available())


if __name__ == '__main__':
    unittest.main()
