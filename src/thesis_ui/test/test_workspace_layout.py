"""Navigation regressions for the reorganized operator workspace."""

import os
import time
from types import MethodType
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import QEvent  # noqa: E402
from PyQt5.QtGui import QCloseEvent  # noqa: E402
from PyQt5.QtWidgets import QApplication, QPushButton  # noqa: E402

from test_gui_state import FakeNode, execution  # noqa: E402
from thesis_ui.joint_control_gui import (  # noqa: E402
    JointControlWindow, JointGuiNode,
)
from thesis_ui.workspace_layout import (  # noqa: E402
    detach_workspace_page, select_workspace_page,
)


class WorkspaceLayoutTests(unittest.TestCase):
    """Exercise navigation against real callbacks and capturing ROS ports."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.node = FakeNode()
        self.window = JointControlWindow(self.node)
        for timer in (
            self.window.ros_timer,
            self.window.ui_timer,
            self.window.preview_timer,
            self.window.jog_timer,
        ):
            timer.stop()

    def tearDown(self):
        self.window.deleteLater()

    def select(self, page_id):
        select_workspace_page(self.window, page_id)

    def test_navigation_does_not_publish_or_change_targets(self):
        targets = [item.value() for item in self.window.target_inputs]
        cartesian = [
            item.value() for item in self.window.cartesian_target_inputs
        ]
        for page_id in self.window.workspace_page_ids:
            self.select(page_id)
        self.assertEqual(
            targets, [item.value() for item in self.window.target_inputs]
        )
        self.assertEqual(
            cartesian,
            [item.value() for item in self.window.cartesian_target_inputs],
        )
        for publisher in (
            self.node.candidate_publisher,
            self.node.jog_intent_publisher,
            self.node.cartesian_candidate_publisher,
            self.node.cartesian_jog_publisher,
            self.node.cartesian_trajectory_publisher,
            self.node.cartesian_trajectory_cancel_publisher,
        ):
            self.assertEqual(publisher.messages, [])

    def test_same_action_widget_survives_navigation(self):
        button = self.window.send_button
        for page_id in ('joints', 'cartesian', 'history', 'joints'):
            self.select(page_id)
            self.assertIs(self.window.send_button, button)
        self.assertEqual(
            len([item for item in self.window.findChildren(QPushButton)
                 if item.objectName() == 'sendButton']),
            1,
        )

    def test_navigation_preserves_active_execution_lock(self):
        self.node.execution_callback(execution(status='ACCEPTED'))
        for page_id in self.window.workspace_page_ids:
            self.select(page_id)
            self.window.refresh_ui()
            self.assertFalse(self.window.send_button.isEnabled())
            self.assertFalse(self.window.cartesian_send_button.isEnabled())
            self.assertFalse(
                self.window.cartesian_trajectory_send_button.isEnabled()
            )

    def test_subview_change_stops_joint_and_cartesian_jog(self):
        self.select('joints')
        tabs = self.window.manual_subviews
        self.assertEqual(tabs.count(), 3)
        tabs.setCurrentIndex(1)
        self.window.jog_mode_button.setChecked(True)
        self.window.wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.begin_deliberate_jog_input()
        self.window.publish_jog_intent()
        self.assertTrue(self.window.jog_intent_streaming)
        tabs.setCurrentIndex(2)
        self.assertFalse(self.window.jog_mode_button.isChecked())
        hold = self.node.jog_intent_publisher.messages[-1]
        self.assertEqual(list(hold.positions), [
            self.node.current_positions[name] for name in hold.joint_names
        ])
        self.window.cartesian_jog_mode_button.setChecked(True)
        self.window.cartesian_wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.publish_cartesian_jog_intent()
        self.assertTrue(self.window.cartesian_jog_intent_streaming)
        tabs.setCurrentIndex(0)
        self.assertFalse(self.window.cartesian_jog_mode_button.isChecked())
        last = self.node.cartesian_jog_publisher.messages[-1].twist
        self.assertEqual((last.linear.x, last.linear.y, last.linear.z,
                          last.angular.x, last.angular.y, last.angular.z),
                         (0.0,) * 6)
        tabs.setCurrentIndex(1)
        self.assertFalse(self.window.jog_mode_button.isChecked())
        self.assertFalse(self.window.cartesian_jog_mode_button.isChecked())

    def test_manual_controls_fit_desktop_viewport(self):
        self.window.resize(1920, 1080)
        self.window.show()
        self.select('joints')
        scroll = self.window.workspace_pages.currentWidget()
        for index, button, sliders in (
            (1, self.window.jog_mode_button, self.window.jog_sliders),
            (2, self.window.cartesian_jog_mode_button,
             self.window.cartesian_jog_sliders),
        ):
            with self.subTest(view=index):
                self.window.manual_subviews.setCurrentIndex(index)
                self.app.processEvents()
                viewport = scroll.viewport()
                for control in (button, *sliders):
                    self.assertTrue(control.isVisible())
                    top_left = control.mapTo(viewport, control.rect().topLeft())
                    bottom_right = control.mapTo(viewport, control.rect().bottomRight())
                    self.assertTrue(viewport.rect().contains(top_left))
                    self.assertTrue(viewport.rect().contains(bottom_right))
                self.assertEqual(scroll.verticalScrollBar().value(), 0)
        self.assertIn('CARTESIANO', self.window.cartesian_jog_mode_button.text())

    def test_objective_view_fits_desktop_and_reflows_when_narrow(self):
        self.window.resize(1920, 1080)
        self.window.show()
        self.select('joints')
        self.window.manual_subviews.setCurrentIndex(0)
        for _ in range(4):
            self.app.processEvents()
        scroll = self.window.workspace_pages.currentWidget()
        viewport = scroll.viewport()
        self.assertFalse(self.window.objective_columns.stacked)
        for control in (self.window.target_sliders[-1], self.window.send_button,
                        self.window.velocity_table):
            self.assertTrue(control.isVisible())
            self.assertTrue(viewport.rect().contains(
                control.mapTo(viewport, control.rect().topLeft())))
            self.assertTrue(viewport.rect().contains(
                control.mapTo(viewport, control.rect().bottomRight())))
        self.assertEqual(scroll.verticalScrollBar().maximum(), 0)
        self.assertEqual(self.window.velocity_table.verticalScrollBar().maximum(), 0)

        self.window.resize(1000, 768)
        for _ in range(4):
            self.app.processEvents()
        self.assertTrue(self.window.objective_columns.stacked)
        self.assertEqual(scroll.horizontalScrollBar().maximum(), 0)

    def test_leaving_manual_stops_joint_jog_and_does_not_resume(self):
        self.select('joints')
        self.window.jog_mode_button.setChecked(True)
        self.window.wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.begin_deliberate_jog_input()
        self.window.publish_jog_intent()
        self.assertTrue(self.window.jog_intent_streaming)
        command_id = self.node.jog_intent_publisher.messages[-1].command_id

        self.select('history')

        self.assertFalse(self.window.jog_mode_button.isChecked())
        self.assertFalse(self.window.jog_intent_streaming)
        hold = self.node.jog_intent_publisher.messages[-1]
        self.assertEqual(hold.command_id, command_id)
        self.assertEqual(
            list(hold.positions),
            [self.node.current_positions[name] for name in hold.joint_names],
        )
        count = len(self.node.jog_intent_publisher.messages)
        self.select('joints')
        self.window.publish_jog_intent()
        self.assertFalse(self.window.jog_mode_button.isChecked())
        self.assertEqual(len(self.node.jog_intent_publisher.messages), count)

    def test_leaving_manual_stops_cartesian_jog_and_does_not_resume(self):
        self.select('joints')
        self.window.cartesian_jog_mode_button.setChecked(True)
        self.window.cartesian_wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.publish_cartesian_jog_intent()
        self.assertTrue(self.window.cartesian_jog_intent_streaming)

        self.select('history')

        self.assertFalse(self.window.cartesian_jog_mode_button.isChecked())
        self.assertFalse(self.window.cartesian_jog_intent_streaming)
        last = self.node.cartesian_jog_publisher.messages[-1]
        self.assertEqual(
            (last.twist.linear.x, last.twist.linear.y, last.twist.linear.z,
             last.twist.angular.x, last.twist.angular.y, last.twist.angular.z),
            (0.0,) * 6,
        )
        count = len(self.node.cartesian_jog_publisher.messages)
        self.select('joints')
        self.window.publish_cartesian_jog_intent()
        self.assertEqual(
            len(self.node.cartesian_jog_publisher.messages), count
        )

    def test_navigation_does_not_clear_pending_trajectory(self):
        self.window.cartesian_trajectory_pending_id = 'pending-path'
        for page_id in self.window.workspace_page_ids:
            self.select(page_id)
        self.assertEqual(
            self.window.cartesian_trajectory_pending_id, 'pending-path'
        )
        self.assertEqual(
            self.node.cartesian_trajectory_cancel_publisher.messages, []
        )

    def test_losing_focus_clears_motion_and_return_does_not_resume(self):
        self.select('joints')
        self.window.cartesian_jog_mode_button.setChecked(True)
        self.window.cartesian_wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.publish_cartesian_jog_intent()
        self.assertTrue(self.window.cartesian_jog_intent_streaming)

        with patch.object(self.window, 'isActiveWindow', return_value=False):
            self.window.changeEvent(QEvent(QEvent.ActivationChange))
        self.window.publish_cartesian_jog_intent()
        last = self.node.cartesian_jog_publisher.messages[-1]
        self.assertEqual(last.twist.linear.x, 0.0)
        self.assertFalse(self.window.cartesian_jog_intent_streaming)
        count = len(self.node.cartesian_jog_publisher.messages)
        with patch.object(self.window, 'isActiveWindow', return_value=True):
            self.window.changeEvent(QEvent(QEvent.ActivationChange))
        self.window.publish_cartesian_jog_intent()
        self.assertEqual(
            len(self.node.cartesian_jog_publisher.messages), count
        )

    def test_workspace_and_legacy_publish_equivalent_user_requests(self):
        legacy_node = FakeNode()
        legacy_node.ui_layout = 'legacy'
        legacy = JointControlWindow(legacy_node)
        for timer in (
            legacy.ros_timer, legacy.ui_timer,
            legacy.preview_timer, legacy.jog_timer,
        ):
            timer.stop()
        try:
            captured = []
            for window, node in (
                (self.window, self.node), (legacy, legacy_node),
            ):
                node.publish_candidate = MethodType(
                    JointGuiNode.publish_candidate, node
                )
                window.copy_current_pose()
                window.target_inputs[0].setValue(0.5)
                window.duration_input.setValue(3.0)
                window.send_candidate()
                point = node.candidate_publisher.messages[-1]

                window.pending_command_id = None
                window.copy_current_cartesian_pose()
                window.adjust_cartesian_target(0, 0.01)
                window.send_cartesian_candidate()
                cartesian = node.cartesian_candidate_publisher.messages[-1]

                window.pending_command_id = None
                window.cartesian_trajectory_inputs[0].setValue(0.1)
                window.send_cartesian_trajectory()
                path = node.cartesian_trajectory_publisher.messages[-1]
                captured.append((
                    tuple(point.joint_names), tuple(point.positions),
                    point.duration.sec, point.duration.nanosec,
                    cartesian.frame_id,
                    (cartesian.x, cartesian.y, cartesian.z,
                     cartesian.roll, cartesian.pitch, cartesian.yaw),
                    cartesian.duration.sec, cartesian.duration.nanosec,
                    path.frame_id, path.mode,
                    (path.x, path.y, path.z,
                     path.roll, path.pitch, path.yaw),
                    path.linear_speed_mps, path.angular_speed_radps,
                ))
            self.assertEqual(captured[0], captured[1])
        finally:
            legacy.deleteLater()

    def test_main_close_cancels_pending_trajectory_and_stops_timers(self):
        self.window.cartesian_trajectory_pending_id = 'pending-path'
        event = QCloseEvent()
        with patch(
                'thesis_ui.joint_control_gui.rclpy.ok', return_value=True), \
                patch(
                    'thesis_ui.joint_control_gui.rclpy.shutdown') as shutdown:
            self.window.closeEvent(event)
        self.assertTrue(event.isAccepted())
        self.assertEqual(
            self.node.cartesian_trajectory_cancel_publisher.messages[-1].data,
            'pending-path',
        )
        self.assertTrue(self.window.shutdown_started)
        shutdown.assert_called_once()
        for timer in (
            self.window.ros_timer, self.window.ui_timer,
            self.window.preview_timer, self.window.jog_timer,
        ):
            self.assertFalse(timer.isActive())

    def test_auxiliary_close_reattaches_same_page_without_stopping_owner(self):
        timers = (
            self.window.ros_timer, self.window.ui_timer,
            self.window.preview_timer, self.window.jog_timer,
        )
        for page_id in ('history',):
            self.select(page_id)
            index = self.window.workspace_page_ids.index(page_id)
            original_page = self.window.workspace_pages.widget(index)
            for timer in timers:
                timer.start(1_000_000)
            try:
                with patch(
                        'thesis_ui.joint_control_gui.rclpy.shutdown'
                ) as shutdown:
                    dialog = detach_workspace_page(self.window, page_id)
                    self.assertIsNot(
                        self.window.workspace_pages.widget(index),
                        original_page,
                    )
                    dialog.close()
                    shutdown.assert_not_called()
                self.assertIs(
                    self.window.workspace_pages.widget(index), original_page
                )
                self.assertNotIn(
                    page_id, self.window.workspace_detached_dialogs
                )
                self.assertFalse(self.window.shutdown_started)
                self.assertTrue(all(timer.isActive() for timer in timers))
                for publisher in (
                    self.node.candidate_publisher,
                    self.node.jog_intent_publisher,
                    self.node.cartesian_candidate_publisher,
                    self.node.cartesian_jog_publisher,
                    self.node.cartesian_trajectory_publisher,
                    self.node.cartesian_trajectory_cancel_publisher,
                ):
                    self.assertEqual(publisher.messages, [])
            finally:
                for timer in timers:
                    timer.stop()

    def test_reopening_auxiliary_reuses_window_without_duplicate_page(self):
        self.select('history')
        original_count = self.window.workspace_pages.count()
        index = self.window.workspace_page_ids.index('history')
        page = self.window.workspace_pages.widget(index)
        first = detach_workspace_page(self.window, 'history')
        self.assertIs(
            detach_workspace_page(self.window, 'history'), first
        )
        self.assertEqual(len(self.window.workspace_detached_dialogs), 1)
        self.assertEqual(self.window.workspace_pages.count(), original_count)
        first.close()
        second = detach_workspace_page(self.window, 'history')
        second.close()
        self.assertEqual(self.window.workspace_pages.count(), original_count)
        self.assertIs(self.window.workspace_pages.widget(index), page)

    def test_operational_pages_cannot_be_detached(self):
        for page_id in ('overview', 'joints', 'cartesian', 'trajectory'):
            with self.assertRaises(ValueError):
                detach_workspace_page(self.window, page_id)

    def test_integrated_details_and_detached_diagnostics_stop_jog(self):
        self.window.show()
        self.select('joints')
        self.assertNotIn('overview', self.window.workspace_page_ids)
        self.assertNotIn('diagnostics', self.window.workspace_page_ids)
        self.window.workspace_details_toggle.setChecked(True)
        self.app.processEvents()
        self.assertTrue(self.window.workspace_details_widget.isVisible())
        content = self.window.workspace_diagnostic_content
        self.window.manual_subviews.setCurrentIndex(2)
        self.window.cartesian_jog_mode_button.setChecked(True)
        self.window.cartesian_wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.publish_cartesian_jog_intent()
        self.assertTrue(self.window.cartesian_jog_intent_streaming)
        dialog = detach_workspace_page(self.window, 'diagnostics')
        self.assertFalse(self.window.cartesian_jog_mode_button.isChecked())
        twist = self.node.cartesian_jog_publisher.messages[-1].twist
        self.assertEqual((twist.linear.x, twist.linear.y, twist.linear.z,
                          twist.angular.x, twist.angular.y, twist.angular.z), (0.0,) * 6)
        self.assertTrue(dialog.isAncestorOf(content))
        self.assertIs(detach_workspace_page(self.window, 'diagnostics'), dialog)
        dialog.close()
        self.assertTrue(self.window.workspace_details_widget.isAncestorOf(content))
        self.assertFalse(self.window.cartesian_jog_mode_button.isChecked())

    def test_auxiliary_round_trip_does_not_resume_manual_motion(self):
        self.select('joints')
        self.window.cartesian_jog_mode_button.setChecked(True)
        self.window.cartesian_wheel_intentions[0].add(time.monotonic(), 1.0)
        self.window.publish_cartesian_jog_intent()
        self.select('history')
        self.assertFalse(self.window.cartesian_jog_intent_streaming)
        count = len(self.node.cartesian_jog_publisher.messages)
        dialog = detach_workspace_page(self.window, 'diagnostics')
        dialog.close()
        self.select('joints')
        self.window.publish_cartesian_jog_intent()
        self.assertFalse(self.window.cartesian_jog_mode_button.isChecked())
        self.assertEqual(
            len(self.node.cartesian_jog_publisher.messages), count
        )

    def test_main_close_with_two_auxiliaries_shuts_down_once(self):
        originals = {
            page_id: self.window.workspace_pages.widget(
                self.window.workspace_page_ids.index(page_id)
            ) for page_id in ('history',)
        }
        dialogs = [
            detach_workspace_page(self.window, page_id)
            for page_id in originals
        ]
        self.assertEqual(len(self.window.workspace_detached_dialogs), 1)
        event = QCloseEvent()
        with patch(
                'thesis_ui.joint_control_gui.rclpy.ok', return_value=True), \
                patch(
                    'thesis_ui.joint_control_gui.rclpy.shutdown') as shutdown:
            self.window.closeEvent(event)
            self.window.closeEvent(QCloseEvent())
        shutdown.assert_called_once()
        self.assertTrue(event.isAccepted())
        self.assertEqual(self.window.workspace_detached_dialogs, {})
        self.assertTrue(all(not dialog.isVisible() for dialog in dialogs))
        for page_id, page in originals.items():
            self.assertIs(
                self.window.workspace_pages.widget(
                    self.window.workspace_page_ids.index(page_id)
                ), page,
            )
        self.assertEqual(self.node.jog_intent_publisher.messages, [])
        self.assertEqual(self.node.cartesian_jog_publisher.messages, [])
        self.assertEqual(
            self.node.cartesian_trajectory_cancel_publisher.messages, []
        )

    def test_desktop_pages_fit_width_without_publishing_motion(self):
        self.window.show()
        for width, height in ((1366, 768), (1920, 1080)):
            self.window.resize(width, height)
            for page_id in self.window.workspace_page_ids:
                self.select(page_id)
                # Layout/reflow and deferred deletion may need successive
                # event turns, but all ROS and command timers remain stopped.
                for _ in range(4):
                    self.app.processEvents()
                page = self.window.workspace_pages.currentWidget()
                self.assertEqual(
                    page.horizontalScrollBar().maximum(), 0,
                    f'Unexpected horizontal scroll at {width}px: {page_id}',
                )
                self.assertLessEqual(self.window.width(), width)
        for publisher in (
            self.node.candidate_publisher,
            self.node.jog_intent_publisher,
            self.node.cartesian_candidate_publisher,
            self.node.cartesian_jog_publisher,
            self.node.cartesian_trajectory_publisher,
            self.node.cartesian_trajectory_cancel_publisher,
        ):
            self.assertEqual(publisher.messages, [])


if __name__ == '__main__':
    unittest.main()
