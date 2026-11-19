"""A read-only overview for the three simulated arms and logical source."""

import math

from PyQt5.QtWidgets import (
    QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget,
)

from thesis_ui.multiarm_observer import ROBOT_IDS


class MultiArmDashboard(QWidget):
    """Render observations; this widget has no command actions or timers."""

    def __init__(self, observer, parent=None):
        super().__init__(parent)
        self.observer = observer
        layout = QVBoxLayout(self)
        heading = QLabel('Vista del escenario de cuatro brazos')
        heading.setObjectName('sectionTitle')
        layout.addWidget(heading)
        note = QLabel(
            'OBSERVACIÓN · arm_01 es una fuente sintética sin actuador. '
            'arm_02, arm_03 y arm_04 pertenecen a Gazebo. '
            'La recencia indica recepción local; no valida las fuentes '
            'ni autoriza movimiento.'
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        self.readiness_label = QLabel()
        self.readiness_label.setWordWrap(True)
        layout.addWidget(self.readiness_label)
        safety_group = QGroupBox('Evaluación global informada')
        safety_layout = QVBoxLayout(safety_group)
        self.safety_label = QLabel()
        self.safety_label.setWordWrap(True)
        self.safety_detail = QLabel()
        self.safety_detail.setWordWrap(True)
        self.clearance_label = QLabel()
        safety_layout.addWidget(self.safety_label)
        safety_layout.addWidget(self.safety_detail)
        safety_layout.addWidget(self.clearance_label)
        layout.addWidget(safety_group)
        cards = QGridLayout()
        self.arm_labels = {}
        self.joint_labels = {}
        for index, robot_id in enumerate(ROBOT_IDS):
            group = QGroupBox(robot_id.upper())
            card = QVBoxLayout(group)
            role = QLabel(
                'Fuente sintética · sin actuador'
                if robot_id == 'arm_01' else 'Brazo simulado · Gazebo'
            )
            role.setWordWrap(True)
            card.addWidget(role)
            status = QLabel('Sin datos')
            card.addWidget(status)
            joints = QLabel('J1–J6: —')
            joints.setWordWrap(True)
            card.addWidget(joints)
            topic = QLabel(f'/{robot_id}/joint_states')
            topic.setWordWrap(True)
            topic.setStyleSheet('color: #71665c; font-size: 11px;')
            card.addWidget(topic)
            cards.addWidget(group, index // 2, index % 2)
            self.arm_labels[robot_id] = status
            self.joint_labels[robot_id] = joints
        layout.addLayout(cards)
        layout.addStretch(1)
        self.refresh()

    def refresh(self):
        """Update from passive observations using the owning window timer."""
        self.readiness_label.setText(self.observer.readiness_text())
        state, detail, clearance = self.observer.safety_state()
        color = {
            'ALLOW': '#3f654f', 'STOP': '#a43636',
        }.get(state, '#87621f')
        self.safety_label.setText('Estado global: ' + state)
        self.safety_label.setStyleSheet(
            f'color: {color}; font-weight: bold; font-size: 18px;'
        )
        self.safety_detail.setText(detail)
        self.clearance_label.setText(
            'Distancia mínima informada: —' if clearance is None else
            f'Distancia mínima informada: {clearance:.3f} m'
        )
        for robot_id in ROBOT_IDS:
            status, sample = self.observer.arm_state(robot_id)
            self.arm_labels[robot_id].setText(status)
            live = status == 'Estado reciente'
            self.arm_labels[robot_id].setStyleSheet(
                'font-weight: bold; color: '
                + ('#3f654f;' if live else '#87621f;')
            )
            if sample is None or not live:
                self.joint_labels[robot_id].setText('J1–J6: —')
            else:
                self.joint_labels[robot_id].setText('\n'.join(
                    f'J{index + 1}: {math.degrees(value):.1f}°'
                    for index, value in enumerate(sample.positions)
                ))


def main(args=None):
    """Run the hybrid overview without creating any control interface."""
    import sys

    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication, QMainWindow, QScrollArea
    import rclpy
    from rclpy.executors import ExternalShutdownException, ShutdownException
    from rclpy.node import Node

    from thesis_ui.multiarm_observer import MultiArmObserver

    rclpy.init(args=args)
    app = QApplication.instance() or QApplication([sys.argv[0]])
    node = Node('multiarm_observation_gui')
    observer = MultiArmObserver(node)
    window = QMainWindow()
    window.setWindowTitle('Tesis · Cuatro brazos · Observación')
    window.resize(1280, 800)
    window.setMinimumSize(760, 520)
    dashboard = MultiArmDashboard(observer)
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setWidget(dashboard)
    window.setCentralWidget(scroll)
    window.setStyleSheet(
        'QMainWindow, QWidget { background: #f5f0e8; color: #352e28; }'
        'QLabel { font-size: 14px; }'
        '#sectionTitle { font-size: 24px; font-weight: bold; }'
        'QGroupBox { background: #fffaf3; border: 1px solid #d6c8b8; '
        'border-radius: 8px; margin-top: 12px; padding: 16px; '
        'font-weight: bold; }'
        'QGroupBox::title { subcontrol-origin: margin; left: 12px; '
        'padding: 0 5px; }'
    )
    timer = QTimer(window)

    def tick():
        try:
            if rclpy.ok(context=node.context):
                # Four joint streams at 50 Hz plus coordinator/readiness:
                # one callback per tick would accumulate old observations.
                # Keep source queues shallow and drain a bounded batch.
                for _ in range(16):
                    if not rclpy.ok(context=node.context):
                        break
                    rclpy.spin_once(node, timeout_sec=0.0)
                dashboard.refresh()
                return
        except (ExternalShutdownException, ShutdownException):
            pass
        if not rclpy.ok(context=node.context):
            timer.stop()
            window.close()

    timer.timeout.connect(tick)
    timer.start(20)
    window.showMaximized()
    try:
        return app.exec_()
    finally:
        timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
