#!/usr/bin/env python3
"""
Qt operator interface for the profile-driven simulation supervisors.

This frontend never talks to a controller or to hardware.  It only asks
``MultiRobotGuiNode`` to submit a candidate to the selected robot supervisor.
The loose field access helpers deliberately accept dataclasses as well as
dictionaries so the ROS backend remains independently testable.
"""

from __future__ import annotations

import math
import sys
from typing import Any, Iterable

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args

from thesis_multi_robot.multi_robot_gui_node import MultiRobotGuiNode


def _field(source: Any, name: str, default: Any = None) -> Any:
    """Read one backend field from either a mapping or an object."""
    if source is None:
        return default
    if isinstance(source, dict):
        return source.get(name, default)
    return getattr(source, name, default)


def _names(node: MultiRobotGuiNode) -> tuple[str, ...]:
    value = getattr(node, 'robot_names')
    return tuple(value() if callable(value) else value)


def _joint_names(profile: Any) -> tuple[str, ...]:
    return tuple(_field(profile, 'joint_names', ()))


def _limits(profile: Any, names: Iterable[str]) -> tuple[tuple[float, float], ...]:
    value = _field(profile, 'joint_limits', ())
    if isinstance(value, dict):
        value = tuple(value[name] for name in names)
    return tuple((float(pair[0]), float(pair[1])) for pair in value)


def _vector(snapshot: Any, field: str, names: tuple[str, ...]) -> tuple[float, ...]:
    value = _field(snapshot, field, ())
    if isinstance(value, dict):
        return tuple(float(value.get(name, math.nan)) for name in names)
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return tuple(math.nan for _ in names)
    if len(vector) != len(names):
        return tuple(math.nan for _ in names)
    return vector


def _finite_vector(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class MultiRobotWindow(QMainWindow):
    """Display all simulation profiles and command only the selected one."""

    def __init__(self, node: MultiRobotGuiNode):
        super().__init__()
        self.node = node
        self.robot_names = _names(node)
        if not self.robot_names:
            raise RuntimeError('El manifiesto no contiene perfiles de robot.')
        self.selected_name = self.robot_names[0]
        self.joint_targets: list[QDoubleSpinBox] = []
        self.joint_current_labels: list[QLabel] = []
        self.joint_minus_buttons: list[QPushButton] = []
        self.joint_plus_buttons: list[QPushButton] = []
        self._joint_target_initialized = False
        self._cartesian_target_initialized = False
        self.last_command_id = ''
        self._build()
        self._select_robot(0)
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(20)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)

    def _build(self):
        self.setWindowTitle('Célula colaborativa — interfaz de simulación')
        self.resize(1120, 820)
        root = QWidget()
        root_layout = QVBoxLayout(root)

        heading = QLabel(
            'Control supervisado JACO2 · UR17g · P3020 · CS625'
        )
        heading.setStyleSheet('font-size: 20px; font-weight: bold;')
        root_layout.addWidget(heading)
        notice = QLabel(
            'Solo simulación. Toda orden se publica como candidato al '
            'supervisor del robot seleccionado; nunca al controlador directo.'
        )
        notice.setWordWrap(True)
        root_layout.addWidget(notice)

        selector_row = QHBoxLayout()
        selector_row.addWidget(QLabel('Brazo activo:'))
        self.robot_selector = QComboBox()
        for name in self.robot_names:
            profile = self.node.profile(name)
            label = _field(profile, 'display_name', name.upper())
            self.robot_selector.addItem(str(label), name)
        self.robot_selector.currentIndexChanged.connect(self._select_robot)
        selector_row.addWidget(self.robot_selector, 1)
        self.selected_status = QLabel('WAITING')
        self.selected_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        selector_row.addWidget(self.selected_status, 2)
        root_layout.addLayout(selector_row)

        self.summary = QTableWidget(len(self.robot_names), 5)
        self.summary.setHorizontalHeaderLabels([
            'Robot', 'Estado', 'Proximidad', 'Decisión', 'Seguimiento',
        ])
        self.summary.verticalHeader().setVisible(False)
        self.summary.setEditTriggers(QTableWidget.NoEditTriggers)
        self.summary.setSelectionBehavior(QTableWidget.SelectRows)
        self.summary.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.summary.cellClicked.connect(
            lambda row, _column: self.robot_selector.setCurrentIndex(row)
        )
        root_layout.addWidget(self.summary)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        controls_layout.addWidget(self._build_joint_group())
        controls_layout.addWidget(self._build_cartesian_group())
        controls_layout.addWidget(self._build_feedback_group())
        controls_layout.addStretch(1)
        scroll.setWidget(controls)
        root_layout.addWidget(scroll, 1)
        self.setCentralWidget(root)

    def _build_joint_group(self) -> QGroupBox:
        group = QGroupBox('Control articular supervisado')
        layout = QVBoxLayout(group)
        controls = QHBoxLayout()
        self.copy_joints_button = QPushButton('Copiar estado actual')
        self.copy_joints_button.clicked.connect(self._copy_current_joints)
        controls.addWidget(self.copy_joints_button)
        controls.addWidget(QLabel('Δ pequeño:'))
        self.delta_degrees = QDoubleSpinBox()
        self.delta_degrees.setRange(0.01, 10.0)
        self.delta_degrees.setDecimals(2)
        self.delta_degrees.setValue(1.0)
        self.delta_degrees.setSuffix('°')
        controls.addWidget(self.delta_degrees)
        controls.addWidget(QLabel('Duración:'))
        self.duration = QDoubleSpinBox()
        self.duration.setRange(0.1, 30.0)
        self.duration.setDecimals(1)
        self.duration.setValue(2.0)
        self.duration.setSuffix(' s')
        controls.addWidget(self.duration)
        self.send_joints_button = QPushButton('Enviar objetivo articular')
        self.send_joints_button.clicked.connect(self._send_joint_target)
        controls.addWidget(self.send_joints_button)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.joint_table = QTableWidget(0, 7)
        self.joint_table.setHorizontalHeaderLabels([
            'Articulación', 'Actual', 'Objetivo', 'Mínimo', 'Máximo',
            '−Δ', '+Δ',
        ])
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        layout.addWidget(self.joint_table)
        return group

    def _build_cartesian_group(self) -> QGroupBox:
        group = QGroupBox('Objetivo cartesiano por IK del perfil')
        layout = QGridLayout(group)
        self.cartesian_fields: dict[str, QDoubleSpinBox] = {}
        specifications = (
            ('X', -10.0, 10.0, 4, ' m'),
            ('Y', -10.0, 10.0, 4, ' m'),
            ('Z', -1.0, 10.0, 4, ' m'),
            ('Roll', -360.0, 360.0, 2, '°'),
            ('Pitch', -360.0, 360.0, 2, '°'),
            ('Yaw', -360.0, 360.0, 2, '°'),
        )
        for index, (name, lower, upper, decimals, suffix) in enumerate(
            specifications
        ):
            field = QDoubleSpinBox()
            field.setRange(lower, upper)
            field.setDecimals(decimals)
            field.setSuffix(suffix)
            row, column = divmod(index, 3)
            layout.addWidget(QLabel(name), row * 2, column)
            layout.addWidget(field, row * 2 + 1, column)
            self.cartesian_fields[name] = field
        self.position_only = QCheckBox('Solo posición (sin orientar herramienta)')
        self.position_only.toggled.connect(self._update_cartesian_mode)
        layout.addWidget(self.position_only, 4, 0, 1, 2)
        self.copy_pose_button = QPushButton('Copiar pose actual')
        self.copy_pose_button.clicked.connect(self._copy_current_pose)
        layout.addWidget(self.copy_pose_button, 5, 0)
        self.send_pose_button = QPushButton('Resolver IK y enviar candidato')
        self.send_pose_button.clicked.connect(self._send_cartesian_target)
        layout.addWidget(self.send_pose_button, 5, 1, 1, 2)
        return group

    def _build_feedback_group(self) -> QGroupBox:
        group = QGroupBox('Feedback del supervisor')
        layout = QFormLayout(group)
        self.freshness_value = QLabel('WAITING')
        self.proximity_value = QLabel('WAITING')
        self.decision_value = QLabel('Sin decisión')
        self.tracking_value = QLabel('Sin objetivo activo')
        for widget in (
            self.freshness_value,
            self.proximity_value,
            self.decision_value,
            self.tracking_value,
        ):
            widget.setWordWrap(True)
        layout.addRow('Estado:', self.freshness_value)
        layout.addRow('Proximidad:', self.proximity_value)
        layout.addRow('Última decisión:', self.decision_value)
        layout.addRow('Seguimiento:', self.tracking_value)
        return group

    def _select_robot(self, index: int):
        if index < 0 or index >= len(self.robot_names):
            return
        self.selected_name = self.robot_names[index]
        profile = self.node.profile(self.selected_name)
        names = _joint_names(profile)
        limits = _limits(profile, names)
        self.joint_targets.clear()
        self.joint_current_labels.clear()
        self.joint_minus_buttons.clear()
        self.joint_plus_buttons.clear()
        self.joint_table.setRowCount(len(names))
        for row, (name, limit) in enumerate(zip(names, limits)):
            lower, upper = map(float, limit)
            lower_deg = math.degrees(lower) if math.isfinite(lower) else -360.0
            upper_deg = math.degrees(upper) if math.isfinite(upper) else 360.0
            current = QLabel('—')
            target = QDoubleSpinBox()
            target.setRange(lower_deg, upper_deg)
            target.setDecimals(3)
            target.setSuffix('°')
            minus = QPushButton('−')
            plus = QPushButton('+')
            minus.clicked.connect(
                lambda _checked=False, item=row: self._send_delta(item, -1.0)
            )
            plus.clicked.connect(
                lambda _checked=False, item=row: self._send_delta(item, 1.0)
            )
            self.joint_table.setItem(row, 0, QTableWidgetItem(name))
            self.joint_table.setCellWidget(row, 1, current)
            self.joint_table.setCellWidget(row, 2, target)
            self.joint_table.setItem(
                row, 3, QTableWidgetItem(f'{lower_deg:.2f}°')
            )
            self.joint_table.setItem(
                row, 4, QTableWidgetItem(f'{upper_deg:.2f}°')
            )
            self.joint_table.setCellWidget(row, 5, minus)
            self.joint_table.setCellWidget(row, 6, plus)
            self.joint_current_labels.append(current)
            self.joint_targets.append(target)
            self.joint_minus_buttons.append(minus)
            self.joint_plus_buttons.append(plus)
        self.last_command_id = ''
        self._joint_target_initialized = False
        self._cartesian_target_initialized = False
        self._copy_current_joints(silent=True)
        self._copy_current_pose(silent=True)
        self._refresh()

    def _snapshot(self, name: str | None = None) -> Any:
        return self.node.snapshot(name or self.selected_name)

    def _is_fresh(self, snapshot: Any) -> bool:
        return bool(
            _field(snapshot, 'state_fresh', _field(snapshot, 'fresh', False))
        )

    def _proximity(self, snapshot: Any) -> str:
        return str(
            _field(
                snapshot,
                'proximity_state',
                _field(snapshot, 'proximity', 'WAITING'),
            )
        )

    def _command_enabled(self, snapshot: Any) -> bool:
        proximity_fresh = bool(_field(snapshot, 'proximity_fresh', True))
        return (
            self._is_fresh(snapshot)
            and proximity_fresh
            and self._proximity(snapshot) in ('ALLOW', 'WARNING')
        )

    def _copy_current_joints(self, _checked=False, silent=False):
        profile = self.node.profile(self.selected_name)
        names = _joint_names(profile)
        snapshot = self._snapshot()
        positions = _vector(snapshot, 'positions', names)
        if not self._is_fresh(snapshot) or not _finite_vector(positions):
            if not silent:
                self._error('No existe un estado articular completo y reciente.')
            return
        for field, value in zip(self.joint_targets, positions):
            field.setValue(math.degrees(value))
        self._joint_target_initialized = True

    def _send_joint_target(self):
        if not self._joint_target_initialized:
            self._error('Copie primero un estado articular reciente.')
            return
        target = tuple(math.radians(field.value()) for field in self.joint_targets)
        try:
            result = self.node.send_joint_target(
                self.selected_name, target, self.duration.value()
            )
            self._record_submission(result)
        except (KeyError, RuntimeError, TypeError, ValueError) as exception:
            self._error(str(exception))

    def _send_delta(self, index: int, direction: float):
        delta = math.radians(direction * self.delta_degrees.value())
        try:
            result = self.node.send_joint_delta(
                self.selected_name, index, delta, self.duration.value()
            )
            self._record_submission(result)
        except (KeyError, RuntimeError, TypeError, ValueError) as exception:
            self._error(str(exception))

    def _current_pose(self) -> tuple[float, ...] | None:
        snapshot = self._snapshot()
        pose = _field(snapshot, 'tool_pose', _field(snapshot, 'pose', None))
        if pose is None:
            xyz = _field(snapshot, 'tool_xyz', None)
            rpy = _field(snapshot, 'tool_rpy', None)
            if xyz is not None and rpy is not None:
                pose = tuple(xyz) + tuple(rpy)
        if pose is None and hasattr(self.node, 'current_tool_pose'):
            pose = self.node.current_tool_pose(self.selected_name)
        if pose is None:
            return None
        if isinstance(pose, dict):
            keys = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
            pose = tuple(pose.get(key, math.nan) for key in keys)
        try:
            values = tuple(float(value) for value in pose)
        except (TypeError, ValueError):
            return None
        return values if len(values) == 6 and _finite_vector(values) else None

    def _copy_current_pose(self, _checked=False, silent=False):
        pose = self._current_pose()
        if not self._is_fresh(self._snapshot()) or pose is None:
            if not silent:
                self._error('La pose cartesiana actual no está disponible.')
            return
        for name, value in zip(('X', 'Y', 'Z'), pose[:3]):
            self.cartesian_fields[name].setValue(value)
        for name, value in zip(('Roll', 'Pitch', 'Yaw'), pose[3:]):
            self.cartesian_fields[name].setValue(math.degrees(value))
        self._cartesian_target_initialized = True

    def _send_cartesian_target(self):
        if not self._cartesian_target_initialized:
            self._error('Copie primero una pose cartesiana reciente.')
            return
        xyz = tuple(self.cartesian_fields[name].value() for name in ('X', 'Y', 'Z'))
        rpy = tuple(
            math.radians(self.cartesian_fields[name].value())
            for name in ('Roll', 'Pitch', 'Yaw')
        )
        position_only = self.position_only.isChecked()
        try:
            result = self.node.send_cartesian_target(
                self.selected_name,
                xyz,
                None if position_only else rpy,
                self.duration.value(),
                position_only,
            )
            self._record_submission(result)
        except (KeyError, RuntimeError, TypeError, ValueError) as exception:
            self._error(str(exception))

    def _record_submission(self, result: Any):
        if isinstance(result, str):
            self.last_command_id = result
        else:
            self.last_command_id = str(
                _field(result, 'command_id', _field(result, 'id', result))
            )
        self.tracking_value.setText(
            f'Candidato enviado: {self.last_command_id}'
        )

    def _update_cartesian_mode(self, checked: bool):
        for name in ('Roll', 'Pitch', 'Yaw'):
            self.cartesian_fields[name].setEnabled(not checked)

    def _spin_ros(self):
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.0)
        except (ExternalShutdownException, KeyboardInterrupt):
            application = QApplication.instance()
            if application is not None:
                application.quit()

    def _refresh(self):
        for row, name in enumerate(self.robot_names):
            snapshot = self._snapshot(name)
            fresh = self._is_fresh(snapshot)
            proximity = self._proximity(snapshot)
            decision = str(
                _field(snapshot, 'decision_state', _field(snapshot, 'decision', '—'))
            )
            tracking = str(
                _field(snapshot, 'tracking_state', _field(snapshot, 'command_status', '—'))
            )
            values = (
                name.upper(), 'FRESH' if fresh else 'STALE', proximity,
                decision, tracking,
            )
            for column, value in enumerate(values):
                self.summary.setItem(row, column, QTableWidgetItem(value))

        profile = self.node.profile(self.selected_name)
        names = _joint_names(profile)
        snapshot = self._snapshot()
        positions = _vector(snapshot, 'positions', names)
        if self._is_fresh(snapshot) and not self._joint_target_initialized:
            self._copy_current_joints(silent=True)
        if self._is_fresh(snapshot) and not self._cartesian_target_initialized:
            self._copy_current_pose(silent=True)
        for label, value in zip(self.joint_current_labels, positions):
            label.setText('—' if not math.isfinite(value) else f'{math.degrees(value):.3f}°')
        fresh = self._is_fresh(snapshot)
        proximity = self._proximity(snapshot)
        enabled = self._command_enabled(snapshot)
        self.selected_status.setText(
            f'{self.selected_name.upper()} · '
            f'{"FRESH" if fresh else "STALE"} · {proximity}'
        )
        self.freshness_value.setText('FRESH' if fresh else 'STALE / esperando estado')
        clearance = _field(
            snapshot, 'minimum_clearance', _field(snapshot, 'clearance', None)
        )
        clearance_text = ''
        if clearance is not None:
            try:
                clearance_text = f' · margen={float(clearance):.3f} m'
            except (TypeError, ValueError):
                pass
        self.proximity_value.setText(proximity + clearance_text)
        decision_state = _field(snapshot, 'decision_state', 'Sin decisión')
        decision_reason = _field(snapshot, 'decision_reason', '')
        self.decision_value.setText(
            f'{decision_state}: {decision_reason}'.rstrip(': ')
        )
        tracking = _field(
            snapshot,
            'tracking_state',
            _field(snapshot, 'command_status', 'Sin objetivo activo'),
        )
        tracking_error = _field(
            snapshot,
            'tracking_error',
            _field(snapshot, 'maximum_target_error', None),
        )
        tracking_text = str(tracking)
        if tracking_error is not None:
            try:
                tracking_text += f' · error={float(tracking_error):.4f} rad'
            except (TypeError, ValueError):
                pass
        self.tracking_value.setText(tracking_text)
        self.send_joints_button.setEnabled(
            enabled and self._joint_target_initialized
        )
        self.send_pose_button.setEnabled(
            enabled and self._cartesian_target_initialized
        )
        for button in (*self.joint_minus_buttons, *self.joint_plus_buttons):
            button.setEnabled(enabled)
        self.copy_joints_button.setEnabled(fresh)
        self.copy_pose_button.setEnabled(fresh)

    def _error(self, detail: str):
        QMessageBox.warning(self, 'Orden no enviada', detail)

    def closeEvent(self, event):  # noqa: N802 - Qt API name
        self.spin_timer.stop()
        self.refresh_timer.stop()
        event.accept()


def main(argv=None):
    """Run the simulation-only ROS/Qt interface."""
    process_arguments = sys.argv if argv is None else argv
    qt_arguments = remove_ros_args(args=process_arguments)
    rclpy.init(args=process_arguments)
    node = None
    try:
        node = MultiRobotGuiNode()
        application = QApplication.instance() or QApplication(qt_arguments)
        window = MultiRobotWindow(node)
        window.show()
        return application.exec_()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
