"""
Profile-driven ROS backend for the simulation-only multi-robot GUI.

The backend never talks to a controller or hardware driver. It publishes only
namespaced ``JointCommand`` candidates, so the profile supervisor remains the
sole admission path for JACO2, UR17g and P3020.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import time
from typing import Sequence
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import CommandDecision, JointCommand, ProximityStatus

from .kinematics import RobotKinematics
from .supervisor import MAX_AGE, MAX_SPEED, stamp_is_fresh


TRACKING_TOLERANCE = 0.008
MAX_CARTESIAN_POSITION_STEP = 0.10
MAX_CARTESIAN_ORIENTATION_STEP = math.radians(20.0)
DECISION_TIMEOUT = 5.0

DISPLAY_NAMES = {
    'jaco2': 'JACO2',
    'ur17g': 'UR17g-1300',
    'p3020': 'P3020',
    'cs625': 'Elite CS625',
}


@dataclass(frozen=True)
class UiRobotProfile:
    """Immutable metadata required by the generic operator interface."""

    name: str
    display_name: str
    profile: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float, float], ...]
    initial_positions: tuple[float, ...]
    tip_link: str
    base_link: str
    urdf: str
    model: RobotKinematics


@dataclass(frozen=True)
class UiRobotSnapshot:
    """One read-only view of live state and supervised command progress."""

    name: str
    positions: tuple[float, ...] | None
    velocities: tuple[float, ...] | None
    state_fresh: bool
    state_age: float | None
    proximity_state: str
    proximity_fresh: bool
    clearance: float | None
    decision_state: str
    decision_code: str
    decision_reason: str
    command_id: str
    command_status: str
    target_positions: tuple[float, ...] | None
    maximum_target_error: float | None
    tool_xyz: tuple[float, float, float] | None
    tool_rpy: tuple[float, float, float] | None


@dataclass
class _LiveState:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    received_at: float
    stamp_ns: int


@dataclass
class _Proximity:
    state: str
    clearance: float
    received_at: float
    stamp_ns: int


@dataclass
class _TrackedCommand:
    command_id: str
    target: tuple[float, ...]
    published_at: float
    accepted: bool | None = None
    decision_state: str = 'PENDING'
    reason_code: str = 'PENDING'
    reason: str = 'Esperando decisión del supervisor.'


def _stamp_ns(stamp) -> int:
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        return -1
    return seconds * 1_000_000_000 + nanoseconds


def _finite_vector(
    values: Sequence[float], length: int, label: str
) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain {length} finite values')
    return result


def load_ui_profiles(manifest_path: str | Path) -> dict[str, UiRobotProfile]:
    """Load and validate the runtime manifest without creating ROS entities."""
    path = Path(manifest_path)
    if not path.is_file():
        raise ValueError('manifest_path must identify an existing cell.json')
    manifest = json.loads(path.read_text())
    robots = manifest.get('robots')
    if (
        manifest.get('schema_version') != 1
        or not isinstance(robots, dict)
        or not robots
    ):
        raise ValueError('unsupported or empty runtime manifest')
    result = {}
    for name, raw in robots.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]*', name) or not isinstance(raw, dict):
            raise ValueError('invalid robot entry in runtime manifest')
        joint_names = tuple(raw.get('joint_names', ()))
        if not joint_names or len(set(joint_names)) != len(joint_names):
            raise ValueError(f'{name}: joint_names must be nonempty and unique')
        urdf_path = Path(str(raw.get('urdf', '')))
        if not urdf_path.is_file():
            raise ValueError(f'{name}: generated URDF is missing')
        root = ET.parse(urdf_path).getroot()
        model = RobotKinematics(root, joint_names)
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        limits = []
        for joint_name in joint_names:
            limit = joints[joint_name].find('limit')
            if limit is None or limit.get('velocity') is None:
                raise ValueError(f'{name}: missing velocity limit for {joint_name}')
            lower = float(limit.get('lower', str(-2.0 * math.pi)))
            upper = float(limit.get('upper', str(2.0 * math.pi)))
            velocity = float(limit.get('velocity'))
            if (
                not all(math.isfinite(value) for value in (lower, upper, velocity))
                or lower >= upper
                or velocity <= 0.0
            ):
                raise ValueError(f'{name}: invalid joint limit for {joint_name}')
            limits.append((lower, upper, velocity))
        initial = _finite_vector(
            raw.get('initial_positions', ()),
            len(joint_names),
            f'{name} initial_positions',
        )
        tip_link = str(raw.get('tip_link', ''))
        base_link = str(raw.get('base_link', ''))
        poses = model.forward(initial)
        if tip_link not in poses or base_link not in poses:
            raise ValueError(f'{name}: base or tip link is absent from generated URDF')
        result[name] = UiRobotProfile(
            name=name,
            display_name=DISPLAY_NAMES.get(name, name.upper()),
            profile=str(raw.get('profile', name)),
            joint_names=joint_names,
            joint_limits=tuple(limits),
            initial_positions=initial,
            tip_link=tip_link,
            base_link=base_link,
            urdf=str(urdf_path.resolve()),
            model=model,
        )
    return result


class MultiRobotGuiNode(Node):
    """
    Simulation-only backend shared by the four profile-driven GUI views.

    Public API used by the Qt frontend:

    - ``robot_names``: ordered tuple of robots in the runtime manifest;
    - ``profile(name)``: immutable profile metadata;
    - ``snapshot(name)``: current state/proximity/decision view;
    - ``send_joint_target`` and ``send_joint_delta``: supervised joint input;
    - ``send_cartesian_target``: URDF-driven local IK followed by the same
      supervised joint-candidate path.
    """

    def __init__(
        self,
        manifest_path: str | None = None,
        use_sim_time: bool | None = None,
    ):
        overrides = []
        if use_sim_time is not None:
            overrides.append(Parameter('use_sim_time', value=use_sim_time))
        super().__init__('multi_robot_gui', parameter_overrides=overrides)
        self.declare_parameter('manifest_path', manifest_path or '')
        configured = manifest_path or str(self.get_parameter('manifest_path').value)
        self._profiles = load_ui_profiles(configured)
        self.robot_names = tuple(self._profiles)
        self._states: dict[str, _LiveState] = {}
        self._proximity: dict[str, _Proximity] = {}
        self._commands: dict[str, _TrackedCommand] = {}
        self._input_counts = {
            name: {'joint_state': 0, 'proximity': 0, 'decision': 0}
            for name in self.robot_names
        }
        self._input_errors = {
            name: {'joint_state': '', 'proximity': ''}
            for name in self.robot_names
        }
        self._candidate_publishers = {}
        self._subscriptions_keepalive = []
        for name in self.robot_names:
            self._candidate_publishers[name] = self.create_publisher(
                JointCommand, f'/{name}/thesis/candidate_command', 10
            )
            self._subscriptions_keepalive.extend([
                self.create_subscription(
                    JointState,
                    f'/{name}/joint_states',
                    lambda message, robot=name: self._state_callback(robot, message),
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    ProximityStatus,
                    f'/{name}/thesis/proximity_status',
                    lambda message, robot=name: self._proximity_callback(
                        robot, message
                    ),
                    10,
                ),
                self.create_subscription(
                    CommandDecision,
                    f'/{name}/thesis/command_decision',
                    lambda message, robot=name: self._decision_callback(
                        robot, message
                    ),
                    10,
                ),
            ])

    def profile(self, name: str) -> UiRobotProfile:
        """Return validated metadata for one robot name."""
        if name not in self._profiles:
            raise KeyError(f'unknown robot: {name}')
        return self._profiles[name]

    def _state_callback(self, name: str, message: JointState) -> None:
        self._input_counts[name]['joint_state'] += 1
        try:
            profile = self.profile(name)
            if len(message.name) != len(message.position):
                raise ValueError('joint state arrays do not match')
            if len(set(message.name)) != len(message.name):
                raise ValueError('joint state contains duplicate names')
            positions_by_name = dict(zip(message.name, message.position))
            velocities_by_name = dict(zip(message.name, message.velocity))
            positions = _finite_vector(
                [positions_by_name[joint] for joint in profile.joint_names],
                len(profile.joint_names),
                'joint state positions',
            )
            velocities = _finite_vector(
                [velocities_by_name.get(joint, 0.0) for joint in profile.joint_names],
                len(profile.joint_names),
                'joint state velocities',
            )
            stamp_ns = _stamp_ns(message.header.stamp)
            now_ns = self.get_clock().now().nanoseconds
            if not stamp_is_fresh(now_ns, stamp_ns):
                raise ValueError('joint state timestamp is stale')
            self._states[name] = _LiveState(
                positions, velocities, time.monotonic(), stamp_ns
            )
            self._input_errors[name]['joint_state'] = ''
        except (KeyError, TypeError, ValueError) as error:
            self._input_errors[name]['joint_state'] = str(error)
            self._states.pop(name, None)

    def _proximity_callback(self, name: str, message: ProximityStatus) -> None:
        self._input_counts[name]['proximity'] += 1
        try:
            state = str(message.state)
            clearance = float(message.minimum_clearance)
            stamp_ns = _stamp_ns(message.stamp)
            if state not in {'ALLOW', 'WARNING', 'REDUCTION', 'STOP'}:
                raise ValueError('unknown proximity state')
            if not math.isfinite(clearance):
                raise ValueError('non-finite proximity clearance')
            if not stamp_is_fresh(self.get_clock().now().nanoseconds, stamp_ns):
                raise ValueError('proximity timestamp is stale')
            self._proximity[name] = _Proximity(
                state, clearance, time.monotonic(), stamp_ns
            )
            self._input_errors[name]['proximity'] = ''
        except (TypeError, ValueError) as error:
            self._input_errors[name]['proximity'] = str(error)
            self._proximity.pop(name, None)

    def _decision_callback(self, name: str, message: CommandDecision) -> None:
        self._input_counts[name]['decision'] += 1
        tracked = self._commands.get(name)
        if tracked is None or message.command_id != tracked.command_id:
            return
        tracked.accepted = bool(message.accepted)
        tracked.decision_state = str(message.state)
        tracked.reason_code = str(message.reason_code)
        tracked.reason = str(message.reason)

    def diagnostics(self, name: str) -> dict[str, object]:
        """Return compact callback/clock diagnostics for validation evidence."""
        self.profile(name)
        return {
            'use_sim_time': bool(self.get_parameter('use_sim_time').value),
            'clock_nanoseconds': self.get_clock().now().nanoseconds,
            'input_counts': dict(self._input_counts[name]),
            'input_errors': dict(self._input_errors[name]),
        }

    def _fresh_state(self, name: str) -> _LiveState:
        state = self._states.get(name)
        if state is None or time.monotonic() - state.received_at > MAX_AGE:
            raise RuntimeError(f'{name}: joint state is unavailable or stale')
        if not stamp_is_fresh(self.get_clock().now().nanoseconds, state.stamp_ns):
            raise RuntimeError(f'{name}: joint state simulation timestamp is stale')
        return state

    def _fresh_proximity(self, name: str) -> _Proximity:
        proximity = self._proximity.get(name)
        if proximity is None or time.monotonic() - proximity.received_at > MAX_AGE:
            raise RuntimeError(f'{name}: proximity state is unavailable or stale')
        if not stamp_is_fresh(self.get_clock().now().nanoseconds, proximity.stamp_ns):
            raise RuntimeError(f'{name}: proximity simulation timestamp is stale')
        if proximity.state not in {'ALLOW', 'WARNING'}:
            raise RuntimeError(f'{name}: supervisor state is {proximity.state}')
        return proximity

    def send_joint_target(
        self, name: str, positions: Sequence[float], duration: float = 2.0
    ) -> str:
        """Publish one bounded, namespaced candidate after fail-closed checks."""
        profile = self.profile(name)
        state = self._fresh_state(name)
        self._fresh_proximity(name)
        target = _finite_vector(positions, len(profile.joint_names), 'joint target')
        duration = float(duration)
        if not math.isfinite(duration) or not 0.1 <= duration <= 30.0:
            raise ValueError('duration must be within [0.1, 30] seconds')
        for current, desired, (lower, upper, velocity) in zip(
            state.positions, target, profile.joint_limits
        ):
            if not lower <= desired <= upper:
                raise ValueError('joint target lies outside the selected profile limits')
            if abs(desired - current) / duration > min(MAX_SPEED, velocity) + 1.0e-9:
                raise ValueError('joint target exceeds the conservative velocity limit')
        if self._candidate_publishers[name].get_subscription_count() < 1:
            raise RuntimeError(f'{name}: profile supervisor is not available')
        command = JointCommand()
        command.command_id = f'gui-{name}-{uuid.uuid4().hex}'
        command.stamp = self.get_clock().now().to_msg()
        command.joint_names = list(profile.joint_names)
        command.positions = list(target)
        duration_ns = round(duration * 1.0e9)
        command.duration.sec, command.duration.nanosec = divmod(
            duration_ns, 1_000_000_000
        )
        self._commands[name] = _TrackedCommand(
            command.command_id, target, time.monotonic()
        )
        self._candidate_publishers[name].publish(command)
        return command.command_id

    def send_joint_delta(
        self, name: str, joint_index: int, delta: float, duration: float = 2.0
    ) -> str:
        """Offset a zero-based joint index from the latest fresh state."""
        state = self._fresh_state(name)
        if not isinstance(joint_index, int) or not 0 <= joint_index < len(state.positions):
            raise ValueError('joint_index is outside the selected profile')
        delta = float(delta)
        if not math.isfinite(delta):
            raise ValueError('joint delta must be finite')
        target = list(state.positions)
        target[joint_index] += delta
        return self.send_joint_target(name, target, duration)

    def send_cartesian_target(
        self,
        name: str,
        xyz: Sequence[float],
        rpy: Sequence[float] | None = None,
        duration: float = 2.0,
        position_only: bool = False,
    ) -> str:
        """Solve profile URDF IK and submit the result through supervision."""
        profile = self.profile(name)
        state = self._fresh_state(name)
        self._fresh_proximity(name)
        translation = _finite_vector(xyz, 3, 'Cartesian XYZ target')
        current = profile.model.forward(state.positions)[profile.tip_link]
        target = current.copy()
        translation_step = float(
            np.linalg.norm(np.asarray(translation) - current[:3, 3])
        )
        if translation_step > MAX_CARTESIAN_POSITION_STEP + 1.0e-9:
            raise ValueError(
                'Cartesian position step exceeds the 0.10 m interface limit'
            )
        target[:3, 3] = np.asarray(translation)
        if rpy is not None:
            angles = _finite_vector(rpy, 3, 'Cartesian RPY target')
            target_rotation = Rotation.from_euler('xyz', angles).as_matrix()
            orientation_step = float(
                np.linalg.norm(
                    Rotation.from_matrix(
                        target_rotation @ current[:3, :3].T
                    ).as_rotvec()
                )
            )
            if orientation_step > MAX_CARTESIAN_ORIENTATION_STEP + 1.0e-9:
                raise ValueError(
                    'Cartesian orientation step exceeds the 20 degree '
                    'interface limit'
                )
            target[:3, :3] = target_rotation
        positions = profile.model.inverse(
            target, state.positions, profile.tip_link, position_only=position_only
        )
        return self.send_joint_target(name, positions, duration)

    def snapshot(self, name: str) -> UiRobotSnapshot:
        """Return a defensive snapshot suitable for rendering from a Qt timer."""
        profile = self.profile(name)
        now = time.monotonic()
        state = self._states.get(name)
        state_age = None if state is None else now - state.received_at
        state_fresh = state is not None and state_age <= MAX_AGE
        if state_fresh:
            state_fresh = stamp_is_fresh(
                self.get_clock().now().nanoseconds, state.stamp_ns
            )
        proximity = self._proximity.get(name)
        proximity_fresh = (
            proximity is not None
            and now - proximity.received_at <= MAX_AGE
            and stamp_is_fresh(
                self.get_clock().now().nanoseconds, proximity.stamp_ns
            )
        )
        tracked = self._commands.get(name)
        error = None
        command_status = 'IDLE'
        if tracked is not None:
            if state_fresh:
                error = max(
                    abs(actual - desired)
                    for actual, desired in zip(state.positions, tracked.target)
                )
            if tracked.accepted is False:
                command_status = 'REJECTED'
            elif (
                tracked.accepted is True
                and tracked.reason_code == 'SIMULATION_GOAL_SUCCEEDED'
                and error is not None
                and error <= TRACKING_TOLERANCE
            ):
                command_status = 'REACHED'
            elif tracked.accepted is True:
                command_status = 'TRACKING'
            elif now - tracked.published_at > DECISION_TIMEOUT:
                command_status = 'DECISION_TIMEOUT'
            else:
                command_status = 'PENDING'
        tool_xyz = None
        tool_rpy = None
        if state_fresh:
            transform = profile.model.forward(state.positions)[profile.tip_link]
            tool_xyz = tuple(float(value) for value in transform[:3, 3])
            tool_rpy = tuple(
                float(value)
                for value in Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
            )
        return UiRobotSnapshot(
            name=name,
            positions=None if state is None else state.positions,
            velocities=None if state is None else state.velocities,
            state_fresh=bool(state_fresh),
            state_age=state_age,
            proximity_state=(proximity.state if proximity_fresh else 'STALE'),
            proximity_fresh=bool(proximity_fresh),
            clearance=(proximity.clearance if proximity_fresh else None),
            decision_state=(tracked.decision_state if tracked is not None else 'IDLE'),
            decision_code=(tracked.reason_code if tracked is not None else 'IDLE'),
            decision_reason=(
                tracked.reason if tracked is not None else 'Sin comando activo.'
            ),
            command_id=(tracked.command_id if tracked is not None else ''),
            command_status=command_status,
            target_positions=(tracked.target if tracked is not None else None),
            maximum_target_error=error,
            tool_xyz=tool_xyz,
            tool_rpy=tool_rpy,
        )
