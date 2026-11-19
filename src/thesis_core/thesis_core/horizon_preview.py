"""Render the nominal or accepted short-horizon arm occupancy volume."""

import math
import time
from typing import Callable, Optional, Sequence, Tuple

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Point
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments
from thesis_core.swept_volume import (
    build_swept_volume,
    maximum_endpoint_travel,
)
from thesis_core.joint_model import (
    CONTINUOUS_JOINTS,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
)
from thesis_core.joint_prediction import (
    is_state_fresh,
    predict_joint_samples,
)
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import (
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
    JointTrajectoryPrediction,
    ProximityStatus,
)


FRAME_ID = 'j2n6s300_link_base'
VALID_SOURCES = ('auto', 'intent', 'execution')
STATE_COLORS = {
    'ALLOW': (0.10, 0.65, 1.00, 0.14),
    'WARNING': (1.00, 0.82, 0.05, 0.20),
    'REDUCTION': (1.00, 0.35, 0.02, 0.24),
    'STOP': (1.00, 0.03, 0.03, 0.32),
}


def movement_directions(
    measured: Sequence[float], target: Sequence[float],
) -> Tuple[int, ...]:
    """Determine target motion signs while ignoring encoder scale noise."""
    return tuple(
        1 if goal - current > 1e-3 else
        -1 if current - goal > 1e-3 else 0
        for current, goal in zip(measured, target)
    )


def direction_reversed(
    previous: Optional[Tuple[int, ...]], current: Tuple[int, ...],
) -> bool:
    """Detect a genuine change of direction on a commanded joint."""
    return previous is not None and any(
        before * after < 0 for before, after in zip(previous, current)
    )


def predict_samples(current, target, duration, horizon, count):
    """Compatibility wrapper around the deterministic predictor."""
    return list(predict_joint_samples(
        current,
        target,
        duration,
        horizon,
        count,
    ).positions)


def message_time_seconds(value):
    """Convert a ROS time message to seconds."""
    return float(value.sec) + float(value.nanosec) * 1e-9


def validate_preview_model(
    joint_names: Sequence[str],
    capsule_radii: Sequence[float],
    segment_function: Callable,
) -> Tuple[Tuple[str, ...], Tuple[float, ...]]:
    """Validate the model-independent part of a preview profile."""
    names = tuple(str(name) for name in joint_names)
    radii = tuple(float(radius) for radius in capsule_radii)
    if not names or len(set(names)) != len(names) or any(
        not name for name in names
    ):
        raise ValueError('joint_names must be non-empty and unique')
    if not radii or any(
        not math.isfinite(radius) or radius < 0.0 for radius in radii
    ):
        raise ValueError(
            'capsule_radii must contain finite non-negative values'
        )
    if not callable(segment_function):
        raise TypeError('segment_function must be callable')
    return names, radii


def preview_model_from_profile(profile):
    """Adapt a canonical RobotProfile to swept-volume inputs."""
    capsules = tuple(profile.collision_capsules)
    radii = tuple(float(capsule.radius) for capsule in capsules)
    joint_names, radii = validate_preview_model(
        profile.joint_names,
        radii,
        profile.capsule_segments,
    )
    frame_id = str(profile.world_frame)
    if not frame_id:
        raise ValueError('profile world_frame must be non-empty')

    def segment_function(positions):
        evaluated = tuple(profile.capsule_segments(positions))
        if len(evaluated) != len(capsules):
            raise ValueError(
                'profile capsule metadata and kinematics size mismatch'
            )
        segments = []
        for capsule, evaluated_capsule in zip(capsules, evaluated):
            name, start, end, radius = evaluated_capsule
            if (
                name != capsule.name
                or not math.isclose(
                    float(radius),
                    float(capsule.radius),
                    rel_tol=0.0,
                    abs_tol=1.0e-12,
                )
            ):
                raise ValueError(
                    'profile capsule order or radius changed at runtime'
                )
            segments.append((start, end))
        return tuple(segments)

    return joint_names, radii, segment_function, frame_id


class HorizonPreview(Node):
    """Render discrete future capsules in the model base frame."""

    def __init__(
        self,
        joint_names=JOINT_NAMES,
        capsule_radii=CAPSULE_RADII,
        segment_function=capsule_segments,
    ):
        super().__init__('horizon_preview')
        self.declare_parameter('profile_manifest', '')
        self.declare_parameter('robot_name', 'jaco2')
        profile_manifest = str(
            self.get_parameter('profile_manifest').value
        )
        robot_name = str(self.get_parameter('robot_name').value)
        self.robot_profile = None
        default_frame = FRAME_ID
        if profile_manifest:
            from thesis_core.robot_profile import load_robot_profile

            self.robot_profile = load_robot_profile(
                profile_manifest, robot_name,
            )
            (
                joint_names,
                capsule_radii,
                segment_function,
                default_frame,
            ) = preview_model_from_profile(self.robot_profile)
        self.joint_names, self.capsule_radii = validate_preview_model(
            joint_names, capsule_radii, segment_function,
        )
        if self.robot_profile is None:
            self.position_limits = JOINT_POSITION_LIMITS
            self.velocity_limits = JOINT_VELOCITY_LIMITS
            self.continuous_joints = CONTINUOUS_JOINTS
        else:
            self.position_limits = (
                self.robot_profile.joint_position_limits
            )
            self.velocity_limits = (
                self.robot_profile.joint_velocity_limits
            )
            self.continuous_joints = self.robot_profile.continuous_joints
        self.segment_function = segment_function
        for name, default in (
            ('horizon_sec', 1.0),
            ('rate_hz', 10.0),
            ('samples', 21),
            ('margin_m', 0.02),
            ('spatial_spacing_m', 0.04),
            ('max_state_age_sec', 0.5),
            ('source', 'auto'),
            ('model_frame', default_frame),
            ('marker_topic', '/thesis/horizon_volume'),
            (
                'joint_prediction_topic',
                '/thesis/joint_trajectory_prediction',
            ),
            ('joint_state_topic', '/joint_states'),
            ('preview_intent_topic', '/thesis/preview_intent'),
            (
                'supervised_intent_topic',
                '/thesis/supervised_command',
            ),
            ('nominal_jog_topic', '/thesis/jog_intent'),
            (
                'supervised_jog_topic',
                '/thesis/supervised_jog_command',
            ),
            (
                'execution_trajectory_topic',
                '/thesis/execution_trajectory',
            ),
            ('execution_control_topic', '/thesis/execution_control'),
            ('proximity_topic', '/thesis/proximity_status'),
            (
                'controller_state_topic',
                '/arm_controller/controller_state',
            ),
        ):
            self.declare_parameter(name, default)

        self.horizon = float(self.get_parameter('horizon_sec').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.count = int(self.get_parameter('samples').value)
        self.margin = float(self.get_parameter('margin_m').value)
        self.spacing = float(
            self.get_parameter('spatial_spacing_m').value
        )
        self.max_state_age = float(
            self.get_parameter('max_state_age_sec').value
        )
        self.source = str(self.get_parameter('source').value).lower()
        self.frame_id = str(self.get_parameter('model_frame').value)
        if (not all(math.isfinite(value) for value in (
                self.horizon, self.rate, self.margin, self.spacing,
                self.max_state_age)) or
                not 0 < self.horizon <= 5 or
                not 0 < self.rate <= 30 or
                not 2 <= self.count <= 101 or
                not 0 <= self.margin <= 0.2 or
                not 0 < self.spacing <= 0.10 or
                not 0 < self.max_state_age <= 5.0 or
                self.source not in VALID_SOURCES or not self.frame_id):
            raise ValueError('Invalid preview parameters')

        self.state = None
        self.intent = None
        self.supervised_intent = None
        self.nominal_jog = None
        self.jog = None
        self._previous_jog_direction = None
        self.execution = None
        self.control = None
        self.proximity_status = None
        self.controller_feedback = None
        self.state_time = -math.inf
        self.state_stamp = None
        self.intent_time = -math.inf
        self.supervised_intent_time = -math.inf
        self.nominal_jog_time = -math.inf
        self.jog_time = -math.inf
        self.execution_time = -math.inf
        self.control_time = -math.inf
        self.proximity_time = -math.inf
        self.controller_time = -math.inf
        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )
        self.prediction_publisher = self.create_publisher(
            JointTrajectoryPrediction,
            str(self.get_parameter('joint_prediction_topic').value),
            10,
        )
        self.state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self.receive_state,
            10,
        )
        self.intent_sub = self.create_subscription(
            JointCommand,
            str(self.get_parameter('preview_intent_topic').value),
            self.receive_intent,
            10,
        )
        self.supervised_intent_sub = self.create_subscription(
            JointCommand,
            str(self.get_parameter('supervised_intent_topic').value),
            self.receive_supervised_intent,
            10,
        )
        jog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.jog_sub = self.create_subscription(
            JointCommand,
            str(self.get_parameter('supervised_jog_topic').value),
            self.receive_jog,
            jog_qos,
        )
        self.nominal_jog_sub = self.create_subscription(
            JointCommand,
            str(self.get_parameter('nominal_jog_topic').value),
            self.receive_nominal_jog,
            jog_qos,
        )
        self.execution_sub = self.create_subscription(
            ExecutionTrajectory,
            str(self.get_parameter('execution_trajectory_topic').value),
            self.receive_execution,
            10,
        )
        self.control_sub = self.create_subscription(
            ExecutionControl,
            str(self.get_parameter('execution_control_topic').value),
            self.receive_control,
            10,
        )
        self.proximity_sub = self.create_subscription(
            ProximityStatus,
            str(self.get_parameter('proximity_topic').value),
            self.receive_proximity,
            10,
        )
        self.controller_sub = self.create_subscription(
            JointTrajectoryControllerState,
            str(self.get_parameter('controller_state_topic').value),
            self.receive_controller_state,
            10,
        )
        # The preview is a user-facing visualization.  Scheduling it from the
        # simulated ROS clock makes its apparent refresh rate fall with
        # Gazebo's real-time factor (10 Hz became roughly 6--7 wall-clock Hz
        # under load).  Keep message stamps on ROS time, but drive rendering
        # from a monotonic clock so RViz remains fluid without changing any
        # safety or trajectory semantics.
        self.preview_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1 / self.rate,
            self.tick,
            clock=self.preview_clock,
        )
        self.previous_tick = None
        self._last_ros_time = None
        self._last_clock_advance = time.monotonic()
        self._clock_valid = False
        self.get_logger().info(
            f'Horizon preview started: robot={robot_name}, '
            f'source={self.source}, '
            f'horizon={self.horizon:.2f}s, rate={self.rate:.1f}Hz'
        )

    def receive_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if all(joint in values and math.isfinite(values[joint])
               for joint in self.joint_names):
            self.state = tuple(
                values[joint] for joint in self.joint_names
            )
            self.state_stamp = msg.header.stamp
            self.state_time = time.monotonic()

    def receive_intent(self, msg):
        self.intent = (
            msg if tuple(msg.joint_names) == self.joint_names else None
        )
        self.intent_time = time.monotonic()

    def receive_supervised_intent(self, msg):
        self.supervised_intent = (
            msg if tuple(msg.joint_names) == self.joint_names else None
        )
        self.supervised_intent_time = time.monotonic()

    def receive_nominal_jog(self, msg):
        self.nominal_jog = (
            msg if tuple(msg.joint_names) == self.joint_names
            and len(msg.positions) == len(self.joint_names) else None
        )
        self.nominal_jog_time = time.monotonic()

    def receive_jog(self, msg):
        self.jog = (
            msg if tuple(msg.joint_names) == self.joint_names
            and len(msg.positions) == len(self.joint_names) else None
        )
        now = time.monotonic()
        self.jog_time = now
        if self.jog is None or self.state is None:
            self._previous_jog_direction = None
            return
        current = movement_directions(self.state, msg.positions)
        reversed_motion = direction_reversed(
            self._previous_jog_direction, current,
        )
        self._previous_jog_direction = current
        if (not reversed_motion or not self.state_is_fresh(now)
                or not getattr(self, '_clock_valid', False)):
            return
        now_ros = self.get_clock().now().nanoseconds / 1e9
        if (self.state_stamp is None or not
                -0.05 <= now_ros - message_time_seconds(self.state_stamp)
                <= self.max_state_age):
            return
        if self.select_source(now, now_ros) != 'jog':
            return
        try:
            _, metadata, prediction = self.make_poses('jog', now_ros)
        except (TypeError, ValueError, ZeroDivisionError):
            return
        self.publish_joint_prediction(
            prediction, metadata, self.get_clock().now().to_msg(),
        )

    def receive_execution(self, msg):
        if msg.status == 'ACCEPTED':
            self.execution = msg
            self.execution_time = time.monotonic()
        elif (
            msg.status in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED')
            and self.execution is not None
            and msg.command_id == self.execution.command_id
        ):
            # An accepted reference no longer has authority after its
            # terminal result. Otherwise it can mask a later rolling JOG.
            self.execution = None

    def receive_control(self, msg):
        self.control = msg
        self.control_time = time.monotonic()

    def receive_proximity(self, msg):
        self.proximity_status = msg
        self.proximity_time = time.monotonic()

    def receive_controller_state(self, msg):
        positions = dict(zip(
            msg.joint_names,
            msg.feedback.positions,
        ))
        if all(joint in positions and math.isfinite(positions[joint])
               for joint in self.joint_names):
            self.controller_feedback = tuple(
                positions[joint] for joint in self.joint_names
            )
            self.controller_time = time.monotonic()

    def state_is_fresh(self, now):
        return (
            self.state is not None
            and self.state_stamp is not None
            and is_state_fresh(
                self.state_time, now, self.max_state_age
            )
        )

    def intent_is_fresh(self, now):
        return self.intent is not None and now - self.intent_time <= 0.5

    def supervised_intent_is_fresh(self, now):
        return (
            self.supervised_intent is not None
            and now - self.supervised_intent_time <= 0.5
        )

    def jog_is_fresh(self, now):
        return self.jog is not None and now - self.jog_time <= 0.25

    def execution_is_active(self, now_ros):
        if self.execution is None:
            return False
        if tuple(self.execution.joint_names) != self.joint_names:
            return False
        if len(self.execution.start_positions) != len(self.joint_names):
            return False
        if len(self.execution.target_positions) != len(self.joint_names):
            return False
        if not math.isfinite(self.execution.duration_sec):
            return False
        start = message_time_seconds(self.execution.start_time)
        if now_ros <= start + self.execution.duration_sec + 0.5:
            return True
        return (
            self.control is not None
            and self.control.command_id == self.execution.command_id
            and self.control.state == 'STOP'
            and time.monotonic() - self.control_time <= 1.0
        )

    def runtime_state(self):
        if (
            self.control is None
            or time.monotonic() - self.control_time > 1.0
            or self.execution is None
            or self.control.command_id != self.execution.command_id
        ):
            return 'UNKNOWN', None, None
        return (
            self.control.state,
            self.control.speed_scale,
            self.control.minimum_clearance,
        )

    def matching_jog_decision(self, command_id, now_monotonic):
        """Return fresh decision evidence tied to the same nominal JOG."""
        nominal_jog = getattr(self, 'nominal_jog', None)
        control = getattr(self, 'control', None)
        if (
            nominal_jog is None
            or nominal_jog.command_id != command_id
            or now_monotonic - getattr(
                self, 'nominal_jog_time', -math.inf
            ) > 0.25
            or control is None
            or control.command_id != command_id
            or now_monotonic - getattr(
                self, 'control_time', -math.inf
            ) > 1.0
            or message_time_seconds(control.intent_stamp)
            != message_time_seconds(nominal_jog.stamp)
            or not math.isfinite(control.predictive_threshold)
            or control.predictive_threshold < 0.0
        ):
            return None
        return control

    def runtime_minimum(self):
        """Return where the supervisor found its minimum in the horizon."""
        if (
            self.control is None
            or time.monotonic() - self.control_time > 1.0
            or self.execution is None
            or self.control.command_id != self.execution.command_id
            or self.control.minimum_sample_count < 2
        ):
            return None
        sample_count = int(self.control.minimum_sample_count)
        sample_number = min(
            int(self.control.minimum_sample_index) + 1,
            sample_count,
        )
        return (
            float(self.control.minimum_time_from_now),
            sample_number,
            sample_count,
            float(self.control.protective_margin),
            float(self.control.time_to_protective_volume),
            float(self.control.time_to_collision),
        )

    def select_source(self, now_monotonic, now_ros):
        execution_active = self.execution_is_active(now_ros)
        supervised_active = self.supervised_intent_is_fresh(now_monotonic)
        intent_active = self.intent_is_fresh(now_monotonic)
        if self.source == 'execution':
            return 'execution' if execution_active else 'zero'
        if self.source == 'intent':
            if supervised_active:
                return 'supervised_intent'
            return 'intent' if intent_active else 'zero'
        if execution_active:
            return 'execution'
        if self.jog_is_fresh(now_monotonic):
            return 'jog'
        if supervised_active:
            return 'supervised_intent'
        if intent_active:
            return 'intent'
        return 'zero'

    def make_poses(self, source, now_ros):
        if source in ('intent', 'supervised_intent', 'jog', 'zero'):
            if source == 'zero':
                command = None
                target = self.state
                duration = self.horizon
                prediction_source = 'zero'
            else:
                command = {
                    'intent': self.intent,
                    'supervised_intent': self.supervised_intent,
                    'jog': self.jog,
                }[source]
                target = tuple(command.positions)
                duration = message_time_seconds(command.duration)
                prediction_source = (
                    'jog' if source == 'jog' else 'intent'
                )
            prediction = predict_joint_samples(
                self.state,
                target,
                duration,
                self.horizon,
                self.count,
                joint_names=self.joint_names,
                position_limits=self.position_limits,
                velocity_limits=self.velocity_limits,
                continuous_joints=self.continuous_joints,
            )
            titles = {
                'intent': 'INTENCIÓN NOMINAL',
                'supervised_intent': 'INTENCIÓN SUPERVISADA',
                'jog': 'JOG SUPERVISADO',
                'zero': 'VELOCIDAD CERO',
            }
            details = {
                'intent': 'previsualización nominal de la interfaz',
                'supervised_intent': 'intención limitada por el supervisor',
                'jog': (f'horizonte aprobado por supervisor: {duration:.2f}s; '
                        'no confirma movimiento real'),
                'zero': 'sin intención vigente; postura medida constante',
            }
            if source == 'jog':
                decision = HorizonPreview.matching_jog_decision(
                    self,
                    command.command_id, time.monotonic()
                )
                if decision is not None:
                    details[source] += (
                        f'\nSupervisor {decision.state}: '
                        f'escala={decision.speed_scale:.2f}; '
                        f'd_nominal={decision.nominal_clearance:.3f}m; '
                        f'umbral={decision.predictive_threshold:.3f}m; '
                        f'{decision.reason_code}'
                    )
            return list(prediction.positions), {
                'title': titles[source],
                'detail': details[source],
                'command_id': '' if command is None else command.command_id,
                'elapsed': None,
                'tracking_error': None,
                'prediction_source': prediction_source,
            }, prediction

        target = tuple(self.execution.target_positions)
        start_time = message_time_seconds(self.execution.start_time)
        elapsed = max(0.0, now_ros - start_time)
        remaining = max(
            float(self.execution.duration_sec) - elapsed,
            1.0e-6,
        )
        runtime_state, speed_scale, clearance = self.runtime_state()
        runtime_minimum = self.runtime_minimum()
        if runtime_state == 'UNKNOWN':
            raise ValueError('No hay decisión de ejecución reciente y correlacionada')
        if runtime_state == 'STOP':
            prediction_target = self.state
            prediction_duration = self.horizon
            prediction_source = 'zero'
        else:
            prediction_target = target
            prediction_duration = remaining / max(speed_scale, 1.0e-6)
            prediction_source = 'execution'
        prediction = predict_joint_samples(
            self.state,
            prediction_target,
            prediction_duration,
            self.horizon,
            self.count,
            joint_names=self.joint_names,
            position_limits=self.position_limits,
            velocity_limits=self.velocity_limits,
            continuous_joints=self.continuous_joints,
        )
        clearance_text = (
            'n/a' if clearance is None else f'{clearance:.3f}m'
        )
        minimum_text = ''
        if runtime_minimum is not None:
            (
                minimum_time,
                sample_number,
                sample_count,
                protective_margin,
                protective_ttc,
                collision_ttc,
            ) = runtime_minimum
            protective_text = (
                'n/a' if protective_ttc < 0.0
                else f'{protective_ttc:.2f}s'
            )
            collision_text = (
                'n/a' if collision_ttc < 0.0
                else f'{collision_ttc:.2f}s'
            )
            minimum_text = (
                f'\nt_min=+{minimum_time:.2f}s '
                f'muestra={sample_number}/{sample_count}; '
                f'margen={protective_margin:.3f}m; '
                f'TTCp={protective_text}; TTCc={collision_text}'
            )
        tracking_error = None
        if (self.controller_feedback is not None and
                time.monotonic() - self.controller_time <= 0.5):
            tracking_error = max(
                abs(a - b)
                for a, b in zip(target, self.controller_feedback)
            )
        return list(prediction.positions), {
            'title': 'REFERENCIA ACEPTADA',
            'detail': (
                f'control={runtime_state} escala={speed_scale:.2f}; '
                f'd_futuro={clearance_text}{minimum_text}'
            ),
            'command_id': self.execution.command_id,
            'elapsed': elapsed,
            'tracking_error': tracking_error,
            'runtime_state': runtime_state,
            'prediction_source': prediction_source,
        }, prediction

    def publish_joint_prediction(self, prediction, metadata, stamp):
        output = JointTrajectoryPrediction()
        output.stamp = stamp
        output.origin_stamp = self.state_stamp
        output.command_id = metadata['command_id']
        output.source = metadata['prediction_source']
        output.joint_names = list(self.joint_names)
        output.sample_count = len(prediction.positions)
        output.sample_period = prediction.sample_period
        output.horizon = prediction.horizon
        output.effective_duration = prediction.effective_duration
        output.sample_times = list(prediction.sample_times)
        output.positions = [
            value
            for sample in prediction.positions
            for value in sample
        ]
        self.prediction_publisher.publish(output)

    def empty_output(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        return MarkerArray(markers=[clear])

    def invalidate_output(self, stamp):
        """Explicitly revoke the last structured prediction and markers."""
        output = JointTrajectoryPrediction()
        output.stamp = stamp
        output.source = 'invalid'
        self.prediction_publisher.publish(output)
        self.publisher.publish(self.empty_output())

    def reset_inputs(self):
        """Require new state and commands after a clock discontinuity."""
        self.state = self.intent = self.supervised_intent = None
        self.nominal_jog = self.jog = None
        self.execution = self.control = self.proximity_status = None
        self.controller_feedback = None
        self.state_stamp = None
        self._previous_jog_direction = None

    def clock_is_valid(self, now_ros, now_wall):
        if self._last_ros_time is None:
            self._last_ros_time = now_ros
            self._last_clock_advance = now_wall
            return False
        if now_ros < self._last_ros_time:
            self.reset_inputs()
            self._last_ros_time = now_ros
            self._last_clock_advance = now_wall
            return False
        if now_ros > self._last_ros_time:
            self._last_clock_advance = now_wall
        self._last_ros_time = now_ros
        if now_ros <= 0.0 or now_wall - self._last_clock_advance > self.max_state_age:
            self.reset_inputs()
            return False
        return True

    def visualization_state(self, source, now_monotonic):
        if source == 'execution':
            return self.runtime_state()[0]
        if source == 'jog':
            decision = (
                HorizonPreview.matching_jog_decision(
                    self, self.jog.command_id, now_monotonic
                )
                if self.jog is not None else None
            )
            if (
                decision is not None
                and decision.state.upper() in STATE_COLORS
            ):
                return decision.state.upper()
            return 'UNKNOWN'
        if (
            self.proximity_status is not None
            and now_monotonic - self.proximity_time <= 0.5
            and self.proximity_status.state.upper() in STATE_COLORS
        ):
            return self.proximity_status.state.upper()
        return 'UNKNOWN'

    def append_volume(self, output, volume, state, stamp):
        # Geometry keeps a stable visual identity independently of risk state.
        red, green, blue, predicted_alpha = (0.10, 0.65, 1.00, 0.14)
        for index, capsule in enumerate(volume.current_capsules):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'current_capsules'
            marker.id = index
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * capsule.radius
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = min(0.48, predicted_alpha + 0.20)
            marker.lifetime.sec = 1
            marker.points = [
                Point(x=x, y=y, z=z) for x, y, z in capsule.points
            ]
            output.markers.append(marker)

        for index, points in enumerate(volume.predicted_points):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'predicted_volume'
            marker.id = index
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * volume.samples[0][index].radius
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = predicted_alpha
            marker.lifetime.sec = 1
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
            output.markers.append(marker)

    def append_time_traces(self, output, volume, stamp):
        maximum_travel = maximum_endpoint_travel(volume)
        for link, endpoints in enumerate(volume.endpoint_traces):
            trace = Marker()
            trace.header.frame_id = self.frame_id
            trace.header.stamp = stamp
            trace.ns = 'horizon_endpoint_traces'
            trace.id = link
            trace.type = Marker.LINE_STRIP
            trace.pose.orientation.w = 1.0
            trace.scale.x = 0.006
            trace.color.r, trace.color.g, trace.color.b = 1.0, 0.8, 0.1
            trace.color.a = 0.9
            trace.lifetime.sec = 1
            trace.points = [Point(x=x, y=y, z=z) for x, y, z in endpoints]
            output.markers.append(trace)

        for slot, fraction in enumerate((0.25, 0.5, 1.0)):
            index = round(fraction * (len(volume.samples) - 1))
            skeleton = Marker()
            skeleton.header.frame_id = self.frame_id
            skeleton.header.stamp = stamp
            skeleton.ns = 'horizon_time_poses'
            skeleton.id = slot
            skeleton.type = Marker.LINE_LIST
            skeleton.pose.orientation.w = 1.0
            skeleton.scale.x = 0.008
            skeleton.color.r = fraction
            skeleton.color.g = 1.0 - 0.5 * fraction
            skeleton.color.b = 1.0
            skeleton.color.a = 0.9
            skeleton.lifetime.sec = 1
            for capsule in volume.samples[index]:
                skeleton.points.extend([
                    Point(
                        x=capsule.start[0],
                        y=capsule.start[1],
                        z=capsule.start[2],
                    ),
                    Point(
                        x=capsule.end[0],
                        y=capsule.end[1],
                        z=capsule.end[2],
                    ),
                ])
            output.markers.append(skeleton)

            label = Marker()
            label.header = skeleton.header
            label.ns = 'horizon_labels'
            label.id = slot
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.orientation.w = 1.0
            endpoint = volume.samples[index][-1].end
            label.pose.position = Point(
                x=endpoint[0],
                y=endpoint[1],
                z=endpoint[2] + 0.05 + slot * 0.035,
            )
            label.scale.z = 0.025
            label.color = skeleton.color
            label.lifetime.sec = 1
            label.text = f'+{fraction * self.horizon:.2f} s'
            output.markers.append(label)
        return maximum_travel

    def append_nominal_predictive_envelope(self, output, volume, stamp):
        """Draw nominal JOG geometry expanded by the decision threshold."""
        for index, points in enumerate(volume.predicted_points):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'nominal_predictive_envelope'
            marker.id = index
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * volume.samples[0][index].radius
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color.r = 1.0
            marker.color.g = 0.38
            marker.color.b = 0.02
            marker.color.a = 0.18
            marker.lifetime.sec = 1
            marker.points = [
                Point(x=x, y=y, z=z) for x, y, z in points
            ]
            output.markers.append(marker)

    def append_label(
        self,
        output,
        source,
        description,
        metadata,
        hz,
        elapsed,
        compute_ms,
        state,
    ):
        label = Marker()
        stamp = self.get_clock().now().to_msg()
        label.header.frame_id = self.frame_id
        label.header.stamp = stamp
        label.ns = 'horizon_labels'
        label.id = 100
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.orientation.w = 1.0
        label.pose.position.z = 1.5
        label.scale.z = 0.045
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 1.0
        label.lifetime.sec = 1
        common = (
            f'state={state} cycle={hz:.1f}Hz '
            f'compute={compute_ms:.1f}ms'
        )
        if source == 'execution':
            error = metadata['tracking_error']
            error_text = 'n/a' if error is None else f'{error:.3f}rad'
            label.text = (
                f'EJECUCIÓN H={self.horizon:.1f}s N={self.count} '
                f'cmd={metadata["command_id"]}\n'
                f't={elapsed:.2f}s {common} error={error_text}\n'
                f'{description}'
            )
        else:
            title = (
                'JOG SUPERVISADO'
                if source == 'jog'
                else 'PREVIEW'
            )
            command = (
                f' cmd={metadata["command_id"]}'
                if source == 'jog' else ''
            )
            label.text = (
                f'{title} H={self.horizon:.1f}s N={self.count}'
                f'{command} '
                f'margin={self.margin:.3f}m\n'
                f'{common}\n{description}'
            )
        output.markers.append(label)

    def tick(self):
        start = time.monotonic()
        hz = 0.0 if self.previous_tick is None else 1 / max(
            start - self.previous_tick,
            1e-9,
        )
        self.previous_tick = start
        output = self.empty_output()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        self._clock_valid = self.clock_is_valid(now_ros, start)
        stamp = self.get_clock().now().to_msg()
        if (not self._clock_valid or not self.state_is_fresh(start)
                or not -0.05 <= now_ros - message_time_seconds(self.state_stamp)
                <= self.max_state_age):
            self.invalidate_output(stamp)
            return

        source = self.select_source(start, now_ros)
        if source is None:
            self.invalidate_output(stamp)
            return

        try:
            poses, metadata, prediction = self.make_poses(
                source, now_ros
            )
        except (TypeError, ValueError, ZeroDivisionError):
            self.invalidate_output(stamp)
            return

        self.publish_joint_prediction(prediction, metadata, stamp)
        volume = build_swept_volume(
            poses,
            self.margin,
            self.spacing,
            capsule_radii=self.capsule_radii,
            segment_function=self.segment_function,
            joint_count=len(self.joint_names),
        )
        jog_decision = (
            self.matching_jog_decision(self.jog.command_id, start)
            if source == 'jog' and self.jog is not None else None
        )
        if (
            jog_decision is not None
            and jog_decision.state in ('REDUCTION', 'STOP')
            and jog_decision.reason_code in (
                'PREDICTIVE_INTRUSION',
                'NO_SAFE_SCALE',
                'CURRENT_CLEARANCE_STOP',
            )
        ):
            try:
                nominal_prediction = predict_joint_samples(
                    self.state,
                    self.nominal_jog.positions,
                    message_time_seconds(self.nominal_jog.duration),
                    self.horizon,
                    self.count,
                    joint_names=self.joint_names,
                    position_limits=self.position_limits,
                    velocity_limits=self.velocity_limits,
                    continuous_joints=self.continuous_joints,
                )
                nominal_volume = build_swept_volume(
                    nominal_prediction.positions,
                    jog_decision.predictive_threshold,
                    self.spacing,
                    capsule_radii=self.capsule_radii,
                    segment_function=self.segment_function,
                    joint_count=len(self.joint_names),
                )
                self.append_nominal_predictive_envelope(
                    output, nominal_volume, stamp
                )
                metadata['detail'] += (
                    '\nNaranja: volumen JOG nominal expandido por el '
                    'umbral predictivo del supervisor'
                )
            except (TypeError, ValueError, ZeroDivisionError) as exc:
                self.get_logger().warning(
                    f'Could not render nominal JOG envelope: {exc}'
                )
        state = self.visualization_state(source, start)
        self.append_volume(output, volume, state, stamp)
        travel = self.append_time_traces(output, volume, stamp)
        metadata['detail'] += (
            f'\nRecorrido máximo de extremos: {travel * 100:.1f} cm'
            '\nAmarillo: trazas; líneas de color: posturas futuras'
        )
        compute_ms = (time.monotonic() - start) * 1000
        self.append_label(
            output,
            source,
            metadata['detail'],
            metadata,
            hz,
            metadata['elapsed'] if source == 'execution' else 0.0,
            compute_ms,
            state,
        )
        self.publisher.publish(output)


def main(args=None):
    """Run the short-horizon occupancy visualizer."""
    rclpy.init(args=args)
    spin_node(HorizonPreview())


if __name__ == '__main__':
    main()
