"""ROS adapter for synchronized, fail-closed four-arm coordination."""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from thesis_core.geometry_profile import legacy_jaco_geometry_profile
from thesis_core.geometry_snapshot_contract import (
    geometry_snapshot_from_message,
)
from thesis_core.geometry_snapshot_projection import (
    project_snapshot_to_world,
)
from thesis_core.multiarm_coordination import (
    GeometryExpectation,
    fail_closed_decision,
)
from thesis_core.multiarm_ros_contract import (
    geometry_snapshot_qos,
    geometry_decision_is_valid,
    required_snapshot_frames,
    stamp_is_strictly_newer,
    time_message_key,
    transform_message_matrix,
)
from thesis_core.multiarm_runtime import (
    CANONICAL_ROBOT_IDS,
    RuntimeDecision,
    RuntimeOccupancy,
    evaluate_runtime_batch,
)
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import GeometrySnapshot as GeometrySnapshotMessage
from thesis_interfaces.msg import MultiArmSafetyStatus


def create_dedicated_tf_listener(buffer):
    """Receive TF concurrently while the coordinator waits at exact stamps."""
    return TransformListener(buffer, None, spin_thread=True)


def _duration_message(seconds):
    """Convert a finite non-negative duration to a normalized ROS message."""
    value = float(seconds)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError('duration must be finite and non-negative')
    sec = int(value)
    nanosec = int(round((value - sec) * 1.0e9))
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    return DurationMessage(sec=sec, nanosec=nanosec)


class MultiArmCoordinatorNode(Node):
    """Project four geometry sources and publish one global authorization."""

    def __init__(self):
        super().__init__('multiarm_coordinator')
        legacy = legacy_jaco_geometry_profile()
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('session_id', 'hybrid-three-sim')
        self.declare_parameter('evaluation_rate_hz', 20.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.035)
        self.declare_parameter('clearance_threshold', 0.05)
        self.declare_parameter('maximum_receive_age_sec', 0.30)
        self.declare_parameter('maximum_snapshot_skew_sec', 0.08)
        self.declare_parameter('status_valid_for_sec', 0.12)
        self.declare_parameter(
            'geometry_topics',
            [f'/{robot_id}/geometry_snapshot'
             for robot_id in CANONICAL_ROBOT_IDS],
        )
        self.declare_parameter(
            'expected_source_kinds',
            ['synthetic_state', 'gazebo', 'gazebo', 'gazebo'],
        )
        self.declare_parameter(
            'expected_profile_ids', [legacy.profile_id] * 4
        )
        self.declare_parameter(
            'expected_geometry_versions', [legacy.geometry_version] * 4
        )
        self.declare_parameter(
            'expected_asset_hashes', [legacy.asset_hash] * 4
        )
        self.declare_parameter(
            'status_topic', '/thesis/multiarm_safety_status'
        )

        self.reference_frame = str(
            self.get_parameter('reference_frame').value
        )
        self.session_id = str(self.get_parameter('session_id').value)
        self.rate_hz = float(
            self.get_parameter('evaluation_rate_hz').value
        )
        self.tf_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )
        self.clearance_threshold = float(
            self.get_parameter('clearance_threshold').value
        )
        self.maximum_receive_age_sec = float(
            self.get_parameter('maximum_receive_age_sec').value
        )
        self.maximum_snapshot_skew_sec = float(
            self.get_parameter('maximum_snapshot_skew_sec').value
        )
        self.status_valid_for_sec = float(
            self.get_parameter('status_valid_for_sec').value
        )
        topics = tuple(self.get_parameter('geometry_topics').value)
        source_kinds = tuple(
            self.get_parameter('expected_source_kinds').value
        )
        profile_ids = tuple(
            self.get_parameter('expected_profile_ids').value
        )
        versions = tuple(
            self.get_parameter('expected_geometry_versions').value
        )
        asset_hashes = tuple(
            self.get_parameter('expected_asset_hashes').value
        )
        self._validate_configuration(
            topics, source_kinds, profile_ids, versions, asset_hashes
        )
        self.expected_sources = dict(zip(CANONICAL_ROBOT_IDS, source_kinds))
        self.expected_geometry = {
            robot_id: GeometryExpectation(profile_id, version, asset_hash)
            for robot_id, profile_id, version, asset_hash in zip(
                CANONICAL_ROBOT_IDS, profile_ids, versions, asset_hashes
            )
        }

        self.publisher = self.create_publisher(
            MultiArmSafetyStatus,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self._records = {}
        self._last_snapshot_stamps = {}
        self.geometry_subscriptions = []
        for robot_id, topic in zip(CANONICAL_ROBOT_IDS, topics):
            subscription = self.create_subscription(
                GeometrySnapshotMessage,
                str(topic),
                lambda message, expected=robot_id: self._snapshot_callback(
                    expected, message
                ),
                geometry_snapshot_qos(),
            )
            self.geometry_subscriptions.append(subscription)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = create_dedicated_tf_listener(self.tf_buffer)
        self._sequence = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.evaluate)
        self.get_logger().info(
            'Multi-arm coordinator ready: arm_01 synthetic + '
            'arm_02..arm_04 Gazebo; output is global fail-closed status only'
        )

    def _validate_configuration(
        self, topics, source_kinds, profile_ids, versions, asset_hashes
    ):
        """Reject anonymous, physical, incomplete, or unbounded settings."""
        if self.reference_frame != 'world':
            raise ValueError('reference_frame must be world')
        if not self.session_id or self.session_id != self.session_id.strip():
            raise ValueError('session_id must be a non-empty trimmed string')
        arrays = (topics, source_kinds, profile_ids, versions, asset_hashes)
        if any(len(values) != len(CANONICAL_ROBOT_IDS) for values in arrays):
            raise ValueError(
                'all four-arm parameter arrays must have length four'
            )
        if (
            len(set(topics)) != len(topics)
            or any(not value for value in topics)
        ):
            raise ValueError('geometry_topics must be unique and non-empty')
        if source_kinds != ('synthetic_state', 'gazebo', 'gazebo', 'gazebo'):
            raise ValueError(
                'offline source kinds must be synthetic,gazebo x3'
            )
        scalars = (
            self.rate_hz,
            self.tf_timeout_sec,
            self.maximum_receive_age_sec,
            self.status_valid_for_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in scalars):
            raise ValueError(
                'rates and time bounds must be finite and positive'
            )
        if self.tf_timeout_sec > 1.0 / self.rate_hz:
            raise ValueError('tf timeout must not exceed evaluation period')
        if (
            not math.isfinite(self.maximum_snapshot_skew_sec)
            or self.maximum_snapshot_skew_sec < 0.0
        ):
            raise ValueError(
                'maximum snapshot skew must be finite and non-negative'
            )
        if (
            not math.isfinite(self.clearance_threshold)
            or self.clearance_threshold < 0.0
        ):
            raise ValueError(
                'clearance threshold must be finite and non-negative'
            )

    def _snapshot_callback(self, expected_robot_id, message):
        """Retain only the newest transport sample for the expected topic."""
        if str(message.robot_id) != expected_robot_id:
            self._records.pop(expected_robot_id, None)
            self.get_logger().warning(
                f'Rejected identity {message.robot_id!r} on '
                f'{expected_robot_id} geometry topic'
            )
            return
        try:
            stamp = time_message_key(message.stamp)
        except ValueError:
            self._records.pop(expected_robot_id, None)
            self.get_logger().warning(
                f'Rejected invalid stamp on {expected_robot_id} '
                'geometry topic'
            )
            return
        if not stamp_is_strictly_newer(
            stamp, self._last_snapshot_stamps.get(expected_robot_id)
        ):
            return
        self._last_snapshot_stamps[expected_robot_id] = stamp
        self._records[expected_robot_id] = (message, time.monotonic())

    def _project_record(self, robot_id, record, deadline):
        """Validate one descriptor and resolve every TF at its exact stamp."""
        message, received = record
        if str(message.reference_frame) != self.reference_frame:
            raise ValueError('snapshot reference frame differs from world')
        snapshot = geometry_snapshot_from_message(message)
        if snapshot.profile.robot_id != robot_id:
            raise ValueError('snapshot robot identity differs from its topic')
        transforms = {}
        for frame in required_snapshot_frames(snapshot):
            remaining = max(0.0, deadline - time.monotonic())
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                frame,
                Time.from_msg(message.stamp),
                timeout=Duration(seconds=remaining),
            )
            transforms[frame] = transform_message_matrix(transform)
        occupancy = project_snapshot_to_world(
            snapshot, transforms, self.reference_frame
        )
        return RuntimeOccupancy(
            occupancy,
            str(message.session_id),
            str(message.source_kind),
            received,
        )

    def _stop(self, reason_code):
        return RuntimeDecision(
            fail_closed_decision(reason_code, self.clearance_threshold),
            float('nan'),
            float('nan'),
        )

    def evaluate(self):
        """Publish ALLOW only after one complete, exact-stamp atomic batch."""
        missing = tuple(
            robot_id for robot_id in CANONICAL_ROBOT_IDS
            if robot_id not in self._records
        )
        if missing:
            self._publish(self._stop('GEOMETRY_SOURCE_MISSING'))
            return
        deadline = time.monotonic() + self.tf_timeout_sec
        try:
            values = tuple(
                self._project_record(
                    robot_id, self._records[robot_id], deadline
                )
                for robot_id in CANONICAL_ROBOT_IDS
            )
        except TransformException as exception:
            self.get_logger().warning(f'Multi-arm TF STOP: {exception}')
            self._publish(self._stop('TF_UNAVAILABLE'))
            return
        except (TypeError, ValueError) as exception:
            self.get_logger().warning(f'Multi-arm geometry STOP: {exception}')
            self._publish(self._stop('GEOMETRY_DESCRIPTOR_INVALID'))
            return

        result = evaluate_runtime_batch(
            values,
            now_sec=self.get_clock().now().nanoseconds * 1.0e-9,
            monotonic_now_sec=time.monotonic(),
            clearance_threshold=self.clearance_threshold,
            session_id=self.session_id,
            expected_geometry=self.expected_geometry,
            expected_sources=self.expected_sources,
            maximum_receive_age_sec=self.maximum_receive_age_sec,
            maximum_snapshot_skew_sec=self.maximum_snapshot_skew_sec,
        )
        self._publish(result)

    @staticmethod
    def _finite_or_missing(value):
        return float(value) if math.isfinite(float(value)) else -1.0

    def _publish(self, runtime_result):
        """Encode one decision; admission failures are always invalid STOP."""
        decision = runtime_result.decision
        valid = geometry_decision_is_valid(decision.reason_code)
        message = MultiArmSafetyStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.valid_for = _duration_message(self.status_valid_for_sec)
        message.session_id = self.session_id
        self._sequence += 1
        message.decision_sequence = self._sequence
        message.reference_frame = self.reference_frame
        message.state = decision.state if valid else 'STOP'
        message.valid = valid
        message.reason_code = decision.reason_code
        message.reason = decision.reason_code.replace('_', ' ').lower()
        message.clearance_threshold = decision.clearance_threshold
        message.minimum_clearance = (
            decision.minimum_clearance
            if decision.minimum_clearance is not None else -1.0
        )
        message.maximum_snapshot_age = self._finite_or_missing(
            runtime_result.maximum_snapshot_age
        )
        message.snapshot_skew = self._finite_or_missing(
            runtime_result.snapshot_skew
        )
        if decision.limiting is not None:
            message.first_robot_id = decision.limiting.first_robot_id
            message.first_primitive = decision.limiting.first_primitive
            message.second_robot_id = decision.limiting.second_robot_id
            message.second_primitive = decision.limiting.second_primitive
        message.evaluated_robot_pairs = len(decision.pair_clearances)
        message.evaluated_primitive_pairs = sum(
            pair.evaluated_combinations for pair in decision.pair_clearances
        )
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    spin_node(MultiArmCoordinatorNode())
