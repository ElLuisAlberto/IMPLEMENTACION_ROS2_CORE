"""Publish versioned arm geometry at the exact incoming joint-state stamp."""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from thesis_core.geometry_profile import (
    NOMINAL,
    legacy_jaco_geometry_profile,
    offline_fixture_jaco_geometry_profile,
    synthetic_gripper_jaco_geometry_profile,
)
from thesis_core.multiarm_ros_contract import (
    geometry_snapshot_qos,
    stamp_is_strictly_newer,
    time_message_key,
)
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import GeometrySnapshot
from thesis_simulation.geometry_snapshot_source_contract import (
    complete_joint_sample,
    geometry_transport_manifest,
    validate_offline_identity,
)


def _duration(seconds: float) -> Duration:
    value = float(seconds)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError('valid_for_sec must be finite and positive')
    sec = int(value)
    nanosec = int(round((value - sec) * 1.0e9))
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    return Duration(sec=sec, nanosec=nanosec)


def _vector(values) -> Vector3:
    return Vector3(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
    )


class GeometrySnapshotSourceNode(Node):
    """Bind one trusted joint-state stream to one immutable geometry profile."""

    def __init__(self):
        super().__init__('geometry_snapshot_source')
        self.declare_parameter('robot_id', 'arm_01')
        self.declare_parameter('source_kind', 'synthetic_state')
        self.declare_parameter('tf_prefix', 'arm_01_jaco')
        self.declare_parameter('session_id', 'hybrid-three-sim')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('tool_mode', 'interface')
        self.declare_parameter('state', NOMINAL)
        self.declare_parameter('valid_for_sec', 0.25)
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('geometry_topic', 'geometry_snapshot')
        self.declare_parameter(
            'required_joint_names',
            [f'arm_01_jaco_joint_{index}' for index in range(1, 7)],
        )

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.source_kind = str(self.get_parameter('source_kind').value)
        validate_offline_identity(self.robot_id, self.source_kind)
        self.session_id = str(self.get_parameter('session_id').value)
        if not self.session_id or self.session_id != self.session_id.strip():
            raise ValueError('session_id must be a non-empty trimmed string')
        self.reference_frame = str(
            self.get_parameter('reference_frame').value
        )
        if self.reference_frame != 'world':
            raise ValueError('reference_frame must be world')
        self.state = str(self.get_parameter('state').value)
        self.valid_for = _duration(
            self.get_parameter('valid_for_sec').value
        )
        self.required_joint_names = tuple(
            self.get_parameter('required_joint_names').value
        )
        prefix = str(self.get_parameter('tf_prefix').value)
        expected_prefix = f'{self.robot_id}_jaco'
        if prefix != expected_prefix:
            raise ValueError(
                f'tf_prefix for {self.robot_id} must be {expected_prefix}'
            )
        tool_mode = str(self.get_parameter('tool_mode').value)
        if tool_mode == 'synthetic_gripper':
            self.profile = synthetic_gripper_jaco_geometry_profile(
                robot_id=self.robot_id, frame_prefix=prefix,
            )
        elif tool_mode == 'fixture':
            self.profile = offline_fixture_jaco_geometry_profile(
                robot_id=self.robot_id,
                frame_prefix=prefix,
            )
        elif tool_mode in ('interface', 'legacy'):
            self.profile = legacy_jaco_geometry_profile(
                robot_id=self.robot_id,
                frame_prefix=prefix,
            )
        else:
            raise ValueError('unsupported tool_mode')
        self.manifest = geometry_transport_manifest(
            self.profile, self.state
        )
        self.expected_dynamic_frames = frozenset(
            f'{prefix}_link_{index}' for index in range(1, 7)
        )
        self._joint_stamps = set()
        self._tf_frames_by_stamp = {}
        self._last_published_stamp = None
        self.publisher = self.create_publisher(
            GeometrySnapshot,
            str(self.get_parameter('geometry_topic').value),
            geometry_snapshot_qos(),
        )
        self.subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_state_callback,
            10,
        )
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self._tf_callback,
            qos_profile_sensor_data,
        )

    def _joint_state_callback(self, joint_state):
        """Publish nothing on incomplete evidence; the coordinator watchdog stops."""
        if not complete_joint_sample(
            joint_state.name,
            joint_state.position,
            self.required_joint_names,
        ):
            self.get_logger().warning('Rejected incomplete joint-state sample')
            return
        try:
            stamp = time_message_key(joint_state.header.stamp)
        except ValueError:
            self.get_logger().warning('Rejected invalid joint-state stamp')
            return
        if not stamp_is_strictly_newer(
            stamp, self._last_published_stamp
        ):
            return
        self._joint_stamps.add(stamp)
        self._prune_stamp_caches(stamp)
        if (
            stamp_is_strictly_newer(stamp, self._last_published_stamp)
            and self.expected_dynamic_frames.issubset(
                self._tf_frames_by_stamp.get(stamp, set())
            )
        ):
            self._publish_snapshot(joint_state.header.stamp)
            self._last_published_stamp = stamp

    def _prune_stamp_caches(self, newest_stamp):
        cutoff = newest_stamp - 2_000_000_000
        self._joint_stamps = {
            stamp for stamp in self._joint_stamps if stamp >= cutoff
        }
        self._tf_frames_by_stamp = {
            stamp: frames
            for stamp, frames in self._tf_frames_by_stamp.items()
            if stamp >= cutoff
        }

    def _tf_callback(self, tf_message):
        """Emit evidence only after TF contains the exact complete chain stamp."""
        stamp_messages = {}
        for transform in tf_message.transforms:
            child_frame = str(transform.child_frame_id)
            if child_frame not in self.expected_dynamic_frames:
                continue
            try:
                stamp = time_message_key(transform.header.stamp)
            except ValueError:
                continue
            if not stamp_is_strictly_newer(
                stamp, self._last_published_stamp
            ):
                continue
            frames = self._tf_frames_by_stamp.setdefault(stamp, set())
            frames.add(child_frame)
            stamp_messages[stamp] = transform.header.stamp
        for stamp in sorted(stamp_messages):
            if (
                stamp in self._joint_stamps
                and stamp_is_strictly_newer(
                    stamp, self._last_published_stamp
                )
                and self.expected_dynamic_frames.issubset(
                    self._tf_frames_by_stamp[stamp]
                )
            ):
                self._publish_snapshot(stamp_messages[stamp])
                self._last_published_stamp = stamp
                self._prune_stamp_caches(stamp)

    def _publish_snapshot(self, stamp):
        """Publish a descriptor whose complete dynamic TF was observed."""
        message = GeometrySnapshot()
        message.stamp = stamp
        message.valid_for = self.valid_for
        message.session_id = self.session_id
        message.source_kind = self.source_kind
        message.robot_id = self.profile.robot_id
        message.profile_id = self.profile.profile_id
        message.geometry_version = self.profile.geometry_version
        message.asset_hash = self.profile.asset_hash
        message.state = self.state
        message.reference_frame = self.reference_frame
        message.mount_frame = self.profile.mount_frame
        message.tcp_frame = self.profile.tcp_frame
        message.valid = True
        message.primitive_names = list(self.manifest.primitive_names)
        message.primitive_types = list(self.manifest.primitive_types)
        message.start_frames = list(self.manifest.start_frames)
        message.end_frames = list(self.manifest.end_frames)
        message.start_offsets = [
            _vector(values) for values in self.manifest.start_offsets
        ]
        message.end_offsets = [
            _vector(values) for values in self.manifest.end_offsets
        ]
        message.radii = list(self.manifest.radii)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    spin_node(GeometrySnapshotSourceNode())
