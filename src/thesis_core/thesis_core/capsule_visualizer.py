import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point, Quaternion
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import ProximityStatus, TrajectoryPrediction


def quaternion_from_z_axis(dx, dy, dz):
    """Return a quaternion that rotates the positive Z axis onto a vector."""
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1.0e-9:
        return Quaternion(w=1.0)

    ux = dx / length
    uy = dy / length
    uz = dz / length

    if uz < -0.999999:
        return Quaternion(x=1.0, w=0.0)

    scale = math.sqrt(2.0 * (1.0 + uz))
    return Quaternion(
        x=-uy / scale,
        y=ux / scale,
        z=0.0,
        w=0.5 * scale,
    )


def validate_capsule_configuration(
    segment_names, start_frames, end_frames, radii, color,
):
    """Validate one profile's RViz capsule-marker configuration."""
    segment_count = len(segment_names)
    sizes = (
        segment_count,
        len(start_frames),
        len(end_frames),
        len(radii),
    )
    if segment_count == 0 or len(set(sizes)) != 1:
        raise ValueError(
            'segment_names, segment_start_frames, '
            'segment_end_frames and segment_radii must have '
            'the same non-zero length'
        )
    if any(
        not math.isfinite(float(radius)) or float(radius) <= 0.0
        for radius in radii
    ):
        raise ValueError(
            'Every capsule radius must be positive and finite'
        )
    if len(color) != 4 or any(
        not math.isfinite(float(channel)) for channel in color
    ):
        raise ValueError('color_rgba must contain four finite values')


class CapsuleVisualizer(Node):
    """Publish a configured robot profile's capsules as RViz markers."""

    def __init__(self):
        super().__init__('capsule_visualizer')

        self.per_obstacle_markers = self.declare_parameter('per_obstacle_markers', False).value
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('marker_topic', '/thesis/robot_capsules')
        self.declare_parameter(
            'proximity_topic', '/thesis/proximity_status'
        )
        self.declare_parameter(
            'prediction_topic', '/thesis/trajectory_prediction'
        )
        self.declare_parameter('segment_names', ['segment'])
        self.declare_parameter('segment_start_frames', ['world'])
        self.declare_parameter('segment_end_frames', ['world'])
        self.declare_parameter('segment_radii', [0.05])
        self.declare_parameter('color_rgba', [0.05, 0.80, 0.35, 0.38])
        self.declare_parameter('max_evidence_age_sec', 0.5)

        self.reference_frame = str(
            self.get_parameter('reference_frame').value
        )
        self.segment_names = list(
            self.get_parameter('segment_names').value
        )
        self.start_frames = list(
            self.get_parameter('segment_start_frames').value
        )
        self.end_frames = list(
            self.get_parameter('segment_end_frames').value
        )
        self.radii = [
            float(value)
            for value in self.get_parameter('segment_radii').value
        ]
        self.color = [
            float(value)
            for value in self.get_parameter('color_rgba').value
        ]
        self.proximity_status = None
        self.trajectory_prediction = None
        self.max_evidence_age = float(self.get_parameter('max_evidence_age_sec').value)
        if not math.isfinite(self.max_evidence_age) or self.max_evidence_age <= 0:
            raise ValueError('max_evidence_age_sec must be positive and finite')
        self.proximity_received = None
        self.prediction_received = None
        self.last_clock_ns = None
        self.clock_changed_at = time.monotonic()
        self.state_colors = {
            'ALLOW': [0.05, 0.80, 0.35, 0.38],
            'WARNING': [1.00, 0.82, 0.05, 0.70],
            'REDUCTION': [1.00, 0.35, 0.02, 0.78],
            'STOP': [1.00, 0.03, 0.03, 0.88],
        }

        self._validate_configuration()

        marker_topic = str(
            self.get_parameter('marker_topic').value
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            qos,
        )
        self.proximity_subscription = self.create_subscription(
            ProximityStatus,
            str(self.get_parameter('proximity_topic').value),
            self.proximity_callback,
            10,
        )
        self.prediction_subscription = self.create_subscription(
            TrajectoryPrediction,
            str(self.get_parameter('prediction_topic').value),
            self.prediction_callback,
            10,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.missing_frames = set()

        publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_capsules,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            f'Capsule visualizer ready: {len(self.segment_names)} '
            f'segments on {marker_topic}'
        )

    def _validate_configuration(self):
        validate_capsule_configuration(
            self.segment_names,
            self.start_frames,
            self.end_frames,
            self.radii,
            self.color,
        )

    def _frame_point(self, frame_name):
        transform = self.tf_buffer.lookup_transform(
            self.reference_frame,
            frame_name,
            Time(),
        )
        stamp_ns = (transform.header.stamp.sec * 1_000_000_000
                    + transform.header.stamp.nanosec)
        if stamp_ns:
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if not -0.05 <= age <= self.max_evidence_age:
                raise TransformException('Transform is stale or in the future')
        translation = transform.transform.translation
        return Point(
            x=translation.x,
            y=translation.y,
            z=translation.z,
        )

    def proximity_callback(self, message):
        self.proximity_status = message
        self.proximity_received = time.monotonic()

    def prediction_callback(self, message):
        self.trajectory_prediction = message
        self.prediction_received = time.monotonic()

    def _fresh(self, message, received, now, ros_ns):
        if message is None or received is None:
            return False
        stamp_ns = message.stamp.sec * 1000000000 + message.stamp.nanosec
        age = (ros_ns - stamp_ns) / 1e9
        return (stamp_ns > 0 and -0.05 <= age <= self.max_evidence_age
                and now - received <= self.max_evidence_age)

    def _base_marker(
        self,
        marker_id,
        marker_type,
        segment_name,
        color=None,
        namespace='current_capsules',
    ):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        selected_color = self.color if color is None else color
        marker.color.r = selected_color[0]
        marker.color.g = selected_color[1]
        marker.color.b = selected_color[2]
        marker.color.a = selected_color[3]
        marker.text = segment_name
        return marker

    def _sphere_marker(
        self,
        marker_id,
        point,
        radius,
        segment_name,
        color=None,
    ):
        marker = self._base_marker(
            marker_id,
            Marker.SPHERE,
            segment_name,
            color=color,
        )
        marker.pose.position = point
        diameter = 2.0 * radius
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        return marker

    def _capsule_markers(self, index, start, end, radius):
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 1.0e-6:
            return []

        marker_id = index * 3
        segment_name = self.segment_names[index]
        color = self.color
        if (
            self.proximity_status is not None
            and self.proximity_status.limiting_segment == segment_name
        ):
            color = self.state_colors.get(
                self.proximity_status.state,
                self.color,
            )
        cylinder = self._base_marker(
            marker_id,
            Marker.CYLINDER,
            segment_name,
            color=color,
        )
        cylinder.pose.position = Point(
            x=(start.x + end.x) * 0.5,
            y=(start.y + end.y) * 0.5,
            z=(start.z + end.z) * 0.5,
        )
        cylinder.pose.orientation = quaternion_from_z_axis(dx, dy, dz)
        cylinder.scale.x = 2.0 * radius
        cylinder.scale.y = 2.0 * radius
        cylinder.scale.z = length

        return [
            cylinder,
            self._sphere_marker(
                marker_id + 1,
                start,
                radius,
                segment_name,
                color=color,
            ),
            self._sphere_marker(
                marker_id + 2,
                end,
                radius,
                segment_name,
                color=color,
            ),
        ]

    def _proximity_markers(self):
        if self.per_obstacle_markers or self.proximity_status is None:
            return []

        status = self.proximity_status
        obstacle = self._base_marker(
            1000,
            Marker.SPHERE,
            'virtual_obstacle',
            color=[0.60, 0.15, 0.95, 0.68],
            namespace='proximity',
        )
        obstacle.pose.position = status.obstacle_center
        diameter = 2.0 * status.obstacle_radius
        obstacle.scale.x = diameter
        obstacle.scale.y = diameter
        obstacle.scale.z = diameter

        state_color = self.state_colors.get(status.state, self.color)
        line = self._base_marker(
            1001,
            Marker.LINE_LIST,
            'minimum_distance',
            color=[
                state_color[0],
                state_color[1],
                state_color[2],
                1.0,
            ],
            namespace='proximity',
        )
        line.scale.x = 0.012
        line.points = [
            status.closest_robot_point,
            status.obstacle_center,
        ]

        label = self._base_marker(
            1002,
            Marker.TEXT_VIEW_FACING,
            'proximity_label',
            color=[
                state_color[0],
                state_color[1],
                state_color[2],
                1.0,
            ],
            namespace='proximity',
        )
        label.pose.position = Point(
            x=(
                status.closest_robot_point.x
                + status.obstacle_center.x
            ) * 0.5,
            y=(
                status.closest_robot_point.y
                + status.obstacle_center.y
            ) * 0.5,
            z=(
                status.closest_robot_point.z
                + status.obstacle_center.z
            ) * 0.5 + 0.08,
        )
        label.scale.z = 0.055
        label.text = (
            f'ACTUAL {status.state} | d={status.minimum_clearance:.3f} m | '
            f'{status.limiting_segment}'
        )

        return [obstacle, line, label]

    def _prediction_markers(self):
        if self.trajectory_prediction is None:
            return []

        prediction = self.trajectory_prediction
        start = prediction.capsule_start
        end = prediction.capsule_end
        radius = prediction.capsule_radius
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 1.0e-6:
            return []

        base_color = self.state_colors.get(
            prediction.state,
            self.color,
        )
        color = [
            base_color[0],
            base_color[1],
            base_color[2],
            0.48,
        ]

        cylinder = self._base_marker(
            2000,
            Marker.CYLINDER,
            'predicted_limiting_capsule',
            color=color,
            namespace='prediction',
        )
        cylinder.pose.position = Point(
            x=(start.x + end.x) * 0.5,
            y=(start.y + end.y) * 0.5,
            z=(start.z + end.z) * 0.5,
        )
        cylinder.pose.orientation = quaternion_from_z_axis(dx, dy, dz)
        cylinder.scale.x = 2.0 * radius
        cylinder.scale.y = 2.0 * radius
        cylinder.scale.z = length

        spheres = []
        for marker_id, point in ((2001, start), (2002, end)):
            sphere = self._base_marker(
                marker_id,
                Marker.SPHERE,
                'predicted_limiting_capsule',
                color=color,
                namespace='prediction',
            )
            sphere.pose.position = point
            sphere.scale.x = 2.0 * radius
            sphere.scale.y = 2.0 * radius
            sphere.scale.z = 2.0 * radius
            spheres.append(sphere)

        label = self._base_marker(
            2003,
            Marker.TEXT_VIEW_FACING,
            'prediction_label',
            color=[color[0], color[1], color[2], 1.0],
            namespace='prediction',
        )
        label.pose.position = Point(
            x=(start.x + end.x) * 0.5,
            y=(start.y + end.y) * 0.5,
            z=(start.z + end.z) * 0.5 + 0.12,
        )
        label.scale.z = 0.055
        protective_ttc = (
            'n/a'
            if prediction.time_to_protective_volume < 0.0
            else f'{prediction.time_to_protective_volume:.2f}s'
        )
        collision_ttc = (
            'n/a'
            if prediction.time_to_collision < 0.0
            else f'{prediction.time_to_collision:.2f}s'
        )
        label.text = (
            f'CANDIDATO {prediction.state} | '
            f'd={prediction.minimum_clearance:.3f} m | '
            f't_min={prediction.minimum_time_from_now:.2f}s | '
            f'm={prediction.protective_margin:.3f}m | '
            f'TTCp={protective_ttc} | TTCc={collision_ttc}'
        )

        witness_line = self._base_marker(
            2004,
            Marker.LINE_LIST,
            'predicted_minimum_witness',
            color=[color[0], color[1], color[2], 1.0],
            namespace='prediction',
        )
        witness_line.scale.x = 0.012
        witness_line.points = [
            prediction.closest_robot_point,
            prediction.obstacle_center_at_minimum,
        ]

        witnesses = []
        for marker_id, point, witness_color in (
            (2005, prediction.closest_robot_point, [0.1, 1.0, 1.0, 1.0]),
            (
                2006,
                prediction.obstacle_center_at_minimum,
                [0.8, 0.2, 1.0, 1.0],
            ),
        ):
            marker = self._base_marker(
                marker_id,
                Marker.SPHERE,
                'predicted_minimum_witness',
                color=witness_color,
                namespace='prediction',
            )
            marker.pose.position = point
            marker.scale.x = marker.scale.y = marker.scale.z = 0.035
            witnesses.append(marker)

        return [cylinder, *spheres, label, witness_line, *witnesses]

    def publish_capsules(self):
        now = time.monotonic()
        ros_ns = self.get_clock().now().nanoseconds
        previous = self.last_clock_ns
        if previous is None or ros_ns != previous:
            self.clock_changed_at = now
        reversed_clock = previous is not None and ros_ns < previous
        self.last_clock_ns = ros_ns
        frozen = (bool(self.get_parameter('use_sim_time').value) and
                  (ros_ns <= 0 or now - self.clock_changed_at > self.max_evidence_age))
        if reversed_clock or frozen:
            self.proximity_status = None
            self.trajectory_prediction = None
        if not self._fresh(self.proximity_status, self.proximity_received, now, ros_ns):
            self.proximity_status = None
        if not self._fresh(self.trajectory_prediction, self.prediction_received, now, ros_ns):
            self.trajectory_prediction = None
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        unavailable = set()

        for index, (start_frame, end_frame, radius) in enumerate(zip(
                self.start_frames,
                self.end_frames,
                self.radii) if not frozen and not reversed_clock else ()):
            try:
                start = self._frame_point(start_frame)
                end = self._frame_point(end_frame)
            except TransformException:
                unavailable.update((start_frame, end_frame))
                continue

            markers.extend(
                self._capsule_markers(index, start, end, radius)
            )

        if unavailable != self.missing_frames:
            if unavailable:
                frames = ', '.join(sorted(unavailable))
                self.get_logger().warning(
                    f'Waiting for TF frames: {frames}'
                )
            elif self.missing_frames:
                self.get_logger().info(
                    'All capsule TF frames are available'
                )
            self.missing_frames = unavailable

        if len(markers) > 1:
            markers.extend(self._proximity_markers())
            markers.extend(self._prediction_markers())
        message = MarkerArray()
        message.markers = markers
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    spin_node(CapsuleVisualizer())


if __name__ == '__main__':
    main()
