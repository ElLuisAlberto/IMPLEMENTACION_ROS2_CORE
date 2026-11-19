"""Profile-driven current-clearance monitor for simulated robot arms."""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMessage
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import Obstacle, ProximityStatus
from visualization_msgs.msg import Marker, MarkerArray

from thesis_core.capsule_visualizer import quaternion_from_z_axis
from thesis_core.clearance_geometry import minimum_configuration_clearance
from thesis_core.work_table import configured_table, table_marker
from thesis_core.robot_profile import load_robot_profile
from thesis_core.ros_runtime import spin_node


FUTURE_TOLERANCE_SEC = 0.05


def stamp_nanoseconds(stamp):
    """Convert a ROS timestamp or reject malformed nanoseconds."""
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        return -1
    return seconds * 1_000_000_000 + nanoseconds


def stamp_is_fresh(now_ns, stamp, maximum_age_sec):
    """Accept only nonzero, recent, non-future source timestamps."""
    source_ns = stamp_nanoseconds(stamp)
    age = (int(now_ns) - source_ns) * 1.0e-9
    return (
        source_ns > 0
        and math.isfinite(float(maximum_age_sec))
        and maximum_age_sec > 0.0
        and -FUTURE_TOLERANCE_SEC <= age <= maximum_age_sec
    )


def profile_clearance(profile, positions, center, obstacle_radius, table=None):
    """Evaluate one profile configuration against a spherical obstacle."""
    capsules = tuple(profile.capsule_segments(positions))
    axes = tuple((c.start, c.end) for c in capsules)
    names = tuple(c.name for c in capsules)
    radii = tuple(c.radius for c in capsules)
    result = minimum_configuration_clearance(axes, names, radii, center, obstacle_radius)
    return table.minimum(result, axes, names, radii) if table else result


def classify_clearance(clearance, warning, stop):
    """Classify current clearance without implying predictive speed scaling."""
    values = tuple(float(value) for value in (clearance, warning, stop))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('clearance thresholds must be finite')
    if not 0.0 <= stop < warning:
        raise ValueError(
            'thresholds must satisfy 0 <= stop < warning'
        )
    if clearance <= stop:
        return 'STOP'
    if clearance <= warning:
        return 'WARNING'
    return 'ALLOW'


class ProfileProximityMonitor(Node):
    """Publish historical proximity evidence from a canonical robot profile."""

    def __init__(self):
        """Load one profile and create its simulated proximity pipeline."""
        super().__init__('profile_proximity_monitor')
        for name, default in (
            ('manifest_path', ''),
            ('robot_name', 'jaco2'),
            ('state_topic', 'joint_states'),
            ('obstacle_topic', 'thesis/obstacle'),
            ('status_topic', 'thesis/proximity_status'),
            ('capsule_marker_topic', 'thesis/robot_capsules'),
            ('obstacle_marker_topic', 'thesis/obstacle_visualization'),
            ('evaluation_rate_hz', 20.0),
            ('max_state_age_sec', 0.5),
            ('max_obstacle_age_sec', 0.5),
            ('warning_distance', 0.30),
            ('stop_distance', 0.05),
        ):
            self.declare_parameter(name, default)

        manifest_path = str(self.get_parameter('manifest_path').value)
        robot_name = str(self.get_parameter('robot_name').value)
        self.profile = load_robot_profile(manifest_path or None, robot_name)
        self.table = configured_table(self, self.profile)
        self.positions = None
        self.state_stamp = None
        self.state_arrival = None
        self.obstacle = None
        self.obstacle_arrival = None
        self.previous_state = None

        self.status_publisher = self.create_publisher(
            ProximityStatus,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.capsule_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('capsule_marker_topic').value),
            10,
        )
        self.obstacle_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('obstacle_marker_topic').value),
            10,
        )
        self.state_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('state_topic').value),
            self.receive_state,
            20,
        )
        self.obstacle_subscription = self.create_subscription(
            Obstacle,
            str(self.get_parameter('obstacle_topic').value),
            self.receive_obstacle,
            20,
        )

        rate = float(self.get_parameter('evaluation_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('evaluation_rate_hz must be positive')
        classify_clearance(
            1.0,
            float(self.get_parameter('warning_distance').value),
            float(self.get_parameter('stop_distance').value),
        )
        self.timer = self.create_timer(1.0 / rate, self.evaluate)
        self.get_logger().info(
            f'Profile proximity ready for {self.profile.display_name}: '
            f'{self.profile.joint_count} DOF, '
            f'{len(self.profile.collision_capsules)} capsules'
        )

    def receive_state(self, message):
        """Store one complete finite profile-ordered joint state."""
        values = dict(zip(message.name, message.position))
        if not all(
            name in values and math.isfinite(float(values[name]))
            for name in self.profile.joint_names
        ):
            self.positions = None
            return
        self.positions = tuple(
            float(values[name]) for name in self.profile.joint_names
        )
        self.state_stamp = message.header.stamp
        self.state_arrival = time.monotonic()

    def receive_obstacle(self, message):
        """Store one validated obstacle already expressed in world."""
        values = (
            message.center.x,
            message.center.y,
            message.center.z,
            message.velocity.x,
            message.velocity.y,
            message.velocity.z,
            message.radius,
            message.uncertainty,
        )
        if (
            message.header.frame_id != self.profile.world_frame
            or not all(math.isfinite(float(value)) for value in values)
            or message.radius <= 0.0
            or message.uncertainty < 0.0
        ):
            self.obstacle = None
            return
        self.obstacle = message
        self.obstacle_arrival = time.monotonic()

    def _fresh(self, stamp, age_parameter):
        return stamp_is_fresh(
            self.get_clock().now().nanoseconds,
            stamp,
            float(self.get_parameter(age_parameter).value),
        )

    def _publish_stop(self, reason):
        if self.obstacle is None:
            return
        message = ProximityStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.reference_frame = self.profile.world_frame
        message.state = 'STOP'
        message.geometry_valid = False
        message.geometry_state = str(reason)
        message.minimum_clearance = 0.0
        message.limiting_segment = reason
        message.obstacle_center = self.obstacle.center
        message.obstacle_velocity = self.obstacle.velocity
        message.obstacle_radius = (
            self.obstacle.radius + self.obstacle.uncertainty
        )
        self.status_publisher.publish(message)
        self.previous_state = 'STOP'

    def evaluate(self):
        """Publish current clearance or a fail-closed stale-data status."""
        if self.obstacle is None:
            return
        if not self._fresh(
            self.obstacle.header.stamp, 'max_obstacle_age_sec'
        ):
            self._publish_stop('OBSTACLE_STALE')
            return
        if (
            self.positions is None
            or self.state_stamp is None
            or not self._fresh(self.state_stamp, 'max_state_age_sec')
        ):
            self._publish_stop('JOINT_STATE_STALE')
            return

        center = (
            self.obstacle.center.x,
            self.obstacle.center.y,
            self.obstacle.center.z,
        )
        radius = self.obstacle.radius + self.obstacle.uncertainty
        try:
            result = profile_clearance(
                self.profile, self.positions, center, radius, self.table
            )
            state = classify_clearance(
                result.clearance,
                float(self.get_parameter('warning_distance').value),
                float(self.get_parameter('stop_distance').value),
            )
        except (TypeError, ValueError, ZeroDivisionError) as error:
            self.get_logger().error(f'Invalid profile geometry: {error}')
            self._publish_stop('PROFILE_GEOMETRY_INVALID')
            return

        message = ProximityStatus()
        message.stamp = self.obstacle.header.stamp
        message.reference_frame = self.profile.world_frame
        message.state = state
        message.geometry_valid = True
        message.geometry_state = 'VALID'
        message.minimum_clearance = result.clearance
        message.limiting_segment = result.segment_name
        message.closest_robot_point = Point(
            x=result.closest_robot_point[0],
            y=result.closest_robot_point[1],
            z=result.closest_robot_point[2],
        )
        message.obstacle_center = self.obstacle.center
        message.obstacle_velocity = self.obstacle.velocity
        message.obstacle_radius = radius
        self.status_publisher.publish(message)
        self._publish_markers(state, radius, result.segment_name)
        if state != self.previous_state:
            self.get_logger().info(
                f'{state}: clearance={result.clearance:.3f} m, '
                f'segment={result.segment_name}'
            )
            self.previous_state = state

    @staticmethod
    def _color(state):
        # Keep the profile-driven overlay visually identical to the historical
        # JACO2 capsule visualizer.  In particular, the nominal state must stay
        # transparent enough for the physical mesh to remain visible in RViz.
        return {
            'ALLOW': (0.05, 0.80, 0.35, 0.38),
            'WARNING': (1.00, 0.82, 0.05, 0.70),
            'REDUCTION': (1.00, 0.35, 0.02, 0.78),
            'STOP': (1.00, 0.03, 0.03, 0.88),
        }[state]

    def _publish_markers(self, state, obstacle_radius, limiting_segment):
        """Render current profile capsules and the inflated obstacle."""
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for index, capsule in enumerate(
            self.profile.capsule_segments(self.positions)
        ):
            # Match the legacy JACO2 display: only the nearest capsule
            # changes color. The others remain translucent green, making the
            # current geometry distinguishable from the orange horizon.
            color = self._color(state if capsule.name == limiting_segment else 'ALLOW')
            marker = Marker()
            marker.header.frame_id = self.profile.world_frame
            marker.header.stamp = stamp
            marker.ns = 'current_capsules'
            marker.id = 3 * index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = (capsule.start[0] + capsule.end[0]) / 2.0
            marker.pose.position.y = (capsule.start[1] + capsule.end[1]) / 2.0
            marker.pose.position.z = (capsule.start[2] + capsule.end[2]) / 2.0
            delta = tuple(
                end - start
                for start, end in zip(capsule.start, capsule.end)
            )
            marker.pose.orientation = quaternion_from_z_axis(*delta)
            marker.scale.x = marker.scale.y = 2.0 * capsule.radius
            marker.scale.z = max(math.dist(capsule.start, capsule.end), 1e-6)
            (
                marker.color.r,
                marker.color.g,
                marker.color.b,
                marker.color.a,
            ) = color
            marker.lifetime = DurationMessage(sec=0, nanosec=200_000_000)
            array.markers.append(marker)
            # A capsule is a cylinder plus hemispherical ends. Two full
            # spheres yield exactly that union, including zero-length axes.
            for offset, position in enumerate((capsule.start, capsule.end), 1):
                sphere = Marker()
                sphere.header = marker.header
                sphere.ns = marker.ns
                sphere.id = marker.id + offset
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position = Point(x=position[0], y=position[1], z=position[2])
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * capsule.radius
                (sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a) = color
                sphere.lifetime = marker.lifetime
                array.markers.append(sphere)
        self.capsule_publisher.publish(array)

        obstacle = Marker()
        obstacle.header.frame_id = self.profile.world_frame
        obstacle.header.stamp = stamp
        obstacle.ns = 'input_obstacle'
        obstacle.id = 0
        obstacle.type = Marker.SPHERE
        obstacle.action = Marker.ADD
        obstacle.pose.position = self.obstacle.center
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = obstacle.scale.y = obstacle.scale.z = (
            2.0 * obstacle_radius
        )
        obstacle.color.r = 0.72
        obstacle.color.g = 0.12
        obstacle.color.b = 0.92
        obstacle.color.a = 0.78
        obstacle.lifetime = DurationMessage(sec=0, nanosec=300_000_000)
        obstacle_array = MarkerArray()
        obstacle_array.markers.append(obstacle)
        if getattr(self, 'table', None):
            obstacle_array.markers.append(table_marker(self.table, obstacle.header))
        self.obstacle_marker_publisher.publish(obstacle_array)


def main(args=None):
    """Run the profile proximity monitor until ROS shutdown."""
    rclpy.init(args=args)
    spin_node(ProfileProximityMonitor())


if __name__ == '__main__':
    main()
