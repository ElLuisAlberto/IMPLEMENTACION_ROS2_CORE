"""Live ROS TF expiry removes the capsule rather than reusing cached TF."""
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from thesis_core.capsule_visualizer import CapsuleVisualizer


def test_lost_dynamic_tf_removes_capsule():
    rclpy.init(args=['--ros-args', '-p', 'segment_end_frames:=[tip]'])
    visualizer = CapsuleVisualizer()
    observer = rclpy.create_node('capsule_tf_integration')
    broadcaster = TransformBroadcaster(observer)
    messages = []
    observer.create_subscription(MarkerArray, '/thesis/robot_capsules',
                                 messages.append, 10)

    def pump(seconds, publish):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish:
                transform = TransformStamped()
                transform.header.stamp = observer.get_clock().now().to_msg()
                transform.header.frame_id = 'world'
                transform.child_frame_id = 'tip'
                transform.transform.translation.z = 1.
                transform.transform.rotation.w = 1.
                broadcaster.sendTransform(transform)
            rclpy.spin_once(visualizer, timeout_sec=0.01)
            rclpy.spin_once(observer, timeout_sec=0.01)
            time.sleep(0.02)
    try:
        pump(1.5, True)
        assert any(any(m.action == Marker.ADD for m in msg.markers)
                   for msg in messages)
        pump(1.3, False)
        assert messages[-1].markers
        assert all(m.action == Marker.DELETEALL for m in messages[-1].markers)
    finally:
        visualizer.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
