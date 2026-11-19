"""ROS integration of clock pause, rewind and prediction invalidation."""
import time

import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import JointTrajectoryPrediction
from visualization_msgs.msg import MarkerArray, Marker

from thesis_core.horizon_preview import HorizonPreview
from thesis_core.joint_model import JOINT_NAMES


def test_preview_revokes_and_recovers_across_clock_discontinuities():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    preview = HorizonPreview()
    observer = rclpy.create_node('preview_clock_integration')
    clock_pub = observer.create_publisher(Clock, '/clock', 10)
    state_pub = observer.create_publisher(JointState, '/joint_states', 10)
    predictions, marker_arrays = [], []
    observer.create_subscription(JointTrajectoryPrediction,
                                 '/thesis/joint_trajectory_prediction',
                                 predictions.append, 10)
    observer.create_subscription(MarkerArray, '/thesis/horizon_volume',
                                 marker_arrays.append, 10)

    def run(seconds, clock_value, state=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            stamp_ns = int(clock_value() * 1e9)
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 10**9)
            clock_pub.publish(clock)
            if state:
                joint = JointState()
                joint.header.stamp = clock.clock
                joint.name = list(JOINT_NAMES)
                joint.position = [0., 3.14, 3.14, 0., 0., 0.]
                state_pub.publish(joint)
            rclpy.spin_once(preview, timeout_sec=0.01)
            rclpy.spin_once(observer, timeout_sec=0.01)
            time.sleep(0.02)

    try:
        start = time.monotonic()
        run(1.5, lambda: 10. + time.monotonic() - start)
        assert any(p.sample_count > 0 for p in predictions)
        before_pause = len(predictions)
        run(1.2, lambda: 12.)
        assert any(p.source == 'invalid' and p.sample_count == 0
                   for p in predictions[before_pause:])
        assert any(any(m.action == Marker.DELETEALL for m in array.markers)
                   for array in marker_arrays)
        before_resume = len(predictions)
        start = time.monotonic()
        run(1.2, lambda: 12. + time.monotonic() - start)
        assert any(p.sample_count > 0 for p in predictions[before_resume:])
        before_rewind = len(predictions)
        run(0.5, lambda: 1., state=False)
        assert any(p.source == 'invalid' for p in predictions[before_rewind:])
        before_resume = len(predictions)
        start = time.monotonic()
        run(1.2, lambda: 1. + time.monotonic() - start)
        assert any(p.sample_count > 0 for p in predictions[before_resume:])
    finally:
        preview.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
