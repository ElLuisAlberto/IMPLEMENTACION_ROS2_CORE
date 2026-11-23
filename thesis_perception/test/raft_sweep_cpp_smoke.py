#!/usr/bin/env python3
"""Run in an isolated ROS domain with raft_sweep_node already running."""
import array
import sys
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts' / 'alternativas'))
from point_flow_raft_preview import swept_cells


def main():
    rclpy.init()
    node = rclpy.create_node('raft_sweep_cpp_smoke')
    received = []
    publisher = node.create_publisher(PointCloud2, '/thesis/perception/raft_sweep_input', qos_profile_sensor_data)
    node.create_subscription(Marker, '/thesis/perception/raft_swept_surface_cpp', received.append, qos_profile_sensor_data)
    deadline = time.monotonic() + 5
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert publisher.get_subscription_count(), 'C++ sweep node not discovered'
    rng = np.random.default_rng(42)
    for count in (0, 1, 100, 1000, 5000):
        p = rng.uniform(-.8, .8, (count, 3)).astype(np.float32)
        v = rng.uniform(-.8, .8, (count, 3)).astype(np.float32)
        cloud = PointCloud2()
        cloud.header.frame_id = 'test_frame'
        cloud.header.stamp = node.get_clock().now().to_msg()
        cloud.height, cloud.width, cloud.point_step = 1, count, 24
        cloud.row_step = count * 24
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'vx', 'vy', 'vz'))]
        cloud.data = array.array('B', np.concatenate((p, v), axis=1).astype('<f4').tobytes())
        received.clear()
        t0 = time.monotonic()
        last_send = 0.0
        while not received and time.monotonic() - t0 < 10:
            if time.monotonic() - last_send > .5:
                publisher.publish(cloud)
                last_send = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.1)
        assert received, 'no marker received'
        elapsed_ms = (time.monotonic() - t0) * 1000
        marker = received[-1]
        assert marker.header.frame_id == 'test_frame'
        expected = {tuple(row) for row in swept_cells(p, v)}
        actual = {(round(q.x / .025 - .5), round(q.y / .025 - .5), round(q.z / .025 - .5))
                  for q in marker.points}
        assert expected == actual, f'{count} inputs: {len(expected ^ actual)} cells differ'
        assert marker.action == (Marker.ADD if count else Marker.DELETE)
        assert marker.lifetime.nanosec == 600_000_000
        print(f'inputs={count} cells={len(actual)} receive_ms={elapsed_ms:.1f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
