# Copyright 2026 Luis Alberto Munoz Marin
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Exercise cloud prediction with known motion; run in an isolated ROS graph."""
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


def main():
    rclpy.init()
    node = Node('prediction_test_client')
    received = []
    markers = []
    node.create_subscription(
        Marker, '/thesis/perception/swept_volume',
        lambda message: markers.append(message), qos_profile_sensor_data)
    node.create_subscription(
        PointCloud2, '/thesis/perception/predicted_points',
        lambda message: received.append(message), qos_profile_sensor_data)
    publisher = node.create_publisher(
        PointCloud2, '/thesis/perception/points_filtered', qos_profile_sensor_data)
    process = subprocess.Popen([
        'ros2', 'run', 'thesis_perception', 'cloud_prediction_node', '--ros-args',
        '-p', 'horizon_s:=0.5'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def send(x, other=-0.4):
        stamp = node.get_clock().now().nanoseconds
        header = Header()
        header.frame_id = 'camera'
        header.stamp.sec = stamp // 1000000000
        header.stamp.nanosec = stamp % 1000000000
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
        points = [(x + (i % 10) * 0.005, (i // 10) * 0.005, 0.8, 0.0)
                  for i in range(100)]
        points += [(other + (i % 10) * 0.005, (i // 10) * 0.005, 0.8, 0.0)
                   for i in range(100)]
        points.append((0.6, 0.5, 0.8, 0.0))
        publisher.publish(point_cloud2.create_cloud(header, fields, points))
        pump(0.05)

    try:
        pump(2)
        for _ in range(12):
            send(0.0)
        assert received and received[-1].width == 201, 'Stationary points missing'
        stationary = list(point_cloud2.read_points(
            received[-1], field_names=['x'], skip_nans=True))
        assert abs(float(stationary[0][0])) < 0.001
        for i in range(1, 18):
            send(i * 0.006)
        assert received[-1].width == 201, 'Future cloud must contain all points'
        points = list(point_cloud2.read_points(
            received[-1], field_names=['x'], skip_nans=True))
        assert float(points[0][0]) > 0.12, points[0]
        populated = [marker for marker in markers if marker.action == Marker.ADD and marker.points]
        assert populated, 'Moving object produced no swept volume'
        assert populated[-1].type == Marker.CUBE_LIST
        assert abs(populated[-1].scale.x - 0.02) < 0.0001
        xs = [point.x for point in populated[-1].points]
        assert min(xs) <= 0.112 and max(xs) > 0.12, 'Sweep lacks observed or future positions'
        assert abs(float(points[100][0]) + 0.4) < 0.001, points[100]
        assert abs(float(points[200][0]) - 0.6) < 0.001, points[200]
        for _ in range(15):
            send(0.102)
        points = list(point_cloud2.read_points(
            received[-1], field_names=['x'], skip_nans=True))
        assert abs(float(points[0][0]) - 0.102) < 0.01, 'Did not return to rest'
        assert markers[-1].action == Marker.DELETE, 'Sweep persisted after rest'
        for i in range(1, 18):
            send(0.102, -0.4 - i * 0.006)
        points = list(point_cloud2.read_points(
            received[-1], field_names=['x'], skip_nans=True))
        assert abs(float(points[0][0]) - 0.102) < 0.01, 'First group moved unexpectedly'
        assert float(points[100][0]) < -0.52, points[100]
        pump(0.8)
        assert received[-1].width == 0, 'Stale prediction was not cleared'
        assert markers[-1].action == Marker.DELETE, 'Sweep persisted after loss'
        print('PASS: all points retained; moving and stationary groups; rest and timeout')
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
