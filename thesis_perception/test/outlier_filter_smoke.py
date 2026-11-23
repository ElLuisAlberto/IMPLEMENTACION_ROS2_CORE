# Copyright 2026 Luis Alberto Munoz Marin
#
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
"""Check the optional filters with an isolated cluster and a stray point."""
import os
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def main():
    rclpy.init()
    node = rclpy.create_node('outlier_smoke')
    before, after = [], []
    node.create_subscription(PointCloud2, '/thesis/perception/points_before_outlier',
                             before.append, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/test/filtered', after.append, qos_profile_sensor_data)
    publisher = node.create_publisher(PointCloud2, '/test/input', qos_profile_sensor_data)
    executable = os.path.join(get_package_prefix('thesis_perception'), 'lib',
                              'thesis_perception', 'pointcloud_preprocessor_node')

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        for method in ['none', 'statistical', 'radius']:
            process = subprocess.Popen([
                executable, '--ros-args', '-p', 'input_topic:=/test/input',
                '-p', 'output_topic:=/test/filtered', '-p', 'enable_voxel:=false',
                '-p', 'max_valid_range_m:=1.0', '-p', 'outlier_method:=' + method],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                pump(1.0)
                before.clear()
                after.clear()
                header = Header()
                header.frame_id = 'camera'
                header.stamp = node.get_clock().now().to_msg()
                fields = [PointField(name=name, offset=offset,
                                     datatype=PointField.FLOAT32, count=1)
                          for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
                points = [(0.005 * (i % 10), 0.005 * (i // 10), 0.6, 0.0)
                          for i in range(100)]
                points.append((0.45, 0.45, 0.6, 0.0))
                publisher.publish(point_cloud2.create_cloud(header, fields, points))
                pump(0.4)
                assert before and after, f'{method}: missing output'
                assert before[-1].width == 101, f'{method}: original cloud changed'
                expected = 101 if method == 'none' else 100
                assert after[-1].width == expected, (method, after[-1].width)
                assert before[-1].header.stamp == after[-1].header.stamp
            finally:
                process.terminate()
                process.wait(timeout=5)
        print('PASS: preserved original; statistical and radius remove synthetic stray point')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
