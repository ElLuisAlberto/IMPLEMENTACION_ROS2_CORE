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
"""Verify local movement on a connected, mostly stationary large cloud."""
import os
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def main():
    rclpy.init()
    node = rclpy.create_node('occlusion_smoke')
    outputs = []
    node.create_subscription(PointCloud2, '/thesis/perception/predicted_points',
                             outputs.append, qos_profile_sensor_data)
    publisher = node.create_publisher(PointCloud2, '/thesis/perception/points_filtered',
                                      qos_profile_sensor_data)
    executable = os.path.join(get_package_prefix('thesis_perception'), 'lib',
                              'thesis_perception', 'cloud_prediction_node')
    process = subprocess.Popen([executable], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
              for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def send(position=0.0):
        background = [(ix * 0.01, iy * 0.01, 0.8, 0.0)
                      for ix in range(30) for iy in range(40)]
        foreground = [(0.29 + ix * 0.01, 0.10 + iy * 0.01 + position, 0.8, 0.0)
                      for ix in range(1, 21) for iy in range(10)]
        points = background + foreground
        header = Header()
        header.frame_id = 'camera'
        header.stamp = node.get_clock().now().to_msg()
        publisher.publish(point_cloud2.create_cloud(header, fields, points))
        pump(0.05)
        matches = [m for m in outputs if m.header.stamp == header.stamp and m.width]
        if not matches:
            return None
        result = point_cloud2.read_points_numpy(matches[-1], field_names=['x', 'y', 'z'])
        return result, np.asarray(points)[:, :3], len(background)

    try:
        pump(1.5)
        for _ in range(24):
            send(0.0)
        moved = 0
        fixed_moved = 0
        count = 0
        for i in range(1, 36):
            sample = send(i * 0.006)
            if sample is None:
                continue
            result, source, n_background = sample
            delta = np.linalg.norm(result - source, axis=1)
            fixed_moved += np.count_nonzero(delta[:n_background] > 0.005)
            moved += np.count_nonzero(delta[n_background:] > 0.005)
            count += 1
        print(f'PASS: {count} frames; static body fixed, moving part projected')
        assert count >= 20, 'Too few matched frames'
        assert moved > 100, 'Local motion was not predicted'
        assert fixed_moved == 0, 'Static part moved'
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
