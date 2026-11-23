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
"""Verify a stationary surface remains fixed when a moving patch occludes it."""
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

    def send(position=None):
        def visible(ix, iy):
            return (position is None or not
                    (position <= ix * 0.01 < position + 0.10 and -0.05 <= iy * 0.01 < 0.05))

        background = [(ix * 0.01, iy * 0.01, 0.80, 0.0)
                      for ix in range(-20, 21) for iy in range(-15, 16) if visible(ix, iy)]
        foreground = [] if position is None else [
            (position + ix * 0.01, iy * 0.01, 0.76, 0.0)
            for ix in range(10) for iy in range(-5, 5)]
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
            send()
        # The foreground also remains stationary long enough to enter the map.
        for _ in range(24):
            send(-0.17)
        moved = 0
        checked = 0
        coverage = []
        for i in range(35):
            sample = send(-0.17 + i * 0.006)
            if sample is None:
                continue
            result, source, n_background = sample
            delta = np.linalg.norm(result - source, axis=1)
            assert np.count_nonzero(delta[:n_background] > 0.002) == 0, (
                'Stationary background was translated', np.max(delta[:n_background]))
            moving_points = np.count_nonzero(delta[n_background:] > 0.005)
            moved += moving_points
            coverage.append(moving_points / (len(delta) - n_background))
            checked += 1
        assert checked >= 20, checked
        assert moved > 0, 'Moving foreground never produced a prediction'
        assert np.median(coverage[-10:]) >= 0.9, ('Incomplete moving object', coverage)
        print(f'PASS: {checked} occlusion frames; background fixed, foreground predicted')
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
