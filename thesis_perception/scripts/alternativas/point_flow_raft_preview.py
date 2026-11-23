#!/usr/bin/python3
# Copyright 2026 Luis Alberto Munoz Marin
# Licensed under the Apache License, Version 2.0
"""Optional RAFT RGB-D preview with a topic separate from Farneback."""
import os
import array
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy

# The source is grouped under scripts/alternativas; installed executions keep
# the original executable name and find the shared module alongside it.
shared_scripts = Path(__file__).resolve().parent.parent
if (shared_scripts / 'point_flow_preview.py').is_file():
    sys.path.insert(0, str(shared_scripts))
from point_flow_preview import PointFlow
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField


def swept_cells(points, velocities, horizon=0.5, voxel_size=0.025, steps=11):
    """Sample visible moving points; cells are a visualization, not a safety bound."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.int32)
    samples = [np.floor((points + velocities * t) / voxel_size).astype(np.int32)
               for t in np.linspace(0.0, horizon, steps)]
    return np.unique(np.concatenate(samples), axis=0)


class RaftPointFlow(PointFlow):
    """Reuse the RGB-D preview and replace only its correspondence estimator."""

    def __init__(self):
        try:
            import torch
            from torchvision.models.optical_flow import raft_small
        except ImportError as exc:
            raise RuntimeError('RAFT requires an approved PyTorch/TorchVision environment') from exc
        if not torch.cuda.is_available():
            raise RuntimeError('RAFT requires a CUDA-capable PyTorch installation')
        weight_path = Path(os.environ.get('THESIS_RAFT_WEIGHTS', '')).expanduser()
        if not weight_path.is_file():
            raise RuntimeError('Set THESIS_RAFT_WEIGHTS to the local raft_small C_T_V2 weights')
        self.torch = torch
        self.model = raft_small(weights=None, progress=False)
        self.model.load_state_dict(torch.load(str(weight_path), map_location='cpu', weights_only=True))
        self.model = self.model.eval().to('cuda')
        super().__init__(node_name='point_flow_raft_preview',
                         output_topic='/thesis/perception/point_flow_raft_preview')
        self.sweep_publisher = self.create_publisher(
            Marker, '/thesis/perception/raft_swept_surface', qos_profile_sensor_data)
        self.sweep_header = None
        self.declare_parameter('sweep_mode', 'legacy')
        self.sweep_mode = self.get_parameter('sweep_mode').value
        if self.sweep_mode not in ('legacy', 'cpp', 'both'):
            raise ValueError('sweep_mode must be legacy, cpp, or both')
        self.sweep_input_publisher = self.create_publisher(
            PointCloud2, '/thesis/perception/raft_sweep_input', qos_profile_sensor_data)
        self.get_logger().info('RAFT small C_T_V2 running on CUDA; original preview is separate')

    def clear_extra_preview(self):
        if self.sweep_header is not None:
            marker = Marker()
            marker.header = self.sweep_header
            marker.ns = 'raft_swept_surface'
            marker.action = Marker.DELETE
            self.sweep_publisher.publish(marker)
            if self.sweep_mode in ('cpp', 'both'):
                self.publish_sweep_input(self.sweep_header, np.empty((0, 3), np.float32),
                                         np.empty((0, 3), np.float32))

    def publish_sweep_input(self, header, points, velocities):
        values = np.concatenate((points, velocities), axis=1).astype('<f4', copy=False)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, len(values)
        cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'vx', 'vy', 'vz'))]
        cloud.point_step, cloud.row_step, cloud.is_dense = 24, 24 * len(values), True
        cloud.data = array.array('B', values.tobytes())
        self.sweep_input_publisher.publish(cloud)

    def publish_extra_preview(self, header, point, velocity, moving):
        self.sweep_header = header
        if self.sweep_mode in ('cpp', 'both'):
            self.publish_sweep_input(header, point[moving], velocity[moving])
        if self.sweep_mode == 'cpp':
            return
        if not np.any(moving):
            marker = Marker()
            marker.header = header
            marker.ns, marker.action = 'raft_swept_surface', Marker.DELETE
            self.sweep_publisher.publish(marker)
            return
        voxel_size = 0.025
        cells = swept_cells(point[moving], velocity[moving], voxel_size=voxel_size)
        marker = Marker()
        marker.header = header
        marker.ns = 'raft_swept_surface'
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = voxel_size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.85, 1.0, 0.32
        marker.lifetime.sec, marker.lifetime.nanosec = 0, 600_000_000
        marker.points = [Point(x=float(x), y=float(y), z=float(z))
                         for x, y, z in (cells + 0.5) * voxel_size]
        self.sweep_publisher.publish(marker)

    def motion_image(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def grayscale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def flow_pair(self, current, previous):
        torch = self.torch
        # TorchVision requires RGB values in [-1, 1] and dimensions divisible by 8.
        arrays = np.stack((current, previous, previous, current))
        frames = torch.from_numpy(arrays).permute(0, 3, 1, 2).to('cuda', dtype=torch.float32)
        frames = frames.div(127.5).sub(1.0)
        with torch.inference_mode():
            result = self.model(frames[:2], frames[2:], num_flow_updates=12)[-1]
        flow = result.permute(0, 2, 3, 1).cpu().numpy()
        return flow[0], flow[1]


def main():
    rclpy.init()
    node = None
    try:
        node = RaftPointFlow()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
