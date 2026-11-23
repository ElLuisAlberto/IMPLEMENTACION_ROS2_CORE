#!/usr/bin/python3
# Copyright 2026 Luis Alberto Munoz Marin
# Licensed under the Apache License, Version 2.0
"""Experimental RGB-D local motion preview; never used for robot control."""
import os
os.environ.setdefault('OPENCV_OPENCL_DEVICE', ':GPU:')
import time
import array
from collections import deque
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class PointFlow(Node):
    def __init__(self, node_name='point_flow_preview',
                 output_topic='/thesis/perception/point_flow_preview'):
        super().__init__(node_name)
        cv2.ocl.setUseOpenCL(True)
        if not cv2.ocl.useOpenCL():
            raise RuntimeError('OpenCL GPU unavailable; refusing silent CPU fallback')
        device = cv2.ocl.Device_getDefault().name()
        if 'NVIDIA' not in device:
            raise RuntimeError(f'Expected NVIDIA GPU, got {device}')
        self.get_logger().info(f'OpenCV OpenCL device: {device}')
        self.bridge = CvBridge()
        self.info = None
        self.colors = deque(maxlen=8)
        self.depths = deque(maxlen=8)
        self.previous = None
        self.last_frame = None
        self.count = 0
        self.last_published = None
        self.cleared = True
        self.declare_parameter('profile_timing', False)
        self.profile_timing = self.get_parameter('profile_timing').value
        self.timing_samples = []
        self.create_timer(0.25, self.clear_stale)
        self.create_subscription(CameraInfo, '/camera/d435i/aligned_depth_to_color/camera_info', self.on_info, qos_profile_sensor_data)
        self.declare_parameter('image_queue_depth', 1)
        image_queue_depth = self.get_parameter('image_queue_depth').value
        if not isinstance(image_queue_depth, int) or not 1 <= image_queue_depth <= 8:
            raise ValueError('image_queue_depth must be an integer between 1 and 8')
        latest_image_qos = QoSProfile(depth=image_queue_depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/d435i/color/image_raw', self.on_color, latest_image_qos)
        self.create_subscription(Image, '/camera/d435i/aligned_depth_to_color/image_raw', self.on_depth, latest_image_qos)
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)

    def motion_image(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def grayscale(self, image):
        return image

    def flow_pair(self, current, previous):
        backward = cv2.calcOpticalFlowFarneback(
            cv2.UMat(current), cv2.UMat(previous), None, .5, 3, 15, 3, 5, 1.2, 0).get()
        forward = cv2.calcOpticalFlowFarneback(
            cv2.UMat(previous), cv2.UMat(current), None, .5, 3, 15, 3, 5, 1.2, 0).get()
        return backward, forward

    def clear_stale(self):
        if self.last_published is None or self.cleared:
            return
        if time.monotonic() - self.last_published > 0.5:
            empty = PointCloud2()
            empty.header = self.last_header
            empty.height, empty.width = 1, 0
            empty.point_step, empty.row_step = 16, 0
            empty.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                            for n, o in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
            self.publisher.publish(empty)
            self.clear_extra_preview()
            self.previous = None
            self.cleared = True

    def clear_extra_preview(self):
        pass

    def publish_extra_preview(self, header, point, velocity, moving):
        pass

    def on_info(self, msg):
        self.info = msg

    def on_color(self, msg):
        self.colors.append(msg)
        self.match_frames()

    def on_depth(self, msg):
        self.depths.append(msg)
        self.match_frames()

    def match_frames(self):
        if self.info is None:
            return
        while self.depths and self.colors:
            depth = self.depths[0]
            target = self.stamp(depth)
            nearest = min(self.colors, key=lambda c: abs(self.stamp(c) - target))
            delta = abs(self.stamp(nearest) - target)
            if delta <= .008:
                self.depths.popleft()
                self.colors.remove(nearest)
                self.process_pair(depth, nearest)
            elif self.stamp(self.colors[-1]) > target + .008:
                self.depths.popleft()
            else:
                break

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def process_pair(self, msg, color_msg):
        now = time.monotonic()
        stamp = self.stamp(msg)
        if self.last_frame is not None and 0 <= stamp - self.last_frame < 0.033:
            return
        self.last_frame = stamp
        try:
            flow_seconds = 0.0
            color = self.motion_image(color_msg)
            if msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
            raw_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = raw_depth.astype(np.float32) * (0.001 if msg.encoding == '16UC1' else 1.0)
            w, h = 256, 192
            motion_image = cv2.resize(color, (w, h), interpolation=cv2.INTER_AREA)
            gray = self.grayscale(motion_image)
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
            info = self.info
            fx, fy = info.k[0] * w / info.width, info.k[4] * h / info.height
            cx, cy = info.k[2] * w / info.width, info.k[5] * h / info.height
            if fx <= 0 or fy <= 0 or msg.header.frame_id != info.header.frame_id:
                self.previous = None
                return
            y, x = np.indices((h, w), dtype=np.float32)
            point = np.stack(((x - cx) * depth / fx, (y - cy) * depth / fy, depth), axis=-1)
            valid = np.isfinite(depth) & (depth >= .1) & (np.linalg.norm(point, axis=2) <= 1.0)
            velocity = np.zeros((h, w, 3), np.float32)
            variance = np.full((h, w), 0.04, np.float32)
            usable = np.zeros((h, w), bool)
            if self.previous is not None:
                prev_gray, prev_image, prev_depth, prev_vel, prev_var, prev_stamp, prev_frame = self.previous
                dt = stamp - prev_stamp
                if .015 < dt < .5 and msg.header.frame_id == prev_frame:
                    # Backward flow provides a previous-image coordinate for each current pixel.
                    flow_start = time.monotonic() if self.profile_timing else None
                    flow, forward = self.flow_pair(motion_image, prev_image)
                    if self.profile_timing:
                        flow_seconds = time.monotonic() - flow_start
                    mx, my = x + flow[:, :, 0], y + flow[:, :, 1]
                    forward_at_match = cv2.remap(
                        forward, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    round_trip = np.linalg.norm(flow + forward_at_match, axis=2)
                    prev_intensity = cv2.remap(
                        prev_gray, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    photometric_error = np.abs(gray.astype(np.float32) - prev_intensity)
                    pd = cv2.remap(prev_depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
                    pv = cv2.remap(prev_vel, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    pp = cv2.remap(prev_var, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    inside = (mx >= 0) & (mx < w - 1) & (my >= 0) & (my < h - 1)
                    usable = (valid & inside & np.isfinite(pd) & (pd >= .1)
                              & (np.abs(pd - depth) < .08) & (round_trip < 1.5)
                              & (photometric_error < 30.0))
                    old = np.stack(((mx - cx) * pd / fx, (my - cy) * pd / fy, pd), axis=-1)
                    measured = (point - old) / dt
                    speed = np.linalg.norm(measured, axis=2)
                    usable &= np.isfinite(speed) & (speed < 2.0)
                    # Independent scalar Kalman velocity uncertainty for each correspondence.
                    prior_var = pp + 0.04 * dt
                    gain = prior_var / (prior_var + 0.025)
                    velocity = pv + gain[:, :, None] * (measured - pv)
                    variance = (1.0 - gain) * prior_var
                    velocity[~usable] = 0
                    variance[~usable] = .04
                    # Reject isolated correspondence noise and smooth each component locally.
                    # Unobserved pixels remain unprojected; no geometry is invented.
                    support = cv2.boxFilter(usable.astype(np.float32), -1, (5, 5),
                                            normalize=False, borderType=cv2.BORDER_CONSTANT)
                    velocity = cv2.medianBlur(velocity, 5)
                    usable &= support >= 13
                    velocity[~usable] = 0
                    variance[~usable] = .04
            self.previous = (gray, motion_image, depth, velocity, variance, stamp, msg.header.frame_id)
            speed = np.linalg.norm(velocity, axis=2)
            moving = usable & (speed > .06)
            # Publish only observed points; orange points are displaced by 0.5 s.
            predicted = point.copy()
            predicted[moving] += velocity[moving] * .5
            keep = valid & np.isfinite(predicted).all(axis=2)
            coords = predicted[keep].astype('<f4')
            colors = np.where(moving[keep], np.uint32(0xff6400), np.uint32(0x00ff00))
            packed = np.empty(coords.shape[0], dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            packed['x'], packed['y'], packed['z'], packed['rgb'] = coords[:, 0], coords[:, 1], coords[:, 2], colors
            cloud = PointCloud2()
            cloud.header = msg.header
            cloud.height, cloud.width = 1, len(packed)
            cloud.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1) for n, o in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
            cloud.is_bigendian, cloud.point_step, cloud.row_step, cloud.is_dense = False, 16, 16 * len(packed), True
            cloud.data = array.array('B', packed.tobytes())
            publish_start = time.monotonic() if self.profile_timing else None
            self.publisher.publish(cloud)
            self.publish_extra_preview(msg.header, point, velocity, moving)
            if self.profile_timing:
                end = time.monotonic()
                total = end - now
                publication = end - publish_start
                self.timing_samples.append((flow_seconds, total - flow_seconds - publication,
                                            publication, total))
                if len(self.timing_samples) >= 60:
                    ms = np.percentile(np.asarray(self.timing_samples) * 1000, [50, 95], axis=0)
                    self.get_logger().info(
                        'timing_ms p50/p95 flow=%.1f/%.1f cpu_other=%.1f/%.1f '
                        'publish=%.1f/%.1f total=%.1f/%.1f' %
                        tuple(ms.T.flatten()))
                    self.timing_samples.clear()
            self.last_published, self.last_header, self.cleared = time.monotonic(), msg.header, False
            self.count += 1
            if self.count % 30 == 0:
                self.get_logger().info(f'points={len(packed)} tracked={usable.sum()} moving={moving.sum()} processing_ms={(time.monotonic()-now)*1000:.1f}')
        except (ValueError, cv2.error) as exc:
            self.previous = None
            self.get_logger().error(f'frame rejected: {exc}')


def main():
    rclpy.init()
    node = PointFlow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
