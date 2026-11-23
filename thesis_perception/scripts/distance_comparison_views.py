#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DistanceComparisonViews(Node):
    def __init__(self):
        super().__init__("comparacion_distancia_suavizado")
        self.info = None
        self.rgb = None
        self.last_publish = 0.0
        self.bridge = CvBridge()
        self.distance_pub = self.create_publisher(
            PointCloud2, "/thesis/comparacion/distancia", qos_profile_sensor_data
        )
        self.smoothed_pub = self.create_publisher(
            PointCloud2, "/thesis/comparacion/suavizado", qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/camera/d435i/aligned_depth_to_color/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/d435i/aligned_depth_to_color/image_raw",
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/d435i/color/image_raw",
            self.rgb_callback,
            qos_profile_sensor_data,
        )

    def camera_info_callback(self, msg):
        self.info = msg

    def rgb_callback(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"Error converting RGB image: {exc}")

    def depth_callback(self, msg):
        now = time.monotonic()
        if self.info is None or self.rgb is None or now - self.last_publish < 0.1:
            return
        self.last_publish = now

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)
            if msg.encoding == "16UC1":
                depth *= 0.001

            height, width = depth.shape
            rgb = cv2.resize(self.rgb, (width, height), interpolation=cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

            fx, fy = self.info.k[0], self.info.k[4]
            cx, cy = self.info.k[2], self.info.k[5]
            if fx <= 0 or fy <= 0:
                return

            yy, xx = np.indices((height, width), dtype=np.float32)
            xn = (xx - cx) / fx
            yn = (yy - cy) / fy
            valid = np.isfinite(depth) & (depth >= 0.1)
            valid &= depth * np.sqrt(1.0 + xn * xn + yn * yn) <= 1.2

            finite_depth = np.where(np.isfinite(depth), depth, 0).astype(np.float32)
            smoothed = cv2.bilateralFilter(finite_depth, 5, 0.025, 3.0)

            for publisher, values in (
                (self.distance_pub, depth),
                (self.smoothed_pub, smoothed),
            ):
                x = xn[valid] * values[valid]
                y = yn[valid] * values[valid]
                z = values[valid]
                rgb_valid = rgb[valid]
                packed_rgb = (
                    (rgb_valid[:, 0].astype(np.uint32) << 16)
                    | (rgb_valid[:, 1].astype(np.uint32) << 8)
                    | rgb_valid[:, 2].astype(np.uint32)
                )

                cloud = PointCloud2()
                cloud.header = msg.header
                cloud.height = 1
                cloud.width = len(x)
                cloud.fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
                ]
                cloud.point_step = 16
                cloud.row_step = 16 * len(x)
                cloud.is_bigendian = False
                cloud.is_dense = True

                points = np.empty(len(x), dtype=[
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("rgb", "<u4"),
                ])
                points["x"] = x
                points["y"] = y
                points["z"] = z
                points["rgb"] = packed_rgb
                cloud.data = points.tobytes()
                publisher.publish(cloud)
        except Exception as exc:
            self.get_logger().error(f"Error creating comparison clouds: {exc}")


def main():
    rclpy.init()
    node = DistanceComparisonViews()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
