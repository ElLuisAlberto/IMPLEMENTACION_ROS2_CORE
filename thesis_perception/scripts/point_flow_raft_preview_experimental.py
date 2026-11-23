#!/usr/bin/python3
# Copyright 2026 Luis Alberto Munoz Marin
# Licensed under the Apache License, Version 2.0
"""Experimental CUDA-postprocessed RAFT preview.

The legacy preview remains unchanged. This variant keeps correspondence,
Kalman filtering, support filtering and predicted-point assembly on CUDA
before copying only the final observed points to the CPU for ROS publication.
"""
import array
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as torch_nn
from torchvision.models.optical_flow import raft_small
from sensor_msgs.msg import PointField, PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data

from point_flow_preview import PointFlow


def median_velocity_5x5(velocity):
    """Match OpenCV's per-component 5x5 median with replicated borders."""
    channels = velocity.permute(2, 0, 1).unsqueeze(0)
    padded = torch_nn.pad(channels, (2, 2, 2, 2), mode="replicate")
    windows = padded.unfold(2, 5, 1).unfold(3, 5, 1)
    medians = windows.contiguous().view(*windows.shape[:4], 25).kthvalue(13, dim=-1).values
    return medians.squeeze(0).permute(1, 2, 0)


def supported_motion_5x5(moving):
    """Keep moving pixels with at least fifteen moving pixels in their 5x5 window."""
    fraction = torch_nn.avg_pool2d(
        moving.float()[None, None], kernel_size=5, stride=1, padding=2
    )[0, 0]
    return moving & (fraction >= 14.5 / 25.0)


def adaptive_motion_5x5(moving, velocity):
    """Retain large groups or small groups with coherent local 3D velocity."""
    mask = moving.float()[None, None]
    count = torch_nn.avg_pool2d(mask, 5, stride=1, padding=2)[0, 0] * 25.0
    weighted = velocity.permute(2, 0, 1)[None] * mask
    mean_vector = torch_nn.avg_pool2d(weighted, 5, stride=1, padding=2)[0].permute(1, 2, 0)
    mean_vector = mean_vector * (25.0 / count.clamp_min(1.0))[..., None]
    speed = torch.linalg.vector_norm(velocity, dim=-1)
    mean_speed = torch_nn.avg_pool2d((speed * moving.float())[None, None], 5, stride=1, padding=2)[0, 0]
    mean_speed = mean_speed * 25.0 / count.clamp_min(1.0)
    mean_norm = torch.linalg.vector_norm(mean_vector, dim=-1)
    coherence = mean_norm / mean_speed.clamp_min(1e-6)
    alignment = (velocity * mean_vector).sum(dim=-1) / (speed * mean_norm).clamp_min(1e-6)
    speed_ratio = speed / mean_speed.clamp_min(1e-6)
    large = moving & (count >= 14.5)
    small = (moving & (count >= 4.5) & (count < 14.5) & (coherence >= 0.85)
             & (alignment >= 0.85) & (speed_ratio >= 0.5) & (speed_ratio <= 1.8))
    return large, small


def confirmed_motion(current, previous, iy, ix):
    """Confirm current candidates using their correspondence in the previous frame."""
    if previous is None or iy is None or ix is None:
        return torch.zeros_like(current)
    return current & previous[iy, ix]


def confirmed_motion_3x3(current, previous, iy, ix, depth, prev_depth, velocity, prev_velocity):
    """Add nearby temporal matches with compatible depth and 3D velocity."""
    exact = confirmed_motion(current, previous, iy, ix)
    if previous is None or iy is None or ix is None:
        return exact
    height, width = current.shape
    mask_windows = torch_nn.unfold(
        previous.float()[None, None], kernel_size=3, padding=1
    ).reshape(9, height, width)
    depth_windows = torch_nn.unfold(
        prev_depth[None, None], kernel_size=3, padding=1
    ).reshape(9, height, width)
    velocity_windows = torch_nn.unfold(
        prev_velocity.permute(2, 0, 1)[None], kernel_size=3, padding=1
    ).reshape(3, 9, height, width)
    neighbor_mask = mask_windows[:, iy, ix] > 0.5
    neighbor_depth = depth_windows[:, iy, ix]
    neighbor_velocity = velocity_windows[:, :, iy, ix]
    current_velocity = velocity.permute(2, 0, 1)[:, None]
    current_speed = torch.linalg.vector_norm(current_velocity, dim=0)
    neighbor_speed = torch.linalg.vector_norm(neighbor_velocity, dim=0)
    alignment = (current_velocity * neighbor_velocity).sum(dim=0) / (
        current_speed * neighbor_speed
    ).clamp_min(1e-6)
    ratio = current_speed / neighbor_speed.clamp_min(1e-6)
    compatible = (neighbor_mask & torch.isfinite(neighbor_depth) & (neighbor_depth >= 0.1)
                  & (torch.abs(neighbor_depth - depth[None]) < 0.08)
                  & (alignment >= 0.85) & (ratio >= 0.5) & (ratio <= 2.0))
    return exact | (current & compatible.any(dim=0))


class ExperimentalRaftPointFlow(PointFlow):
    """Use compiled RAFT and CUDA correspondence postprocessing."""

    def __init__(self):
        if not torch.cuda.is_available():
            raise RuntimeError("RAFT requires a CUDA-capable PyTorch installation")

        weight_path = Path(os.environ.get("THESIS_RAFT_WEIGHTS", "")).expanduser()
        if not weight_path.is_file():
            raise RuntimeError(
                "Set THESIS_RAFT_WEIGHTS to the local raft_small C_T_V2 weights"
            )

        self.torch = torch
        torch.backends.cudnn.benchmark = False
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False

        model = raft_small(weights=None, progress=False)
        model.load_state_dict(
            torch.load(str(weight_path), map_location="cpu", weights_only=True)
        )
        model = model.eval().to("cuda")
        try:
            self.model = torch.compile(model, mode="reduce-overhead")
            compile_status = "torch.compile reduce-overhead"
        except Exception as error:
            self.model = model
            compile_status = f"eager fallback ({error})"

        super().__init__(
            node_name="point_flow_raft_preview_experimental",
            output_topic="/thesis/perception/point_flow_raft_preview_experimental",
        )
        self.sweep_publisher = self.create_publisher(
            Marker,
            "/thesis/perception/raft_swept_surface_experimental",
            qos_profile_sensor_data,
        )
        self.declare_parameter("sweep_mode", "python")
        self.declare_parameter("temporal_motion_confirm", False)
        self.declare_parameter("temporal_search_3x3", False)
        self.declare_parameter("adaptive_small_motion", False)
        self.declare_parameter("profile_motion", False)
        self.temporal_motion_confirm = self.get_parameter("temporal_motion_confirm").value
        self.temporal_search_3x3 = self.get_parameter("temporal_search_3x3").value
        self.adaptive_small_motion = self.get_parameter("adaptive_small_motion").value
        self.profile_motion = self.get_parameter("profile_motion").value
        self.previous_motion_candidates = None
        self.declare_parameter("max_valid_range_m", 1.2)
        self.max_valid_range_m = self.get_parameter("max_valid_range_m").value
        if not np.isfinite(self.max_valid_range_m) or self.max_valid_range_m <= 0.1:
            raise ValueError("max_valid_range_m must be finite and greater than 0.1")
        self.declare_parameter("profile_stages", False)
        self.declare_parameter("profile_pairing", False)
        self.declare_parameter("async_pairing", False)
        self.declare_parameter("max_pair_age_s", 0.5)
        self.async_pairing = self.get_parameter("async_pairing").value
        self.max_pair_age_s = self.get_parameter("max_pair_age_s").value
        if not 0.0 < self.max_pair_age_s <= 1.0:
            raise ValueError("max_pair_age_s must be between 0 and 1 second")
        self.worker_condition = threading.Condition()
        self.pending_pair = None
        self.worker_running = self.async_pairing
        self.worker_busy = False
        self.worker_thread = None
        self.profile_stages = self.get_parameter("profile_stages").value
        self.profile_pairing = self.profile_stages or self.get_parameter("profile_pairing").value
        self.stage_samples = []
        self.match_stats = {"rgb": 0, "depth": 0, "paired": 0, "throttled": 0, "expired_depth": 0, "replaced": 0, "stale": 0}
        self.match_started = time.monotonic()
        self.match_deltas = []
        self.sweep_mode = self.get_parameter("sweep_mode").value
        if self.sweep_mode not in ("python", "cpp"):
            raise ValueError("sweep_mode must be python or cpp")
        self.line_input = self.create_publisher(
            PointCloud2, "/thesis/perception/raft_lines_input_experimental",
            qos_profile_sensor_data,
        )
        self.sweep_header = None
        if self.async_pairing:
            self.worker_thread = threading.Thread(target=self.run_worker, name="raft_worker")
            self.worker_thread.start()
        self.get_logger().info(
            f"Experimental RAFT small C_T_V2 running on CUDA: {compile_status}; "
            f"async_pairing={self.async_pairing}"
        )

    def run_worker(self):
        while True:
            with self.worker_condition:
                self.worker_condition.wait_for(lambda: not self.worker_running or self.pending_pair is not None)
                if not self.worker_running:
                    return
                depth, color, received = self.pending_pair
                self.pending_pair = None
                self.worker_busy = True
            try:
                # A delayed worker must never revive an old visualization.
                if time.monotonic() - received <= 0.5:
                    self.process_pair(depth, color)
            except Exception as exc:
                self.get_logger().error(f"RAFT worker failed: {exc}")
                self.previous = None
            finally:
                with self.worker_condition:
                    self.worker_busy = False

    def stop_worker(self):
        if self.worker_thread is not None:
            with self.worker_condition:
                self.worker_running = False
                self.pending_pair = None
                self.worker_condition.notify_all()
            self.worker_thread.join()

    def clear_stale(self):
        if not getattr(self, "async_pairing", False):
            return super().clear_stale()
        with self.worker_condition:
            if not self.worker_busy:
                super().clear_stale()

    def submit_pair(self, depth, color):
        if not self.async_pairing:
            self.process_pair(depth, color)
            return
        # ROS time is used here, so simulated playback must provide /clock.
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self.stamp(depth)
        if now <= 0 or age < -0.1 or age > self.max_pair_age_s:
            if self.profile_pairing:
                self.match_stats["stale"] += 1
            return
        with self.worker_condition:
            if self.profile_pairing and self.pending_pair is not None:
                self.match_stats["replaced"] += 1
            self.pending_pair = (depth, color, time.monotonic())
            self.worker_condition.notify()

    def clear_extra_preview(self):
        if self.sweep_header is not None:
            if self.sweep_mode == "cpp":
                self.publish_line_input(self.sweep_header, np.empty((0, 6), dtype="<f4"))
                return
            marker = Marker()
            marker.header = self.sweep_header
            marker.ns = "raft_swept_surface_experimental"
            marker.action = Marker.DELETE
            self.sweep_publisher.publish(marker)

    def publish_line_input(self, header, segments):
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, len(segments)
        cloud.fields = [
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(("x", "y", "z", "end_x", "end_y", "end_z"))
        ]
        cloud.is_bigendian = False
        cloud.point_step, cloud.row_step = 24, 24 * len(segments)
        cloud.is_dense = True
        cloud.data = array.array("B", segments.tobytes())
        self.line_input.publish(cloud)

    def publish_extra_preview(self, header, point, moving, velocity):
        self.sweep_header = header
        if self.sweep_mode == "cpp":
            points = point[moving]
            ends = points + velocity[moving] * 0.5
            segments = torch.cat((points, ends), dim=1).detach().cpu().numpy().astype("<f4", copy=False)
            self.publish_line_input(header, segments)
            return
        marker = Marker()
        marker.header = header
        marker.ns = "raft_swept_surface_experimental"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.001
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.0,
            0.85,
            1.0,
            0.05,
        )
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 600_000_000
        if bool(moving.any()):
            points = point[moving]
            ends = points + velocity[moving] * 0.5
            segments = torch.stack((points, ends), dim=1).detach().cpu().numpy()
            marker.action = Marker.ADD
            marker.points = [
                Point(x=float(x), y=float(y), z=float(z))
                for x, y, z in segments.reshape(-1, 3)
            ]
        self.sweep_publisher.publish(marker)

    def motion_image(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def on_color(self, msg):
        if self.profile_pairing:
            self.match_stats["rgb"] += 1
        super().on_color(msg)

    def on_depth(self, msg):
        if self.profile_pairing:
            self.match_stats["depth"] += 1
        super().on_depth(msg)

    def match_frames(self):
        if not self.profile_pairing and not self.async_pairing:
            return super().match_frames()
        if self.info is None:
            return
        while self.depths and self.colors:
            depth = self.depths[0]
            target = self.stamp(depth)
            nearest = min(self.colors, key=lambda c: abs(self.stamp(c) - target))
            delta = abs(self.stamp(nearest) - target)
            if delta <= 0.008:
                self.depths.popleft()
                self.colors.remove(nearest)
                if self.profile_pairing:
                    self.match_stats["paired"] += 1
                    self.match_deltas.append(delta * 1000)
                    if not self.async_pairing and self.last_frame is not None and 0 <= target - self.last_frame < 0.033:
                        self.match_stats["throttled"] += 1
                self.submit_pair(depth, nearest)
            elif self.stamp(self.colors[-1]) > target + 0.008:
                self.depths.popleft()
                if self.profile_pairing:
                    self.match_stats["expired_depth"] += 1
            else:
                break
        elapsed = time.monotonic() - self.match_started
        if self.profile_pairing and elapsed >= 10.0:
            self.get_logger().info(
                "matching elapsed_s=%.2f %s delta_ms_p50/p95=%s" % (
                    elapsed, " ".join(f"{k}={v}" for k, v in self.match_stats.items()),
                    "/".join(f"{x:.2f}" for x in np.percentile(self.match_deltas, [50, 95]))
                    if self.match_deltas else "n/a",
                )
            )
            self.match_stats = dict.fromkeys(self.match_stats, 0)
            self.match_deltas.clear()
            self.match_started = time.monotonic()

    def grayscale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def process_pair(self, msg, color_msg):
        started = time.monotonic()
        stamp = self.stamp(msg)
        if self.last_frame is not None and 0 <= stamp - self.last_frame < 0.033:
            return
        self.last_frame = stamp
        stage_times = {}
        stage_start = time.monotonic()

        def checkpoint(name):
            nonlocal stage_start
            if self.profile_stages:
                torch.cuda.synchronize()
                now = time.monotonic()
                stage_times[name] = (now - stage_start) * 1000
                stage_start = now

        try:
            if msg.encoding not in ("16UC1", "32FC1"):
                raise ValueError("Unsupported depth encoding")
            rgb = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="rgb8"
            )
            raw_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            rgb = cv2.resize(rgb, (256, 192), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            depth_cpu = cv2.resize(
                raw_depth.astype(np.float32)
                * (0.001 if msg.encoding == "16UC1" else 1.0),
                (256, 192),
                interpolation=cv2.INTER_NEAREST,
            )
            info = self.info
            fx = info.k[0] * 256 / info.width
            fy = info.k[4] * 192 / info.height
            cx = info.k[2] * 256 / info.width
            cy = info.k[5] * 192 / info.height
            if fx <= 0 or fy <= 0 or msg.header.frame_id != info.header.frame_id:
                self.previous = None
                return

            checkpoint("cpu_prepare")
            gray_cuda = torch.from_numpy(gray).to("cuda", dtype=torch.float32)[None, None]
            depth_cuda = torch.from_numpy(depth_cpu).to("cuda", dtype=torch.float32)[None, None]
            checkpoint("gray_depth_upload")
            rgb_cuda = (
                torch.from_numpy(rgb)
                .permute(2, 0, 1)
                .contiguous()
                .to("cuda", dtype=torch.float32)
            )
            checkpoint("unused_rgb_upload")

            height, width = 192, 256
            y, x = torch.meshgrid(
                torch.arange(height, device="cuda", dtype=torch.float32),
                torch.arange(width, device="cuda", dtype=torch.float32),
                indexing="ij",
            )
            checkpoint("grid")
            point = torch.stack(
                ((x - cx) * depth_cuda[0, 0] / fx,
                 (y - cy) * depth_cuda[0, 0] / fy,
                 depth_cuda[0, 0]),
                dim=-1,
            )
            valid = (
                torch.isfinite(depth_cuda[0, 0])
                & (depth_cuda[0, 0] >= 0.1)
                & (torch.linalg.vector_norm(point, dim=-1) <= self.max_valid_range_m)
            )
            usable = torch.zeros((height, width), device="cuda", dtype=torch.bool)
            velocity = torch.zeros((height, width, 3), device="cuda", dtype=torch.float32)
            variance = torch.full(
                (height, width), 0.04, device="cuda", dtype=torch.float32
            )
            previous_candidates = None
            previous_depth = previous_velocity = None
            ix = iy = None

            if self.previous is not None:
                prev_gray, prev_rgb, prev_depth, prev_vel, prev_var, prev_stamp, prev_frame = self.previous
                dt = stamp - prev_stamp
                if 0.015 < dt < 0.5 and msg.header.frame_id == prev_frame:
                    previous_candidates = self.previous_motion_candidates
                    previous_depth, previous_velocity = prev_depth[0, 0], prev_vel
                    current_rgb = cv2.resize(rgb, (256, 192), interpolation=cv2.INTER_AREA)
                    arrays = np.stack((current_rgb, prev_rgb, prev_rgb, current_rgb))
                    frames = (
                        torch.from_numpy(arrays)
                        .permute(0, 3, 1, 2)
                        .contiguous()
                        .to("cuda", dtype=torch.float32)
                        .div_(127.5)
                        .sub_(1.0)
                    )
                    checkpoint("setup_and_frames")
                    with torch.inference_mode():
                        flows = self.model(
                            frames[:2], frames[2:], num_flow_updates=12
                        )[-1]
                    checkpoint("raft_inference")
                    flow = flows.permute(0, 2, 3, 1)
                    backward = flow[0]
                    forward = flow[1]

                    mx = x + backward[:, :, 0]
                    my = y + backward[:, :, 1]
                    ix = torch.round(mx).long().clamp(0, width - 1)
                    iy = torch.round(my).long().clamp(0, height - 1)
                    forward_at_match = forward[iy, ix]
                    round_trip = torch.linalg.vector_norm(
                        backward + forward_at_match, dim=-1
                    )
                    prev_intensity = prev_gray[0, 0, iy, ix]
                    photometric_error = torch.abs(gray_cuda[0, 0] - prev_intensity)
                    pd = prev_depth[0, 0, iy, ix]
                    pv = prev_vel[iy, ix]
                    pp = prev_var[iy, ix]
                    inside = (mx >= 0) & (mx < width - 1) & (my >= 0) & (my < height - 1)
                    usable = (
                        valid & inside & torch.isfinite(pd) & (pd >= 0.1)
                        & (torch.abs(pd - depth_cuda[0, 0]) < 0.08)
                        & (round_trip < 1.5)
                        & (photometric_error < 30.0)
                    )
                    old = torch.stack(
                        ((mx - cx) * pd / fx,
                         (my - cy) * pd / fy,
                         pd),
                        dim=-1,
                    )
                    measured = (point - old) / dt
                    speed = torch.linalg.vector_norm(measured, dim=-1)
                    usable = usable & torch.isfinite(speed) & (speed < 2.0)
                    prior_var = pp + 0.04 * dt
                    gain = prior_var / (prior_var + 0.025)
                    velocity = pv + gain[:, :, None] * (measured - pv)
                    variance = (1.0 - gain) * prior_var
                    velocity = torch.where(usable[:, :, None], velocity, torch.zeros_like(velocity))
                    variance = torch.where(usable, variance, torch.full_like(variance, 0.04))

                    checkpoint("correspondence_kalman")
                    support = torch_nn.avg_pool2d(
                        usable.float()[None, None], kernel_size=5,
                        stride=1, padding=2
                    )[0, 0]
                    velocity = median_velocity_5x5(velocity)
                    usable = usable & (support >= 13.0 / 25.0)
                    velocity = torch.where(usable[:, :, None], velocity, torch.zeros_like(velocity))
                    checkpoint("median_and_support")

            checkpoint("state_finalize")
            self.previous = (
                gray_cuda,
                rgb,
                depth_cuda,
                velocity,
                variance,
                stamp,
                msg.header.frame_id,
            )
            speed = torch.linalg.vector_norm(velocity, dim=-1)
            raw_moving = usable & (speed > 0.06)
            large = small = None
            def confirm(candidates):
                if self.temporal_search_3x3:
                    return confirmed_motion_3x3(
                        candidates, previous_candidates, iy, ix, depth_cuda[0, 0],
                        previous_depth, velocity, previous_velocity,
                    )
                return confirmed_motion(candidates, previous_candidates, iy, ix)

            if self.adaptive_small_motion:
                large, small = adaptive_motion_5x5(raw_moving, velocity)
                candidates = large | small
                confirmed = confirm(candidates)
                moving = confirmed if self.temporal_motion_confirm else (large | (confirmed & small))
            else:
                candidates = supported_motion_5x5(raw_moving)
                moving = (confirm(candidates)
                          if self.temporal_motion_confirm else candidates)
            self.previous_motion_candidates = (
                candidates if self.temporal_motion_confirm or self.adaptive_small_motion else None
            )
            if self.profile_motion and self.count % 30 == 0:
                self.get_logger().info(
                    "motion_debug valid=%d usable=%d speed_candidates=%d large=%d small=%d confirmed=%d"
                    % (int(valid.sum().item()), int(usable.sum().item()),
                       int(raw_moving.sum().item()), int(large.sum().item()) if large is not None else -1,
                       int(small.sum().item()) if small is not None else -1, int(moving.sum().item()))
                )
            predicted = point + torch.where(
                moving[:, :, None],
                velocity * 0.5,
                torch.zeros_like(velocity),
            )
            keep = valid & torch.isfinite(predicted).all(dim=-1)
            checkpoint("mask_and_predict")
            keep_cpu = keep.detach().cpu().numpy()
            coords = predicted.detach().cpu().numpy()[keep_cpu].astype("<f4")
            moving_cpu = moving.detach().cpu().numpy()[keep_cpu]
            checkpoint("download_and_select")
            rgb32 = np.where(moving_cpu, np.uint32(0xff6400), np.uint32(0x00ff00))
            packed = np.empty(
                coords.shape[0],
                dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
            )
            packed["x"], packed["y"], packed["z"], packed["rgb"] = (
                coords[:, 0],
                coords[:, 1],
                coords[:, 2],
                rgb32,
            )
            cloud = PointCloud2()
            cloud.header = msg.header
            cloud.height, cloud.width = 1, len(packed)
            cloud.fields = [
                PointField(
                    name=name,
                    offset=4 * index,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
                for index, name in enumerate(("x", "y", "z", "rgb"))
            ]
            cloud.is_bigendian = False
            cloud.point_step = 16
            cloud.row_step = 16 * len(packed)
            cloud.is_dense = True
            cloud.data = array.array("B", packed.tobytes())
            checkpoint("pack_cloud")
            publish_started = time.monotonic()
            self.publisher.publish(cloud)
            self.publish_extra_preview(msg.header, point, moving, velocity)
            checkpoint("publish_and_lines")
            self.last_published = time.monotonic()
            self.last_header = msg.header
            self.cleared = False
            self.count += 1
            if self.profile_stages and "raft_inference" in stage_times:
                self.stage_samples.append(stage_times)
                if len(self.stage_samples) >= 60:
                    names = tuple(stage_times)
                    if all(all(name in sample for name in names) for sample in self.stage_samples):
                        values = np.asarray([[sample[name] for name in names] for sample in self.stage_samples])
                        p50, p95 = np.percentile(values, [50, 95], axis=0)
                        self.get_logger().info("stages_ms p50/p95 " + " ".join(
                            f"{name}={median:.3f}/{tail:.3f}"
                            for name, median, tail in zip(names, p50, p95)
                        ))
                    self.stage_samples.clear()
            if self.profile_timing:
                total = time.monotonic() - started
                publish = time.monotonic() - publish_started
                self.timing_samples.append((0.0, total - publish, publish, total))
                if len(self.timing_samples) >= 60:
                    ms = np.percentile(
                        np.asarray(self.timing_samples) * 1000,
                        [50, 95],
                        axis=0,
                    )
                    self.get_logger().info(
                        "timing_ms p50/p95 flow=%.1f/%.1f cpu_other=%.1f/%.1f "
                        "publish=%.1f/%.1f total=%.1f/%.1f"
                        % tuple(ms.T.flatten())
                    )
                    self.timing_samples.clear()
            if self.count % 30 == 0:
                self.get_logger().info(
                    "points=%d tracked=%d moving=%d processing_ms=%.1f"
                    % (
                        len(packed),
                        int(usable.sum().item()),
                        int(moving.sum().item()),
                        (time.monotonic() - started) * 1000,
                    )
                )
        except (ValueError, cv2.error, RuntimeError) as exc:
            self.previous = None
            self.get_logger().error("frame rejected: %s" % exc)


def main():
    rclpy.init()
    node = None
    try:
        node = ExperimentalRaftPointFlow()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.stop_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
