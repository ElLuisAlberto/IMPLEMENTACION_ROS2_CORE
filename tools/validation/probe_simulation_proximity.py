#!/usr/bin/env python3
"""Observe simulation proximity without publishing commands or changing parameters.

Output is JSON on stdout. Latest samples are diagnostic observations, not a
synchronized safety decision. Use one instance at a time in the launch domain.
"""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

from thesis_core.geometry_profile import legacy_jaco_geometry_profile
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII, SEGMENT_NAMES, capsule_segments, minimum_sphere_clearance,
    transform_matrix,
)
from thesis_core.multiarm_coordination import (
    CapsuleOccupancy, RobotOccupancySnapshot, evaluate_four_arm_safety,
)
from thesis_interfaces.msg import (
    GeometrySnapshot, MultiArmSafetyStatus, Obstacle, ProximityStatus,
)
from thesis_simulation.hybrid_scenario import build_hybrid_three_sim_scenario


ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.0e-9


def point(message):
    return (message.x, message.y, message.z)


def statistics_or_none(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None
    return {'min': min(finite), 'max': max(finite), 'median': statistics.median(finite)}


class ProximityProbe(Node):
    """Keep observations only; no command publisher, client or timer exists."""

    def __init__(self, mode):
        super().__init__('simulation_proximity_readonly_probe')
        self.mode = mode
        self.counts = Counter()
        self.states = Counter()
        self.reasons = Counter()
        self.joints = {}
        self.samples = deque(maxlen=200)
        self.latest_proximity = None
        self.latest_global = None
        self.latest_scene = None
        self.latest_obstacle = None
        self.readiness = None
        self.geometry = {}
        self.clearances = []
        self.numeric_errors = []
        self.scene_disagreements = []
        self.observation_subscriptions = []
        if mode == 'single':
            self.subscribe(JointState, '/joint_states', self.single_joints, sensor=True)
            self.subscribe(ProximityStatus, '/thesis/proximity_status', self.proximity)
            self.subscribe(TFMessage, '/world/jaco_world/pose/info', self.scene, sensor=True)
            self.subscribe(Obstacle, '/thesis/obstacle_in_model_frame', self.obstacle)
            self.subscribe(String, '/thesis/system_readiness', self.ready)
        else:
            for robot_id in ROBOT_IDS:
                self.subscribe(
                    JointState, f'/{robot_id}/joint_states',
                    lambda msg, identity=robot_id: self.hybrid_joints(identity, msg),
                    sensor=True,
                )
                self.subscribe(
                    GeometrySnapshot, f'/{robot_id}/geometry_snapshot',
                    lambda msg, identity=robot_id: self.snapshot(identity, msg),
                )
            self.subscribe(
                MultiArmSafetyStatus, '/thesis/multiarm_safety_status', self.global_status
            )
            self.subscribe(String, '/thesis/hybrid_system_readiness', self.ready)

    def subscribe(self, message_type, topic, callback, sensor=False):
        qos = qos_profile_sensor_data if sensor else 10
        self.observation_subscriptions.append(
            self.create_subscription(message_type, topic, callback, qos)
        )

    def ready(self, message):
        self.readiness = message.data
        self.counts['readiness'] += 1

    def single_joints(self, message):
        names = [f'j2n6s300_joint_{index}' for index in range(1, 7)]
        self.collect_joints('single', names, message)

    def hybrid_joints(self, robot_id, message):
        names = [f'{robot_id}_jaco_joint_{index}' for index in range(1, 7)]
        self.collect_joints(robot_id, names, message)

    def collect_joints(self, identity, names, message):
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(values[name]) for name in names):
            self.counts[f'{identity}_invalid_joint_sample'] += 1
            return
        sample = (seconds(message.header.stamp), tuple(values[name] for name in names))
        self.joints[identity] = sample
        self.counts[f'{identity}_joints'] += 1
        if identity == 'single':
            self.samples.append(sample)

    def snapshot(self, robot_id, message):
        self.counts[f'{robot_id}_geometry'] += 1
        self.geometry[robot_id] = {
            'profile': message.profile_id, 'valid': message.valid,
            'source': message.source_kind, 'state': message.state,
        }

    def global_status(self, message):
        self.counts['global_status'] += 1
        self.states[message.state] += 1
        self.reasons[message.reason_code] += 1
        self.latest_global = {
            'state': message.state, 'valid': message.valid,
            'reason': message.reason_code, 'clearance': message.minimum_clearance,
            'pair': [message.first_robot_id, message.second_robot_id],
        }
        if message.valid:
            self.clearances.append(message.minimum_clearance)

    def obstacle(self, message):
        self.counts['obstacle'] += 1
        self.latest_obstacle = {
            'center': point(message.center), 'radius': message.radius,
            'uncertainty': message.uncertainty,
        }

    def scene(self, message):
        for transform in message.transforms:
            child = str(transform.child_frame_id)
            if child.replace('::', '/').split('/', 1)[0] == 'safety_obstacle':
                self.latest_scene = point(transform.transform.translation)
                self.counts['scene_obstacle'] += 1
                break

    def proximity(self, message):
        self.counts['proximity'] += 1
        self.states[message.state] += 1
        self.reasons[message.limiting_segment] += 1
        center = point(message.obstacle_center)
        self.latest_proximity = {
            'state': message.state, 'segment': message.limiting_segment,
            'clearance': message.minimum_clearance, 'obstacle_center': center,
            'effective_obstacle_radius': message.obstacle_radius,
            'geometry_valid': message.geometry_valid,
            'acquisition_time_sec': seconds(message.stamp),
        }
        if not message.geometry_valid or message.limiting_segment not in SEGMENT_NAMES:
            return
        if self.latest_scene is not None:
            self.scene_disagreements.append(math.dist(center, self.latest_scene))
        self.clearances.append(message.minimum_clearance)
        if self.samples:
            stamp = seconds(message.stamp)
            nearest = min(self.samples, key=lambda sample: abs(sample[0] - stamp))
            if abs(nearest[0] - stamp) < 0.05:
                expected = minimum_sphere_clearance(nearest[1], center, message.obstacle_radius)[0]
                self.numeric_errors.append(abs(expected - message.minimum_clearance))

    def hybrid_fk_check(self):
        if set(self.joints) != set(ROBOT_IDS):
            return None
        path = (
            Path(__file__).resolve().parents[2]
            / 'src/thesis_simulation/config/hybrid_three_sim.yaml'
        )
        with path.open() as stream:
            scenario = build_hybrid_three_sim_scenario(yaml.safe_load(stream))
        snapshots = []
        stamps = [sample[0] for sample in self.joints.values()]
        for instance in scenario.instances:
            profile = legacy_jaco_geometry_profile(
                instance.robot_id, frame_prefix=instance.tf_prefix
            )
            positions = self.joints[instance.robot_id][1]
            axes = capsule_segments(
                positions, transform_matrix(instance.base_pose[:3], instance.base_pose[3:])
            )
            snapshots.append(RobotOccupancySnapshot(
                instance.robot_id, profile.profile_id, profile.geometry_version,
                profile.asset_hash, 'world', 10.0, 1.0,
                tuple(CapsuleOccupancy(name, *axis, radius)
                      for name, axis, radius in zip(SEGMENT_NAMES, axes, CAPSULE_RADII)),
            ))
        decision = evaluate_four_arm_safety(snapshots, 10.1, 0.05)
        return {
            'note': 'Latest joint samples, diagnostic FK only; not a safety authorization.',
            'sample_stamp_skew_sec': max(stamps) - min(stamps),
            'state': decision.state, 'clearance': decision.minimum_clearance,
            'pairs': {f'{pair.first_robot_id}/{pair.second_robot_id}': pair.minimum.clearance
                      for pair in decision.pair_clearances},
        }

    def result(self):
        writers = self.get_publisher_names_and_types_by_node(
            self.get_name(), self.get_namespace()
        )
        return {
            'mode': self.mode, 'counts': dict(self.counts), 'states': dict(self.states),
            'reasons': dict(self.reasons), 'readiness': self.readiness,
            'clearance_m': statistics_or_none(self.clearances),
            'fk_vs_reported_clearance_error_m': statistics_or_none(self.numeric_errors),
            'latest_scene_vs_reported_center_distance_m': statistics_or_none(
                self.scene_disagreements
            ),
            'latest_proximity': self.latest_proximity, 'latest_scene_center': self.latest_scene,
            'latest_obstacle': self.latest_obstacle, 'latest_global': self.latest_global,
            'geometry': self.geometry,
            'hybrid_fk_check': self.hybrid_fk_check() if self.mode == 'hybrid' else None,
            'own_publishers': writers,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('single', 'hybrid'), required=True)
    parser.add_argument('--duration', type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    rclpy.init(args=[])
    node = ProximityProbe(args.mode)
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        print(json.dumps(node.result(), ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
