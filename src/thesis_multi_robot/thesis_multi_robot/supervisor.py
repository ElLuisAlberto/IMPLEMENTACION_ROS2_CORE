"""Experimental, simulation-only admission and monitoring for robot profiles.

This supervisor uses the same candidate/decision message contracts as the
thesis, with URDF kinematics and conservative collision envelopes.  It does
not claim equivalence to the validated JACO2 supervisor or certified safety.
It checks the external sphere and other robots; self-collision exclusions
have not been validated and self-collision checking is outside this scope.
"""

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


MAX_AGE = 0.5
FUTURE_TOLERANCE = 0.05
MAX_SPEED = 0.15
GEOMETRIC_MARGIN = 0.05


@dataclass(frozen=True)
class Evaluation:
    state: str
    clearance: float
    accepted: bool


def classify_clearance(clearance):
    """Reject missing geometry and proximity requiring active reduction."""
    if not math.isfinite(clearance) or clearance <= 0.0:
        return Evaluation('STOP', clearance, False)
    if clearance < 0.10:
        return Evaluation('REDUCTION', clearance, False)
    if clearance < 0.20:
        return Evaluation('WARNING', clearance, True)
    return Evaluation('ALLOW', clearance, True)


def evaluate_horizon(start, target, duration, clearance_at, samples=20):
    """Evaluate endpoints and equally spaced joint-interpolation samples.

    ``clearance_at(positions, offset_seconds)`` supplies conservative clearance
    after all obstacle uncertainty and protective margins are applied.
    """
    if not start or len(start) != len(target):
        raise ValueError('joint vectors must have equal nonzero lengths')
    if not all(math.isfinite(float(v)) for v in (*start, *target)):
        raise ValueError('joint positions must be finite')
    if not math.isfinite(duration) or not 0.1 <= duration <= 30.0:
        raise ValueError('duration must be within [0.1, 30] seconds')
    if not isinstance(samples, int) or samples < 2:
        raise ValueError('at least two horizon intervals are required')
    minimum = math.inf
    for index in range(samples + 1):
        ratio = index / samples
        position = tuple(a + (b - a) * ratio for a, b in zip(start, target))
        clearance = float(clearance_at(position, duration * ratio))
        if not math.isfinite(clearance):
            return Evaluation('STOP', clearance, False)
        minimum = min(minimum, clearance)
    return classify_clearance(minimum)


def stamp_is_fresh(now_ns, stamp_ns, maximum_age=MAX_AGE):
    """Reject zero, stale and significantly future source stamps."""
    age = (now_ns - stamp_ns) / 1.0e9
    return stamp_ns > 0 and -FUTURE_TOLERANCE <= age <= maximum_age


def duration_seconds(seconds, nanoseconds):
    """Validate ROS Duration fields before converting them to seconds."""
    if (not isinstance(seconds, int) or not isinstance(nanoseconds, int)
            or seconds < 0 or not 0 <= nanoseconds < 1_000_000_000):
        raise ValueError('malformed ROS duration')
    result = seconds + nanoseconds / 1e9
    if not 0.1 <= result <= 30.0:
        raise ValueError('duration must be within [0.1, 30] seconds')
    return result


def remaining_duration(now_ns, deadline_ns):
    """Keep runtime horizon in simulation time even with slow real-time factor."""
    return max(0.1, min(30.0, (deadline_ns - now_ns) / 1e9))


def horizon_sample_count(start, target, duration, own_reach, peer_motion=(),
                         obstacle_speed=0.0, maximum_intervals=400):
    """Bound relative travel per sample to 2 cm or reject excessive work.

    A point on a rotating serial chain travels no more than the sum of each
    joint's absolute angular travel multiplied by a conservative robot reach.
    Peer motion entries contain ``(reach_bound, sum_absolute_joint_speed)``.
    The sum of all peer travels is conservative for every individual pair.
    """
    if not start or len(start) != len(target):
        raise ValueError('joint vectors must have equal nonzero lengths')
    values = (*start, *target, duration, own_reach, obstacle_speed,
              *(value for peer in peer_motion for value in peer))
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('sampling bound requires finite values')
    if duration <= 0 or own_reach <= 0 or obstacle_speed < 0:
        raise ValueError('sampling duration/reach/speed must be positive')
    if any(reach <= 0 or speed < 0 for reach, speed in peer_motion):
        raise ValueError('invalid peer sampling bound')
    own_travel = own_reach * sum(abs(b - a) for a, b in zip(start, target))
    other_travel = duration * (
        obstacle_speed + sum(reach * speed for reach, speed in peer_motion))
    intervals = max(20, math.ceil((own_travel + other_travel) / 0.02))
    if intervals > maximum_intervals:
        raise ValueError('sampling budget exceeded; split or shorten the command')
    return intervals


def reach_bound(root, capsules):
    """Conservatively bound every robot collision point's rotational lever."""
    length = 0.0
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        if parent is not None and parent.get('link') == 'world':
            continue  # Cell placement must not increase robot arm length.
        origin = joint.find('origin')
        xyz = (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0')
        length += math.sqrt(sum(float(value) ** 2 for value in xyz.split()))
    local_extent = max(
        math.sqrt(sum(float(value) ** 2 for value in point)) + float(capsule['radius'])
        for capsule in capsules for point in (capsule['start'], capsule['end']))
    result = length + local_extent
    if not math.isfinite(result) or result <= 0:
        raise ValueError('robot reach bound must be positive and finite')
    return result


def validate_target(start, target, limits, duration):
    """Validate limits and conservative angular speed before action dispatch."""
    if len(start) != len(target) or len(target) != len(limits):
        raise ValueError('joint count mismatch')
    if not math.isfinite(duration) or not 0.1 <= duration <= 30.0:
        raise ValueError('duration must be within [0.1, 30] seconds')
    for current, desired, (lower, upper, velocity) in zip(start, target, limits):
        if not math.isfinite(desired) or not lower <= desired <= upper:
            raise ValueError('target outside finite URDF limits')
        if not math.isfinite(current) or not math.isfinite(velocity) or velocity <= 0:
            raise ValueError('invalid starting state or URDF velocity limit')
        if abs(desired - current) / duration > min(MAX_SPEED, velocity) + 1e-9:
            raise ValueError('target exceeds conservative joint velocity')


def main(args=None):
    """Run the profile supervisor with explicitly namespaced simulation I/O."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from thesis_interfaces.msg import (
        CommandDecision, JointCommand, Obstacle, ProximityStatus,
    )
    from trajectory_msgs.msg import JointTrajectoryPoint
    from .kinematics import RobotKinematics
    from thesis_core.work_table import SIM_TABLE_SIZE as TABLE_SIZE, fixed_mount_links, WorkTable
    from .geometry import capsule_sets_clearance, sphere_clearance, world_capsules

    def nanoseconds(stamp):
        if not 0 <= int(stamp.nanosec) < 1_000_000_000:
            return -1
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    class ProfileSupervisor(Node):
        def __init__(self):
            super().__init__('profile_supervisor')
            self.declare_parameter('manifest_path', '')
            self.declare_parameter('robot_name', '')
            manifest = json.loads(Path(self.get_parameter('manifest_path').value).read_text())
            self.name = self.get_parameter('robot_name').value
            self.profiles = manifest['robots']
            if self.name not in self.profiles:
                raise ValueError('robot_name is absent from runtime manifest')
            self.profile = self.profiles[self.name]
            self.models = {}
            self.reaches = {}
            for name, profile in self.profiles.items():
                root = ET.parse(profile['urdf']).getroot()
                self.models[name] = RobotKinematics(root, profile['joint_names'])
                self.reaches[name] = reach_bound(root, profile['capsules'])
            root = ET.parse(self.profile['urdf']).getroot()
            joints = {joint.get('name'): joint for joint in root.findall('joint')}
            self.fixed_links = fixed_mount_links(root)
            self.table_centers = manifest.get('table_centers', {})
            self.limits = []
            for name in self.profile['joint_names']:
                joint = joints[name]
                limit = joint.find('limit')
                if limit is None:
                    raise ValueError(f'missing joint limit: {name}')
                self.limits.append((
                    float(limit.get('lower', '-6.283185307179586')),
                    float(limit.get('upper', '6.283185307179586')),
                    float(limit.get('velocity')),
                ))
            self.states = {}
            self.obstacle = None
            self.goal = None
            self.pending = False
            self.cancel_requested = False
            self.cancel_sent = False
            self.cancel_attempts = 0
            self.cancel_inflight = False
            self.active_target = None
            self.active_id = ''
            self.active_until_ns = 0
            self.seen_ids = deque(maxlen=256)
            self.decisions = self.create_publisher(
                CommandDecision, f'/{self.name}/thesis/command_decision', 10)
            self.proximity = self.create_publisher(
                ProximityStatus, f'/{self.name}/thesis/proximity_status', 10)
            self.client = ActionClient(
                self, FollowJointTrajectory,
                f'/{self.name}/arm_controller/follow_joint_trajectory')
            self.subscriptions_keepalive = []
            for name in self.profiles:
                self.subscriptions_keepalive.append(self.create_subscription(
                    JointState, f'/{name}/joint_states',
                    lambda message, name=name: self.state_callback(name, message),
                    qos_profile_sensor_data))
            self.subscriptions_keepalive.extend([
                self.create_subscription(Obstacle, f'/{self.name}/thesis/obstacle',
                                         self.obstacle_callback, 10),
                self.create_subscription(JointCommand, f'/{self.name}/thesis/candidate_command',
                                         self.command_callback, 10),
            ])
            self.timer = self.create_timer(0.1, self.monitor)

        def state_callback(self, name, message):
            try:
                if len(set(message.name)) != len(message.name):
                    raise ValueError('duplicate joint names')
                values = dict(zip(message.name, message.position))
                positions = tuple(float(values[j]) for j in self.profiles[name]['joint_names'])
                if not all(math.isfinite(v) for v in positions):
                    raise ValueError('nonfinite joint state')
                velocities_map = dict(zip(message.name, message.velocity))
                velocities = tuple(float(velocities_map.get(j, 0.0))
                                   for j in self.profiles[name]['joint_names'])
                if not all(math.isfinite(v) and abs(v) <= MAX_SPEED + 0.05
                           for v in velocities):
                    raise ValueError('invalid or unexpectedly fast joint state')
                now = self.get_clock().now().nanoseconds
                if not stamp_is_fresh(now, nanoseconds(message.header.stamp)):
                    raise ValueError('stale joint source timestamp')
                self.states[name] = (positions, velocities, time.monotonic(),
                                     nanoseconds(message.header.stamp))
            except (KeyError, ValueError, TypeError):
                self.states.pop(name, None)

        def obstacle_callback(self, message):
            values = (message.center.x, message.center.y, message.center.z,
                      message.velocity.x, message.velocity.y, message.velocity.z,
                      message.radius, message.uncertainty)
            now = self.get_clock().now().nanoseconds
            if (message.header.frame_id != 'world'
                    or not all(math.isfinite(v) for v in values)
                    or message.radius <= 0 or message.uncertainty < 0
                    or not stamp_is_fresh(now, nanoseconds(message.header.stamp))):
                self.obstacle = None
                return
            self.obstacle = (message, time.monotonic(), nanoseconds(message.header.stamp))

        def ready(self):
            now = time.monotonic()
            clock = self.get_clock().now().nanoseconds
            if self.obstacle is None:
                return False
            if (now - self.obstacle[1] > MAX_AGE
                    or not stamp_is_fresh(clock, self.obstacle[2])):
                return False
            return all(name in self.states
                       and now - self.states[name][2] <= MAX_AGE
                       and stamp_is_fresh(clock, self.states[name][3])
                       for name in self.profiles)

        def evaluate(self, target, duration, current_only=False):
            if not self.ready():
                return Evaluation('STOP', math.nan, False)
            start = self.states[self.name][0]
            obstacle, _, obstacle_stamp = self.obstacle
            obstacle_age = max(0.0, (self.get_clock().now().nanoseconds - obstacle_stamp) / 1e9)

            def clearance_at(positions, offset):
                own = world_capsules(self.profile['capsules'],
                                     self.models[self.name].forward(positions))
                center = tuple(c + v * (offset + obstacle_age) for c, v in zip(
                    (obstacle.center.x, obstacle.center.y, obstacle.center.z),
                    (obstacle.velocity.x, obstacle.velocity.y, obstacle.velocity.z)))
                clearance = min(
                    sphere_clearance(
                        capsule,
                        center,
                        obstacle.radius + obstacle.uncertainty,
                    )
                    for capsule in own
                )
                for name, table in self.table_centers.items():
                    for c in own:
                        if name == self.name and c['link'] in self.fixed_links:
                            continue
                        a, b, r = c['start'], c['end'], c['radius']
                        if min(a[2], b[2]) - r - table[2] - TABLE_SIZE[2]/2 >= clearance:
                            continue
                        clearance = min(clearance, WorkTable(
                            tuple(table), TABLE_SIZE, frozenset(), with_legs=True
                        ).closest([(a, b)], ['segment'], [r]).clearance)
                for peer_name, peer in self.profiles.items():
                    if peer_name == self.name:
                        continue
                    state, velocity, _, stamp = self.states[peer_name]
                    age = max(0.0, (self.get_clock().now().nanoseconds - stamp) / 1e9)
                    predicted = tuple(q + v * (offset + age) for q, v in zip(state, velocity))
                    others = world_capsules(peer['capsules'],
                                            self.models[peer_name].forward(predicted))
                    clearance = min(clearance, capsule_sets_clearance(own, others))
                return clearance - GEOMETRIC_MARGIN

            if current_only:
                result = classify_clearance(float(clearance_at(start, 0.0)))
            else:
                peers = [(self.reaches[name], sum(abs(v) for v in self.states[name][1]))
                         for name in self.profiles if name != self.name]
                obstacle_speed = math.sqrt(sum(value ** 2 for value in (
                    obstacle.velocity.x, obstacle.velocity.y, obstacle.velocity.z)))
                samples = horizon_sample_count(
                    start, target, duration,
                    self.reaches[self.name], peers, obstacle_speed,
                )
                result = evaluate_horizon(start, target, duration, clearance_at, samples)
            # Geometric work may take longer than the freshness budget.  It
            # cannot authorize a goal from inputs that expired during work.
            return result if self.ready() else Evaluation('STOP', math.nan, False)

        def publish_decision(self, command_id, evaluation, reason):
            message = CommandDecision()
            message.stamp = self.get_clock().now().to_msg()
            message.command_id = command_id
            message.accepted = evaluation.accepted
            message.state = evaluation.state
            message.reason_code = reason
            message.reason = reason
            self.decisions.publish(message)

        def command_callback(self, message):
            rejected = Evaluation('STOP', math.nan, False)
            reason = 'INVALID_COMMAND'
            try:
                if not message.command_id.strip() or message.command_id in self.seen_ids:
                    raise ValueError('empty or replayed command identifier')
                self.seen_ids.append(message.command_id)
                if not stamp_is_fresh(
                    self.get_clock().now().nanoseconds,
                    nanoseconds(message.stamp),
                ):
                    raise ValueError('stale/future command timestamp')
                if list(message.joint_names) != self.profile['joint_names']:
                    raise ValueError('joint ordering mismatch')
                if self.pending or self.goal is not None:
                    reason = 'BUSY'
                    raise ValueError('another goal is still pending or active')
                if not self.ready():
                    reason = 'STALE_INPUT'
                    raise ValueError('fresh state and obstacle are required')
                duration = duration_seconds(message.duration.sec, message.duration.nanosec)
                target = tuple(message.positions)
                validate_target(self.states[self.name][0], target, self.limits, duration)
                evaluation = self.evaluate(target, duration)
                if not evaluation.accepted:
                    self.publish_decision(message.command_id, evaluation, 'GEOMETRIC_REJECTION')
                    return
                if not self.client.server_is_ready():
                    reason = 'CONTROLLER_UNAVAILABLE'
                    raise ValueError('trajectory controller is unavailable')
                self.pending = True
                self.cancel_requested = False
                self.cancel_sent = False
                self.cancel_attempts = 0
                self.cancel_inflight = False
                self.active_target = target
                self.active_id = message.command_id
                self.active_until_ns = self.get_clock().now().nanoseconds + int(duration * 1e9)
                self.send_goal(target, duration, message.command_id, evaluation)
            except (ValueError, TypeError, KeyError) as error:
                self.publish_decision(message.command_id, rejected, f'{reason}: {error}')

        def send_goal(self, target, duration, command_id, evaluation):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.profile['joint_names']
            point = JointTrajectoryPoint()
            point.positions = list(target)
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
            goal.trajectory.points = [point]
            try:
                future = self.client.send_goal_async(goal)
            except Exception as error:
                self.pending = False
                self.active_target = None
                self.publish_decision(command_id, Evaluation('STOP', math.nan, False),
                                      f'ACTION_DISPATCH_ERROR: {error}')
                return

            def accepted(future):
                self.pending = False
                try:
                    handle = future.result()
                except Exception as error:
                    self.active_target = None
                    self.publish_decision(command_id, Evaluation('STOP', math.nan, False),
                                          f'ACTION_ERROR: {error}')
                    return
                if not handle.accepted:
                    self.active_target = None
                    self.publish_decision(command_id, Evaluation('STOP', math.nan, False),
                                          'CONTROLLER_REJECTED')
                    return
                self.goal = handle
                handle.get_result_async().add_done_callback(self.goal_finished)
                try:
                    current_evaluation = self.evaluate(target, duration)
                except (KeyError, ValueError, TypeError, ZeroDivisionError):
                    current_evaluation = Evaluation('STOP', math.nan, False)
                if self.cancel_requested or not current_evaluation.accepted:
                    self.stop_goal()
                    self.publish_decision(command_id, Evaluation('STOP', math.nan, False),
                                          'INPUT_CHANGED_DURING_DISPATCH')
                else:
                    self.active_until_ns = self.get_clock().now().nanoseconds + int(duration * 1e9)
                    self.publish_decision(command_id, evaluation, 'SIMULATION_GOAL_ACCEPTED')

            future.add_done_callback(accepted)

        def goal_finished(self, future):
            command_id = self.active_id
            succeeded = False
            try:
                result = future.result()
                if result.result.error_code == 0 and not self.cancel_requested:
                    succeeded = True
                elif result.result.error_code != 0 and not self.cancel_requested:
                    self.publish_decision(self.active_id, Evaluation('STOP', math.nan, False),
                                          f'CONTROLLER_ERROR: {result.result.error_string}')
            except Exception as error:
                self.publish_decision(self.active_id, Evaluation('STOP', math.nan, False),
                                      f'CONTROLLER_RESULT_ERROR: {error}')
            self.goal = None
            self.active_target = None
            self.cancel_requested = False
            self.cancel_sent = False
            self.cancel_attempts = 0
            self.cancel_inflight = False
            if succeeded:
                self.publish_decision(
                    command_id,
                    Evaluation('ALLOW', math.nan, True),
                    'SIMULATION_GOAL_SUCCEEDED',
                )

        def stop_goal(self):
            self.cancel_requested = True
            if (self.goal is None or self.cancel_sent or self.cancel_inflight
                    or self.cancel_attempts >= 3):
                return
            handle = self.goal
            self.cancel_attempts += 1
            self.cancel_inflight = True

            def failed(reason):
                self.cancel_inflight = False
                self.publish_decision(self.active_id, Evaluation('STOP', math.nan, False),
                                      f'CANCEL_FAILED: {reason}; attempt {self.cancel_attempts}/3')

            def canceled(future):
                if self.goal is not handle:
                    return  # The result callback already made this goal terminal.
                try:
                    response = future.result()
                    if not response.goals_canceling:
                        failed('controller did not confirm goal cancellation')
                        return
                except Exception as error:
                    failed(str(error))
                    return
                # JTC enters its built-in hold only after accepted cancellation.
                self.cancel_sent = True
                self.cancel_inflight = False

            try:
                handle.cancel_goal_async().add_done_callback(canceled)
            except Exception as error:
                failed(str(error))

        def monitor(self):
            if not self.ready():
                evaluation = Evaluation('STOP', math.nan, False)
            else:
                target = (self.active_target if self.active_target is not None
                          else self.states[self.name][0])
                duration = (remaining_duration(self.get_clock().now().nanoseconds,
                                               self.active_until_ns)
                            if self.active_target is not None else 0.1)
                try:
                    evaluation = self.evaluate(target, duration,
                                               current_only=self.active_target is None)
                except (KeyError, ValueError, TypeError, ZeroDivisionError):
                    evaluation = Evaluation('STOP', math.nan, False)
            status = ProximityStatus()
            status.stamp = self.get_clock().now().to_msg()
            status.reference_frame = 'world'
            status.state = evaluation.state
            status.minimum_clearance = evaluation.clearance
            status.limiting_segment = 'profile_envelopes'  # No witness point is published.
            self.proximity.publish(status)
            if not evaluation.accepted and (self.pending or self.goal is not None):
                if not self.cancel_requested:
                    self.publish_decision(
                        self.active_id,
                        Evaluation('STOP', evaluation.clearance, False),
                        'RUNTIME_STOP',
                    )
                self.stop_goal()

    rclpy.init(args=args)
    node = ProfileSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
