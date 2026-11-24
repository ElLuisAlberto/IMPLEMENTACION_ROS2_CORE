"""Report when the preventive simulation stack is ready to accept commands."""

import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from thesis_core.robot_profile import load_robot_profile
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import Obstacle, ProximityStatus
from tf2_msgs.msg import TFMessage

from thesis_core.jaco_kinematics import SEGMENT_NAMES
from thesis_simulation.readiness_contract import sample_is_fresh


def default_readiness_endpoints(profile):
    """Return profile-scoped readiness endpoints without ROS side effects."""
    namespace = profile.controller_paths.namespace
    prefix = '' if namespace == '/' else str(namespace).rstrip('/')
    manager = profile.controller_paths.controller_manager.rstrip('/')
    return {
        'prefix': prefix,
        'joint_states': profile.joint_states_topic,
        'proximity': f'{prefix}/thesis/proximity_status',
        'obstacle_input': f'{prefix}/thesis/obstacle_in_model_frame',
        'ready': f'{prefix}/thesis/system_ready',
        'status': f'{prefix}/thesis/system_readiness',
        'controller_service': f'{manager}/list_controllers',
        'controller_action': profile.controller_action,
        'obstacle_pose': f'/world/{profile.world_name}/pose/info',
    }


class SystemReadinessNode(Node):
    """Check graph connections, fresh data and active Gazebo control."""

    def __init__(self):
        super().__init__('system_readiness')
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('robot_name', 'jaco2')
        self.declare_parameter('obstacle_source_mode', 'topic')
        self.declare_parameter('use_demo_obstacle', True)
        self.declare_parameter('require_gui', True)
        self.declare_parameter('static_preview', False)
        self.declare_parameter('readiness_timeout_sec', 0.75)
        self.declare_parameter('readiness_rate_hz', 1.0)
        self.declare_parameter('controller_request_timeout_sec', 2.0)
        self.declare_parameter(
            'controller_manager_service',
            '',
        )
        self.declare_parameter(
            'controller_action',
            '',
        )
        self.declare_parameter('controller_name', 'arm_controller')
        self.declare_parameter('joint_state_topic', '')
        self.declare_parameter('proximity_topic', '')
        self.declare_parameter('obstacle_input_topic', '')
        self.declare_parameter('obstacle_pose_topic', '')
        self.declare_parameter('ready_topic', '')
        self.declare_parameter('status_topic', '')

        manifest_path = str(self.get_parameter('manifest_path').value)
        robot_name = str(self.get_parameter('robot_name').value)
        self.profile_mode = bool(manifest_path)
        self.robot_profile = load_robot_profile(
            manifest_path or None, robot_name
        )
        defaults = default_readiness_endpoints(self.robot_profile)
        self.topic_prefix = defaults['prefix']

        def configured_topic(parameter, default):
            value = str(self.get_parameter(parameter).value)
            return value or default

        self.joint_state_topic = configured_topic(
            'joint_state_topic', defaults['joint_states']
        )
        self.proximity_topic = configured_topic(
            'proximity_topic', defaults['proximity']
        )
        self.obstacle_input_topic = configured_topic(
            'obstacle_input_topic', defaults['obstacle_input']
        )
        self.ready_topic = configured_topic(
            'ready_topic', defaults['ready']
        )
        self.status_topic = configured_topic(
            'status_topic', defaults['status']
        )
        self.obstacle_pose_topic = str(
            self.get_parameter('obstacle_pose_topic').value
        ) or defaults['obstacle_pose']

        self.obstacle_source_mode = str(
            self.get_parameter('obstacle_source_mode').value
        ).lower()
        self.use_demo_obstacle = bool(
            self.get_parameter('use_demo_obstacle').value
        )
        self.require_gui = bool(self.get_parameter('require_gui').value)
        self.static_preview = bool(
            self.get_parameter('static_preview').value
        )
        self.timeout_sec = float(
            self.get_parameter('readiness_timeout_sec').value
        )
        rate_hz = float(
            self.get_parameter('readiness_rate_hz').value
        )
        self.controller_request_timeout_sec = float(
            self.get_parameter('controller_request_timeout_sec').value
        )
        if self.obstacle_source_mode not in ('topic', 'gazebo'):
            raise ValueError(
                'obstacle_source_mode must be topic or gazebo'
            )
        if (not math.isfinite(self.timeout_sec)
                or not math.isfinite(rate_hz)
                or not math.isfinite(self.controller_request_timeout_sec)
                or self.timeout_sec <= 0.0
                or rate_hz <= 0.0
                or self.controller_request_timeout_sec <= 0.0):
            raise ValueError('readiness timeouts and rate must be positive')

        self.last_received = {
            '/clock': None,
            '/joint_states': None,
            '/thesis/proximity_status': None,
        }
        self.last_received['/thesis/obstacle_source'] = None
        self.proximity_issue = 'esperando geometría de proximidad válida'
        self.last_status = None
        self.controller_active = False
        self.controller_error = 'esperando controller_manager'
        self.controller_future = None
        self.controller_future_started_at = None

        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Clock, '/clock', self.clock_callback, clock_qos
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            ProximityStatus,
            self.proximity_topic,
            self.proximity_callback,
            10,
        )
        if self.obstacle_source_mode == 'topic':
            self.create_subscription(
                Obstacle,
                self.obstacle_input_topic,
                self.obstacle_callback,
                10,
            )
        else:
            self.create_subscription(
                TFMessage,
                self.obstacle_pose_topic,
                self.obstacle_pose_callback,
                10,
            )

        self.ready_publisher = self.create_publisher(
            Bool, self.ready_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10
        )
        service_name = str(
            self.get_parameter('controller_manager_service').value
        )
        if not service_name:
            service_name = defaults['controller_service']
        action_name = str(
            self.get_parameter('controller_action').value
        )
        if not action_name:
            action_name = defaults['controller_action']
        self.controller_action_name = action_name
        self.controller_name = str(
            self.get_parameter('controller_name').value
        )
        if not self.controller_name:
            raise ValueError('controller_name must be nonempty')
        self.controller_client = self.create_client(
            ListControllers, service_name
        )
        self.action_client = ActionClient(
            self, FollowJointTrajectory, action_name
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.evaluate)
        self.get_logger().info(
            'System readiness monitor started; waiting for the full stack'
        )

    def _mark_received(self, key):
        self.last_received[key] = time.monotonic()

    def clock_callback(self, _msg):
        """Record receipt of a live simulation clock sample."""
        self._mark_received('/clock')

    def joint_state_callback(self, _msg):
        """Record receipt of a live robot state sample."""
        if all(
            name in _msg.name for name in self.robot_profile.joint_names
        ):
            self._mark_received('/joint_states')

    def proximity_callback(self, message):
        """Receipt alone does not establish that proximity was measurable."""
        self._mark_received('/thesis/proximity_status')
        valid = (
            bool(message.geometry_valid)
            and message.reference_frame == getattr(getattr(self, 'robot_profile', None), 'world_frame', 'world')
            and (bool(message.limiting_segment) if getattr(self, 'profile_mode', False) else message.limiting_segment in SEGMENT_NAMES)
            and math.isfinite(message.minimum_clearance)
            and math.isfinite(message.obstacle_radius)
            and message.obstacle_radius > 0.0
        )
        self.proximity_issue = (
            '' if valid else
            'geometría de proximidad no disponible: ' + message.limiting_segment
        )

    def obstacle_callback(self, _msg):
        """Record receipt of a transformed perception input."""
        self._mark_received('/thesis/obstacle_source')

    def obstacle_pose_callback(self, _msg):
        """Record receipt of the Gazebo obstacle pose bridge."""
        self._mark_received('/thesis/obstacle_source')

    def _update_controller_state(self):
        now = time.monotonic()
        if self.controller_future is not None:
            if not self.controller_future.done():
                elapsed = now - self.controller_future_started_at
                if elapsed < self.controller_request_timeout_sec:
                    return
                self.controller_future.cancel()
                self.controller_future = None
                self.controller_future_started_at = None
                self.controller_active = False
                self.controller_error = (
                    'timeout list_controllers; reintentando'
                )
            else:
                try:
                    response = self.controller_future.result()
                    controller_name = getattr(
                        self, 'controller_name', 'arm_controller'
                    )
                    self.controller_active = any(
                        item.name == controller_name
                        and item.state.lower() == 'active'
                        for item in response.controller
                    )
                    self.controller_error = (
                        '' if self.controller_active
                        else f'{controller_name} no está active'
                    )
                # ROS service failures are runtime input, not programming
                # errors, so the monitor reports them and retries.
                except Exception as exc:
                    self.controller_active = False
                    self.controller_error = f'falló list_controllers: {exc}'
                self.controller_future = None
                self.controller_future_started_at = None

        if (self.controller_future is None
                and self.controller_client.service_is_ready()):
            self.controller_future = self.controller_client.call_async(
                ListControllers.Request()
            )
            self.controller_future_started_at = now

    def _node_graph_status(self):
        common = {
            'robot_state_publisher',
            'safety_supervisor_node', 'simulation_command_adapter',
            'cartesian_command_adapter', 'cartesian_jog_adapter',
            'cartesian_trajectory_adapter', 'horizon_preview',
            'system_readiness',
        }
        if not self.static_preview:
            common.add('joint_state_broadcaster')
        if getattr(self, 'profile_mode', False):
            expected = common | {
                'cell_clock', 'profile_proximity_monitor',
            }
            if self.use_demo_obstacle:
                expected.add('cell_demo_obstacle')
        else:
            expected = common | {
                'gazebo_clock_bridge', 'proximity_monitor',
            }
            if self.obstacle_source_mode == 'topic':
                expected.add('obstacle_input_bridge')
                if self.use_demo_obstacle:
                    expected.add('obstacle_demo_source')
        if self.require_gui:
            expected.add('joint_control_gui_node')

        counts = {}
        for name, _namespace in self.get_node_names_and_namespaces():
            counts[name] = counts.get(name, 0) + 1

        missing = [
            f'nodo ausente: /{name}'
            for name in sorted(expected)
            if counts.get(name, 0) == 0
        ]
        duplicates = [
            f'nodo duplicado: /{name} ({counts[name]} instancias)'
            for name in sorted(expected)
            if counts.get(name, 0) > 1
        ]
        return missing + duplicates

    def _freshness_status(self, now):
        required = [
            ('/clock', 'reloj de simulación'),
            ('/joint_states', 'estado articular'),
            ('/thesis/proximity_status', 'cálculo de proximidad'),
            ('/thesis/obstacle_source', 'fuente de obstáculo'),
        ]
        missing = []
        for key, label in required:
            if not sample_is_fresh(
                    self.last_received[key], now, self.timeout_sec):
                missing.append(f'sin dato reciente: {label}')
        if getattr(self, 'proximity_issue', ''):
            missing.append(self.proximity_issue)
        return missing

    def _connection_status(self):
        prefix = self.topic_prefix
        expected_subscribers = {
            f'{prefix}/thesis/candidate_command': 'supervisor',
            f'{prefix}/thesis/jog_intent': 'supervisor',
            f'{prefix}/thesis/proximity_status': 'supervisor',
            f'{prefix}/thesis/supervised_command': 'adaptador',
            f'{prefix}/thesis/supervised_jog_command': 'adaptador',
            f'{prefix}/thesis/cartesian_candidate': 'adaptador cartesiano',
            f'{prefix}/thesis/cartesian_jog_intent': (
                'adaptador JOG cartesiano'
            ),
            f'{prefix}/thesis/cartesian_trajectory_candidate': (
                'coordinador cartesiano'
            ),
        }
        missing = []
        for topic, consumer in expected_subscribers.items():
            if self.count_subscribers(topic) == 0:
                missing.append(
                    f'{consumer} no está conectado a {topic}'
                )
        return missing

    def evaluate(self):
        """Publish READY or a concise list of unavailable components."""
        if not self.static_preview:
            self._update_controller_state()
        missing = self._node_graph_status()
        missing.extend(self._freshness_status(time.monotonic()))
        missing.extend(self._connection_status())
        if not self.static_preview:
            if not self.controller_active:
                missing.append(self.controller_error)
            if not self.action_client.server_is_ready():
                action_name = getattr(
                    self,
                    'controller_action_name',
                    '/arm_controller/follow_joint_trajectory',
                )
                missing.append(
                    f'acción {action_name} no disponible'
                )

        ready = not missing
        status = 'READY' if ready else 'WAITING: ' + '; '.join(missing)
        ready_message = Bool()
        ready_message.data = ready
        status_message = String()
        status_message.data = status
        self.ready_publisher.publish(ready_message)
        self.status_publisher.publish(status_message)

        if status != self.last_status:
            if ready:
                self.get_logger().info(
                    'SYSTEM READY: all required links are live'
                )
            else:
                self.get_logger().warning(status)
            self.last_status = status


def main(args=None):
    """Run the readiness monitor using a wall-clock timer."""
    rclpy.init(args=args)
    spin_node(SystemReadinessNode())


if __name__ == '__main__':
    main()
