"""Convert bounded Cartesian pose requests into supervised joint commands."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from thesis_core.cartesian_kinematics import (
    align_relative_target,
    pose_error,
    solve_inverse_kinematics_transform,
    transform_from_translation_quaternion,
)
from thesis_core.jaco_kinematics import (
    end_effector_transform,
    transform_matrix,
    translation,
)
from thesis_core.joint_model import (
    CONTINUOUS_JOINTS,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    shortest_joint_delta,
)
from thesis_core.robot_profile import load_robot_profile
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import (
    CartesianCommand,
    CommandDecision,
    JointCommand,
)
import tf2_ros

from thesis_simulation.profile_cartesian import (
    CartesianProfileModel,
    position_only_is_permitted,
)


class CartesianCommandAdapter(Node):
    """Provide a simulation-only Cartesian front end to the joint pipeline."""

    def __init__(self):
        super().__init__('cartesian_command_adapter')
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('robot_name', 'jaco2')
        self.declare_parameter('maximum_state_age_sec', 0.50)
        self.declare_parameter('maximum_position_step_m', 0.10)
        self.declare_parameter(
            'maximum_orientation_step_rad', math.radians(20.0)
        )
        self.declare_parameter('position_tolerance_m', 0.003)
        self.declare_parameter(
            'orientation_tolerance_rad', math.radians(1.0)
        )
        self.maximum_state_age_sec = float(
            self.get_parameter('maximum_state_age_sec').value
        )
        self.maximum_position_step_m = float(
            self.get_parameter('maximum_position_step_m').value
        )
        self.maximum_orientation_step_rad = float(
            self.get_parameter('maximum_orientation_step_rad').value
        )
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.orientation_tolerance_rad = float(
            self.get_parameter('orientation_tolerance_rad').value
        )
        parameters = (
            self.maximum_state_age_sec,
            self.maximum_position_step_m,
            self.maximum_orientation_step_rad,
            self.position_tolerance_m,
            self.orientation_tolerance_rad,
        )
        if not all(
            math.isfinite(value) and value > 0.0 for value in parameters
        ):
            raise ValueError('Cartesian adapter parameters must be positive')

        manifest_path = str(self.get_parameter('manifest_path').value)
        robot_name = str(self.get_parameter('robot_name').value)
        self.robot_profile = load_robot_profile(
            manifest_path or None, robot_name
        )
        self.joint_names = self.robot_profile.joint_names
        self.position_limits = self.robot_profile.joint_position_limits
        self.velocity_limits = self.robot_profile.joint_velocity_limits
        self.continuous_joints = self.robot_profile.continuous_joints
        self.world_frame = self.robot_profile.world_frame
        self.tip_link = self.robot_profile.tip_link
        self.cartesian_model = CartesianProfileModel(
            self.robot_profile, legacy_jaco2=not manifest_path
        )

        self.positions = {}
        self.state_received_at = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )
        self.command_publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10
        )
        self.decision_publisher = self.create_publisher(
            CommandDecision, '/thesis/command_decision', 10
        )
        self.create_subscription(
            JointState, '/joint_states', self.receive_state, 20
        )
        self.create_subscription(
            CartesianCommand,
            '/thesis/cartesian_candidate',
            self.receive_command,
            10,
        )
        self.get_logger().info(
            'Simulation Cartesian adapter ready: '
            f'maximum step={self.maximum_position_step_m:.3f} m, '
            f'{math.degrees(self.maximum_orientation_step_rad):.1f} deg, '
            f'IK tolerance={self.position_tolerance_m:.3f} m'
        )

    def receive_state(self, message):
        positions = {
            name: float(value)
            for name, value in zip(message.name, message.position)
            if name in self.joint_names and math.isfinite(value)
        }
        if all(name in positions for name in self.joint_names):
            self.positions = positions
            self.state_received_at = time.monotonic()

    def reject(self, message, reason_code, detail):
        decision = CommandDecision()
        decision.stamp = self.get_clock().now().to_msg()
        decision.command_id = message.command_id
        decision.accepted = False
        decision.state = 'REJECTED'
        decision.reason_code = reason_code
        decision.reason = detail
        self.decision_publisher.publish(decision)
        self.get_logger().warning(
            f'CARTESIAN REJECTED {message.command_id}: {detail}'
        )

    @staticmethod
    def duration_seconds(message):
        return (
            float(message.duration.sec)
            + float(message.duration.nanosec) * 1.0e-9
        )

    def current_joint_vector(self):
        if self.state_received_at is None:
            return None
        age = time.monotonic() - self.state_received_at
        if age > self.maximum_state_age_sec:
            return None
        return tuple(self.positions[name] for name in self.joint_names)

    def current_world_transform(self):
        """Read the same Gazebo TF reference used by the GUI."""
        message = self.tf_buffer.lookup_transform(
            self.world_frame, self.tip_link, Time()
        )
        transform = message.transform
        return transform_from_translation_quaternion(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def receive_command(self, message):
        world_frame = getattr(self, 'world_frame', 'world')
        if not message.command_id:
            self.reject(message, 'INVALID_CARTESIAN_COMMAND', 'ID vacío.')
            return
        if message.frame_id != world_frame:
            self.reject(
                message,
                'INVALID_CARTESIAN_FRAME',
                'El comando cartesiano debe usar '
                f'frame_id={world_frame}.',
            )
            return
        duration = self.duration_seconds(message)
        if not math.isfinite(duration) or not 0.1 <= duration <= 30.0:
            self.reject(
                message,
                'INVALID_CARTESIAN_DURATION',
                'La duración debe pertenecer a [0.1, 30.0] s.',
            )
            return
        target_pose = (
            message.x,
            message.y,
            message.z,
            message.roll,
            message.pitch,
            message.yaw,
        )
        if not all(math.isfinite(value) for value in target_pose):
            self.reject(
                message,
                'INVALID_CARTESIAN_POSE',
                'La pose contiene un valor no finito.',
            )
            return

        current = self.current_joint_vector()
        if current is None:
            self.reject(
                message,
                'CARTESIAN_STATE_UNAVAILABLE',
                'No existe un estado articular completo y reciente.',
            )
            return
        cartesian_model = getattr(self, 'cartesian_model', None)
        current_model_transform = (
            end_effector_transform(current)
            if cartesian_model is None
            else cartesian_model.forward(current)
        )
        target_world_transform = transform_matrix(
            target_pose[:3], target_pose[3:]
        )
        try:
            current_world_transform = self.current_world_transform()
        except (tf2_ros.TransformException, ValueError) as exception:
            self.reject(
                message,
                'CARTESIAN_TF_UNAVAILABLE',
                f'No se pudo leer {world_frame} -> '
                f'{getattr(self, "tip_link", "j2n6s300_end_effector")}: '
                f'{exception}',
            )
            return
        position_error, orientation_error = pose_error(
            target_world_transform, current_world_transform
        )
        position_step = math.sqrt(
            sum(value * value for value in position_error)
        )
        orientation_step = math.sqrt(
            sum(value * value for value in orientation_error)
        )
        if position_step > self.maximum_position_step_m + 1.0e-9:
            self.reject(
                message,
                'CARTESIAN_STEP_TOO_LARGE',
                f'Paso lineal {position_step:.3f} m; máximo '
                f'{self.maximum_position_step_m:.3f} m.',
            )
            return
        if orientation_step > self.maximum_orientation_step_rad + 1.0e-9:
            self.reject(
                message,
                'CARTESIAN_ROTATION_TOO_LARGE',
                f'Paso angular {math.degrees(orientation_step):.1f} grados; '
                f'máximo '
                f'{math.degrees(self.maximum_orientation_step_rad):.1f}.',
            )
            return

        try:
            model_target_transform = align_relative_target(
                current_world_transform,
                target_world_transform,
                current_model_transform,
            )
            if cartesian_model is None:
                result = solve_inverse_kinematics_transform(
                    current,
                    model_target_transform,
                    position_tolerance_m=self.position_tolerance_m,
                    orientation_tolerance_rad=self.orientation_tolerance_rad,
                )
            else:
                result = cartesian_model.inverse(
                    current,
                    model_target_transform,
                    self.position_tolerance_m,
                    self.orientation_tolerance_rad,
                    position_only=position_only_is_permitted(
                        self.robot_profile, orientation_step
                    ),
                )
        except ValueError as exception:
            self.reject(
                message,
                'INVERSE_KINEMATICS_INVALID_INPUT',
                f'No se pudo iniciar la IK: {exception}',
            )
            return
        if not result.success:
            self.reject(
                message,
                'INVERSE_KINEMATICS_FAILED',
                'Pose no alcanzable desde la configuración actual: '
                f'error lineal={result.position_error_m:.4f} m, '
                'error angular='
                f'{math.degrees(result.orientation_error_rad):.2f} grados.',
            )
            return

        joint_names = getattr(self, 'joint_names', JOINT_NAMES)
        position_limits = getattr(
            self, 'position_limits', JOINT_POSITION_LIMITS
        )
        velocity_limits = getattr(
            self, 'velocity_limits', JOINT_VELOCITY_LIMITS
        )
        continuous_joints = getattr(
            self, 'continuous_joints', CONTINUOUS_JOINTS
        )
        minimum_duration = max(
            abs(shortest_joint_delta(
                name,
                target,
                start,
                position_limits=position_limits,
                continuous_joints=continuous_joints,
            ))
            / velocity_limits[name]
            for name, start, target in zip(
                joint_names, current, result.positions
            )
        )
        if duration + 1.0e-9 < minimum_duration:
            self.reject(
                message,
                'CARTESIAN_DURATION_TOO_SHORT',
                f'Duración insuficiente; mínimo estimado '
                f'{minimum_duration:.2f} s.',
            )
            return

        output = JointCommand()
        output.stamp = message.stamp
        output.command_id = message.command_id
        output.joint_names = list(joint_names)
        output.positions = list(result.positions)
        output.duration = message.duration
        self.command_publisher.publish(output)
        current_world_position = translation(current_world_transform)
        reference_offset = math.sqrt(sum(
            (world - model) ** 2
            for world, model in zip(
                current_world_position,
                translation(current_model_transform),
            )
        ))
        self.get_logger().info(
            f'CARTESIAN CONVERTED {message.command_id}: '
            f'iterations={result.iterations}, '
            f'position_error={result.position_error_m:.4f} m, '
            'orientation_error='
            f'{math.degrees(result.orientation_error_rad):.2f} deg, '
            f'from_tf=({current_world_position[0]:.3f}, '
            f'{current_world_position[1]:.3f}, '
            f'{current_world_position[2]:.3f}), '
            f'reference_offset={reference_offset:.4f} m'
        )


def main(args=None):
    """Run the simulation-only Cartesian conversion node."""
    rclpy.init(args=args)
    spin_node(CartesianCommandAdapter())


if __name__ == '__main__':
    main()
