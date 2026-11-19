"""A spherical obstacle source for reproducible simulation checks."""
from tf2_msgs.msg import TFMessage
import rclpy
from rclpy.node import Node
from thesis_interfaces.msg import Obstacle


class DemoObstacle(Node):
    def __init__(self):
        super().__init__('cell_demo_obstacle')
        self.declare_parameter('robot_names', ['jaco2', 'ur17g', 'p3020', 'cs625'])
        for name, value in [('x', 0.), ('y', -8.), ('z', 1.), ('radius', .12)]:
            self.declare_parameter(name, value)
        self.publishers_ = [self.create_publisher(Obstacle, f'/{name}/thesis/obstacle', 10)
                            for name in self.get_parameter('robot_names').value]
        self.previous_pose = None
        if self.declare_parameter('track_gazebo', False).value:
            self.create_subscription(TFMessage, '/model/safety_obstacle/pose', self.pose, 10)
        else:
            self.create_timer(.05, self.publish_obstacle)

    def pose(self, message):
        for tf in message.transforms:
            if tf.child_frame_id == 'safety_obstacle':
                p = tf.transform.translation
                stamp = tf.header.stamp
                now = stamp.sec + stamp.nanosec * 1e-9
                xyz = (p.x, p.y, p.z)
                velocity = (0., 0., 0.)
                if self.previous_pose:
                    previous, point = self.previous_pose
                    if now > previous:
                        velocity = tuple((a-b)/(now-previous) for a, b in zip(xyz, point))
                self.previous_pose = now, xyz
                self.publish_obstacle(xyz, stamp, velocity)
                return

    def publish_obstacle(self, xyz=None, stamp=None, velocity=(0., 0., 0.)):
        msg = Obstacle()
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        # Contract is the same Obstacle message as the existing thesis.
        msg.header.frame_id = 'world'
        msg.center.x, msg.center.y, msg.center.z = xyz or tuple(
            float(self.get_parameter(a).value) for a in 'xyz')
        msg.velocity.x, msg.velocity.y, msg.velocity.z = velocity
        msg.radius = float(self.get_parameter('radius').value)
        msg.uncertainty = .02
        for publisher in self.publishers_:
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
