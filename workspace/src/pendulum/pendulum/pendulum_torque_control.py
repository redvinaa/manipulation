# other
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS modules
import rclpy
from rclpy.node import Node

# ROS message types
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


MOTOR_LENGTH = 0.2
MOTOR_DIAMETER = 0.2
ROD_DIAMETER = 0.05
WEIGHT_DIAMETER = 0.3


class Pendulum(Node):
    """!
    @brief Pendulum and controller model

    MIT Manipulation course
    02 - Let's get you a robot
    01_reflected_inertia
    Simple pendulum with motor and gearbox

    Subscribes to joint state messages for the position control of the pendulum.
    Publishes a marker array for visualization in RViz.
    Publishes wrenches for the motor and gearbox.
    """

    def __init__(self):
        """!@brief Get parameters, initialize pub sub and marker."""

        super().__init__('pendulum')
        self.get_logger().info('Pendulum node started.')

        # Parameters
        self.declare_parameter('mass', 1.0)
        self.declare_parameter('length', 0.5)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('gearbox_ratio', 160.0)
        self.declare_parameter('rotor_inertia', 3.46e-4)
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('frame_id', 'world')

        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value
        self.gearbox_ratio = self.get_parameter('gearbox_ratio').get_parameter_value().double_value
        self.rotor_inertia = self.get_parameter('rotor_inertia').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publishers
        self.wrench_motor_pub = self.create_publisher(WrenchStamped, 'wrench_motor', 10)
        self.wrench_gearbox_pub = self.create_publisher(WrenchStamped, 'wrench_gearbox', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'pendulum_marker', 10)

        # Subscriptions
        self.joint_msg: JointState | None = None
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Timer for publishing at a fixed frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        # System state [q, q_dot]
        self.q = np.zeros(2)
        self.now = self.get_clock().now()

    def publish_markers(self):
        """!@brief Create and publish markers for the pendulum based on q."""
        quat = R.from_euler('xy', [np.pi/2, -self.gearbox_ratio * self.q[0]]).as_quat(canonical=False)
        motor_marker = Marker()
        motor_marker.header.frame_id = self.frame_id
        motor_marker.ns = 'motor'
        motor_marker.type = Marker.CYLINDER
        motor_marker.action = Marker.ADD
        motor_marker.pose.position.y = MOTOR_LENGTH * 1.5
        motor_marker.pose.position.z = self.length
        motor_marker.pose.orientation.x = quat[0]
        motor_marker.pose.orientation.y = quat[1]
        motor_marker.pose.orientation.z = quat[2]
        motor_marker.pose.orientation.w = quat[3]
        motor_marker.scale.x = MOTOR_DIAMETER
        motor_marker.scale.y = MOTOR_DIAMETER
        motor_marker.scale.z = MOTOR_LENGTH
        motor_marker.color.r = 1.0
        motor_marker.color.a = 1.0

        quat = R.from_euler('xy', [np.pi/2, -self.q[0]]).as_quat(canonical=False)
        gearbox_marker = Marker()
        gearbox_marker.header.frame_id = self.frame_id
        gearbox_marker.ns = 'gearbox'
        gearbox_marker.type = Marker.CYLINDER
        gearbox_marker.action = Marker.ADD
        gearbox_marker.pose.position.y = MOTOR_LENGTH * 0.5
        gearbox_marker.pose.position.z = self.length
        gearbox_marker.pose.orientation.x = quat[0]
        gearbox_marker.pose.orientation.y = quat[1]
        gearbox_marker.pose.orientation.z = quat[2]
        gearbox_marker.pose.orientation.w = quat[3]
        gearbox_marker.scale.x = MOTOR_DIAMETER
        gearbox_marker.scale.y = MOTOR_DIAMETER
        gearbox_marker.scale.z = MOTOR_LENGTH
        gearbox_marker.color.r = 1.0
        gearbox_marker.color.g = 1.0
        gearbox_marker.color.a = 1.0

        quat = R.from_euler('y', -self.q[0]).as_quat(canonical=False)
        rod_marker = Marker()
        rod_marker.header.frame_id = self.frame_id
        rod_marker.ns = 'rod'
        rod_marker.type = Marker.CYLINDER
        rod_marker.action = Marker.ADD
        rod_marker.pose.position.x = self.length / 2.0 * np.sin(self.q[0])
        rod_marker.pose.position.z = self.length / 2.0 * (2 - np.cos(self.q[0]))
        rod_marker.pose.orientation.x = quat[0]
        rod_marker.pose.orientation.y = quat[1]
        rod_marker.pose.orientation.z = quat[2]
        rod_marker.pose.orientation.w = quat[3]
        rod_marker.scale.x = ROD_DIAMETER
        rod_marker.scale.y = ROD_DIAMETER
        rod_marker.scale.z = self.length
        rod_marker.color.r = 1.0
        rod_marker.color.g = 1.0
        rod_marker.color.b = 1.0
        rod_marker.color.a = 1.0

        weight_marker = Marker()
        weight_marker.header.frame_id = self.frame_id
        weight_marker.ns = 'weight'
        weight_marker.type = Marker.SPHERE
        weight_marker.action = Marker.ADD
        weight_marker.pose.position.x = self.length * np.sin(self.q[0])
        weight_marker.pose.position.z = self.length * (1 - np.cos(self.q[0]))
        weight_marker.scale.x = WEIGHT_DIAMETER
        weight_marker.scale.y = WEIGHT_DIAMETER
        weight_marker.scale.z = WEIGHT_DIAMETER
        weight_marker.color.g = 1.0
        weight_marker.color.a = 1.0

        self.marker_array = MarkerArray()
        self.marker_array.markers = [motor_marker, gearbox_marker, rod_marker, weight_marker]
        self.marker_pub.publish(self.marker_array)

    def update_system(self):
        """!@brief Update the pendulum state based on joint state."""
        if self.joint_msg is None:
            return

        dt = (self.get_clock().now() - self.now).nanoseconds * 1e-9
        self.now = self.get_clock().now()

        # Get torque from joint state
        input_torque = self.joint_msg.position[0]

        # Reflected inertia
        I_pendulum = self.mass * self.length**2
        I_reflected = self.rotor_inertia * self.gearbox_ratio**2 + I_pendulum

        sum_torque = - self.mass * self.gravity * self.length * np.sin(self.q[0]) + input_torque

        q_dot_dot = sum_torque / I_reflected

        # Update state
        self.q[1] += q_dot_dot * dt
        self.q[0] += self.q[1] * dt

    def joint_state_callback(self, msg):
        """!@brief Save joint state data."""
        self.joint_msg = msg

    def timer_callback(self):
        self.update_system()
        self.publish_markers()


def main():
    rclpy.init()
    rclpy.spin(Pendulum())


if __name__ == '__main__':
    main()
