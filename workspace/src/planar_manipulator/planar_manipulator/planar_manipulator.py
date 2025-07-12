# other
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS modules
import rclpy
from rclpy.node import Node

# ROS message types
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


ARM_DIAMETER = 0.05
MOTOR_MAX_TORQUE = 3.0

def shortest_angular_distance(a1, a2):
    """Returns the shortest angular distance from angle a1 to a2 in radians."""
    diff = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    return diff

def clamp_angle(angle):
    """Clamp angle to the range [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class PlanarManipulator(Node):
    """!
    @brief Planar manipulator

    MIT Manipulation course
    03 - Basic Pick and Place
    05_planar_manipulator

    a) Derive the forward kinematics of the manipulator.
    b) Derive the Jacobian matrix of the manipulator.
    c) Analyze the kinematic singularities of the manipulator from the Jacobian.
    """

    def __init__(self):
        """!@brief Get parameters, initialize pub sub and marker."""

        super().__init__('planar_manipulator')
        self.get_logger().info('Planar manipulator node started.')

        # Parameters
        self.declare_parameter('arm_length', 0.5)
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('frame_id', 'map')

        self.arm_length = self.get_parameter('arm_length').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'planar_manipulator_marker', 10)

        # Subscriptions
        self.joint_msg: JointState | None = None
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Timer for publishing at a fixed frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        # System state [q_0, q_1]
        self.q = np.zeros(2)
        self.now = self.get_clock().now()

    def publish_markers(self):
        """!@brief Create and publish markers for the planar_manipulator based on q."""
        quat_0 = R.from_euler('yz', [np.pi / 2, self.q[0]]).as_quat(canonical=False)
        arm_0_marker = Marker()
        arm_0_marker.header.frame_id = self.frame_id
        arm_0_marker.ns = 'arm_0'
        arm_0_marker.type = Marker.CYLINDER
        arm_0_marker.action = Marker.ADD
        arm_0_marker.pose.position.x = self.arm_length / 2.0 * np.cos(self.q[0])
        arm_0_marker.pose.position.y = self.arm_length / 2.0 * np.sin(self.q[0])
        arm_0_marker.pose.orientation.x = quat_0[0]
        arm_0_marker.pose.orientation.y = quat_0[1]
        arm_0_marker.pose.orientation.z = quat_0[2]
        arm_0_marker.pose.orientation.w = quat_0[3]
        arm_0_marker.scale.x = ARM_DIAMETER
        arm_0_marker.scale.y = ARM_DIAMETER
        arm_0_marker.scale.z = self.arm_length
        arm_0_marker.color.r = 1.0
        arm_0_marker.color.g = 1.0
        arm_0_marker.color.b = 1.0
        arm_0_marker.color.a = 1.0

        quat_1 = R.from_euler('yz', [np.pi / 2, self.q[0] + self.q[1]]).as_quat(canonical=False)
        arm_1_marker = Marker()
        arm_1_marker.header.frame_id = self.frame_id
        arm_1_marker.ns = 'arm_1'
        arm_1_marker.type = Marker.CYLINDER
        arm_1_marker.action = Marker.ADD
        arm_1_marker.pose.position.x = \
            self.arm_length * np.cos(self.q[0]) + self.arm_length / 2.0 * np.cos(self.q[0] + self.q[1])
        arm_1_marker.pose.position.y = \
            self.arm_length * np.sin(self.q[0]) + self.arm_length / 2.0 * np.sin(self.q[0] + self.q[1])
        arm_1_marker.pose.orientation.x = quat_1[0]
        arm_1_marker.pose.orientation.y = quat_1[1]
        arm_1_marker.pose.orientation.z = quat_1[2]
        arm_1_marker.pose.orientation.w = quat_1[3]
        arm_1_marker.scale.x = ARM_DIAMETER
        arm_1_marker.scale.y = ARM_DIAMETER
        arm_1_marker.scale.z = self.arm_length
        arm_1_marker.color.r = 1.0
        arm_1_marker.color.g = 1.0
        arm_1_marker.color.b = 1.0
        arm_1_marker.color.a = 1.0

        eef_marker = Marker()
        eef_marker.header.frame_id = self.frame_id
        eef_marker.ns = 'eef'
        eef_marker.type = Marker.SPHERE
        eef_marker.action = Marker.ADD
        eef_marker.pose.position.x = self.arm_length * (np.cos(self.q[0]) + np.cos(self.q[0] + self.q[1]))
        eef_marker.pose.position.y = self.arm_length * (np.sin(self.q[0]) + np.sin(self.q[0] + self.q[1]))
        eef_marker.scale.x = ARM_DIAMETER * 3
        eef_marker.scale.y = ARM_DIAMETER * 3
        eef_marker.scale.z = ARM_DIAMETER * 3
        eef_marker.color.r = 1.0
        eef_marker.color.g = 0.0
        eef_marker.color.b = 0.0
        eef_marker.color.a = 1.0

        # Manipulability ellipsoid
        # Maps the q_0_dot, q_1_dot unit circle to end-effector velocity through the Jacobian
        manip_ellipse_marker = Marker()
        manip_ellipse_marker.header.frame_id = self.frame_id
        manip_ellipse_marker.ns = 'manip_ellipse'
        manip_ellipse_marker.type = Marker.LINE_STRIP
        manip_ellipse_marker.action = Marker.ADD
        manip_ellipse_marker.points = []
        manip_ellipse_marker.color.r = 1.0
        manip_ellipse_marker.color.g = 1.0
        manip_ellipse_marker.color.a = 1.0
        manip_ellipse_marker.scale.x = 0.01

        # Divide and loop along the unit circle
        num_points = 30
        jacobian = self.jacobian()
        for i in range(num_points + 1):
            theta = 2 * np.pi * i / num_points
            q_dot = np.array([np.cos(theta), np.sin(theta)])
            ellipse_point = np.dot(jacobian, q_dot)
            point = Point()
            point.x = eef_marker.pose.position.x + ellipse_point[0]
            point.y = eef_marker.pose.position.y + ellipse_point[1]
            point.z = 0.0
            manip_ellipse_marker.points.append(point)

        self.marker_array = MarkerArray()
        self.marker_array.markers = [arm_0_marker, arm_1_marker, eef_marker, manip_ellipse_marker]
        self.marker_pub.publish(self.marker_array)

    def jacobian(self):
        """!@brief Compute the Jacobian matrix of the planar manipulator."""
        j = np.zeros((2, 2))
        j[0, 0] = -self.arm_length * np.sin(self.q[0]) - self.arm_length * np.sin(self.q[0] + self.q[1])
        j[0, 1] = -self.arm_length * np.sin(self.q[0] + self.q[1])
        j[1, 0] = self.arm_length * np.cos(self.q[0]) + self.arm_length * np.cos(self.q[0] + self.q[1])
        j[1, 1] = self.arm_length * np.cos(self.q[0] + self.q[1])
        return j

    def update_system(self):
        """!@brief Update the planar_manipulator state based on joint state."""
        if self.joint_msg is None:
            return

        dt = (self.get_clock().now() - self.now).nanoseconds * 1e-9
        self.now = self.get_clock().now()
        if dt <= 0:
            return

        # We abuse joint state publisher to get arbitrary input
        self.q[0] = clamp_angle(self.joint_msg.position[0])
        self.q[1] = clamp_angle(self.joint_msg.position[1])

    def joint_state_callback(self, msg):
        """!@brief Save joint state data."""
        self.joint_msg = msg

    def timer_callback(self):
        self.update_system()
        self.publish_markers()


def main():
    rclpy.init()
    rclpy.spin(PlanarManipulator())


if __name__ == '__main__':
    main()
