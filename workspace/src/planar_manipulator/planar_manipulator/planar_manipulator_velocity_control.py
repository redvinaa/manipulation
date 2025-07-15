# other
import angles
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS modules
import rclpy
from rclpy.node import Node

# ROS message types
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


"""!
@brief Use differential forward kinematics to control end-effector.

Path is an L width square centered around the origin.
"""


ARM_DIAMETER = 0.05  # Only for visualization

class PlanarManipulator(Node):
    def __init__(self):
        """!@brief Get parameters, initialize pub sub and marker."""

        super().__init__('planar_manipulator')
        self.get_logger().info('Planar manipulator node started.')

        # Parameters
        self.declare_parameter('arm_length', 0.5)
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('eef_speed', 0.3)

        self.arm_length = self.get_parameter('arm_length').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.eef_speed = self.get_parameter('eef_speed').get_parameter_value().double_value

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'planar_manipulator_marker', 10)

        # Timer for publishing at a fixed frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        # System state [q_0, q_1]
        self.q = np.array([0.0, 0.2])  # Joint angles in radians
        self.now = self.get_clock().now()
        self.target_points = np.array([[self.arm_length, 0.0],
                                       [0.0, self.arm_length],
                                       [-self.arm_length * 1.2, self.arm_length * 1.2],
                                       [-self.arm_length, 0.0],
                                       [0.0, -self.arm_length],
                                       [-self.arm_length * 0.3, self.arm_length * 0.3]])

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

        path_marker = Marker()
        path_marker = Marker()
        path_marker.header.frame_id = self.frame_id
        path_marker.ns = 'path_marker'
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.points = []
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.a = 1.0
        path_marker.scale.x = 0.01

        for i in range(len(self.target_points) + 1):
            point = self.target_points[i % len(self.target_points)]
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            path_marker.points.append(p)

        current_target_marker = Marker()
        current_target_marker.header.frame_id = self.frame_id
        current_target_marker.ns = 'current_target_marker'
        current_target_marker.type = Marker.SPHERE
        current_target_marker.action = Marker.ADD
        current_target_marker.pose.position.x = self.target_points[0][0]
        current_target_marker.pose.position.y = self.target_points[0][1]
        current_target_marker.scale.x = 0.05
        current_target_marker.scale.y = 0.05
        current_target_marker.scale.z = 0.05
        current_target_marker.color.r = 1.0
        current_target_marker.color.g = 1.0
        current_target_marker.color.b = 0.0
        current_target_marker.color.a = 1.0


        self.marker_array = MarkerArray()
        self.marker_array.markers = [arm_0_marker, arm_1_marker, eef_marker, path_marker, current_target_marker]
        self.marker_pub.publish(self.marker_array)

    def jacobian(self):
        """!@brief Compute the Jacobian matrix of the planar manipulator."""
        j = np.zeros((2, 2))
        j[0, 0] = -self.arm_length * np.sin(self.q[0]) - self.arm_length * np.sin(self.q[0] + self.q[1])
        j[0, 1] = -self.arm_length * np.sin(self.q[0] + self.q[1])
        j[1, 0] = self.arm_length * np.cos(self.q[0]) + self.arm_length * np.cos(self.q[0] + self.q[1])
        j[1, 1] = self.arm_length * np.cos(self.q[0] + self.q[1])
        return j

    def get_target_velocity(self):
        """!@brief Compute the target end-effector velocity leading to next target point."""
        # Compute the current end-effector position
        x = self.arm_length * (np.cos(self.q[0]) + np.cos(self.q[0] + self.q[1]))
        y = self.arm_length * (np.sin(self.q[0]) + np.sin(self.q[0] + self.q[1]))

        # Compute the desired velocity towards the target point
        direction = self.target_points[0] - np.array([x, y])
        norm = np.linalg.norm(direction)
        if norm < 0.01:
            self.get_logger().info('Target reached, going to next target point.')
            self.target_points = np.roll(self.target_points, -1, axis=0)
            return np.zeros(2)

        direction /= norm
        target_velocity = direction * self.eef_speed
        return target_velocity

    def update_system(self):
        """!@brief Update the planar_manipulator state based on joint state."""
        dt = (self.get_clock().now() - self.now).nanoseconds * 1e-9
        self.now = self.get_clock().now()
        if dt <= 0:
            return

        target_velocity = self.get_target_velocity()
        self.get_logger().info(f'Target velocity: {target_velocity}')

        j = self.jacobian()
        dq = np.linalg.pinv(j) @ target_velocity
        self.q += dq * dt

    def timer_callback(self):
        self.update_system()
        self.publish_markers()


def main():
    rclpy.init()
    rclpy.spin(PlanarManipulator())


if __name__ == '__main__':
    main()
