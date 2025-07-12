# other
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS modules
import rclpy
from rclpy.node import Node
from manipulation_utils import create_frame_marker, to_quaternion_msg

# ROS message types
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Quaternion


"""!
@brief Find grasp pose.

MIT Manipulation course
03 - Basic Pick and Place
07_rigid_transforms
"""



def main():
    rclpy.init()

    node = Node('grasp_pose_node')

    marker_pub = node.create_publisher(MarkerArray, 'grasp_pose_marker', 10)

    world_frame = create_frame_marker(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        name='world',
    )
    marker_pub.publish(world_frame)

    p_WO = np.array([0.5, 0.1, 0])
    R_WO = R.from_euler('xyz', [np.pi / 2, 0, -np.pi / 2])

    object_frame = create_frame_marker(
        position=Point(x=p_WO[0], y=p_WO[1], z=p_WO[2]),
        orientation=to_quaternion_msg(R_WO),
        name='object',
    )
    marker_pub.publish(object_frame)


    p_OG_W = np.array([0.0, 0.0, 0.02])
    R_OW = R_WO.inv()
    p_OG = R_OW.apply(p_OG_W)

    R_OG = R.from_euler('xyz', [np.pi, -np.pi / 2, 0])

    p_WG = p_WO + R_WO.apply(p_OG)
    R_WG = R_WO * R_OG
    grasp_frame = create_frame_marker(
        position=Point(x=p_WG[0], y=p_WG[1], z=p_WG[2]),
        orientation=to_quaternion_msg(R_WG),
        name='grasp',
    )
    marker_pub.publish(grasp_frame)

    # Set float precision for printing
    np.set_printoptions(precision=3, suppress=True)

    # Pring X_OG and X_WG
    print("p_OG:", p_OG)
    print("R_OG:", R_OG.as_matrix())
    print("p_WG:", p_WG)
    print("R_WG:", R_WG.as_matrix())


if __name__ == '__main__':
    main()
