# other
from copy import deepcopy
from scipy.spatial.transform import Rotation as R

# ROS message types
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion


def to_quaternion_msg(rot: R) -> Quaternion:
    q = rot.as_quat(canonical=False)  # [x, y, z, w]
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def create_frame_marker(position: Point, orientation: Quaternion, frame_id = 'map', axis_length = 0.1, name = None) -> MarkerArray:
    """
    Create a MarkerArray representing a coordinate frame trident (X=Red, Y=Green, Z=Blue).
    Each axis arrow has its own orientation computed via scipy.

    Args:
        frame_id (str): The frame ID the marker is attached to.
        position (geometry_msgs.msg.Point): Position of the frame.
        orientation (geometry_msgs.msg.Quaternion): Base frame orientation.
        axis_length (float): Arrow length for each axis.

    Returns:
        MarkerArray: Three markers representing X, Y, Z axes.
    """
    marker_array = MarkerArray()

    ns = 'frame'
    if name:
        ns = f'{ns}_{name}'

    base_marker = Marker()
    base_marker.header.frame_id = frame_id
    base_marker.ns = ns
    base_marker.type = Marker.ARROW
    base_marker.action = Marker.ADD
    base_marker.pose.position = position
    base_marker.scale.x = axis_length
    base_marker.scale.y = axis_length * 0.1
    base_marker.scale.z = axis_length * 0.1
    base_marker.lifetime.sec = 0  # never auto-delete

    # Convert base orientation to scipy rotation
    base_rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

    # Directions relative to X-axis arrow default direction
    axis_rotations = {
        "x": R.identity(),                    # No rotation needed for X
        "y": R.from_euler('z', 90, degrees=True),  # Rotate +X to +Y
        "z": R.from_euler('y', -90, degrees=True), # Rotate +X to +Z
    }

    colors = {
        "x": (1.0, 0.0, 0.0),  # Red
        "y": (0.0, 1.0, 0.0),  # Green
        "z": (0.0, 0.0, 1.0),  # Blue
    }

    for i, axis in enumerate(['x', 'y', 'z']):
        marker = deepcopy(base_marker)
        marker.id = i
        rot = base_rot * axis_rotations[axis]
        marker.pose.orientation = to_quaternion_msg(rot)
        r, g, b = colors[axis]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker_array.markers.append(marker)

    # Add text marker for the frame name
    if name:
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.ns = ns
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = Point(
            x=position.x,
            y=position.y,
            z=position.z - axis_length * 0.3
        )
        text_marker.text = name
        text_marker.scale.z = axis_length * 0.4
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        marker_array.markers.append(text_marker)

    return marker_array
