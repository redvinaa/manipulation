import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction
from moveit_configs_utils import MoveItConfigsBuilder



def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare(package='bin_picking').find('bin_picking')

    actions = []

    # Reset RViz time on startup in simulation
    actions.append(ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/rviz/reset_time', 'std_srvs/srv/Empty', '{}'],
        output='screen'
    ))

    # Include scattered scene (Gazebo + objects)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "scattered_scene_with_ur.launch.py")
        )
    ))

    # Static TFs for cameras
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.5', '--y', '0.3', '--z', '0.4',
                   '--roll', '0.0', '--pitch', '0.7', '--yaw', '-1.57',
                   '--frame-id', 'world', '--child-frame-id', 'bin1_cam_left']
    ))
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.5', '--y', '-0.3', '--z', '0.4',
                   '--roll', '0.0', '--pitch', '0.7', '--yaw', '1.57',
                   '--frame-id', 'world', '--child-frame-id', 'bin1_cam_right']
    ))

    bin1_x = 0.5
    bin1_y = 0.0
    bin2_x = 0.0
    bin2_y = 0.5
    bin_size_x = 0.4
    bin_size_y = 0.4
    wall_cutoff = 0.02  # To avoid including bin walls in the point cloud


    sensors_yaml = PathJoinSubstitution([
        FindPackageShare("ur_with_2f_85_config"), "config", "sensors_3d.yaml"
    ]).perform(context)

    moveit_config = MoveItConfigsBuilder(
        "ur_with_2f_85", package_name="ur_with_2f_85_config").sensors_3d(
            file_path=sensors_yaml
        ).to_moveit_configs()

    actions.append(Node(
        package="bin_picking",
        executable="find_grasp_pose_node",
        name="find_grasp_pose",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.sensors_3d,
            {
                'pointcloud_topics': [
                    '/bin1/left/camera/points',
                    '/bin1/right/camera/points',
                ],
                'voxel_size': 0.01,
                'publish_cropped_clouds': True,
                'target_frame': 'world',
                "min_x": bin1_x - bin_size_x/2 + wall_cutoff,
                "max_x": bin1_x + bin_size_x/2 - wall_cutoff,
                "min_y": bin1_y - bin_size_y/2 + wall_cutoff,
                "max_y": bin1_y + bin_size_y/2 - wall_cutoff,
                "min_z": wall_cutoff, "max_z": 0.5,
                'use_sim_time': True,
            }
        ],
    ))

    #  # PCL CropBox filters for each depth camera
    #  def create_crop_box_node(camera_name: str, parameters: dict) -> ComposableNode:
    #      parameters.update({
    #          'use_sim_time': True,
    #          'input_frame': 'world',
    #          'output_frame': 'world'})
    #      return ComposableNode(
    #          package="pcl_ros",
    #          plugin="pcl_ros::CropBox",
    #          name=f"{camera_name.replace('/', '_')}_crop_box",
    #          remappings=[
    #              ("input", f"/{camera_name}/camera/points"),
    #              ("output", f"/{camera_name}/camera/points_cropped")
    #          ],
    #          parameters=[parameters],
    #      )

    #  actions.append(ComposableNodeContainer(
    #      name='bin_picking_container',
    #      namespace='',
    #      package='rclcpp_components',
    #      executable='component_container',
    #      composable_node_descriptions=[
    #          ComposableNode(
    #              package='bin_picking',
    #              plugin='FindGraspPose',
    #              name='find_grasp_pose',
    #              parameters=[
    #                  moveit_config.robot_description,
    #                  moveit_config.robot_description_semantic,
    #                  moveit_config.robot_description_kinematics,
    #                  {
    #                      'pointcloud_topics': [
    #                          '/bin1/left/camera/points',
    #                          '/bin1/right/camera/points',
    #                      ],
    #                      'voxel_size': 0.01,
    #                      'publish_cropped_clouds': True,
    #                      'target_frame': 'world',
    #                      "min_x": bin1_x - bin_size_x/2 + wall_cutoff,
    #                      "max_x": bin1_x + bin_size_x/2 - wall_cutoff,
    #                      "min_y": bin1_y - bin_size_y/2 + wall_cutoff,
    #                      "max_y": bin1_y + bin_size_y/2 - wall_cutoff,
    #                      "min_z": wall_cutoff, "max_z": 0.5,
    #                      'use_sim_time': True,
    #                  }
    #              ],
    #          ),
    #          #  create_crop_box_node("bin1/left", {
    #          #      "min_x": bin1_x - bin_size_x/2 + wall_cutoff,
    #          #      "max_x": bin1_x + bin_size_x/2 - wall_cutoff,
    #          #      "min_y": bin1_y - bin_size_y/2 + wall_cutoff,
    #          #      "max_y": bin1_y + bin_size_y/2 - wall_cutoff,
    #          #      "min_z": wall_cutoff, "max_z": 0.5,
    #          #  }),
    #          #  create_crop_box_node("bin1/right", {
    #          #      "min_x": bin1_x - bin_size_x/2 + wall_cutoff,
    #          #      "max_x": bin1_x + bin_size_x/2 - wall_cutoff,
    #          #      "min_y": bin1_y - bin_size_y/2 + wall_cutoff,
    #          #      "max_y": bin1_y + bin_size_y/2 - wall_cutoff,
    #          #      "min_z": wall_cutoff, "max_z": 0.5,
    #          #  }),
    #      ],
    #      output='screen',
    #  ))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
