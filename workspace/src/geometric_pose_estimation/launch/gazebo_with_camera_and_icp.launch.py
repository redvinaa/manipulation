import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(
        package='geometric_pose_estimation').find('geometric_pose_estimation')
    world_file = LaunchConfiguration(
        'world_file', default=os.path.join(pkg_share, 'worlds', 'camera_scene.world'))
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='false')
    log_level = LaunchConfiguration('log_level', default='info')

    bridge_config = LaunchConfiguration(
        'bridge_config', default=os.path.join(pkg_share, 'config', 'camera_bridge.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file', default_value=world_file, description='SDF world file to load'),
        DeclareLaunchArgument(
            'bridge_config', default_value=bridge_config, description='YAML config for ros‑gz bridge'),
        DeclareLaunchArgument(
            'gazebo_gui', default_value=gazebo_gui, description='Enable Gazebo GUI'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='Logging level for nodes'),

        # Launch Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -s ', world_file],
                              'on_exit_shutdown': 'true'}.items()
        ),

        # Launch Gazebo GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g ', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gazebo_gui)
        ),

        RosGzBridge(
            bridge_name='gz_bridge',
            config_file=bridge_config,
            log_level=log_level,
            bridge_params=(
                "'qos_overrides./camera/points.publisher.reliability': 'best_effort'"),
            # TODO: Remove this after https://github.com/gazebosim/ros_gz/issues/774
            # gets resolved
            extra_bridge_params=[{}],
        ),

        # Static transform publisher: world → camera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=['--y', '-0.6', '--z', '0.2', '--pitch', '0.2', '--yaw', '1.57',
                       '--frame-id', 'world', '--child-frame-id', 'depth_camera_link']
        ),

        # Your ICP node
        Node(
            package='geometric_pose_estimation',
            executable='icp_with_segmentation',
            name='icp_subscriber',
            output='screen'
        ),
    ])
