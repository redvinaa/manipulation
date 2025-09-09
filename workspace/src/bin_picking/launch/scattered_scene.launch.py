import os
import random
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument('world_file', default_value='',
                              description='SDF world file to load'),
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='Enable Gazebo GUI'),
        DeclareLaunchArgument('paused', default_value='false', description='Start simulation paused'),
        DeclareLaunchArgument('num_objects', default_value='4', description='Number of objects to spawn'),
        DeclareLaunchArgument('spawn_height_min', default_value='0.2', description='Minimum spawn height'),
        DeclareLaunchArgument('spawn_height_diff', default_value='0.1', description='Height difference between stacked objects'),
        DeclareLaunchArgument('spawn_center_x', default_value='0.5', description='Center X position for random spawn'),
        DeclareLaunchArgument('spawn_center_y', default_value='0.0', description='Center Y position for random spawn'),
        DeclareLaunchArgument('spawn_radius', default_value='0.1', description='Radius for random spawn distribution'),
        DeclareLaunchArgument('seed', default_value='2', description='Random seed for reproducibility'),
    ]

    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    pkg_share = FindPackageShare(package='bin_picking').find('bin_picking')

    world_file = LaunchConfiguration('world_file').perform(context) or \
        os.path.join(pkg_share, 'worlds', 'empty_bins.world')

    num_objects = int(LaunchConfiguration('num_objects').perform(context))
    spawn_height_min = float(LaunchConfiguration('spawn_height_min').perform(context))
    spawn_height_diff = float(LaunchConfiguration('spawn_height_diff').perform(context))
    spawn_center_x = float(LaunchConfiguration('spawn_center_x').perform(context))
    spawn_center_y = float(LaunchConfiguration('spawn_center_y').perform(context))
    spawn_radius = float(LaunchConfiguration('spawn_radius').perform(context))

    actions = []

    # Launch Gazebo world
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("ros_gz_sim").find("ros_gz_sim"),
                "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}"
        }.items(),
    ))

    # Discover available models
    model_dir = "/home/ubuntu/manipulation/gazebo_models/ycb"
    available_models = []
    for d in sorted(os.listdir(model_dir)):
        folder_path = os.path.join(model_dir, d)
        if not os.path.isdir(folder_path):
            continue
        if "_" not in d:
            continue
        sdf_basename = d.split("_", 1)[1] + ".sdf"
        sdf_path = os.path.join(folder_path, sdf_basename)
        if os.path.isfile(sdf_path):
            available_models.append((d, sdf_path))

    if not available_models:
        raise RuntimeError(f"No usable models (with SDF) found in {model_dir}")

    # Spawn objects
    random.seed(int(LaunchConfiguration('seed').perform(context)))
    for i in range(num_objects):
        folder_name, sdf_path = random.choice(available_models)
        print(f"Spawning object {i+1}/{num_objects}: {folder_name}")

        x = spawn_center_x + random.uniform(-spawn_radius, spawn_radius)
        y = spawn_center_y + random.uniform(-spawn_radius, spawn_radius)
        z = spawn_height_min + i * spawn_height_diff

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', sdf_path,
                    '-name', f'obj_{i}',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z)
                ],
                output='screen'
            )
        )

    # Gazebo ↔ ROS bridge
    bridge_config = os.path.join(pkg_share, 'config', 'camera_bridge.yaml')
    actions.append(RosGzBridge(
        bridge_name='gz_bridge',
        config_file=bridge_config,
        log_level='info',
    ))

    return actions
