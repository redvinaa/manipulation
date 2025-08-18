import os
import random
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument('world_file', default_value='',
                              description='SDF world file to load'),
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='Enable Gazebo GUI'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logging level for nodes'),
        DeclareLaunchArgument('paused', default_value='false', description='Start simulation paused'),
        DeclareLaunchArgument('num_objects', default_value='7', description='Number of objects to spawn'),
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
    ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package='bin_picking').find('bin_picking')

    # Resolve parameters
    world_file = LaunchConfiguration('world_file').perform(context) or \
        os.path.join(pkg_share, 'worlds', 'empty_bins.world')
    gazebo_gui = LaunchConfiguration('gazebo_gui').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'

    num_objects = int(LaunchConfiguration('num_objects').perform(context))
    spawn_height_min = float(LaunchConfiguration('spawn_height_min').perform(context))
    spawn_height_diff = float(LaunchConfiguration('spawn_height_diff').perform(context))
    spawn_center_x = float(LaunchConfiguration('spawn_center_x').perform(context))
    spawn_center_y = float(LaunchConfiguration('spawn_center_y').perform(context))
    spawn_radius = float(LaunchConfiguration('spawn_radius').perform(context))

    # Discover available models (require sdf file)
    model_dir = "/home/ubuntu/manipulation/gazebo_models/ycb-tools/models/ycb"
    available_models = []
    for d in sorted(os.listdir(model_dir)):
        folder_path = os.path.join(model_dir, d)
        if not os.path.isdir(folder_path):
            continue

        # expected sdf name = folder name without 3-digit prefix
        if "_" not in d:
            continue
        sdf_basename = d.split("_", 1)[1] + ".sdf"
        sdf_path = os.path.join(folder_path, sdf_basename)

        if os.path.isfile(sdf_path):
            available_models.append((d, sdf_path))

    if not available_models:
        raise RuntimeError(f"No usable models (with SDF) found in {model_dir}")

    actions = []

    # Launch Gazebo server
    gz_args = f'-s {world_file}'
    if not paused:
        gz_args = '-r ' + gz_args
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()
        )
    )

    # Launch Gazebo GUI (optional)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g ', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gazebo_gui)
        )
    )

    # Spawn random objects
    random.seed(int(LaunchConfiguration('seed').perform(context)))
    for i in range(num_objects):
        folder_name, sdf_path = random.choice(available_models)
        print(f"Spawning object {i+1}/{num_objects}: {folder_name}")

        x = spawn_center_x + random.uniform(-spawn_radius, spawn_radius)
        y = spawn_center_y + random.uniform(-spawn_radius, spawn_radius)
        z = spawn_height_min + i * spawn_height_diff  # stack progressively

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

    return actions
