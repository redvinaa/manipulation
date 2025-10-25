import os
import random
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.logging import get_logger
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument('world_file', default_value='',
                              description='SDF world file to load'),
        DeclareLaunchArgument('num_objects', default_value='5', description='Number of objects to spawn'),
        DeclareLaunchArgument('spawn_height_min', default_value='0.1', description='Minimum spawn height'),
        DeclareLaunchArgument('spawn_height_diff', default_value='0.1', description='Height difference between stacked objects'),
        DeclareLaunchArgument('spawn_center_x', default_value='0.5', description='Center X position for random spawn'),
        DeclareLaunchArgument('spawn_center_y', default_value='0.0', description='Center Y position for random spawn'),
        DeclareLaunchArgument('spawn_radius', default_value='0.07', description='Radius for random spawn distribution'),
        DeclareLaunchArgument('seed', default_value='5', description='Random seed for reproducibility'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='Whether to launch Gazebo GUI'),
        DeclareLaunchArgument('only_mustard_bottle', default_value='false',
                              description='Spawn only the mustard bottle if true'),
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
    only_mustard_bottle = LaunchConfiguration('only_mustard_bottle').perform(context).lower() in ("true", "1", "yes")
    seed = int(LaunchConfiguration('seed').perform(context))

    actions = []
    logger = get_logger('launch')

    EXCLUDE_OBJECTS = ['059_chain', '053_mini_soccer_ball']

    # ---------------------------
    # Include Gazebo launch file
    # ---------------------------
    gazebo_launch_file = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')  # adjust path
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world_file': world_file,
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items()
    )
    actions.append(gazebo_launch)

    # ---------------------------
    # Discover available models
    # ---------------------------
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
            if any(excl == d for excl in EXCLUDE_OBJECTS):
                logger.warn(f"Excluding model: '{d}'")
                continue
            available_models.append((d, sdf_path))

    if not available_models:
        raise RuntimeError(f"No usable models (with SDF) found in {model_dir}")

    # ---------------------------
    # Spawn objects
    # ---------------------------
    if only_mustard_bottle:
        # Only spawn 006_mustard_bottle
        sdf_path = os.path.join(model_dir, '006_mustard_bottle', 'mustard_bottle.sdf')
        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', sdf_path,
                    '-name', 'obj_0',
                    '-x', str(spawn_center_x),
                    '-y', str(spawn_center_y),
                    '-z', str(spawn_height_min)
                ],
                output='screen'
            )
        )
    else:
        # Spawn random objects
        random.seed(seed)
        for i in range(num_objects):
            folder_name, sdf_path = random.choice(available_models)
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

    # ---------------------------
    # Gazebo ↔ ROS bridge
    # ---------------------------
    bridge_config = os.path.join(pkg_share, 'config', 'camera_bridge.yaml')
    actions.append(RosGzBridge(
        bridge_name='gz_bridge',
        config_file=bridge_config,
        bridge_params=(
            "'qos_overrides./bin1/left/camera/points.publisher."
            "reliability': 'best_effort', "
            "'qos_overrides./bin1/right/camera/points.publisher."
            "reliability': 'best_effort'"),
        log_level='info',
    ))

    # ---------------------------
    # Include UR launch file
    # ---------------------------
    ur_launch_file = os.path.join(
        FindPackageShare('ur_with_2f_85_description').find('ur_with_2f_85_description'),
        'launch',
        'ur_sim_control.launch.py'
    )
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_file),
        launch_arguments={
            'world_file': world_file,
            "ur_fixed_frame": "world"}.items()
    )
    actions.append(ur_launch)

    return actions
