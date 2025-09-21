from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def toBool(launch_config: LaunchConfiguration, context) -> bool:
    return launch_config.perform(context).lower() in ("true", "1", "yes")


def generate_launch_description():
    # Declare launch arguments
    gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Launch Gazebo with GUI"
    )
    gazebo_verbose_arg = DeclareLaunchArgument(
        "gazebo_verbose",
        default_value="false",
        description="Enable verbose output in Gazebo"
    )
    use_bullet_physics_arg = DeclareLaunchArgument(
        "use_bullet_physics",
        default_value="false",
        description="Use Bullet Featherstone physics engine"
    )
    paused_arg = DeclareLaunchArgument(
        "paused",
        default_value="false",
        description="Start simulation paused"
    )
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value=FindPackageShare("ros_gz_sim").find("ros_gz_sim") + "/worlds/empty.sdf",
        description="Path to the Gazebo world file"
    )

    gazebo_gui = LaunchConfiguration("gazebo_gui")
    gazebo_verbose = LaunchConfiguration("gazebo_verbose")
    use_bullet_physics = LaunchConfiguration("use_bullet_physics")
    paused = LaunchConfiguration("paused")
    world_file = LaunchConfiguration("world_file")

    def launch_setup(context, *args, **kwargs):
        # Build gz sim arguments dynamically
        gz_args = " -r"  # run sim on start
        if not toBool(gazebo_gui, context):
            gz_args += " -s"
        if toBool(gazebo_verbose, context):
            gz_args += " -v 4"
        if toBool(use_bullet_physics, context):
            gz_args += " --physics-engine gz-physics-bullet-featherstone-plugin"
        if toBool(paused, context):
            gz_args += " -p"
        gz_args += " " + world_file.perform(context)

        gz_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        )

        return [gz_launch_description]

    return LaunchDescription(
        [
            gazebo_gui_arg,
            gazebo_verbose_arg,
            use_bullet_physics_arg,
            paused_arg,
            world_file_arg,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
