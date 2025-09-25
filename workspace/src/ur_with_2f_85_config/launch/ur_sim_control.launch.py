# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

# Note(Vince Reda): Copied and removed gazebo and rviz related stuff

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    description_file = LaunchConfiguration("description_file")
    ur_parent_frame = LaunchConfiguration("ur_parent_frame")

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        description_file, " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=", "ur", " ",
        "ur_type:=", ur_type, " ",
        "tf_prefix:=", tf_prefix, " ",
        "simulation_controllers:=", controllers_file, " ",
        "parent:=", ur_parent_frame])

    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Gazebo spawner
    gazebo_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.001", "-Y", "0",
        ],
        output="screen",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    initial_joint_controller_spawner_args = [initial_joint_controller, "-c", "/controller_manager"]
    if activate_joint_controller.perform(context).lower() != "true":
        initial_joint_controller_spawner_args.append("--stopped")
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=initial_joint_controller_spawner_args,
    )

    # Spawners for the Robotiq controllers
    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    )

    sensors_yaml = PathJoinSubstitution([
        FindPackageShare("ur_with_2f_85_config"), "config", "sensors_3d.yaml"
    ]).perform(context)

    moveit_config = MoveItConfigsBuilder(
        "ur_with_2f_85", package_name="ur_with_2f_85_config").sensors_3d(
            file_path=sensors_yaml
        ).to_moveit_configs()

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "octomap_resolution": 0.007,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
                "capabilities": ParameterValue(
                    moveit_config.move_group_capabilities["capabilities"], value_type=str),
                "disable_capabilities": ParameterValue(
                    moveit_config.move_group_capabilities["disable_capabilities"], value_type=str),
                # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            }]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        gazebo_spawner,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        robotiq_gripper_controller_spawner,
        move_group,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur15",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            "safety_limits", default_value="true",
            description="Enables the safety limits controller if true."))

    declared_arguments.append(DeclareLaunchArgument(
            "safety_pos_margin", default_value="0.15",
            description="The margin to lower and upper limits in the safety controller."))

    declared_arguments.append(DeclareLaunchArgument(
            "safety_k_position", default_value="20",
            description="k-position factor in the safety controller."))

    declared_arguments.append(DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare('ur_with_2f_85_config'), 'config', 'ur_ros2_controllers.yaml']),
            description=\
                "Absolute path to YAML file with the ros2_control controllers configuration."))

    declared_arguments.append(DeclareLaunchArgument(
            "tf_prefix", default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup."))

    declared_arguments.append(DeclareLaunchArgument(
            "activate_joint_controller", default_value="true",
            description="Start initial joint controller immediately"))

    declared_arguments.append(DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Robot controller to start."))

    declared_arguments.append(DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare('ur_with_2f_85_description'), 'urdf', 'ur_with_2f_85.urdf.xacro']
            ),
            description="URDF/XACRO description file (absolute path) with the robot."))

    declared_arguments.append(DeclareLaunchArgument(
            "ur_parent_frame",
            default_value="world",
            description="Name of the frame to which the robot is attached."))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
