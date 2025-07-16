#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <Eigen/Core>

// Solving excercise
// https://deepnote.com/workspace/Manipulation-ac8201a1-470a-4c77-afd0-2cc45bc229ff/project/65aad364-ef1c-45f5-a796-fac7c122e274/notebook/08robotpainter-d78494343af24c88bfeab80b79c742c4

[[nodiscard]] std::vector<Eigen::Isometry3d> composeCircularKeyFrames(
  std::vector<float> thetas, Eigen::Isometry3d X_WCenter, float radius)
{
  std::vector<Eigen::Isometry3d> key_frames;

  Eigen::Isometry3d X_CStart;
  X_CStart.translation() = Eigen::Vector3d(0.0, radius, 0.0);

  for (const auto& theta : thetas)
  {
    auto R_CK = Eigen::AngleAxisd(-theta, Eigen::Vector3d(0.0, 0.0, 1.0)).toRotationMatrix();
    Eigen::Isometry3d X_CK;
    X_CK.translation() = R_CK * X_CStart.translation();
    X_CK.linear() = R_CK;
    key_frames.push_back(X_WCenter * X_CK);
  }

  return key_frames;
}

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_painter");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("robot_painter_node", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();


  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "ur_manipulator";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.hpp>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.hpp>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);





  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // Set joint to 0
  std::vector<double> joint_values = move_group.getCurrentJointValues();
  for (size_t i = 0; i < joint_values.size(); ++i) {
    joint_values[i] = 0.0; // Set all joints to zero
  }
  joint_values[1] = -M_PI / 2;
  move_group.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to plan to initial joint state.");
    return 1;
  }

  joint_values.back() = -M_PI / 2;
  move_group.setJointValueTarget(joint_values);
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to plan to initial joint state.");
    return 1;
  }

  move_group.setEndEffector("wrist_3_link");
  RCLCPP_WARN(
    LOGGER, "Eef frame: %s",
    move_group.getEndEffector().c_str());

  rviz_visual_tools::RvizVisualTools visual_tools("world", "/rviz_visual_tools", move_group_node);

  {
    float radius = 0.1;

    Eigen::Isometry3d X_WCenter;
    X_WCenter.translation() << 0.45, 0.0, 0.4;

    Eigen::Matrix3d R_WCenter = (
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0, 1.0, 0.0)) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0, 0.0, 1.0))).toRotationMatrix();
    X_WCenter.linear() = R_WCenter;
    visual_tools.publishAxis(X_WCenter, 0.1);
    visual_tools.trigger();

    // Move the robot to the center of the circle
    move_group.setPoseTarget(X_WCenter);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group.execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to plan to the center of the circle.");
      return 1;
    }

    size_t num_keyframes = 10;
    std::vector<float> thetas;
    for (size_t i = 0; i < num_keyframes; ++i) {
      thetas.push_back(2.0 * M_PI * i / num_keyframes);
    }

    auto key_frames = composeCircularKeyFrames(thetas, X_WCenter, radius);
    for (const auto& key_frame : key_frames) {
      visual_tools.publishAxis(key_frame, 0.05);
    }
    visual_tools.trigger();

    std::vector<geometry_msgs::msg::Pose> key_frame_poses;
    for (const auto& key_frame : key_frames) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = key_frame.translation().x();
      pose.position.y = key_frame.translation().y();
      pose.position.z = key_frame.translation().z();
      Eigen::Quaterniond quat(key_frame.linear());
      pose.orientation.x = quat.x();
      pose.orientation.y = quat.y();
      pose.orientation.z = quat.z();
      pose.orientation.w = quat.w();
      key_frame_poses.push_back(pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    move_group.computeCartesianPath(key_frame_poses, 0.01, trajectory);
    if (trajectory.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(LOGGER, "Failed to compute Cartesian path.");
      return 1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    if (move_group.execute(cartesian_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Failed to execute Cartesian path.");
      return 1;
    }
  }

  rclcpp::shutdown();
  return 0;
}
