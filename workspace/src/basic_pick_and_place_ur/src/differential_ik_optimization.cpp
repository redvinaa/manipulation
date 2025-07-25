#include <optional>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <Eigen/Core>
#include "OsqpEigen/OsqpEigen.h"

/*
 * Solving excercise
 * https://deepnote.com/workspace/Manipulation-ac8201a1-470a-4c77-afd0-2cc45bc229ff/project/65aad364-ef1c-45f5-a796-fac7c122e274/notebook/10differentialikoptimization-89cfae31353f4c939d5fc8df21487315
 *
 * Load velocity controller to run this:
 * ros2 control load_controller --set-state active forward_velocity_controller
 *
 * Let's create a moving target for the robot to reach: it should move in a circle.
 * Set feasible joint velocity limits, and a virtual wall that the robot should not cross.
 */

//! @brief Target moving in a circle around the z-axis
class Target
{
public:
  Target(rclcpp::Node::SharedPtr node)
  {
    node->declare_parameter("target_pos_z", 1.0);
    node->declare_parameter("target_trajectory_radius", 0.2);
    node->declare_parameter("target_angular_velocity", 0.1);
    z_pos_ = node->get_parameter("target_pos_z").as_double();
    radius_ = node->get_parameter("target_trajectory_radius").as_double();
    angular_velocity_ = node->get_parameter("target_angular_velocity").as_double();

    clock_ = node->get_clock();
    start_time_ = clock_->now();
  }

  Eigen::Vector3d getPosition()
  {
    double dt = (clock_->now() - start_time_).seconds();
    double angle = angular_velocity_ * dt;

    Eigen::Vector3d position;
    position.x() = radius_ * cos(angle);
    position.y() = radius_ * sin(angle);
    position.z() = z_pos_;
    return position;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time start_time_;
  double z_pos_;
  double radius_;
  double angular_velocity_;
};

class ConstrainedFollower
{
public:
  ConstrainedFollower(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    node->declare_parameter("planning_group", "ur_manipulator");
    node->declare_parameter("base_frame_id", "base_link");
    node->declare_parameter("constrain_x_max", 0.0);
    node->declare_parameter("max_joint_velocity", 1.5);
    node->declare_parameter("target_eef_velocity", 0.5);
    planning_group_ = node->get_parameter("planning_group").as_string();
    base_frame_id_ = node->get_parameter("base_frame_id").as_string();
    constrain_x_max_ = node->get_parameter("constrain_x_max").as_double();
    max_joint_velocity_ = node->get_parameter("max_joint_velocity").as_double();
    target_eef_velocity_ = node->get_parameter("target_eef_velocity").as_double();
    const auto controller_frequency =
      node->get_parameter("controller_frequency").as_double();
    if (controller_frequency <= 0.0) {
      RCLCPP_ERROR(node->get_logger(), "Controller frequency must be positive.");
      throw std::runtime_error("Invalid controller frequency");
    }
    dt_ = 1.0 / controller_frequency;

    joint_velocity_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 1);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "ur_manipulator");
    visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      base_frame_id_, "/rviz_markers", node);
    visual_tools_->deleteAllMarkers();

    using rclcpp::contexts::get_global_default_context;
    get_global_default_context()->add_pre_shutdown_callback(
      [this]() {
        // Publish zero joint velocities to stop the robot
        std_msgs::msg::Float64MultiArray stop_joint_velocity_msg;
        stop_joint_velocity_msg.data.resize(6, 0.0);  // TODO: Get the correct size somehow
        joint_velocity_publisher_->publish(stop_joint_velocity_msg);
        RCLCPP_INFO(node_->get_logger(), "Stopping robot.");
      });
  }

  void updateVelocities(const Eigen::Vector3d & target_position)
  {
    const auto & robot_model = move_group_->getRobotModel();
    moveit::core::RobotState robot_state(robot_model);

    // Get target velocity
    const auto current_position_msg = move_group_->getCurrentPose().pose;
    Eigen::Vector3d current_position(
      current_position_msg.position.x,
      current_position_msg.position.y,
      current_position_msg.position.z);
    Eigen::Vector3d target_velocity_linear = target_position - current_position;
    const double target_vel_norm = target_velocity_linear.norm();
    if (target_vel_norm > target_eef_velocity_) {
      // Normalize and scale to target velocity
      target_velocity_linear *= (target_eef_velocity_ / target_vel_norm);
    }
    Eigen::VectorXd target_velocity(6);
    target_velocity.head<3>() = target_velocity_linear;
    target_velocity.tail<3>().setZero();  // No angular velocity

    // Get jacobian
    robot_state.setJointGroupPositions(planning_group_, move_group_->getCurrentJointValues());
    robot_state.update();
    const auto * jmg = robot_state.getJointModelGroup(planning_group_);
    Eigen::MatrixXd jacobian = robot_state.getJacobian(jmg);

    // Publish visualization of the target and eef poses
    if (!last_target_pose_) {
      last_target_pose_ = target_position;
      last_eef_pose_.emplace(
        current_position_msg.position.x, current_position_msg.position.y,
        current_position_msg.position.z);
    } else {
      const Eigen::Vector3d current_eef_pose(
        current_position_msg.position.x, current_position_msg.position.y,
        current_position_msg.position.z);

      // Target line is blue
      visual_tools_->publishLine(
        *last_target_pose_, target_position, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);

      // EEF line is green normally, red close to singularity
      const auto eef_line_color = isCloseToSingularity(jacobian)
        ? rviz_visual_tools::RED : rviz_visual_tools::GREEN;
      visual_tools_->publishLine(
        *last_eef_pose_, current_eef_pose, eef_line_color, rviz_visual_tools::LARGE);
      visual_tools_->trigger();

      last_target_pose_ = target_position;
      last_eef_pose_.emplace(current_eef_pose);
    }

    /* Get optimal joint velocities
     *
     * The problem:
     * minimize v: ||V_G - J(q) * v||^2,
     * where V_G is the gripper (eef) velocity and v are the joint velocities
     *
     * This can be rewritten as:
     * minimize v: 1/2 * v^T * Q * v + c^T * v,
     * where Q = J(q)^T * J(q) and c = -J(q)^T * V_G
     *
     * Subject to l ≤ A * v ≤ u,
     * where A should be of size: n_constraints x n_joints,
     * and l, u should be of size n_constraints.
     */
    Eigen::SparseMatrix<double> Q = (jacobian.transpose() * jacobian).sparseView();
    Eigen::VectorXd c = -jacobian.transpose() * target_velocity;
    const size_t num_joints = jmg->getVariableCount();

    // TODO: Add position constraints
    const size_t num_constraints = num_joints;
    Eigen::SparseMatrix<double> A(num_constraints, num_joints);
    A.setIdentity();
    Eigen::VectorXd l(num_constraints);
    Eigen::VectorXd u(num_constraints);
    l.setConstant(-max_joint_velocity_);
    u.setConstant(max_joint_velocity_);

    OsqpEigen::Solver solver;

    RCLCPP_INFO(
      node_->get_logger(),
      "num_joints: %zu, jacobian: %zu x %zu, Q: %zu x %zu, c: %zu, A: %zu x %zu, l: %zu, u: %zu",
      num_joints, jacobian.rows(), jacobian.cols(), Q.rows(), Q.cols(), c.size(),
      A.rows(), A.cols(), l.size(), u.size());

    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(num_joints);
    solver.data()->setNumberOfConstraints(num_constraints);
    solver.data()->setHessianMatrix(Q);
    solver.data()->setGradient(c);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    Eigen::VectorXd v_optimal = Eigen::VectorXd::Zero(num_joints);

    if (!solver.initSolver()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize OSQP solver.");
    } else if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      RCLCPP_ERROR(node_->get_logger(), "OSQP solver failed to solve the problem.");
    } else {
      v_optimal = solver.getSolution();
    }

    std_msgs::msg::Float64MultiArray joint_velocity_msg;
    joint_velocity_msg.data.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
      joint_velocity_msg.data[i] = v_optimal(i);
    }
    joint_velocity_publisher_->publish(joint_velocity_msg);
  }

  static bool isCloseToSingularity(const Eigen::MatrixXd & jacobian, double threshold = 0.01)
  {
    // Check if the Jacobian is close to singular by checking its determinant
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.singularValues().minCoeff() < threshold;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_publisher_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  const moveit::core::JointModelGroup * jmg_;

  std::optional<Eigen::Vector3d> last_target_pose_, last_eef_pose_;

  // Parameters
  std::string planning_group_;
  std::string base_frame_id_;
  double max_joint_velocity_;
  double constrain_x_max_;
  double target_eef_velocity_;
  double dt_;
};

int main(int argc, char** argv)
{
  // Init ros
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("differential_ik_optimization");

  // Get parameters
  node->declare_parameter("controller_frequency", 10.0);
  double controller_frequency = node->get_parameter("controller_frequency").as_double();

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  Target target(node);
  ConstrainedFollower follower(node);
  rclcpp::Rate rate(controller_frequency);

  while (rclcpp::ok()) {
    // Get target position from the moving target
    Eigen::Vector3d target_position = target.getPosition();

    // Update joint velocities based on the target position
    follower.updateVelocities(target_position);

    // Sleep for the controller frequency
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
