#ifndef BIN_PICKING__FIND_GRASP_POSE_HPP_
#define BIN_PICKING__FIND_GRASP_POSE_HPP_

#include <memory>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <vector>
#include <map>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>


namespace bin_picking
{

/** 
 * @brief Find grasp poses for a parallel-jaw gripper from point clouds
 *
 * - subscribes to one or more point cloud topics
 * - computes normals and principal curvatures
 * - samples candidate grasps using Darboux frames
 * - checks grasps for collisions
 * - visualizes the results in RViz
 * - finds and publishes best candidate
 */
class FindGraspPose
{
public:
  explicit FindGraspPose();

  /// Wait for spin thread to finish
  void run();

private:
  // Parameters
  std::string pointcloud_topic_left_;
  std::string pointcloud_topic_right_;
  std::string camera_frame_left_;
  std::string camera_frame_right_;
  std::string target_frame_;
  float voxel_size_;

  // Nodes
  /* We need a separate node for MoveGroupInterface, because otherwise getCurrentState()
   * fails. I think it's because even though MGI creates its own callback group and
   * spins it, in getCurrentState it calls CurrentStateMonitor, which uses
   * the node passed to MGI, and doesn't create its own callback group.
   */
  rclcpp::Node::SharedPtr node_, mgi_node_, vis_node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;

  // Interfaces
  moveit::planning_interface::MoveGroupInterfacePtr move_group_arm_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_gripper_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud1_, sub_cloud2_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>> sync_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Transform from grasp frame to gripper base frame
  std::shared_ptr<Eigen::Isometry3d> T_gripper_grasp_;

  // Planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Visualization
  std::shared_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;

  // Subscriber callback
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_left,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_right);

  pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
    const geometry_msgs::msg::Point & viewpoint = geometry_msgs::msg::Point(),
    const int k_neighbors = 10);

  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr computePrincipalCurvatures(
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr & input,
    const int k_neighbors = 10);

  // Checks if the gripper at a given pose is in collision
  bool checkGripperCollision(const Eigen::Isometry3d & gripper_base_pose, double gripper_joint = 0.0);

  /** @brief Visualize the gripper at a given pose
   *
   * @param gripper_joint Gripper joint value (0.0 = open, 0.8 = closed)
   * @return IDs of the created markers
   */
  std::vector<int> visualizeGripper(
    const Eigen::Isometry3d & gripper_base_pose, double gripper_joint = 0.0,
    rviz_visual_tools::Colors color = rviz_visual_tools::BLUE);

  struct CandidateGrasp
  {
    Eigen::Isometry3d pose;
    size_t index;
    bool success;
    size_t inliers;
  };
  CandidateGrasp sampleCandidateGrasp(
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud,
    const pcl::PointCloud<pcl::PrincipalCurvatures>::ConstPtr & pcs,
    bool interactive = false);
};

}  // namespace bin_picking

#endif  // BIN_PICKING__FIND_GRASP_POSE_HPP_
