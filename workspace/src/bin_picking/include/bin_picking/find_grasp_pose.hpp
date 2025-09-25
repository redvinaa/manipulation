#ifndef BIN_PICKING__FIND_GRASP_POSE_HPP_
#define BIN_PICKING__FIND_GRASP_POSE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>


namespace bin_picking
{

class FindGraspPose
{
public:
  explicit FindGraspPose();

  /// Wait for spin thread to finish
  void run();

private:
  // Parameters
  std::vector<std::string> pointcloud_topics_;
  std::string target_frame_;
  float voxel_size_;
  float min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

  // Nodes
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  // We need a separate node for MoveGroupInterface, because otherwise getCurrentState()
  // fails. I think it's because even though MGI creates its own callback group and
  // spins it, in getCurrentState it calls CurrentStateMonitor, which uses
  // the node passed to MGI, and doesn't create its own callback group.
  rclcpp::Node::SharedPtr mgi_node_;
  rclcpp::executors::SingleThreadedExecutor mgi_executor_;
  std::thread mgi_spin_thread_;

  // Interfaces
  moveit::planning_interface::MoveGroupInterfacePtr move_group_arm_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_gripper_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Storage for processed clouds
  std::map<std::string, pcl::PointCloud<pcl::PointNormal>::Ptr> clouds_with_normals_;

  // Planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Visualization
  std::shared_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;

  // Subscriber callback
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    const std::string & topic_name);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
      const std::string & target_frame);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
    float min_x, float max_x,
    float min_y, float max_y,
    float min_z, float max_z);

  pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
    const geometry_msgs::msg::Point & viewpoint = geometry_msgs::msg::Point(),
    const int k_neighbors = 10);

  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr computePrincipalCurvatures(
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr & input,
    const int k_neighbors = 10);

  // Merge all voxelized clouds stored in voxelized_clouds_
  pcl::PointCloud<pcl::PointNormal>::Ptr mergeClouds(
    const std::vector<pcl::PointCloud<pcl::PointNormal>::ConstPtr> & clouds,
    const float voxel_size);

  // Checks if the gripper at a given pose is in collision
  bool checkCollision(const Eigen::Isometry3d & grasp_pose, double gripper_joint = 0.0);

  /** @brief Visualize the gripper at a given pose
   *
   * @return IDs of the created markers
   */
  std::vector<int> visualizeGripper(
    const Eigen::Isometry3d & pose, double gripper_joint,
    rviz_visual_tools::Colors color = rviz_visual_tools::BLUE);

  // TODO organize relevant code into this function
  // geometry_msgs::msg::Pose findGraspPose(
  //   const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud);
};

}  // namespace bin_picking

#endif  // BIN_PICKING__FIND_GRASP_POSE_HPP_
