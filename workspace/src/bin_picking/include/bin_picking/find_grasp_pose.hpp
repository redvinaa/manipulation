#ifndef BIN_PICKING__FIND_GRASP_POSE_HPP_
#define BIN_PICKING__FIND_GRASP_POSE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>


namespace bin_picking
{

class FindGraspPose : public rclcpp::Node
{
public:
  explicit FindGraspPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  std::vector<std::string> pointcloud_topics_;
  std::string target_frame_;
  float voxel_size_;
  float min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

  // Interfaces
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Storage for processed clouds
  std::map<std::string, pcl::PointCloud<pcl::PointNormal>::Ptr> clouds_with_normals_;

  // Visualization
  std::shared_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;
  rclcpp::Node::SharedPtr vis_tools_node_;
  std::thread vis_tools_thread_;

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

  geometry_msgs::msg::Pose findGraspPose(
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud);
};

}  // namespace bin_picking

#endif  // BIN_PICKING__FIND_GRASP_POSE_HPP_
