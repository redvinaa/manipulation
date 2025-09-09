#ifndef BIN_PICKING__FIND_GRASP_POSE_HPP_
#define BIN_PICKING__FIND_GRASP_POSE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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
  float voxel_size_;

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;

  // Storage for processed clouds
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> voxelized_clouds_;

  // Visualization
  rviz_visual_tools::RvizVisualTools visual_tools_;

  // Subscriber callback
  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    const std::string & topic_name);

  // Processing chain
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
    float voxel_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr computeNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
    int k_neighbors);

  // Merge all voxelized clouds stored in voxelized_clouds_
  pcl::PointCloud<pcl::PointXYZ>::Ptr mergeVoxelizedClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds);
};

}  // namespace bin_picking

#endif  // BIN_PICKING__FIND_GRASP_POSE_HPP_
