#include "bin_picking/find_grasp_pose.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h>  // for concatenatePointCloud


namespace bin_picking
{

FindGraspPose::FindGraspPose(const rclcpp::NodeOptions & options)
: Node("find_grasp_pose", options),
  visual_tools_("world", "/rviz_visual_markers", this)
{
  // Declare parameter for pointcloud topics
  declare_parameter<std::vector<std::string>>("pointcloud_topics", {});
  pointcloud_topics_ = get_parameter("pointcloud_topics").as_string_array();

  declare_parameter<float>("voxel_size", 0.01f);
  voxel_size_ = get_parameter("voxel_size").as_double();

  if (pointcloud_topics_.empty()) {
    RCLCPP_WARN(get_logger(), "No pointcloud topics configured!");
  }

  // Create a subscriber for each topic
  for (const auto & topic : pointcloud_topics_) {
    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, topic](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        pointCloudCallback(msg, topic);
      });
    subscriptions_.push_back(sub);
    RCLCPP_INFO(get_logger(), "Subscribed to %s", topic.c_str());
  }
}

void FindGraspPose::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  const std::string & topic_name)
{
  // Convert ROS msg -> PCL cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  RCLCPP_INFO(get_logger(), "Received cloud from %s with %zu points",
              topic_name.c_str(), cloud->size());

  // Voxelize
  float voxel_size = 0.01f;  // could be parameterized
  auto voxelized = voxelize(cloud, voxel_size);

  // Compute normals (optional, may be used later)
  int k_neighbors = 10;
  auto with_normals = computeNormals(voxelized, k_neighbors);

  // Store voxelized cloud
  voxelized_clouds_[topic_name] = voxelized;

  // Visualize (optional, here just show cloud size in console)
  RCLCPP_INFO(get_logger(),
              "Processed voxelized cloud from %s with %zu points",
              topic_name.c_str(), voxelized->size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::voxelize(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  float voxel_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  sor.setInputCloud(input);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::computeNormals(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  int k_neighbors)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(input);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setKSearch(k_neighbors);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  ne.compute(*normals);

  // For now we just return the input cloud again.
  // (In practice, you may want to store normals alongside the points,
  //   e.g. use pcl::PointCloud<pcl::PointNormal> instead.)
  return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*input));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::mergeVoxelizedClouds(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto & c : clouds) {
    if (c && !c->empty()) {
      *merged += *c;
    }
  }
  RCLCPP_INFO(get_logger(), "Merged cloud has %zu points", merged->size());
  return merged;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bin_picking::FindGraspPose)

}  // namespace bin_picking
