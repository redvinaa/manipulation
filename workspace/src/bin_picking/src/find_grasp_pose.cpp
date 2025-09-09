#include "bin_picking/find_grasp_pose.hpp"
#include "bin_picking/utils.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>  // for concatenatePointCloud


namespace bin_picking
{

FindGraspPose::FindGraspPose(const rclcpp::NodeOptions & options)
: Node("find_grasp_pose", options),
  visual_tools_("world", "/rviz_visual_tools", this)  // TODO use target_frame_
{
  // Declare parameter for pointcloud topics
  declare_parameter<std::vector<std::string>>("pointcloud_topics", {});
  pointcloud_topics_ = get_parameter("pointcloud_topics").as_string_array();

  declare_parameter<std::string>("target_frame", "world");
  target_frame_ = get_parameter("target_frame").as_string();

  declare_parameter<float>("voxel_size", 0.01f);
  voxel_size_ = get_parameter("voxel_size").as_double();
  declare_parameter<float>("min_x", -std::numeric_limits<float>::max());
  min_x_ = get_parameter("min_x").as_double();
  declare_parameter<float>("max_x", std::numeric_limits<float>::max());
  max_x_ = get_parameter("max_x").as_double();
  declare_parameter<float>("min_y", -std::numeric_limits<float>::max());
  min_y_ = get_parameter("min_y").as_double();
  declare_parameter<float>("max_y", std::numeric_limits<float>::max());
  max_y_ = get_parameter("max_y").as_double();
  declare_parameter<float>("min_z", -std::numeric_limits<float>::max());
  min_z_ = get_parameter("min_z").as_double();
  declare_parameter<float>("max_z", std::numeric_limits<float>::max());
  max_z_ = get_parameter("max_z").as_double();

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

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void FindGraspPose::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  const std::string & topic_name)
{
  MeasureExecutionTime timer("pointCloudCallback");

  // Convert ROS msg -> PCL cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  RCLCPP_INFO(get_logger(), "Received cloud from %s with %zu points",
              topic_name.c_str(), cloud->size());

  auto transformed_cloud = transformPointCloud(cloud, target_frame_);

  // Voxelize
  float voxel_size = 0.01f;  // could be parameterized
  auto voxelized = voxelize(transformed_cloud, voxel_size);

  // Crop to region of interest
  auto voxelized_cropped = cropPointCloud(voxelized, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);

  // Compute normals (optional, may be used later)
  int k_neighbors = 10;
  auto with_normals = computeNormals(voxelized_cropped, k_neighbors);

  // Store voxelized cloud
  clouds_with_normals_[topic_name] = with_normals;

  // Visualize normals as lines
  visual_tools_.deleteAllMarkers();
  for (size_t i = 0; i < with_normals->size(); i += 10) {  // visualize every 10th normal
    const auto & pt = with_normals->points[i];
    if (!std::isnan(pt.normal_x) && !std::isnan(pt.normal_y) && !std::isnan(pt.normal_z)) {
      Eigen::Vector3d start(pt.x, pt.y, pt.z);
      Eigen::Vector3d end(pt.x + 0.05 * pt.normal_x,
                          pt.y + 0.05 * pt.normal_y,
                          pt.z + 0.05 * pt.normal_z);
      visual_tools_.publishLine(
        start, end, rviz_visual_tools::Colors::RED, rviz_visual_tools::Scales::XXSMALL);
    }
  }
  visual_tools_.trigger();

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

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& target_frame)
{
  // Lookup the transform from cloud frame to target frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
      transform_stamped = tf_buffer_->lookupTransform(
          target_frame, cloud->header.frame_id,
          rclcpp::Time(cloud->header.stamp));  //, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
      return nullptr;
  }

  // Convert geometry_msgs Transform to Eigen Affine3f
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  transform_1.translation() <<
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z;
  Eigen::Quaternionf q(
      transform_stamped.transform.rotation.w,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z);
  transform_1.rotate(q);
  // Alternatively, use Eigen directly
  Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();
  transform_eigen.translation() <<
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z;
  transform_eigen.linear() = Eigen::Quaternionf(
      transform_stamped.transform.rotation.w,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z).toRotationMatrix();

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_eigen);

  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::cropPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z)
{
    // Initialize the CropBox filter
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(input);

    // Define the minimum and maximum points of the box
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1.0);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1.0);

    // Set the crop box parameters
    crop_filter.setMin(min_pt);
    crop_filter.setMax(max_pt);

    // Apply the filter and store the result
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    crop_filter.filter(*cropped_cloud);

    return cropped_cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr FindGraspPose::computeNormals(
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

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*input, *normals, *cloud_with_normals);
  return cloud_with_normals;
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

}  // namespace bin_picking

#include <rclcpp_components/register_node_macro.hpp>
using bin_picking::FindGraspPose;
RCLCPP_COMPONENTS_REGISTER_NODE(FindGraspPose)
