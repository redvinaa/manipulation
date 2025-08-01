#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include "geometric_pose_estimation/utils.hpp"
#include "geometric_pose_estimation/icp_with_segmentation.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace geometric_pose_estimation
{

IcpWithSegmentation::IcpWithSegmentation()
: Node("icp_with_segmentation")
{
  // // Initialize parameters
  // this->declare_parameter<std::string>(
  //   "model_path",
  //   "/home/ubuntu/manipulation/gazebo_models/ACE_Coffee_Mug_Kristen_16_oz_cup/meshes/model.obj");
  // this->get_parameter("model_path", model_path_);

  // Initialize publisher and subscriber
  rclcpp::QoS qos(10);
  qos.best_effort();
  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/points", qos,
    std::bind(&IcpWithSegmentation::pointCloudCallback, this, std::placeholders::_1));

  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
    "depth_camera_link", "/rviz_visual_tools", this);

  // // Load model points
  // Pointcloud model_points = loadModelPoints(model_path_);
}

void IcpWithSegmentation::pointCloudCallback(const sensor_msgs::msg::PointCloud2 & msg)
{
  visual_tools_->deleteAllMarkers();

  // Convert to Pointcloud
  const Pointcloud pcl = convertPointCloud2(msg);

  // Perform RANSAC to find plane
  const auto ransac_result = ransacFindPlane(pcl, cost_functions::gaussian);
  const PlaneEquation & plane_eq = ransac_result.first;
  const size_t inliers_count = ransac_result.second;
  RCLCPP_INFO(
    this->get_logger(),
    "RANSAC found plane with %zu inliers, equation: %.2f x + %.2f y + %.2f z + %.2f = 0",
    inliers_count, plane_eq[0], plane_eq[1], plane_eq[2], plane_eq[3]);
  visual_tools_->publishABCDPlane(
    plane_eq[0], plane_eq[1], plane_eq[2], plane_eq[3], rviz_visual_tools::RED);
  visual_tools_->trigger();

  // Remove the plane from the point cloud
  Pointcloud filtered_pcl = removePlaneFromPointcloud(pcl, ransac_result.first);
  std::vector<geometry_msgs::msg::Point> filtered_points;
  filtered_points.reserve(filtered_pcl.size());
  for (const auto & point : filtered_pcl) {
    geometry_msgs::msg::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    filtered_points.push_back(p);
  }
  visual_tools_->publishSpheres(filtered_points, rviz_visual_tools::Colors::GREEN, 0.01);
  visual_tools_->trigger();

  // Publish or process the filtered point cloud
}

Pointcloud IcpWithSegmentation::convertPointCloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  MeasureExecutionTime measure_time("convertPointCloud2");
  Pointcloud pointcloud;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Skip invalid points (e.g. NaNs)
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    Eigen::Vector3f point(*iter_x, *iter_y, *iter_z);
    pointcloud.push_back(point);
  }

  return pointcloud;
}

RansacResult IcpWithSegmentation::ransacFindPlane(
  const Pointcloud & pcl,
  CostFn cost_fn,
  float tolerance,
  size_t max_iterations)
{
  MeasureExecutionTime measure_time("ransacFindPlane");
  RansacResult best_result;

  for (size_t i = 0; i < max_iterations; ++i) {
    // Sample 3 random points from the point cloud
    std::vector<size_t> indices;
    while (indices.size() < 3) {
      size_t idx = rand() % pcl.size();
      if (std::find(indices.begin(), indices.end(), idx) == indices.end()) {
        indices.push_back(idx);
      }
    }

    // Compute the plane equation from the sampled points
    Eigen::Vector3f p1 = pcl[indices[0]];
    Eigen::Vector3f p2 = pcl[indices[1]];
    Eigen::Vector3f p3 = pcl[indices[2]];
    Eigen::Vector3f normal = (p2 - p1).cross(p3 - p1);
    if (normal.norm() < 1e-6) {
      // Points are collinear, skip this iteration
      continue;
    }
    normal.normalize();
    float d = -normal.dot(p1);

    PlaneEquation plane_eq;
    plane_eq << normal[0], normal[1], normal[2], d;

    // Count inliers based on the plane equation
    size_t inliers_count = 0;
    for (const auto & point : pcl) {
      float distance = std::abs(plane_eq[0] * point[0] + plane_eq[1] * point[1] +
                                plane_eq[2] * point[2] + plane_eq[3]);
      if (distance < tolerance) {
        inliers_count++;
      }
    }

    // If this is the best result so far, save it
    if (inliers_count > best_result.second) {
      best_result.first = plane_eq;
      best_result.second = inliers_count;
    }
  }

  return std::make_pair(best_result.first, best_result.second);
}

Pointcloud IcpWithSegmentation::removePlaneFromPointcloud(
  const Pointcloud & pcl,
  const PlaneEquation & plane_equation,
  float tolerance)
{
  MeasureExecutionTime measure_time("removePlaneFromPointcloud");
  Pointcloud filtered_pcl;

  for (const auto & point : pcl) {
    float distance = std::abs(plane_equation[0] * point[0] +
                              plane_equation[1] * point[1] +
                              plane_equation[2] * point[2] +
                              plane_equation[3]);
    if (distance >= tolerance) {
      filtered_pcl.push_back(point);
    }
  }

  return filtered_pcl;
}

}  // namespace geometric_pose_estimation

int main(int argc, char** argv)
{
  // Init ros
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<geometric_pose_estimation::IcpWithSegmentation>());

  rclcpp::shutdown();
  return 0;
}
