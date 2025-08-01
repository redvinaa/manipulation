#ifndef GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_
#define GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_

// STL includes
#include <vector>
#include <functional>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Interfaces
#include "sensor_msgs/msg/point_cloud2.hpp"

// Other
#include <Eigen/Core>


namespace geometric_pose_estimation
{

using Pointcloud = std::vector<Eigen::Vector3f>;

using PlaneEquation = Eigen::Vector4f;

/// @brief Result of the RANSAC algorithm: best plane equation and the number of inliers.
using RansacResult = std::pair<PlaneEquation, size_t>;

using CostFn = std::function<float(float)>;

namespace cost_functions
{

/// @brief Gaussian
inline float gaussian(float x)
{
  return 1 - std::exp(-0.5f * x * x);
}

/// @brief Truncated least squares
inline float truncatedLeastSquares(float x)
{
  const float threshold = 1.0;
  if (std::abs(x) < threshold) {
    return x * x;
  } else {
    return threshold * threshold;
  }
}

}  // namespace cost_functions

/**
 * @brief Class for performing ICP with segmentation.
 */
class IcpWithSegmentation : public rclcpp::Node
{
public:
  IcpWithSegmentation();

  // static Pointcloud loadModelPoints(const std::string & model_path);

  /// @brief Convert PointCloud2 to Pointcloud
  static Pointcloud convertPointCloud2(const sensor_msgs::msg::PointCloud2 & msg);

  /// @brief Find plane equation using RANSAC
  static RansacResult ransacFindPlane(
    const Pointcloud & pcl,
    CostFn cost_fn,
    float tolerance = 1e-3,
    size_t max_iterations = 10);

  /// @brief Remove plane from point cloud based on equation
  static Pointcloud removePlaneFromPointcloud(
    const Pointcloud & pcl,
    const PlaneEquation & plane_equation,
    float tolerance = 1e-3);

private:
  /// @brief Callback for the point cloud topic.
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2 & pcl);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // // Parameters
  // std::string model_path_;
};

}  // geometric_pose_estimation

#endif  // GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_
