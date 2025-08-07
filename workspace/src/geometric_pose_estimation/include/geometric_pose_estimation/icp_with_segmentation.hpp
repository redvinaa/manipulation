#ifndef GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_
#define GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_

// STL includes
#include <vector>
#include <thread>
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

/// @brief General cost function signature
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
 *
 * TODO(redvinaa): This should have been a namespace
 */
class IcpWithSegmentation : public rclcpp::Node
{
public:
  IcpWithSegmentation();

  /** @brief SVD based model fitting
   *
   * Correspondence is by nearest neighbor up to max distance.
   * Moves sensor points to model points, then inverts transform
   * to decreate effect of partial views
   */
  Eigen::Isometry3f fitModelIcp(
    const Pointcloud & source_points,
    const Pointcloud & target_points,
    const Eigen::Isometry3f & initial_guess = Eigen::Isometry3f::Identity(),
    float max_correspondence_distance = 0.1f,
    size_t max_iterations = 100,
    float convergence_threshold = 1e-3);

  [[nodiscard]]
  static Eigen::Vector3f getCentroid(const Pointcloud & pcl);

  static Pointcloud loadObjVertices(const std::string & model_path, size_t throttle = 10);

  /// @brief Convert PointCloud2 to Pointcloud
  static Pointcloud convertPointCloud2(const sensor_msgs::msg::PointCloud2 & msg);

  /// @brief Convert Pointcloud to std::vector<Points>
  static std::vector<geometry_msgs::msg::Point> convertPointcloudToPoints(
    const Pointcloud & pcl);

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

  static rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  static rclcpp::Node::SharedPtr vis_node_;
  static std::thread visual_tools_thread_;

private:
  /// @brief Callback for the point cloud topic.
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2 & pcl);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  Pointcloud model_points_;

  // Parameters
  std::string model_path_;
};

}  // geometric_pose_estimation

#endif  // GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_
