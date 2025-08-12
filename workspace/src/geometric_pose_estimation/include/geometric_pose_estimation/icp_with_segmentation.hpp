#ifndef GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_
#define GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_

// STL includes

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Interfaces
#include "sensor_msgs/msg/point_cloud2.hpp"

// Other
#include <Eigen/Core>


namespace geometric_pose_estimation
{

using Scalar = float;
using PCLEigen = Eigen::Matrix<Scalar, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Transform = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using Vector = Eigen::Matrix<Scalar, 3, 1>;

// Note: Global variables are discouraged, but this is a simple example.
rclcpp::Node::SharedPtr node;
rclcpp::Logger logger = rclcpp::get_logger("geometric_pose_estimation");
rviz_visual_tools::RvizVisualToolsPtr visual_tools;
PCLEigen model_points;

/// @brief Callback for the point cloud topic.
void pointCloudCallback(const sensor_msgs::msg::PointCloud2 & pcl);
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;

/// @brief Publish point cloud to RViz
void visualizePCL(
  const PCLEigen & pcl,
  const rviz_visual_tools::Colors & color,
  float scale = 0.004f,
  const std::string & ns = "pcl");

/// @brief Gaussian cost function
inline Scalar gaussian(Scalar x)
{
  return 1 - std::exp(-0.5f * x * x);
}

/// @brief Load obj file vertices into a pointcloud
PCLEigen loadObjVertices(const std::string & model_path, size_t throttle = 10);

namespace ICP
{
struct IcpParams
{
  Scalar max_correspondence_distance = 0.1f;
  size_t max_iterations = 500;
  Scalar convergence_threshold = 1e-6;
  bool visualize = false;
};

[[nodiscard]]
Vector getCentroid(const PCLEigen & pcl);

/** @brief Get initial guess for ICP
 *
 * Rotates source points around source centroid, then moves them to target centroid.
 */
Transform getInitialGuess(
  const Vector & source_centroid,
  const Vector & target_centroid,
  const Eigen::Quaternion<Scalar> & rotation = Eigen::Quaternion<Scalar>::Identity());

/** @brief Get initial guess for ICP
 *
 * Rotates source points around source centroid, then moves them to target centroid.
 */
inline Transform getInitialGuess(
  const PCLEigen & source_points,
  const PCLEigen & target_points,
  const Eigen::Quaternion<Scalar> & rotation = Eigen::Quaternion<Scalar>::Identity())
{
  return getInitialGuess(getCentroid(source_points), getCentroid(target_points), rotation);
}

/// @brief Perform one iteration of ICP
Transform performIcpStep(
  const PCLEigen & source_points,
  const PCLEigen & target_points,
  Transform initial_guess,
  Scalar max_correspondence_distance = 0.1f,
  bool visualize = false);

/** @brief Perform ICP
 *
 *  Tries to find the best transformation that moves source_points to target_points
 */
Transform performIcp(
  const PCLEigen & source_points,
  const PCLEigen & target_points,
  Transform initial_guess,
  IcpParams params = IcpParams());

Scalar calculateSquaredError(
  const PCLEigen & source_points,
  const PCLEigen & target_points);

}  // namespace ICP

namespace RANSAC
{
/// @brief Coefficients for a plane equation in the form Ax + By + Cz + D = 0
using PlaneEquation = Eigen::Vector4<Scalar>;

struct RansacResult
{
  PlaneEquation plane_equation;
  size_t inliers_count;
};

/// @brief Find plane equation using RANSAC, gaussian cost
RansacResult findPlane(
  const PCLEigen & pcl,
  Scalar tolerance = 1e-3,
  size_t max_iterations = 20);

/// @brief Remove plane from point cloud based on equation
PCLEigen removePlaneFromPointcloud(
  const PCLEigen & pcl,
  const PlaneEquation & plane_equation,
  Scalar tolerance = 1e-3);

}  // namespace RANSAC

}  // geometric_pose_estimation

#endif  // GEOMETRIC_POSE_ESTIMATION__ICP_WITH_SEGMENTATION_HPP_
