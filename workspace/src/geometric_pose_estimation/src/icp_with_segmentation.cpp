#include <cstdlib>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "geometric_pose_estimation/utils.hpp"
#include "geometric_pose_estimation/icp_with_segmentation.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace geometric_pose_estimation
{

rviz_visual_tools::RvizVisualToolsPtr IcpWithSegmentation::visual_tools_ = nullptr;
rclcpp::Node::SharedPtr IcpWithSegmentation::vis_node_ = nullptr;
std::thread IcpWithSegmentation::visual_tools_thread_;

IcpWithSegmentation::IcpWithSegmentation()
: Node("icp_with_segmentation")
{
  // Initialize parameters
  this->declare_parameter<std::string>(
    "model_path",
    "/home/ubuntu/manipulation/gazebo_models/ACE_Coffee_Mug_Kristen_16_oz_cup/meshes/model.obj");
  this->get_parameter("model_path", model_path_);
  model_points_ = loadObjVertices(model_path_);

  // Move model centroid to origin
  const auto model_centroid = getCentroid(model_points_);
  for (auto & point : model_points_) {
    point -= model_centroid;
  }

  // Initialize publisher and subscriber
  rclcpp::QoS qos(10);
  qos.best_effort();
  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/points", qos,
    std::bind(&IcpWithSegmentation::pointCloudCallback, this, std::placeholders::_1));

  if (!visual_tools_) {
    vis_node_ = rclcpp::Node::make_shared("visual_tools_node");
    visual_tools_thread_ = std::thread(
      []() { rclcpp::spin(vis_node_); });  // Spin visual tools node in a separate thread
    visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      "depth_camera_link", "/rviz_visual_tools", vis_node_);
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
  }
}

void IcpWithSegmentation::pointCloudCallback(const sensor_msgs::msg::PointCloud2 & msg)
{
  RCLCPP_INFO(this->get_logger(), "===== Received point cloud message =====");
  // visual_tools_->deleteAllMarkers();

  // visual_tools_->publishSpheres(
  //   convertPointcloudToPoints(model_points_), rviz_visual_tools::Colors::GREEN, 0.005);
  // visual_tools_->trigger();

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
  // visual_tools_->publishABCDPlane(
  //   plane_eq[0], plane_eq[1], plane_eq[2], plane_eq[3], rviz_visual_tools::RED);
  // visual_tools_->trigger();

  // Remove the plane from the point cloud
  Pointcloud filtered_pcl = removePlaneFromPointcloud(pcl, ransac_result.first);
  // visual_tools_->publishSpheres(
  //   convertPointcloudToPoints(filtered_pcl), rviz_visual_tools::Colors::GREEN, 0.005);
  // visual_tools_->trigger();

  // Fit model using ICP
  Eigen::Isometry3f initial_guess = Eigen::Isometry3f::Identity();
  initial_guess.translation() = -getCentroid(filtered_pcl);
  Eigen::Isometry3f X_model_sensor;
  try {
    X_model_sensor = fitModelIcp(filtered_pcl, model_points_, initial_guess);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "ICP fitting failed: %s", e.what());
    return;
  }
  const auto X_sensor_model = X_model_sensor.inverse();

  // Visualize the result
  visual_tools_->deleteAllMarkers();
  visual_tools_->publishSpheres(
    convertPointcloudToPoints(filtered_pcl), rviz_visual_tools::Colors::GREEN, 0.005);
  Pointcloud model_points_transformed = model_points_;
  for (auto & point : model_points_transformed) {
    point = X_sensor_model * point;  // Transform model points to sensor frame
  }
  visual_tools_->publishSpheres(
    convertPointcloudToPoints(model_points_transformed), rviz_visual_tools::Colors::YELLOW, 0.005);
  visual_tools_->trigger();
  // visual_tools_->prompt(
  //   "ICP fitting complete: Press 'next' to continue");
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

std::vector<geometry_msgs::msg::Point> IcpWithSegmentation::convertPointcloudToPoints(
  const Pointcloud & pcl)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(pcl.size());

  for (const auto & point : pcl) {
    geometry_msgs::msg::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.push_back(p);
  }

  return points;
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

Pointcloud IcpWithSegmentation::loadObjVertices(const std::string & model_path, size_t throttle)
{
  MeasureExecutionTime measure_time("loadObjVertices");
  Pointcloud points;

  std::ifstream file(model_path);
  std::string line;

  for (size_t i = 0; std::getline(file, line); i++) {
    if (i % throttle != 0) {
      continue;  // Throttle the loading
    }

    if (line.substr(0, 2) == "v ") {
      std::istringstream iss(line.substr(2));
      Eigen::Vector3f p;
      iss >> p[0] >> p[1] >> p[2];
      points.push_back(p);
    }
  }

  return points;
}

Eigen::Isometry3f IcpWithSegmentation::fitModelIcp(
  const Pointcloud & source_points,
  const Pointcloud & target_points,
  const Eigen::Isometry3f & initial_guess,
  float max_correspondence_distance,
  size_t max_iterations,
  float convergence_threshold)
{
  MeasureExecutionTime measure_time("fitModelIcp");

  const auto & P_source = source_points;
  const auto & P_target = target_points;

  // Visualize initial guess
  // Pointcloud source_points_transformed = P_source;
  // for (auto & p_source : source_points_transformed) {
  //   p_source = initial_guess * p_source;
  // }
  // visual_tools_->publishSpheres(
  //   convertPointcloudToPoints(source_points_transformed), rviz_visual_tools::Colors::YELLOW, 0.005);
  // visual_tools_->publishSpheres(
  //   convertPointcloudToPoints(target_points), rviz_visual_tools::Colors::GREEN, 0.005);
  // visual_tools_->trigger();
  // visual_tools_->prompt(
  //   "Initial guess: Press 'next' to start ICP iterations");

  Eigen::Isometry3f X_target_source = initial_guess;

  for (size_t iter = 0; iter < max_iterations; ++iter) {
    visual_tools_->deleteAllMarkers();

    // Apply current transform to source points
    std::vector<Eigen::Vector3f> P_source_transformed;
    P_source_transformed.reserve(P_source.size());
    for (const auto & p_source : P_source) {
      Eigen::Vector3f p_source_transformed = X_target_source * p_source;
      P_source_transformed.push_back(p_source_transformed);
    }

    // Find closest point to each transformed source point
    Pointcloud P_target_matched, P_source_matched;
    P_target_matched.reserve(P_source.size());  // Max size, might end up smaller
    P_source_matched.reserve(P_source.size());
    for (const auto & p_source : P_source_transformed) {
      Eigen::Vector3f closest_point = Eigen::Vector3f::Zero();
      float min_distance = std::numeric_limits<float>::max();

      for (const auto & p_target : P_target) {
        float distance = (p_source - p_target).norm();
        if (distance < min_distance && distance < max_correspondence_distance) {
          min_distance = distance;
          closest_point = p_target;
        }
      }

      if (min_distance < max_correspondence_distance) {
        P_target_matched.push_back(closest_point);
        P_source_matched.push_back(p_source);
      }
    }

    /* Fail if no correspondences found
     * (expect that initial_guess is at least close to centroid,
     *  or max_correspondence_distance is large enough)
     */
    if (P_target_matched.empty() || P_source_matched.empty()) {
      throw std::runtime_error("No correspondences found");
    }

    // Center points and
    // calculate covariance matrix and perform SVD
    // (at this point target_pts_matched and source_pts_matched are the same size)
    const auto source_centroid = getCentroid(P_source_matched);
    const auto target_centroid = getCentroid(P_target_matched);
    Eigen::MatrixXf P_target_eigen(3, P_target_matched.size());
    Eigen::MatrixXf P_source_eigen(3, P_source_matched.size());
    for (size_t i = 0; i < P_target_matched.size(); ++i) {
      P_target_eigen.col(i) = P_target_matched[i] - target_centroid;
      P_source_eigen.col(i) = P_source_matched[i] - source_centroid;
    }
    Eigen::Matrix3f covariance = P_source_eigen * P_target_eigen.transpose();

    // Get rotation using SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
    // Ensure R is valid rotation
    if (R.determinant() < 0) {
      R.col(2) *= -1;
    }

    // Get optimal translation
    Eigen::Vector3f translation = target_centroid - R * source_centroid;

    // Update transform
    Eigen::Isometry3f new_X_target_source = Eigen::Isometry3f::Identity();
    new_X_target_source.linear() = R * X_target_source.linear();
    new_X_target_source.translation() = R * X_target_source.translation() + translation;

    // Check convergence
    const float diff_translation = (new_X_target_source.translation() - X_target_source.translation()).norm();
    const float diff_rotation = (new_X_target_source.linear() - X_target_source.linear()).norm();
    if (std::max(diff_translation, diff_rotation) < convergence_threshold) {
      RCLCPP_INFO(get_logger(), "Converged after %zu iterations", iter);
      return new_X_target_source;
    }

    // Update transform
    X_target_source = new_X_target_source;

    RCLCPP_INFO(
      get_logger(), "Iteration %zu: translation diff = %.6f, rotation diff = %.6f",
      iter, diff_translation, diff_rotation);

    {
      // Visualize current transform and prompt user
      std::vector<geometry_msgs::msg::Point> P_source_transformed;
      P_source_transformed.reserve(P_source.size());
      for (const auto & p_source : P_source) {
        geometry_msgs::msg::Point p;
        Eigen::Vector3f p_source_transformed = X_target_source * p_source;
        p.x = p_source_transformed[0];
        p.y = p_source_transformed[1];
        p.z = p_source_transformed[2];
        P_source_transformed.push_back(p);
      }
      // visual_tools_->publishSpheres(
      //   convertPointcloudToPoints(target_points), rviz_visual_tools::Colors::GREEN, 0.005);
      // visual_tools_->publishSpheres(
      //   P_source_transformed, rviz_visual_tools::Colors::YELLOW, 0.005);
      // visual_tools_->trigger();
      // visual_tools_->prompt("Press 'next' to continue ICP iteration");
      // rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  throw std::runtime_error(
    "ICP did not converge within the maximum number of iterations");
}

Eigen::Vector3f IcpWithSegmentation::getCentroid(const Pointcloud & pcl)
{
  Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);

  for (const auto & point : pcl) {
    centroid += point;
  }
  centroid /= static_cast<float>(pcl.size());

  return centroid;
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
