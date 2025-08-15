#include <vector>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <iomanip>  // for scientific notation

#include <rclcpp/rclcpp.hpp>
#include "geometric_pose_estimation/utils.hpp"
#include "geometric_pose_estimation/icp_with_segmentation.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace geometric_pose_estimation
{

void pointCloudCallback(const sensor_msgs::msg::PointCloud2 & msg)
{
  RCLCPP_INFO(logger, "===== Received point cloud message =====");
  visual_tools->deleteAllMarkers();

  // Convert pointcloud to eigen
  PCLEigen pcl;
  {
    MeasureExecutionTime measure_time("convert pcl to eigen");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    const size_t num_points = msg.width * msg.height;
    pcl.resize(3, num_points);
    for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
      pcl.col(i) << *iter_x, *iter_y, *iter_z;
    }
  }

  // Perform RANSAC to find plane
  RANSAC::RansacResult ransac_result;
  {
    MeasureExecutionTime measure_time("RANSAC find plane");
    ransac_result = RANSAC::findPlane(pcl);
  }
  const RANSAC::PlaneEquation & plane_eq = ransac_result.plane_equation;
  RCLCPP_INFO(
    logger,
    "RANSAC found plane with %zu inliers, equation: %.2f x + %.2f y + %.2f z + %.2f = 0",
    ransac_result.inliers_count, plane_eq[0], plane_eq[1], plane_eq[2], plane_eq[3]);
  visual_tools->publishABCDPlane(
    plane_eq[0], plane_eq[1], plane_eq[2], plane_eq[3], rviz_visual_tools::RED);
  visual_tools->trigger();

  // Remove the plane from the point cloud
  {
    MeasureExecutionTime measure_time("remove plane from point cloud");
    pcl = RANSAC::removePlaneFromPointcloud(pcl, ransac_result.plane_equation);
  }
  visualizePCL(
    pcl, rviz_visual_tools::Colors::BLUE, 0.005, "pcl_filtered");
  // visual_tools->prompt("Press 'next' to continue with ICP fitting");

  // Fit model using ICP
  ICP::IcpParams params;
  params.visualize = true;

  Transform X_model_sensor;
  try {
    MeasureExecutionTime measure_time("ICP fitting");

    // Random quat rotated around z
    const float random_yaw = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * M_PI;
    Eigen::AngleAxisf random_angle(random_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf random_quat(random_angle);

    // Fit sensor to model to decrease effect of partial views
    Transform initial_guess = ICP::getInitialGuess(pcl, model_points, random_quat);
    X_model_sensor = ICP::performIcp(
      pcl, model_points, initial_guess, params);
    const auto error = ICP::calculateSquaredError(
      X_model_sensor * pcl, model_points);
    RCLCPP_INFO_STREAM(
      logger, "ICP fitting completed with error: " <<
      std::scientific << std::setprecision(2) << error);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger, "ICP fitting failed: %s", e.what());
    return;
  }

  const Transform X_sensor_model = X_model_sensor.inverse();

  // Visualize (best) result
  visual_tools->deleteAllMarkers();
  visual_tools->publishAxisLabeled(
    X_sensor_model.cast<double>(), "model_fitted_transform");
  visualizePCL(
    X_sensor_model * model_points,
    rviz_visual_tools::Colors::GREEN, 0.005, "model_fitted");
  visual_tools->prompt("Press 'next' to process new message");
}

RANSAC::RansacResult RANSAC::findPlane(
  const PCLEigen & pcl, float tolerance, size_t max_iterations)
{
  RansacResult best_result;

  for (size_t i = 0; i < max_iterations; ++i) {
    // Sample 3 random points from the point cloud
    std::vector<size_t> indices;
    while (indices.size() < 3) {
      size_t idx = rand() % pcl.cols();
      if (std::find(indices.begin(), indices.end(), idx) == indices.end()) {
        indices.push_back(idx);
      }
    }

    // Compute the plane equation from the sampled points
    Vector p1 = pcl.col(indices[0]);
    Vector p2 = pcl.col(indices[1]);
    Vector p3 = pcl.col(indices[2]);
    Vector normal = (p2 - p1).cross(p3 - p1);
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
    for (const auto & point : pcl.colwise()) {
      float distance = std::abs(plane_eq[0] * point[0] + plane_eq[1] * point[1] +
                                plane_eq[2] * point[2] + plane_eq[3]);
      if (distance < tolerance) {
        inliers_count++;
      }
    }

    // If this is the best result so far, save it
    if (inliers_count > best_result.inliers_count) {
      best_result.plane_equation = plane_eq;
      best_result.inliers_count = inliers_count;
    }
  }

  return best_result;
}

PCLEigen RANSAC::removePlaneFromPointcloud(
  const PCLEigen & pcl,
  const PlaneEquation & plane_equation,
  float tolerance)
{
  PCLEigen filtered_pcl;
  filtered_pcl.resize(3, pcl.cols());

  size_t counter = 0;
  for (const auto & point : pcl.colwise()) {
    float distance = std::abs(plane_equation[0] * point[0] +
                              plane_equation[1] * point[1] +
                              plane_equation[2] * point[2] +
                              plane_equation[3]);
    if (distance >= tolerance) {
      filtered_pcl.col(counter) = point;
      counter++;
    }
  }

  filtered_pcl.conservativeResize(3, counter);
  return filtered_pcl;
}

PCLEigen loadObjVertices(const std::string & model_path, size_t throttle)
{
  std::ifstream file(model_path);
  std::string line;

  std::vector<Vector> vertices;
  for (size_t i = 0; std::getline(file, line); i++) {
    if (i % throttle != 0) {
      continue;  // Throttle the loading
    }

    if (line.substr(0, 2) == "v ") {
      std::istringstream iss(line.substr(2));
      Vector p;
      iss >> p[0] >> p[1] >> p[2];
      vertices.push_back(p);
    }
  }

  PCLEigen points;
  points.resize(3, vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    points.col(i) = vertices[i];
  }

  return points;
}

Transform ICP::getInitialGuess(
  const Vector & source_centroid,
  const Vector & target_centroid,
  const Eigen::Quaternion<Scalar> & rotation)
{
  Transform center_source = Transform::Identity();
  center_source.translation() = -source_centroid;

  Transform rotate = Transform::Identity();
  rotate.linear() = rotation.toRotationMatrix();

  Transform move_to_target = Transform::Identity();
  move_to_target.translation() = target_centroid;

  // Compose the initial guess
  Transform initial_guess_random_rot = move_to_target * rotate * center_source;
  return initial_guess_random_rot;
}

Transform ICP::performIcpStep(
  const PCLEigen & source_points,
  const PCLEigen & target_points,
  Transform initial_guess,
  float max_correspondence_distance,
  bool visualize)
{
  MeasureExecutionTime measure_time("ICP step");

  // Apply initial guess
  const auto & P_guess = initial_guess;
  PCLEigen P_source = P_guess * source_points;
  PCLEigen P_target = target_points;

  if (visualize) {
    visualizePCL(
      source_points, rviz_visual_tools::Colors::BLUE, 0.005, "source_points");
    visualizePCL(
      P_source, rviz_visual_tools::Colors::YELLOW, 0.005, "source_points_transformed");
    visualizePCL(
      P_target, rviz_visual_tools::Colors::GREEN, 0.005, "target_points");
  }

  // For each sorce point, find the closest target point (within the max distance)
  std::vector<Vector> P_source_matched_v, P_target_matched_v;
  const auto max_dist_sq = max_correspondence_distance * max_correspondence_distance;
  for (int i = 0; i < P_source.cols(); ++i) {
    Eigen::Vector3f p = P_source.col(i);
    Eigen::Matrix3Xf diff = P_target.colwise() - p;
    Eigen::RowVectorXf dist_sq = diff.colwise().squaredNorm();

    Eigen::Index min_idx;
    dist_sq.minCoeff(&min_idx);

    if (dist_sq[min_idx] <= max_dist_sq) {
      P_source_matched_v.push_back(p);
      P_target_matched_v.push_back(P_target.col(min_idx));
    }
  }

  if (P_target_matched_v.empty()) {
    throw std::runtime_error("No correspondences found");
  }

  // Build matrices from matched points
  PCLEigen P_source_matched(3, P_source_matched_v.size());
  PCLEigen P_target_matched(3, P_target_matched_v.size());
  for (size_t i = 0; i < P_source_matched_v.size(); ++i) {
    P_source_matched.col(i) = P_source_matched_v[i];
    P_target_matched.col(i) = P_target_matched_v[i];
  }

  // Center the point clouds
  const Vector source_centroid_matched = getCentroid(P_source_matched);
  const Vector target_centroid_matched = getCentroid(P_target_matched);
  P_source_matched.colwise() -= source_centroid_matched;
  P_target_matched.colwise() -= target_centroid_matched;

  // Compute the covariance matrix and perform SVD
  Eigen::Matrix3<Scalar> covariance =
    P_source_matched * P_target_matched.transpose();
  Eigen::JacobiSVD<Eigen::MatrixX<Scalar>> svd(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3<Scalar> R =
    svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0) {
    R.col(2) *= -1;
  }

  // Optimal translation: from centroid of the *rotated* source points to the target centroid
  const Vector translation =
    target_centroid_matched - R * source_centroid_matched;
  Transform P_svd = Transform::Identity();
  P_svd.linear() = R;
  P_svd.translation() = translation;

  // Compose final transform
  Transform new_X_target_source = P_svd * P_guess;

  return new_X_target_source;
}

Transform ICP::performIcp(
  const PCLEigen & source_points,
  const PCLEigen & target_points,
  Transform initial_guess,
  ICP::IcpParams params)
{
  // Start with the initial guess
  Transform X_target_source;
  size_t last_spheres_id = 0;

  for (size_t iter = 0; iter < params.max_iterations; ++iter) {
    Transform new_X_target_source;
    if (iter == 0) {
      // Let performIcpStep handle missing guess
      new_X_target_source = performIcpStep(
        source_points, target_points, initial_guess,
        params.max_correspondence_distance, params.visualize);
    } else {
      new_X_target_source = performIcpStep(
        source_points, target_points, X_target_source,
        params.max_correspondence_distance, params.visualize);
    }

    if (params.visualize) {
      visual_tools->deleteMarker("source_points", last_spheres_id);
      last_spheres_id = visualizePCL(
        new_X_target_source * source_points,
        rviz_visual_tools::Colors::YELLOW, 0.005,
        "source_points");
    }

    // Check convergence
    if (iter > 0) {
      const Scalar diff_translation =
        (new_X_target_source.translation() - X_target_source.translation()).norm();
      const Scalar diff_rotation =
        (new_X_target_source.linear() - X_target_source.linear()).norm();

      if (diff_translation < params.convergence_threshold &&
          diff_rotation < params.convergence_threshold)
      {
        RCLCPP_INFO(logger, "Converged after %zu iterations", iter + 1);
        return new_X_target_source;
      } else {
        RCLCPP_INFO(
          logger, "Iteration %zu: translation diff = %.6f, rotation diff = %.6f",
          iter + 1, diff_translation, diff_rotation);
      }
    }

    X_target_source = new_X_target_source;
  }

  throw std::runtime_error(
    "ICP did not converge after " + std::to_string(params.max_iterations) + " iterations");
}

size_t visualizePCL(
  const PCLEigen & pcl,
  const rviz_visual_tools::Colors & color,
  float scale,
  const std::string & ns)
{
  if (pcl.size() == 0) {
    RCLCPP_WARN(logger, "Point cloud is empty, nothing to visualize");
  }

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(pcl.cols());
  for (const auto & point : pcl.colwise()) {
    geometry_msgs::msg::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.push_back(p);
  }

  visual_tools->publishSpheres(points, color, scale, ns);
  visual_tools->trigger();
  return visual_tools->getSpheresId();
}

Vector ICP::getCentroid(const PCLEigen & pcl)
{
  if (pcl.size() == 0) {
    throw std::runtime_error("Point cloud is empty, cannot compute centroid");
  }

  Vector centroid(0.0f, 0.0f, 0.0f);

  for (const auto & point : pcl.colwise()) {
    centroid += point;
  }
  centroid /= static_cast<float>(pcl.cols());

  return centroid;
}

Scalar ICP::calculateSquaredError(
  const PCLEigen & source_points,
  const PCLEigen & target_points)
{
  Scalar error = 0.0f;
  for (const auto & p_source : source_points.colwise()) {
    // Compute squared distances to all points in target_points
    Eigen::Matrix3Xf diff = target_points.colwise() - p_source;
    Eigen::RowVectorXf dist_sq = diff.colwise().squaredNorm();

    // Find the minimum distance
    Scalar min_dist_sq = dist_sq.minCoeff();
    error += min_dist_sq;
  }

  return error / static_cast<Scalar>(source_points.cols());
}

}  // namespace geometric_pose_estimation

int main(int argc, char** argv)
{
  using namespace geometric_pose_estimation;

  // Init ros
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("icp_with_segmentation_node");

  // Initialize parameters
  node->declare_parameter<std::string>(
    "model_path",
    "/home/ubuntu/manipulation/gazebo_models/ACE_Coffee_Mug_Kristen_16_oz_cup/meshes/model.obj");
  std::string model_path;
  node->get_parameter("model_path", model_path);
  model_points = loadObjVertices(model_path);

  // Initialize publisher and subscriber
  rclcpp::QoS qos(10);
  qos.best_effort();
  pcl_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/points", qos,
    std::bind(&pointCloudCallback, std::placeholders::_1));

  // Visual tools node has to be already spinning for remote control
  auto visual_tools_node = rclcpp::Node::make_shared("visual_toolsnode");
  auto visual_tools_thread = std::thread(
    [visual_tools_node]() { rclcpp::spin(visual_tools_node); });
  visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
    "depth_camera_link", "/rviz_visual_tools", visual_tools_node);
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
