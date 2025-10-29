#ifndef MOTION_PLANNING__CROWDED_SCENE_HPP
#define MOTION_PLANNING__CROWDED_SCENE_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>
#include <vector>
#include <angles/angles.h>

namespace motion_planning
{

class CrowdedScene
{
public:
  explicit CrowdedScene(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    // Retrieve parameters
    node_->declare_parameter("root_link_length", 1.0);
    node_->declare_parameter("end_link_length", 1.0);
    node_->declare_parameter("dist_from_obstacle", 0.5);
    node_->declare_parameter("bin_depth", 1.0);
    node_->declare_parameter("bin_width", 1.0);
    node_->declare_parameter("configuration_space_size", 2.0);

    node_->get_parameter("root_link_length", root_link_length_);
    node_->get_parameter("end_link_length", end_link_length_);
    node_->get_parameter("dist_from_obstacle", dist_from_obstacle_);
    node_->get_parameter("bin_depth", bin_depth_);
    node_->get_parameter("bin_width", bin_width_);
    node_->get_parameter("configuration_space_size", configuration_space_size_);

    // Compute wall segments
    walls_.push_back({Eigen::Vector2d(bin_width_, 0.0), Eigen::Vector2d(bin_width_, -bin_depth_)});
    walls_.push_back({Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, -bin_depth_)});
    walls_.push_back({Eigen::Vector2d(-bin_width_, 0.0), Eigen::Vector2d(-bin_width_, -bin_depth_)});
    walls_.push_back({Eigen::Vector2d(bin_width_, -bin_depth_), Eigen::Vector2d(-bin_width_, -bin_depth_)});
    for (auto& wall : walls_)
    {
      wall.first.y() -= dist_from_obstacle_;
      wall.second.y() -= dist_from_obstacle_;
    }

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("crowded_scene_markers", 1);
  }

  bool isInCollision(const Eigen::Vector2d& angles) const
  {
    Eigen::Vector2d base(0.0, 0.0);
    Eigen::Vector2d joint(root_link_length_ * std::cos(angles[0]),
                          root_link_length_ * std::sin(angles[0]));
    Eigen::Vector2d end(joint.x() + end_link_length_ * std::cos(angles[0] + angles[1]),
                        joint.y() + end_link_length_ * std::sin(angles[0] + angles[1]));

    // Check link1 collisions
    for (const auto& wall : walls_)
    {
      if (segmentsIntersect(base, joint, wall.first, wall.second))
        return true;
      if (segmentsIntersect(joint, end, wall.first, wall.second))
        return true;
    }
    return false;
  }

  void visualize(const Eigen::Vector2d& angles) const
  {
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    // Draw wall segments (white lines)
    for (const auto& wall : walls_)
    {
      visualization_msgs::msg::Marker wall_marker;
      wall_marker.header.frame_id = "world";
      wall_marker.header.stamp = node_->now();
      wall_marker.ns = "walls";
      wall_marker.id = id++;
      wall_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      wall_marker.action = visualization_msgs::msg::Marker::ADD;
      wall_marker.scale.x = 0.02;
      wall_marker.color.r = 0.0;
      wall_marker.color.g = 0.0;
      wall_marker.color.b = 1.0;
      wall_marker.color.a = 1.0;

      geometry_msgs::msg::Point p1, p2;
      p1.x = wall.first.x();
      p1.y = wall.first.y();
      p2.x = wall.second.x();
      p2.y = wall.second.y();
      wall_marker.points.push_back(p1);
      wall_marker.points.push_back(p2);

      markers.markers.push_back(wall_marker);
    }

    // Compute link geometry
    Eigen::Vector2d base(0.0, 0.0);
    Eigen::Vector2d joint(root_link_length_ * std::cos(angles[0]),
                          root_link_length_ * std::sin(angles[0]));
    Eigen::Vector2d end(joint.x() + end_link_length_ * std::cos(angles[0] + angles[1]),
                        joint.y() + end_link_length_ * std::sin(angles[0] + angles[1]));

    bool collision = isInCollision(angles);

    // Draw manipulator links (green or red)
    visualization_msgs::msg::Marker links;
    links.header.frame_id = "world";
    links.header.stamp = node_->now();
    links.ns = "manipulator";
    links.id = id++;
    links.type = visualization_msgs::msg::Marker::LINE_LIST;
    links.action = visualization_msgs::msg::Marker::ADD;
    links.scale.x = 0.04;
    if (collision)
    {
      links.color.r = 1.0;
      links.color.g = 0.0;
      links.color.b = 0.0;
    }
    else
    {
      links.color.r = 0.0;
      links.color.g = 1.0;
      links.color.b = 0.0;
    }
    links.color.a = 1.0;

    geometry_msgs::msg::Point p0, p1, p2;
    p0.x = base.x();  p0.y = base.y();
    p1.x = joint.x(); p1.y = joint.y();
    p2.x = end.x();   p2.y = end.y();

    links.points.push_back(p0); links.points.push_back(p1);
    links.points.push_back(p1); links.points.push_back(p2);

    markers.markers.push_back(links);

    // // Draw point in configuration space (yellow sphere)
    // visualization_msgs::msg::Marker conf_space_point;
    // const float angles_root_norm = angles::normalize_angle(angles[0]) / (2 * M_PI) * configuration_space_size_;
    // const float angles_end_norm = angles::normalize_angle(angles[1]) / (2 * M_PI) * configuration_space_size_;

    // conf_space_point.header.frame_id = "world";
    // conf_space_point.header.stamp = node_->now();
    // conf_space_point.ns = "conf_space_point";
    // conf_space_point.id = id++;
    // conf_space_point.type = visualization_msgs::msg::Marker::SPHERE;
    // conf_space_point.action = visualization_msgs::msg::Marker::ADD;
    // conf_space_point.scale.x = 0.1;
    // conf_space_point.scale.y = 0.1;
    // conf_space_point.scale.z = 0.1;
    // conf_space_point.color.r = 1.0;
    // conf_space_point.color.g = 1.0;
    // conf_space_point.color.b = 0.0;
    // conf_space_point.color.a = 1.0;
    // conf_space_point.pose.position.x = angles_root_norm;
    // conf_space_point.pose.position.y = angles_end_norm;
    // conf_space_point.pose.position.z = 0.0;
    // markers.markers.push_back(conf_space_point);

    marker_pub_->publish(markers);
  }

  Eigen::Vector2d getEndPose(const Eigen::Vector2d& angles) const
  {
    Eigen::Vector2d root_end;
    root_end << root_link_length_ * std::cos(angles[0]),
                root_link_length_ * std::sin(angles[0]);
    Eigen::Vector2d end_pose = root_end + Eigen::Vector2d(
                                  end_link_length_ * std::cos(angles[0] + angles[1]),
                                  end_link_length_ * std::sin(angles[0] + angles[1]));
    return end_pose;
  }

  // Normally 2 solutions for 2-link planar arm, none if out of reach
  std::vector<Eigen::Vector2d> getJointsFromIk(const Eigen::Vector2d& end_pose) const
  {
    double x = end_pose.x();
    double y = end_pose.y();
    double L1 = root_link_length_;
    double L2 = end_link_length_;
    double dist2 = x * x + y * y;

    // Check reachability: must satisfy triangle inequality
    if (dist2 > (L1 + L2) * (L1 + L2) || dist2 < (L1 - L2) * (L1 - L2))
      return {};  // unreachable target

    double D = (dist2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    D = std::max(-1.0, std::min(1.0, D));  // numerical safety

    double theta2a = std::acos(D);
    double theta2b = -std::acos(D);

    double theta1a = std::atan2(y, x) - std::atan2(L2 * std::sin(theta2a), L1 + L2 * std::cos(theta2a));
    double theta1b = std::atan2(y, x) - std::atan2(L2 * std::sin(theta2b), L1 + L2 * std::cos(theta2b));

    return {Eigen::Vector2d(theta1a, theta2a), Eigen::Vector2d(theta1b, theta2b)};
  }


  nav_msgs::msg::OccupancyGrid getOccupancyGrid(size_t n_divs = 1000) const
  {
    // configuration_space_size_ defines the total width/height of the grid in meters (for visualization),
    // but internally we still sample angles in [-pi, pi].
    const double resolution = configuration_space_size_ / static_cast<double>(n_divs);

    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = static_cast<uint32_t>(n_divs);
    grid.info.height = static_cast<uint32_t>(n_divs);

    // Origin at bottom-left corner in visualization space
    grid.info.origin.position.x = -configuration_space_size_ / 2.0;
    grid.info.origin.position.y = -configuration_space_size_ / 2.0;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(grid.info.width * grid.info.height, 0);

    // Angular step corresponding to one grid cell
    const double angle_per_cell = (2.0 * M_PI) / static_cast<double>(n_divs);

    // Iterate rows (theta2) and columns (theta1)
    for (size_t row = 0; row < n_divs; ++row)
    {
      double theta2 = -M_PI + (static_cast<double>(row) + 0.5) * angle_per_cell;

      for (size_t col = 0; col < n_divs; ++col)
      {
        double theta1 = -M_PI + (static_cast<double>(col) + 0.5) * angle_per_cell;
        Eigen::Vector2d angles(theta1, theta2);

        if (isInCollision(angles))
        {
          grid.data[row * grid.info.width + col] = 100;  // occupied
        }
      }
    }

    return grid;
  }

  double getConfigurationSpaceSize() const
  {
    return configuration_space_size_;
  }


private:
  static bool segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                const Eigen::Vector2d& q1, const Eigen::Vector2d& q2)
  {
    auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      return a.x() * b.y() - a.y() * b.x();
    };

    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;
    double denom = cross(r, s);
    if (std::fabs(denom) < 1e-9)
      return false; // Parallel or collinear

    double t = cross(q1 - p1, s) / denom;
    double u = cross(q1 - p1, r) / denom;
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> walls_;

  // Parameters
  double root_link_length_;
  double end_link_length_;
  double dist_from_obstacle_;
  double bin_depth_;
  double bin_width_;
  double configuration_space_size_;
};

}  // namespace motion_planning

#endif  // MOTION_PLANNING__CROWDED_SCENE_HPP
