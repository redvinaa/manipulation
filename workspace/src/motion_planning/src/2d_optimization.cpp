// traj_planning_2d_node.cpp

#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "interactive_markers/interactive_marker_server.hpp"

#include "motion_planning/2d_optimization.hpp"  // your header

using namespace std::chrono_literals;

class TrajPlanning2DNode : public rclcpp::Node
{
public:
  TrajPlanning2DNode()
  : Node("traj_planning_2d_node"),
    marker_server_("traj_planning_markers", this)
  {
    this->declare_parameter<int>("n_segments", 10);
    this->declare_parameter<double>("obstacle_radius", 0.5);
    this->declare_parameter<int>("n_obstacles", 2);

    int n_segments_int;
    this->get_parameter("n_segments", n_segments_int);
    n_segments_ = static_cast<size_t>(n_segments_int);
    this->get_parameter("obstacle_radius", obstacle_radius_);
    int n_obstacles = this->get_parameter("n_obstacles").as_int();

    traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);

    createInteractiveMarkerEndpoint("start_marker", Eigen::Vector2d(-1.0, -1.0));
    createInteractiveMarkerEndpoint("goal_marker",  Eigen::Vector2d(1.0, 1.0));

    for (int i = 0; i < n_obstacles; ++i) {
      double angle = i * (2.0 * M_PI / n_obstacles);
      Eigen::Vector2d pos(0.5 * std::cos(angle), 0.5 * std::sin(angle));
      createInteractiveMarkerObstacle("obstacle_marker_" + std::to_string(i), pos, obstacle_radius_);
      obstacles_.push_back(pos);
    }
  }

private:
  void createInteractiveMarkerEndpoint(const std::string & name, Eigen::Vector2d init_xy)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = this->now();
    int_marker.name = name;
    int_marker.description = name;
    int_marker.pose.position.x = init_xy.x();
    int_marker.pose.position.y = init_xy.y();
    int_marker.pose.position.z = 0.0;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_xy";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    marker_server_.insert(int_marker,
      [this, name](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
      {
        this->interactiveFeedback(name, feedback);
      });
    marker_server_.applyChanges();
  }

  void createInteractiveMarkerObstacle(const std::string & name, Eigen::Vector2d init_xy, double radius)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = this->now();
    int_marker.name = name;
    int_marker.description = name;
    int_marker.pose.position.x = init_xy.x();
    int_marker.pose.position.y = init_xy.y();
    int_marker.pose.position.z = 0.0;

    visualization_msgs::msg::InteractiveMarkerControl sphere_control;
    sphere_control.orientation.w = 1;
    sphere_control.orientation.x = 0;
    sphere_control.orientation.y = 1;
    sphere_control.orientation.z = 0;
    sphere_control.always_visible = true;
    sphere_control.markers.resize(1);
    visualization_msgs::msg::Marker & m = sphere_control.markers[0];
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "obstacle_sphere";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = init_xy.x();
    m.pose.position.y = init_xy.y();
    m.pose.position.z = 0;
    m.scale.x = radius * 2;
    m.scale.y = radius * 2;
    m.scale.z = 0.1;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;

    sphere_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(sphere_control);

    marker_server_.insert(int_marker,
      [this, name, radius](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
      {
        this->interactiveFeedback(name, feedback);
      });
    marker_server_.applyChanges();
  }

  void interactiveFeedback(const std::string & name,
                           const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (name == "start_marker") {
      start_.x() = feedback->pose.position.x;
      start_.y() = feedback->pose.position.y;
    }
    else if (name == "goal_marker") {
      goal_.x() = feedback->pose.position.x;
      goal_.y() = feedback->pose.position.y;
    }
    else {
      // Update corresponding obstacle position
      for (size_t i = 0; i < obstacles_.size(); ++i) {
        if (name == "obstacle_marker_" + std::to_string(i)) {
          obstacles_[i].x() = feedback->pose.position.x;
          obstacles_[i].y() = feedback->pose.position.y;
          break;
        }
      }
    }

    planAndPublishTrajectory();
  }

  void planAndPublishTrajectory()
  {
    static Eigen::MatrixXd previous_traj = Eigen::MatrixXd::Zero(2, n_segments_);
    const auto & initial_guess = previous_traj;

    auto result = traj_planning_2d::plan2DTrajectory(
      start_, goal_, obstacles_, obstacle_radius_, n_segments_, initial_guess);

    if (!result) {
      RCLCPP_WARN(this->get_logger(), "Trajectory planner returned no solution");
      return;
    }
    Eigen::MatrixXd traj = *result;
    previous_traj = traj;

    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = this->now();
    line.ns = "trajectory_line";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.02;
    line.color.r = 0.0f;
    line.color.g = 0.0f;
    line.color.b = 1.0f;
    line.color.a = 1.0f;

    for (size_t i = 0; i < traj.cols(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = traj(0, i);
      p.y = traj(1, i);
      p.z = 0.0;
      line.points.push_back(p);
    }
    array.markers.push_back(line);

    visualization_msgs::msg::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = this->now();
    points.ns = "trajectory_points";
    points.id = 1;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0.05;
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0f;

    for (size_t i = 0; i < traj.cols(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = traj(0, i);
      p.y = traj(1, i);
      p.z = 0.0;
      points.points.push_back(p);
    }
    array.markers.push_back(points);

    traj_pub_->publish(array);
  }

  size_t n_segments_{20};
  double obstacle_radius_{0.2};
  Eigen::Vector2d start_{0.0, 0.0};
  Eigen::Vector2d goal_{1.0, 1.0};
  std::vector<Eigen::Vector2d> obstacles_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
  interactive_markers::InteractiveMarkerServer marker_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajPlanning2DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
