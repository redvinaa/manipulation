#include "motion_planning/crowded_scene.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("crowded_scene_with_marker");

  auto occupancy_pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>("configuration_space_occupancy", 1);

  motion_planning::CrowdedScene crowded_scene(node);
  auto grid = crowded_scene.getOccupancyGrid();
  grid.header.frame_id = "world";
  grid.header.stamp = node->now();
  occupancy_pub->publish(grid);

  // Interactive Marker Server
  auto server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "end_effector_marker", node);

  // Create interactive marker
  InteractiveMarker marker;
  marker.header.frame_id = "world";
  marker.name = "end_effector";
  marker.description = "Drag to move end effector";
  marker.scale = 0.2;

  // Initial position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  // Control for planar (XY) motion
  InteractiveMarkerControl move_control;
  move_control.name = "move_plane";
  move_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  move_control.orientation.w = 1.0;
  move_control.orientation.x = 0.0;
  move_control.orientation.y = 1.0;
  move_control.orientation.z = 0.0;
  move_control.orientation_mode = InteractiveMarkerControl::FIXED;

  // Add a simple visual marker (sphere)
  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.scale.x = 0.05;
  sphere.scale.y = 0.05;
  sphere.scale.z = 0.05;
  sphere.color.r = 1.0;
  sphere.color.g = 0.0;
  sphere.color.b = 0.0;
  sphere.color.a = 1.0;
  move_control.markers.push_back(sphere);
  move_control.always_visible = true;
  marker.controls.push_back(move_control);

  server->insert(marker);

  constexpr bool CONTROL_CONFIG_SPACE = true;

  // Handle marker feedback
  server->setCallback(
    "end_effector",
    [&crowded_scene, node](const InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
      if (CONTROL_CONFIG_SPACE) {
        const double theta1 = feedback->pose.position.x / (crowded_scene.getConfigurationSpaceSize()) * (2.0 * M_PI);
        const double theta2 = feedback->pose.position.y / (crowded_scene.getConfigurationSpaceSize()) * (2.0 * M_PI);
        const Eigen::Vector2d angles(theta1, theta2);
        crowded_scene.visualize(angles);
      } else {
        if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
        {
          const double x = feedback->pose.position.x;
          const double y = feedback->pose.position.y;
          const auto joints_v = crowded_scene.getJointsFromIk(Eigen::Vector2d(x, y));

          if (joints_v.empty()) {
            RCLCPP_WARN(
              node->get_logger(),
              "No IK solution found for end-effector pose (%.2f, %.2f)", x, y);
          } else {
            if (crowded_scene.isInCollision(joints_v[0])) {
              crowded_scene.visualize(joints_v[1]);
            } else {
              crowded_scene.visualize(joints_v[0]);
            }
          }
        }
      }
    });

  server->applyChanges();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
