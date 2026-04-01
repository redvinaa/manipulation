/**
 * @file cspace_explorer.cpp
 * @brief ROS 2 node for interactive C-space visualisation of a two-link planar arm.
 *
 * This node visualises the configuration space (C-space) of a two-link
 * planar manipulator operating inside a U-shaped bin (see
 * motion_planning::CrowdedScene).  It publishes the precomputed C-space
 * occupancy grid once at start-up and then lets the user drag an interactive
 * marker to explore arm configurations in real time.
 *
 * The marker XY position is linearly mapped to (θ1, θ2) and the arm is
 * visualised directly in the bin scene.
 *
 * ### ROS 2 node name
 * `cspace_explorer`
 *
 * ### Published topics
 * | Topic                          | Type                           | Description                      |
 * |--------------------------------|--------------------------------|----------------------------------|
 * | `configuration_space_occupancy`| `nav_msgs/OccupancyGrid`       | C-space collision map (once)     |
 * | `crowded_scene_markers`        | `visualization_msgs/MarkerArray`| Scene + arm visualisation        |
 *
 * ### Interactive marker server
 * | Server name          | Marker name    | Description                      |
 * |----------------------|----------------|----------------------------------|
 * | `end_effector_marker`| `end_effector` | Draggable sphere in XY plane     |
 */

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

  // ---- Publish the C-space occupancy grid once at start-up ----
  // Transient-local durability ensures late-joining subscribers (e.g. RViz)
  // receive the message even after it was published.
  rclcpp::QoS map_qos(1);
  map_qos.transient_local();
  auto occupancy_pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>("configuration_space_occupancy", map_qos);

  motion_planning::CrowdedScene crowded_scene(node);
  auto grid = crowded_scene.getOccupancyGrid();
  grid.header.frame_id = "world";
  grid.header.stamp    = node->now();
  occupancy_pub->publish(grid);

  // ---- Interactive Marker Server ----
  // The server publishes the marker to RViz and delivers drag feedback
  // to the callback below.
  auto server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "end_effector_marker", node);

  // ---- Create the draggable end-effector marker ----
  InteractiveMarker marker;
  marker.header.frame_id = "world";
  marker.name        = "end_effector";
  marker.description = "Drag to move end effector";
  marker.scale       = 0.2;

  // Initial marker position at the world origin.
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  // ---- MOVE_PLANE control: constrain dragging to the XY plane ----
  InteractiveMarkerControl move_control;
  move_control.name             = "move_plane";
  move_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  move_control.orientation.w    = 1.0;
  move_control.orientation.x    = 0.0;
  move_control.orientation.y    = 1.0;  // normal = world Z
  move_control.orientation.z    = 0.0;
  move_control.orientation_mode = InteractiveMarkerControl::FIXED;

  // ---- Small red sphere as the marker visual ----
  visualization_msgs::msg::Marker sphere;
  sphere.type    = visualization_msgs::msg::Marker::SPHERE;
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

  // ---- Exploration mode switch ----
  // Set to true  to control the arm directly in C-space.
  // Set to false to control the end-effector position (task space + IK).
  constexpr bool CONTROL_CONFIG_SPACE = true;

  // ---- Feedback callback: update arm visualisation on every drag event ----
  server->setCallback(
    "end_effector",
    [&crowded_scene, node](const InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
      if (CONTROL_CONFIG_SPACE)
      {
        // ---- C-space mode ----
        // Map the marker XY position linearly to (theta1, theta2) in [-pi, pi].
        const double theta1 = feedback->pose.position.x
          / crowded_scene.getConfigurationSpaceSize() * (2.0 * M_PI);
        const double theta2 = feedback->pose.position.y
          / crowded_scene.getConfigurationSpaceSize() * (2.0 * M_PI);
        const Eigen::Vector2d angles(theta1, theta2);
        crowded_scene.visualize(angles);
      }
      else
      {
        // ---- Task-space mode ----
        // Treat the marker position as a desired end-effector target and
        // solve IK.  Use the collision-free solution where possible.
        if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
        {
          const double x = feedback->pose.position.x;
          const double y = feedback->pose.position.y;
          const auto joints_v = crowded_scene.getJointsFromIk(Eigen::Vector2d(x, y));

          if (joints_v.empty())
          {
            // Target outside the reachable workspace.
            RCLCPP_WARN(
              node->get_logger(),
              "No IK solution found for end-effector pose (%.2f, %.2f)", x, y);
          }
          else
          {
            // Prefer the first (elbow-up) solution; fall back to elbow-down
            // if elbow-up causes a collision.
            if (crowded_scene.isInCollision(joints_v[0]))
              crowded_scene.visualize(joints_v[1]);
            else
              crowded_scene.visualize(joints_v[0]);
          }
        }
      }
    });

  server->applyChanges();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
