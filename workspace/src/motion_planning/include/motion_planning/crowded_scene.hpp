/**
 * @file crowded_scene.hpp
 * @brief 2-D two-link planar manipulator in a "bin" environment.
 *
 * This header defines motion_planning::CrowdedScene, a ROS 2 helper class
 * that models a two-link planar robotic arm operating inside a U-shaped
 * bin.  It is used to:
 *
 *   - Compute the 2-D configuration-space (C-space) occupancy grid, which
 *     maps (θ1, θ2) ∈ [-π, π]² to occupied/free.
 *   - Perform forward kinematics (joint angles → end-effector position).
 *   - Perform analytical inverse kinematics (end-effector → joint angles).
 *   - Detect collisions between the manipulator links and the bin walls.
 *   - Visualise the scene (walls + current arm configuration) via RViz
 *     MarkerArray messages.
 *
 * ### Coordinate frames
 * - The arm base is fixed at the world origin (0, 0).
 * - The bin opening faces upward; its walls are offset downward by
 *   `dist_from_obstacle` to add a safety margin.
 *
 * ### ROS 2 topics published
 * | Topic                   | Type                              | Description          |
 * |-------------------------|-----------------------------------|----------------------|
 * | `crowded_scene_markers` | `visualization_msgs/MarkerArray`  | Scene visualisation  |
 */

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

/**
 * @brief Models a two-link planar arm inside a U-shaped bin and provides
 *        kinematics, collision checking and RViz visualisation.
 *
 * All geometry is 2-D (XY plane).  The arm consists of a *root link* of
 * length L1 attached at the world origin, followed by an *end link* of
 * length L2.  The bin is modelled as four line segments (left wall, right
 * wall, outer-left wall and bottom wall).
 *
 * ### ROS 2 parameters
 * | Parameter                | Type   | Default | Description                                     |
 * |--------------------------|--------|---------|-------------------------------------------------|
 * | root_link_length         | double | 1.0     | Length of the first arm link (m).               |
 * | end_link_length          | double | 1.0     | Length of the second arm link (m).              |
 * | dist_from_obstacle       | double | 0.5     | Safety margin subtracted from wall y-coords.   |
 * | bin_depth                | double | 1.0     | Depth of the bin in the Y direction (m).        |
 * | bin_width                | double | 1.0     | Half-width of the bin opening (m).              |
 * | configuration_space_size | double | 2.0     | Side length of the C-space grid in metres.      |
 */
class CrowdedScene
{
public:
  /**
   * @brief Construct the scene and initialise all ROS 2 resources.
   *
   * Reads parameters from @p node, builds the bin wall geometry, and
   * creates the `crowded_scene_markers` publisher.
   *
   * @param node Shared pointer to an already-created rclcpp::Node used for
   *             parameter access and publisher creation.
   */
  explicit CrowdedScene(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    // ---- Declare and read ROS 2 parameters ----
    node_->declare_parameter("root_link_length", 1.0);
    node_->declare_parameter("end_link_length", 1.0);
    node_->declare_parameter("dist_from_obstacle", 0.5);
    node_->declare_parameter("bin_depth", 1.0);
    node_->declare_parameter("bin_width", 1.0);
    node_->declare_parameter("configuration_space_size", 2.0);

    node_->get_parameter("root_link_length",         root_link_length_);
    node_->get_parameter("end_link_length",          end_link_length_);
    node_->get_parameter("dist_from_obstacle",       dist_from_obstacle_);
    node_->get_parameter("bin_depth",                bin_depth_);
    node_->get_parameter("bin_width",                bin_width_);
    node_->get_parameter("configuration_space_size", configuration_space_size_);

    // ---- Build the bin wall segments ----
    // The bin is U-shaped with an open top.  Each wall is stored as a pair
    // of 2-D endpoints (start, end).  Walls are then shifted downward by
    // dist_from_obstacle_ to add a safety margin around the structure.
    walls_.push_back({Eigen::Vector2d( bin_width_, 0.0),  Eigen::Vector2d( bin_width_, -bin_depth_)}); // right wall
    walls_.push_back({Eigen::Vector2d( 0.0,        0.0),  Eigen::Vector2d( 0.0,        -bin_depth_)}); // inner divider
    walls_.push_back({Eigen::Vector2d(-bin_width_, 0.0),  Eigen::Vector2d(-bin_width_, -bin_depth_)}); // left  wall
    walls_.push_back({Eigen::Vector2d( bin_width_, -bin_depth_), Eigen::Vector2d(-bin_width_, -bin_depth_)}); // bottom

    // Apply safety margin: shift every wall downward.
    for (auto& wall : walls_)
    {
      wall.first.y()  -= dist_from_obstacle_;
      wall.second.y() -= dist_from_obstacle_;
    }

    // ---- Publisher ----
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "crowded_scene_markers", 1);
  }

  /**
   * @brief Check whether a given arm configuration collides with the bin.
   *
   * Forward-kinematics is computed from @p angles and each arm link is
   * tested for intersection against every bin wall segment.
   *
   * @param angles 2-D vector (θ1, θ2) of joint angles in radians.
   * @return `true` if any link intersects a wall segment, `false` otherwise.
   */
  bool isInCollision(const Eigen::Vector2d& angles) const
  {
    // ---- Forward kinematics ----
    Eigen::Vector2d base(0.0, 0.0);
    Eigen::Vector2d joint(
      root_link_length_ * std::cos(angles[0]),
      root_link_length_ * std::sin(angles[0]));
    Eigen::Vector2d end(
      joint.x() + end_link_length_ * std::cos(angles[0] + angles[1]),
      joint.y() + end_link_length_ * std::sin(angles[0] + angles[1]));

    // ---- Collision check for both links against all walls ----
    for (const auto& wall : walls_)
    {
      if (segmentsIntersect(base,  joint, wall.first, wall.second)) return true;
      if (segmentsIntersect(joint, end,   wall.first, wall.second)) return true;
    }
    return false;
  }

  /**
   * @brief Publish RViz markers for the current arm pose and the bin walls.
   *
   * Publishes a `visualization_msgs::msg::MarkerArray` to the
   * `crowded_scene_markers` topic containing:
   *   - LINE_LIST markers for every bin wall (blue).
   *   - A LINE_LIST marker for the two arm links (green = collision-free,
   *     red = in collision).
   *
   * @param angles 2-D vector (θ1, θ2) of joint angles in radians.
   */
  void visualize(const Eigen::Vector2d& angles) const
  {
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    // ---- Draw bin walls as blue line segments ----
    for (const auto& wall : walls_)
    {
      visualization_msgs::msg::Marker wall_marker;
      wall_marker.header.frame_id = "world";
      wall_marker.header.stamp    = node_->now();
      wall_marker.ns    = "walls";
      wall_marker.id    = id++;
      wall_marker.type  = visualization_msgs::msg::Marker::LINE_LIST;
      wall_marker.action = visualization_msgs::msg::Marker::ADD;
      wall_marker.scale.x = 0.02;   // line width
      wall_marker.color.r = 0.0;
      wall_marker.color.g = 0.0;
      wall_marker.color.b = 1.0;    // blue
      wall_marker.color.a = 1.0;

      geometry_msgs::msg::Point p1, p2;
      p1.x = wall.first.x();  p1.y = wall.first.y();
      p2.x = wall.second.x(); p2.y = wall.second.y();
      wall_marker.points.push_back(p1);
      wall_marker.points.push_back(p2);

      markers.markers.push_back(wall_marker);
    }

    // ---- Compute arm link endpoints via forward kinematics ----
    Eigen::Vector2d base(0.0, 0.0);
    Eigen::Vector2d joint(
      root_link_length_ * std::cos(angles[0]),
      root_link_length_ * std::sin(angles[0]));
    Eigen::Vector2d end(
      joint.x() + end_link_length_ * std::cos(angles[0] + angles[1]),
      joint.y() + end_link_length_ * std::sin(angles[0] + angles[1]));

    bool collision = isInCollision(angles);

    // ---- Draw manipulator links (green = free, red = colliding) ----
    visualization_msgs::msg::Marker links;
    links.header.frame_id = "world";
    links.header.stamp    = node_->now();
    links.ns   = "manipulator";
    links.id   = id++;
    links.type = visualization_msgs::msg::Marker::LINE_LIST;
    links.action = visualization_msgs::msg::Marker::ADD;
    links.scale.x = 0.04;  // line width
    if (collision)
    {
      links.color.r = 1.0; links.color.g = 0.0; links.color.b = 0.0; // red
    }
    else
    {
      links.color.r = 0.0; links.color.g = 1.0; links.color.b = 0.0; // green
    }
    links.color.a = 1.0;

    geometry_msgs::msg::Point p0, p1, p2;
    p0.x = base.x();  p0.y = base.y();
    p1.x = joint.x(); p1.y = joint.y();
    p2.x = end.x();   p2.y = end.y();

    // Link 1: base → joint, Link 2: joint → end.
    links.points.push_back(p0); links.points.push_back(p1);
    links.points.push_back(p1); links.points.push_back(p2);

    markers.markers.push_back(links);

    // NOTE: A configuration-space point marker (yellow sphere) is available
    // in the source but is currently commented out.  Uncomment the block
    // below if simultaneous task-space + C-space visualisation is desired.

    marker_pub_->publish(markers);
  }

  /**
   * @brief Compute the 2-D end-effector position for a given configuration.
   *
   * Applies full forward kinematics: base → joint → end-effector.
   *
   * @param angles 2-D vector (θ1, θ2) of joint angles in radians.
   * @return 2-D end-effector position in the world frame.
   */
  Eigen::Vector2d getEndPose(const Eigen::Vector2d& angles) const
  {
    Eigen::Vector2d root_end;
    root_end << root_link_length_ * std::cos(angles[0]),
                root_link_length_ * std::sin(angles[0]);
    Eigen::Vector2d end_pose =
      root_end + Eigen::Vector2d(
        end_link_length_ * std::cos(angles[0] + angles[1]),
        end_link_length_ * std::sin(angles[0] + angles[1]));
    return end_pose;
  }

  /**
   * @brief Compute the joint angles for a desired end-effector position (IK).
   *
   * Uses the standard closed-form solution for a 2-link planar arm.  There
   * are normally two solutions ("elbow up" and "elbow down").  The function
   * returns an empty vector when the target is outside the reachable workspace
   * (i.e. when the triangle inequality is violated).
   *
   * @param end_pose Desired 2-D end-effector position.
   * @return A vector of up to two solutions, each a 2-D vector (θ1, θ2).
   *         Returns an empty vector when the target is unreachable.
   */
  std::vector<Eigen::Vector2d> getJointsFromIk(const Eigen::Vector2d& end_pose) const
  {
    double x  = end_pose.x();
    double y  = end_pose.y();
    double L1 = root_link_length_;
    double L2 = end_link_length_;
    double dist2 = x * x + y * y;

    // ---- Reachability check ----
    // Target must lie within the annulus [|L1-L2|, L1+L2].
    if (dist2 > (L1 + L2) * (L1 + L2) || dist2 < (L1 - L2) * (L1 - L2))
      return {};  // unreachable

    // ---- Elbow angle (two solutions: elbow-up and elbow-down) ----
    double D = (dist2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    D = std::max(-1.0, std::min(1.0, D));  // clamp for numerical safety

    double theta2a =  std::acos(D);  // elbow-up
    double theta2b = -std::acos(D);  // elbow-down

    // ---- Shoulder angle derived from elbow angle ----
    double theta1a = std::atan2(y, x)
      - std::atan2(L2 * std::sin(theta2a), L1 + L2 * std::cos(theta2a));
    double theta1b = std::atan2(y, x)
      - std::atan2(L2 * std::sin(theta2b), L1 + L2 * std::cos(theta2b));

    return {Eigen::Vector2d(theta1a, theta2a), Eigen::Vector2d(theta1b, theta2b)};
  }

  /**
   * @brief Build and return the configuration-space occupancy grid.
   *
   * Discretises the C-space (θ1, θ2) ∈ [-π, π]² into an n_divs×n_divs grid
   * and marks each cell as occupied (100) or free (0) based on isInCollision().
   *
   * The resulting ROS 2 OccupancyGrid uses `configuration_space_size_` as the
   * total physical width/height (in metres) so it can be rendered at a
   * convenient scale in RViz alongside the task-space scene.
   *
   * @param n_divs Number of cells along each axis (default: 1000).
   * @return Fully populated nav_msgs::msg::OccupancyGrid.
   */
  nav_msgs::msg::OccupancyGrid getOccupancyGrid(size_t n_divs = 1000) const
  {
    // Grid resolution in metres per cell (used for RViz display only).
    const double resolution = configuration_space_size_ / static_cast<double>(n_divs);

    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width      = static_cast<uint32_t>(n_divs);
    grid.info.height     = static_cast<uint32_t>(n_divs);

    // Place the grid origin at the bottom-left of the visualisation window.
    grid.info.origin.position.x  = -configuration_space_size_ / 2.0;
    grid.info.origin.position.y  = -configuration_space_size_ / 2.0;
    grid.info.origin.position.z  =  0.0;
    grid.info.origin.orientation.w = 1.0;

    // Initialise all cells to free (0).
    grid.data.assign(grid.info.width * grid.info.height, 0);

    // Angular step per grid cell (maps one cell to one angle increment).
    const double angle_per_cell = (2.0 * M_PI) / static_cast<double>(n_divs);

    // Iterate over rows (theta2) and columns (theta1).
    for (size_t row = 0; row < n_divs; ++row)
    {
      double theta2 = -M_PI + (static_cast<double>(row) + 0.5) * angle_per_cell;

      for (size_t col = 0; col < n_divs; ++col)
      {
        double theta1 = -M_PI + (static_cast<double>(col) + 0.5) * angle_per_cell;
        Eigen::Vector2d angles(theta1, theta2);

        if (isInCollision(angles))
        {
          // Mark cell as fully occupied.
          grid.data[row * grid.info.width + col] = 100;
        }
      }
    }

    return grid;
  }

  /**
   * @brief Return the side length of the C-space visualisation window.
   *
   * This value is used externally (e.g. in rrt_with_optim.cpp) to convert
   * interactive-marker positions into joint-angle values.
   *
   * @return `configuration_space_size_` parameter value (metres).
   */
  double getConfigurationSpaceSize() const
  {
    return configuration_space_size_;
  }


private:
  /**
   * @brief Test whether two 2-D line segments intersect.
   *
   * Uses the parametric cross-product method: both parameters t and u must
   * lie in [0, 1] for a proper intersection.  Parallel/collinear segments
   * are treated as non-intersecting.
   *
   * @param p1 Start of the first  segment.
   * @param p2 End   of the first  segment.
   * @param q1 Start of the second segment.
   * @param q2 End   of the second segment.
   * @return `true` if the segments properly cross, `false` otherwise.
   */
  static bool segmentsIntersect(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const Eigen::Vector2d& q1, const Eigen::Vector2d& q2)
  {
    // 2-D cross product helper.
    auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      return a.x() * b.y() - a.y() * b.x();
    };

    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;
    double denom = cross(r, s);

    // Parallel or collinear → no intersection.
    if (std::fabs(denom) < 1e-9)
      return false;

    double t = cross(q1 - p1, s) / denom;
    double u = cross(q1 - p1, r) / denom;

    // Both parameters in [0,1] means the segments cross.
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
  }

  rclcpp::Node::SharedPtr node_; ///< ROS 2 node used for publishers and parameters.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; ///< Scene visualisation publisher.

  /// Bin walls, each stored as a (start, end) pair of 2-D points.
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> walls_;

  // ---- Arm / scene parameters (loaded from ROS 2 parameter server) ----
  double root_link_length_;        ///< Length of the first  arm link (m).
  double end_link_length_;         ///< Length of the second arm link (m).
  double dist_from_obstacle_;      ///< Safety margin applied to bin walls (m).
  double bin_depth_;               ///< Depth of the bin in Y (m).
  double bin_width_;               ///< Half-width of the bin opening (m).
  double configuration_space_size_;///< C-space visualisation window side length (m).
};

}  // namespace motion_planning

#endif  // MOTION_PLANNING__CROWDED_SCENE_HPP

