/**
 * @file simple_rrt_demo.cpp
 * @brief ROS 2 node that runs and visualises RRT / RRT* in a 2-D square world
 *        using OpenCV for rendering.
 *
 * ### World
 * A square axis-aligned space [0, world_size] × [0, world_size].
 * Obstacles are straight wall segments defined by their two endpoints.
 *
 * ### Algorithm
 * Controlled by the `rrt_star` boolean parameter:
 *   - false → plain RRT   (extend + connect)
 *   - true  → RRT*        (extend + connect + rewire within `rewire_radius`)
 *
 * ### Visualisation
 * An OpenCV window is updated after every `step_delay_ms` milliseconds.
 * - Grey   background
 * - Black  wall obstacles
 * - Blue   RRT tree edges
 * - Dark green  tree nodes (small dots)
 * - Orange start point
 * - Red    goal  point
 * - Bright green  final path (drawn once goal is reached)
 *
 * ### ROS 2 parameters
 * | Parameter        | Type            | Default                     | Description                                         |
 * |------------------|-----------------|-----------------------------|-----------------------------------------------------|
 * | world_size       | double          | 10.0                        | Side length of the square world (m)                 |
 * | start_x          | double          | 0.5                         | Start X                                             |
 * | start_y          | double          | 0.5                         | Start Y                                             |
 * | goal_x           | double          | 9.5                         | Goal X                                              |
 * | goal_y           | double          | 9.5                         | Goal Y                                              |
 * | max_step         | double          | 0.5                         | Maximum extension step length                       |
 * | goal_bias        | double          | 0.1                         | Probability [0,1] of sampling the goal directly     |
 * | goal_threshold   | double          | 0.4                         | Distance at which the goal is considered reached    |
 * | rrt_star         | bool            | false                       | Use RRT* rewiring instead of plain RRT              |
 * | rewire_radius    | double          | 1.5                         | Neighbourhood radius for RRT* rewiring              |
 * | image_size       | int             | 800                         | OpenCV window pixel size (square)                   |
 * | step_delay_ms    | int             | 10                          | Milliseconds between visualisation updates          |
 * | walls            | double[]        | (see below)                 | Flat list [x1,y1,x2,y2, ...] of wall endpoints     |
 *
 * Default obstacles form a simple maze-like layout inside a 10×10 world.
 */

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <vector>

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Wall
{
  Point2D a;  ///< First  endpoint.
  Point2D b;  ///< Second endpoint.
};

struct Node
{
  Point2D  pos;
  int      parent{-1};   ///< Index into nodes vector; -1 for root.
  double   cost{0.0};    ///< Cost-from-root (used by RRT*).
};

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

static double dist(const Point2D& p, const Point2D& q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return std::sqrt(dx * dx + dy * dy);
}

/// 2-D cross product of vectors (b-a) and (c-a).
static double cross2d(const Point2D& a, const Point2D& b, const Point2D& c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/// True if segment [p1,p2] properly intersects segment [q1,q2].
static bool segmentsIntersect(
  const Point2D& p1, const Point2D& p2,
  const Point2D& q1, const Point2D& q2)
{
  const double d1 = cross2d(q1, q2, p1);
  const double d2 = cross2d(q1, q2, p2);
  const double d3 = cross2d(p1, p2, q1);
  const double d4 = cross2d(p1, p2, q2);

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
      ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
  {
    return true;
  }
  return false;
}

/// True if the segment [from, to] collides with any wall.
static bool collides(
  const Point2D& from, const Point2D& to,
  const std::vector<Wall>& walls)
{
  for (const auto& w : walls)
  {
    if (segmentsIntersect(from, to, w.a, w.b))
      return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------

static cv::Point toCv(const Point2D& p, double world_size, int img_size)
{
  // World → pixel: Y is flipped so that +Y points up on screen.
  const int px = static_cast<int>(p.x / world_size * img_size);
  const int py = static_cast<int>((1.0 - p.y / world_size) * img_size);
  return {px, py};
}

static void drawScene(
  cv::Mat& canvas,
  const std::vector<Wall>& walls,
  const std::vector<Node>& nodes,
  const Point2D& start, const Point2D& goal,
  double world_size, int img_size,
  const std::vector<int>& path_indices)   ///< Empty until goal is reached.
{
  // Background
  canvas.setTo(cv::Scalar(230, 230, 230));

  // --- Walls (black, thick) ---
  for (const auto& w : walls)
  {
    cv::line(
      canvas,
      toCv(w.a, world_size, img_size),
      toCv(w.b, world_size, img_size),
      cv::Scalar(20, 20, 20), 3, cv::LINE_AA);
  }

  // --- Tree edges (blue) ---
  for (const auto& node : nodes)
  {
    if (node.parent < 0) continue;
    cv::line(
      canvas,
      toCv(nodes[node.parent].pos, world_size, img_size),
      toCv(node.pos, world_size, img_size),
      cv::Scalar(200, 120, 40), 1, cv::LINE_AA);  // BGR → orange-ish blue = steel blue
  }

  // --- Tree nodes (small dark-green dots) ---
  for (const auto& node : nodes)
  {
    cv::circle(
      canvas,
      toCv(node.pos, world_size, img_size),
      2, cv::Scalar(40, 100, 40), cv::FILLED, cv::LINE_AA);
  }

  // --- Final path (bright green, thick) ---
  for (std::size_t i = 1; i < path_indices.size(); ++i)
  {
    cv::line(
      canvas,
      toCv(nodes[path_indices[i - 1]].pos, world_size, img_size),
      toCv(nodes[path_indices[i]].pos,     world_size, img_size),
      cv::Scalar(0, 220, 0), 3, cv::LINE_AA);
  }

  // --- Start (orange circle) ---
  cv::circle(
    canvas,
    toCv(start, world_size, img_size),
    8, cv::Scalar(0, 140, 255), cv::FILLED, cv::LINE_AA);
  cv::circle(
    canvas,
    toCv(start, world_size, img_size),
    8, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

  // --- Goal (red circle) ---
  cv::circle(
    canvas,
    toCv(goal, world_size, img_size),
    8, cv::Scalar(0, 0, 220), cv::FILLED, cv::LINE_AA);
  cv::circle(
    canvas,
    toCv(goal, world_size, img_size),
    8, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

// ---------------------------------------------------------------------------
// RRT / RRT* step logic
// ---------------------------------------------------------------------------

/// Return the index of the node closest to `sample`.
static int nearestNode(const std::vector<Node>& nodes, const Point2D& sample)
{
  int    best_idx  = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
  {
    const double d = dist(nodes[i].pos, sample);
    if (d < best_dist) { best_dist = d; best_idx = i; }
  }
  return best_idx;
}

/// Steer from `from` toward `to` by at most `max_step`. Returns new point.
static Point2D steer(const Point2D& from, const Point2D& to, double max_step)
{
  const double d = dist(from, to);
  if (d <= max_step) return to;
  const double ratio = max_step / d;
  return {from.x + ratio * (to.x - from.x),
          from.y + ratio * (to.y - from.y)};
}

/// Collect indices of all nodes within `radius` of `point`.
static std::vector<int> nearNeighbours(
  const std::vector<Node>& nodes, const Point2D& point, double radius)
{
  std::vector<int> result;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
  {
    if (dist(nodes[i].pos, point) <= radius)
      result.push_back(i);
  }
  return result;
}

/// Extract path from root to node at `goal_idx` as ordered index list.
static std::vector<int> extractPath(const std::vector<Node>& nodes, int goal_idx)
{
  std::vector<int> path;
  for (int idx = goal_idx; idx >= 0; idx = nodes[idx].parent)
    path.push_back(idx);
  std::reverse(path.begin(), path.end());
  return path;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simple_rrt_demo");

  // ---- Declare parameters ----
  node->declare_parameter("world_size",     10.0);
  node->declare_parameter("start_x",         0.5);
  node->declare_parameter("start_y",         0.5);
  node->declare_parameter("goal_x",          9.5);
  node->declare_parameter("goal_y",          9.5);
  node->declare_parameter("max_step",        0.5);
  node->declare_parameter("goal_bias",       0.1);
  node->declare_parameter("goal_threshold",  0.4);
  node->declare_parameter("rrt_star",       false);
  node->declare_parameter("rewire_radius",   1.5);
  node->declare_parameter("image_size",      800);
  node->declare_parameter("step_delay_ms",   10);

  // Default obstacle layout: a simple zig-zag maze inside a 10×10 world.
  // Format: [x1, y1, x2, y2,  x1, y1, x2, y2, ...]
  node->declare_parameter(
    "walls",
    std::vector<double>{
      // Horizontal wall at y≈3, from x=0 to x=7
      0.0, 3.0,  7.0, 3.0,
      // Horizontal wall at y≈6, from x=3 to x=10
      3.0, 6.0,  10.0, 6.0,
      // Short vertical blocker at x≈5, y=3..6
      5.0, 4.0,  5.0, 5.0,
    });

  // ---- Read parameters ----
  const double world_size    = node->get_parameter("world_size").as_double();
  const double start_x       = node->get_parameter("start_x").as_double();
  const double start_y       = node->get_parameter("start_y").as_double();
  const double goal_x        = node->get_parameter("goal_x").as_double();
  const double goal_y        = node->get_parameter("goal_y").as_double();
  const double max_step      = node->get_parameter("max_step").as_double();
  const double goal_bias     = node->get_parameter("goal_bias").as_double();
  const double goal_threshold= node->get_parameter("goal_threshold").as_double();
  const bool   use_rrt_star  = node->get_parameter("rrt_star").as_bool();
  const double rewire_radius = node->get_parameter("rewire_radius").as_double();
  const int    image_size    = node->get_parameter("image_size").as_int();
  const int    step_delay_ms = node->get_parameter("step_delay_ms").as_int();

  const std::vector<double> walls_coords =
    node->get_parameter("walls").as_double_array();

  // ---- Build wall list ----
  std::vector<Wall> walls;
  if (walls_coords.size() % 4 != 0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "walls must have a multiple of 4 elements (x1,y1,x2,y2 per wall). "
      "Got %zu — ignoring obstacles.", walls_coords.size());
  }
  else
  {
    for (std::size_t i = 0; i + 3 < walls_coords.size(); i += 4)
    {
      walls.push_back({{walls_coords[i], walls_coords[i + 1]},
                       {walls_coords[i + 2], walls_coords[i + 3]}});
    }
  }

  RCLCPP_INFO(
    node->get_logger(),
    "simple_rrt_demo: %s | world=%.1f | start=(%.2f,%.2f) | goal=(%.2f,%.2f) | "
    "%zu wall(s) | step=%.2f | goal_bias=%.2f",
    use_rrt_star ? "RRT*" : "RRT",
    world_size, start_x, start_y, goal_x, goal_y,
    walls.size(), max_step, goal_bias);

  const Point2D start{start_x, start_y};
  const Point2D goal {goal_x,  goal_y};

  // ---- Initialise tree ----
  std::vector<Node> nodes;
  nodes.push_back({start, -1, 0.0});

  // ---- RNG ----
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> space_dist(0.0, world_size);
  std::uniform_real_distribution<double> bias_dist(0.0, 1.0);

  // ---- OpenCV window ----
  const std::string win_name =
    std::string("simple_rrt_demo — ") + (use_rrt_star ? "RRT*" : "RRT");
  cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
  cv::Mat canvas(image_size, image_size, CV_8UC3);

  std::vector<int> final_path;
  bool goal_reached = false;
  int  goal_node_idx = -1;

  // ---- Single RRT/RRT* step: sample → steer → add → (rewire) → goal check ----
  // Returns true if a new node was added to the tree.
  auto stepOnce = [&]() -> bool
  {
    // 1. Sample
    const Point2D sample = (bias_dist(rng) < goal_bias)
      ? goal
      : Point2D{space_dist(rng), space_dist(rng)};

    // 2. Nearest node
    const int      nearest_idx = nearestNode(nodes, sample);
    const Point2D& nearest_pos = nodes[nearest_idx].pos;

    // 3. Steer
    const Point2D new_pos = steer(nearest_pos, sample, max_step);

    // 4. Collision check — discard sample if the edge hits a wall
    if (collides(nearest_pos, new_pos, walls))
      return false;

    // 5. Build candidate node (default parent = nearest)
    Node new_node;
    new_node.pos    = new_pos;
    new_node.parent = nearest_idx;
    new_node.cost   = nodes[nearest_idx].cost + dist(nearest_pos, new_pos);

    // ---- RRT* : choose best parent in neighbourhood ----
    if (use_rrt_star)
    {
      for (int nb_idx : nearNeighbours(nodes, new_pos, rewire_radius))
      {
        const double candidate_cost =
          nodes[nb_idx].cost + dist(nodes[nb_idx].pos, new_pos);
        if (candidate_cost < new_node.cost &&
            !collides(nodes[nb_idx].pos, new_pos, walls))
        {
          new_node.parent = nb_idx;
          new_node.cost   = candidate_cost;
        }
      }
    }

    const int new_idx = static_cast<int>(nodes.size());
    nodes.push_back(new_node);

    // ---- RRT* : rewire neighbours through new node ----
    if (use_rrt_star)
    {
      for (int nb_idx : nearNeighbours(nodes, new_pos, rewire_radius))
      {
        if (nb_idx == new_node.parent) continue;
        const double rewired_cost =
          new_node.cost + dist(new_pos, nodes[nb_idx].pos);
        if (rewired_cost < nodes[nb_idx].cost &&
            !collides(new_pos, nodes[nb_idx].pos, walls))
        {
          nodes[nb_idx].parent = new_idx;
          nodes[nb_idx].cost   = rewired_cost;
        }
      }
    }

    // 6. Goal check — track the cheapest node inside the goal threshold
    if (dist(new_pos, goal) <= goal_threshold)
    {
      if (!goal_reached || new_node.cost < nodes[goal_node_idx].cost)
      {
        if (!goal_reached)
          RCLCPP_INFO(
            node->get_logger(),
            "Goal first reached! Tree nodes: %zu. Path cost: %.3f",
            nodes.size(), new_node.cost);
        goal_reached  = true;
        goal_node_idx = new_idx;
      }
    }

    return true;
  };

  // ---- Main loop ----
  while (rclcpp::ok())
  {
    // RRT  : stop growing once a solution is found (plain RRT cannot improve it).
    // RRT* : keep growing forever — rewiring continuously refines the path cost.
    if (!goal_reached || use_rrt_star)
      stepOnce();

    // Re-extract path every frame: for RRT* rewiring may have shortened it.
    if (goal_reached)
      final_path = extractPath(nodes, goal_node_idx);

    // 7. Render
    drawScene(canvas, walls, nodes, start, goal,
              world_size, image_size, final_path);

    // Draw mode label (top-left)
    cv::putText(
      canvas,
      use_rrt_star ? "RRT*" : "RRT",
      {10, 28},
      cv::FONT_HERSHEY_SIMPLEX, 1.0,
      cv::Scalar(20, 20, 20), 3, cv::LINE_AA);   // dark outline
    cv::putText(
      canvas,
      use_rrt_star ? "RRT*" : "RRT",
      {10, 28},
      cv::FONT_HERSHEY_SIMPLEX, 1.0,
      use_rrt_star ? cv::Scalar(0, 180, 255) : cv::Scalar(255, 160, 0),
      2, cv::LINE_AA);

    if (goal_reached)
    {
      // Show current best path cost when using RRT*
      const std::string status = use_rrt_star
        ? ("cost: " + [&](){
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << nodes[goal_node_idx].cost;
            return ss.str();
          }())
        : "Goal reached!";
      cv::putText(
        canvas, status,
        {10, 58},
        cv::FONT_HERSHEY_SIMPLEX, 0.65,
        cv::Scalar(0, 0, 0), 3, cv::LINE_AA);   // outline
      cv::putText(
        canvas, status,
        {10, 58},
        cv::FONT_HERSHEY_SIMPLEX, 0.65,
        cv::Scalar(0, 200, 60), 2, cv::LINE_AA);
    }

    cv::imshow(win_name, canvas);

    const int key = cv::waitKey(step_delay_ms);
    if (key == 27 || key == 'q')  // ESC or 'q' → quit
      break;

    rclcpp::spin_some(node);
  }

  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
