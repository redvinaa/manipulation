# motion_planning

A ROS 2 package that provides two executables for 2-D motion planning and
configuration-space (C-space) exploration of a two-link planar robotic arm.

---

## Overview

| Executable        | Source file                  | Purpose |
|-------------------|------------------------------|---------|
| `2d_optimization` | `src/2d_optimization.cpp`    | Interactive NLP-based trajectory planning in the task plane |
| `rrt_with_optim`  | `src/rrt_with_optim.cpp`     | Interactive C-space / task-space exploration of a crowded bin scene |

Both nodes use **RViz interactive markers** so the user can drag inputs
(start, goal, obstacles, or end-effector) and see results update in real time.

---

## Packages & Dependencies

| Dependency            | Use |
|-----------------------|-----|
| `rclcpp`              | ROS 2 C++ client library |
| `geometry_msgs`       | `PoseStamped`, `Point` messages |
| `nav_msgs`            | `OccupancyGrid` for C-space map |
| `visualization_msgs`  | `MarkerArray` for RViz visualisation |
| `interactive_markers` | Draggable RViz markers |
| `tf2_eigen`           | Eigen ↔ ROS transform utilities |
| `angles`              | Angle normalisation helpers |
| `ifopt` + IPOPT       | NLP formulation and solver |
| `bin_picking`         | Shared types / scene description |
| `backward_ros`        | Pretty crash stack traces |

---

## Nodes

### `2d_optimization` — interactive 2-D trajectory planner

Plans a collision-free, minimum-length path between a draggable start and
goal through a set of draggable circular obstacles.  The path is represented
as an ordered list of **N waypoints** and computed by solving a Non-Linear
Programme with [IFOPT](https://github.com/ethz-adrl/ifopt) / IPOPT.

#### Subscribed topics
None (input via interactive markers).

#### Published topics
| Topic                | Type                             | Description |
|----------------------|----------------------------------|-------------|
| `trajectory_markers` | `visualization_msgs/MarkerArray` | Optimised path rendered as a blue line and green waypoint spheres |

#### Parameters
| Parameter       | Type   | Default | Description |
|-----------------|--------|---------|-------------|
| `n_segments`    | `int`  | `10`    | Number of waypoints in the trajectory |
| `obstacle_radius` | `double` | `0.5` | Radius of all circular obstacles (m) |
| `n_obstacles`   | `int`  | `2`     | Number of obstacles (placed uniformly on a 0.5 m circle at startup) |

#### Optimisation formulation

```
minimise    Σ ||q_{i+1} - q_i||²          (total squared path length)
subject to  ||q_0 - start||² + ||q_{N-1} - goal||² = 0   (endpoint pins)
            0.10 ≤ ||q_{i+1} - q_i|| ≤ obstacle_radius    (segment lengths)
            ||q_i - o_j||² ≥ r²  ∀ i, j                  (obstacle avoidance)
```

All constraint Jacobians are provided analytically.  The previous solution
is used as a warm-start on each re-plan to improve convergence speed.

---

### `rrt_with_optim` — C-space / task-space explorer

Models a **two-link planar arm** inside a U-shaped bin.  At start-up it
computes the full C-space occupancy grid (1 000 × 1 000 collision checks)
and publishes it as a `nav_msgs/OccupancyGrid`.  The user can then drag
an interactive marker to explore arm configurations.

#### Exploration modes

The mode is selected at compile time via the `CONTROL_CONFIG_SPACE` constant
in `src/rrt_with_optim.cpp`:

| Value  | Mode | Marker semantics |
|--------|------|-----------------|
| `true` (default) | **C-space** | Marker XY → (θ₁, θ₂) mapped linearly over [−π, π] |
| `false` | **Task-space** | Marker XY → desired end-effector position; solved via closed-form IK |

#### Published topics
| Topic                           | Type                              | Description |
|---------------------------------|-----------------------------------|-------------|
| `configuration_space_occupancy` | `nav_msgs/OccupancyGrid`          | Precomputed C-space collision map |
| `crowded_scene_markers`         | `visualization_msgs/MarkerArray`  | Bin walls + current arm pose |

#### Parameters (from `motion_planning::CrowdedScene`)
| Parameter                | Type     | Default | Description |
|--------------------------|----------|---------|-------------|
| `root_link_length`       | `double` | `1.0`   | First link length (m) |
| `end_link_length`        | `double` | `1.0`   | Second link length (m) |
| `dist_from_obstacle`     | `double` | `0.5`   | Safety margin subtracted from wall Y-coordinates (m) |
| `bin_depth`              | `double` | `1.0`   | Bin depth in Y (m) |
| `bin_width`              | `double` | `1.0`   | Bin half-width (m) |
| `configuration_space_size` | `double` | `2.0` | Side length of the C-space grid in the RViz view (m) |

---

## Building

```bash
cd /path/to/workspace
colcon build --packages-select motion_planning
source install/setup.bash
```

## Running

### 2-D trajectory optimiser

```bash
ros2 run motion_planning 2d_optimization \
  --ros-args -p n_segments:=15 -p obstacle_radius:=0.4 -p n_obstacles:=3
```

Open RViz, add a **MarkerArray** display on `/trajectory_markers` and an
**InteractiveMarkers** display on `/traj_planning_markers/update`.

### C-space explorer

```bash
ros2 run motion_planning rrt_with_optim
```

Open RViz and add:
- **Map** display on `/configuration_space_occupancy`
- **MarkerArray** display on `/crowded_scene_markers`
- **InteractiveMarkers** display on `/end_effector_marker/update`

---

## Architecture

```
motion_planning/
├── include/motion_planning/
│   ├── 2d_optimization.hpp    # IFOPT variable/constraint/cost classes +
│   │                          #   plan2DTrajectory() entry point
│   └── crowded_scene.hpp      # CrowdedScene: FK, IK, collision, C-space grid
├── src/
│   ├── 2d_optimization.cpp    # TrajPlanning2DNode (interactive NLP planner)
│   └── rrt_with_optim.cpp     # C-space / task-space explorer node
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Key classes (in `2d_optimization.hpp`)

| Class | Role |
|-------|------|
| `Segments` | IFOPT variable set — N 2-D waypoints flattened to a 2N vector |
| `LengthCost` | Minimise total squared path length |
| `EndpointConstraint` | Pin first/last waypoints to start/goal |
| `PointCollisionConstraint` | Keep each waypoint outside circular obstacles |
| `LineCollisionConstraint` | Keep each segment edge outside obstacles (currently disabled) |
| `SegmentLengthConstraint` | Bound individual segment lengths to avoid degenerate solutions |
| `plan2DTrajectory()` | Convenience function: assembles and solves the full NLP |

---

## Notes & Known Limitations

- `LineCollisionConstraint` uses an **approximate Jacobian** (∂t/∂p ≈ 0)
  and is disabled by default.  Enable it in `plan2DTrajectory()` for stricter
  collision avoidance at the cost of slower convergence.
- The C-space grid computation in `rrt_with_optim` is **blocking** and takes
  a few seconds for the default 1 000 × 1 000 resolution; reduce `n_divs`
  for faster startup.
- IPOPT `print_level` is set to `2` (minimal output).  Change it in
  `plan2DTrajectory()` for more or less solver verbosity.

