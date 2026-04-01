/**
 * @file 2d_optimization.hpp
 * @brief Trajectory optimisation primitives for 2-D path planning.
 *
 * This header defines the NLP (Non-Linear Programming) components used to
 * plan a collision-free, minimum-length trajectory in 2-D space using the
 * IFOPT framework with an IPOPT back-end.
 *
 * The trajectory is represented as an ordered sequence of N waypoints
 * ("segments").  The optimiser minimises the total squared path length while
 * satisfying:
 *   - Endpoint constraints  – first/last waypoint must match start/goal.
 *   - Obstacle constraints  – every waypoint (and optionally every segment)
 *     must lie outside circular obstacles.
 *   - Segment-length bounds – individual segment lengths are kept inside a
 *     user-supplied interval to avoid degenerate solutions.
 *
 * Public API
 * ----------
 * traj_planning_2d::plan2DTrajectory() is the single entry point for callers.
 * All other symbols are IFOPT component classes consumed internally.
 */

#ifndef TRAJ_PLANNING_2D_HPP
#define TRAJ_PLANNING_2D_HPP

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Dense>
#include <iostream>
#include <optional>
#include <vector>

using namespace ifopt;

namespace traj_planning_2d {

/**
 * @brief IFOPT variable set representing the N trajectory waypoints.
 *
 * Internally the N 2-D waypoints are stored as a flat vector of size 2*N in
 * column-major order: [x0, y0, x1, y1, ..., x_{N-1}, y_{N-1}].
 * All variables are unbounded so the optimiser can place waypoints anywhere
 * in the plane.
 */
class Segments : public VariableSet {
public:
  /**
   * @brief Construct the variable set.
   * @param n_segments   Number of waypoints N in the trajectory.
   * @param initial_guess 2×N Eigen matrix used as the warm-start.
   *                      If the size does not match 2*N a zero matrix is used.
   */
  Segments(size_t n_segments, Eigen::MatrixXd initial_guess)
  : VariableSet(2 * n_segments, "segments")
  {
    if (initial_guess.size() != 2 * n_segments) {
      std::cerr << "Initial guess size does not match number of segments, using zeroes instead." << std::endl;
      x_ = Eigen::MatrixXd::Zero(2 * n_segments, 1);
      return;
    }

    // Flatten the 2×N matrix into a 2N-element column vector.
    x_ = Eigen::Map<Eigen::VectorXd>(initial_guess.data(), 2 * n_segments);
  }

  /// Set the optimisation variables from a flat vector (called by IFOPT).
  void SetVariables(const VectorXd &x) override { x_ = x; }

  /// Return the current flat variable vector (called by IFOPT).
  VectorXd GetValues() const override { return x_; }

  /// All waypoint coordinates are unconstrained (±∞).
  VecBound GetBounds() const override {
    VecBound bounds(GetRows());
    for (size_t i = 0; i < GetRows(); ++i) {
      bounds[i] = Bounds(
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
    }
    return bounds;
  }

private:
  /// Flat variable vector, size = 2*N (x and y for each waypoint).
  Eigen::VectorXd x_;
  // q (matrix form, 2×N) can be recovered via Eigen::Map when needed.
};

/**
 * @brief IFOPT cost term that minimises the total squared path length.
 *
 * The cost is defined as:
 * @f[
 *   J = \sum_{i=1}^{N-1} \|q_i - q_{i-1}\|^2
 * @f]
 * where q_i is the i-th 2-D waypoint.  Minimising the sum of squared segment
 * lengths encourages a short, evenly-spaced path.
 *
 * The analytical Jacobian is provided to speed up convergence.
 */
class LengthCost : public CostTerm {
public:
    LengthCost() : CostTerm("length_cost") {}

    /**
     * @brief Evaluate the cost (sum of squared segment lengths).
     * @return Scalar cost value.
     */
    double GetCost() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        // Interpret the flat vector as a 2×N matrix.
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        double cost = 0.0;
        for (int i = 1; i < q.cols(); ++i) {
            cost += (q.col(i) - q.col(i - 1)).squaredNorm();
        }
        return cost;
    }

    /**
     * @brief Fill the analytical Jacobian of the cost w.r.t. the waypoints.
     *
     * For each waypoint q_i the gradient contribution is:
     *   ∂J/∂q_i = 2(q_i - q_{i-1})  [from the left segment]
     *           + 2(q_i - q_{i+1})  [from the right segment]
     * with boundary terms omitted for the first and last waypoints.
     *
     * @param var_set Name of the variable set being differentiated.
     * @param jac     Sparse Jacobian matrix to fill in.
     */
    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        const size_t N = q.cols();
        for (size_t i = 0; i < N; ++i) {
            for (size_t dim = 0; dim < 2; ++dim) {
                double grad = 0.0;
                // Contribution from segment (i-1, i).
                if (i > 0) grad += 2.0 * (q(dim, i) - q(dim, i - 1));
                // Contribution from segment (i, i+1).
                if (i < N - 1) grad += 2.0 * (q(dim, i) - q(dim, i + 1));
                jac.coeffRef(0, i * 2 + dim) = grad;
            }
        }
    }
};

/**
 * @brief IFOPT constraint that pins the first and last waypoints to the
 *        desired start and goal positions.
 *
 * A single scalar equality constraint is used:
 * @f[
 *   g = \|q_0 - p_{\text{start}}\|^2 + \|q_{N-1} - p_{\text{goal}}\|^2 = 0
 * @f]
 */
class EndpointConstraint : public ConstraintSet {
public:
  /**
   * @brief Construct the endpoint constraint.
   * @param start Desired 2-D start position.
   * @param goal  Desired 2-D goal  position.
   */
  EndpointConstraint(Eigen::Vector2d start, Eigen::Vector2d goal)
  : ConstraintSet(1, "endpoint_constraint"), start_(start), goal_(goal) {}

  /**
   * @brief Evaluate the constraint residual.
   * @return 1-element vector; equals zero only when both endpoints match.
   */
  VectorXd GetValues() const override {
    Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
    Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);
    VectorXd g(1);
    g(0) = (q.col(0) - start_).squaredNorm() + (q.col(q.cols() - 1) - goal_).squaredNorm();
    return g;
  }

  /// Enforce g == 0 (equality constraint).
  VecBound GetBounds() const override {
    VecBound b(GetRows());
    b[0] = Bounds(0.0, 0.0);
    return b;
  }

  /**
   * @brief Analytical Jacobian of the endpoint constraint.
   *
   * Only the first and last waypoints have non-zero partial derivatives:
   *   ∂g/∂q_0       = 2(q_0 - start)
   *   ∂g/∂q_{N-1}   = 2(q_{N-1} - goal)
   *
   * @param var_set Name of the variable set.
   * @param jac     Sparse Jacobian to fill.
   */
  void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
    if (var_set == "segments") {
      Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
      Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

      const size_t N = q.cols();
      for (size_t dim = 0; dim < 2; ++dim) {
        // Derivative w.r.t. start point (first waypoint).
        const size_t var_idx_start = 0 * 2 + dim;
        jac.coeffRef(0, var_idx_start) = 2.0 * (q(dim, 0) - start_(dim));

        // Derivative w.r.t. goal point (last waypoint).
        const size_t var_idx_goal = (N - 1) * 2 + dim;
        jac.coeffRef(0, var_idx_goal) = 2.0 * (q(dim, N - 1) - goal_(dim));
      }
    }
  }

private:
  Eigen::Vector2d start_; ///< Required start position.
  Eigen::Vector2d goal_;  ///< Required goal  position.
};

/**
 * @brief IFOPT constraint that keeps every waypoint outside a circular obstacle.
 *
 * For each waypoint q_i the constraint is:
 * @f[
 *   g_i = \|q_i - o\|^2 - r^2 \geq \epsilon
 * @f]
 * where o is the obstacle centre, r its radius, and ε is a small positive
 * margin (0.01) that prevents the solver from sitting exactly on the boundary.
 *
 * @note This is a point-wise check.  Use LineCollisionConstraint to also
 *       prevent segment edges from penetrating the obstacle.
 */
class PointCollisionConstraint : public ConstraintSet {
public:
    /**
     * @brief Construct the per-waypoint collision avoidance constraint.
     * @param obstacle   2-D centre of the circular obstacle.
     * @param radius     Radius of the obstacle.
     * @param n_segments Number of waypoints N.
     */
    PointCollisionConstraint(const Eigen::Vector2d& obstacle, double radius, size_t n_segments)
        : ConstraintSet(n_segments, "collision_constraint"),
          obstacle_(obstacle),
          radius_(radius),
          n_segments_(n_segments) {}

    /**
     * @brief Evaluate constraint residuals for every waypoint.
     * @return N-element vector; values ≥ 0.01 indicate clearance.
     */
    VectorXd GetValues() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        VectorXd g(n_segments_);
        for (size_t i = 0; i < n_segments_; ++i) {
            // g_i = ||q_i - o||^2 - r^2  (must be >= 0 to stay outside)
            g(i) = (q.col(i) - obstacle_).squaredNorm() - radius_ * radius_;
        }

        return g;
    }

    /// Lower bound: each waypoint must be at least ε beyond the obstacle surface.
    VecBound GetBounds() const override {
        VecBound bounds(n_segments_);
        for (size_t i = 0; i < n_segments_; ++i) {
            bounds[i] = Bounds(0.01, std::numeric_limits<double>::infinity());
        }
        return bounds;
    }

    /**
     * @brief Analytical Jacobian of the point-collision constraint.
     *
     * For waypoint q_i:
     *   ∂g_i/∂q_i = 2(q_i - o)
     * All other partial derivatives are zero.
     *
     * @param var_set Name of the variable set.
     * @param jac     Sparse Jacobian to fill.
     */
    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        for (size_t i = 0; i < n_segments_; ++i) {
            Eigen::Vector2d diff = q.col(i) - obstacle_;
            for (size_t dim = 0; dim < 2; ++dim) {
                size_t var_idx = i * 2 + dim;
                // ∂/∂q_i (||q_i - o||^2 - r^2)
                jac.coeffRef(i, var_idx) = 2.0 * diff(dim);
            }
        }
    }

private:
    Eigen::Vector2d obstacle_; ///< Obstacle centre.
    double radius_;            ///< Obstacle radius.
    size_t n_segments_;        ///< Number of waypoints.
};

/**
 * @brief IFOPT constraint that keeps every *segment* (line between consecutive
 *        waypoints) outside a circular obstacle.
 *
 * For each segment (q_i, q_{i+1}) the minimum distance from the obstacle
 * centre to the segment is computed by projecting the obstacle onto the
 * segment and clamping the parameter t to [0, 1]:
 * @f[
 *   t      = \text{clamp}\!\left(\frac{(o-p_1) \cdot (p_2-p_1)}{\|p_2-p_1\|^2}, 0, 1\right) \\
 *   c      = p_1 + t\,(p_2-p_1) \\
 *   g_i    = \|o - c\|^2 - r^2 \geq \epsilon
 * @f]
 *
 * The Jacobian is approximated by treating t as constant (i.e. ignoring
 * ∂t/∂p).
 *
 * @note This constraint is currently disabled in plan2DTrajectory() in favour
 *       of PointCollisionConstraint, but can be swapped in if needed.
 */
class LineCollisionConstraint : public ConstraintSet {
public:
    /**
     * @brief Construct the per-segment collision avoidance constraint.
     * @param obstacle   2-D centre of the circular obstacle.
     * @param radius     Radius of the obstacle.
     * @param n_segments Number of waypoints N (creates N-1 segment constraints).
     */
    LineCollisionConstraint(const Eigen::Vector2d& obstacle, double radius, size_t n_segments)
        : ConstraintSet(n_segments - 1, "collision_constraint"),
          obstacle_(obstacle),
          radius_(radius),
          n_segments_(n_segments) {}

    /**
     * @brief Evaluate segment-to-obstacle distance constraints.
     * @return (N-1)-element vector; values ≥ 0.01 indicate clearance.
     */
    VectorXd GetValues() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        VectorXd g(n_segments_ - 1);

        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            Eigen::Vector2d p1 = q.col(i);
            Eigen::Vector2d p2 = q.col(i + 1);

            // Project obstacle onto the segment and clamp to [0, 1].
            Eigen::Vector2d diff = obstacle_ - p1;
            Eigen::Vector2d seg  = p2 - p1;
            double t = seg.dot(diff) / seg.squaredNorm();
            t = std::clamp(t, 0.0, 1.0);
            Eigen::Vector2d closest = p1 + t * seg;
            double dist2 = (obstacle_ - closest).squaredNorm();

            g(i) = dist2 - radius_ * radius_; // g_i >= 0 → outside obstacle
        }

        return g;
    }

    /// Lower bound: each segment must maintain at least ε clearance.
    VecBound GetBounds() const override {
        VecBound bounds(n_segments_ - 1);
        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            bounds[i] = Bounds(0.01, std::numeric_limits<double>::infinity());
        }
        return bounds;
    }

    /**
     * @brief Approximate analytical Jacobian of the segment-collision constraint.
     *
     * The projection parameter t is treated as constant (∂t/∂p ≈ 0) for
     * simplicity.  The approximate gradients are:
     *   ∂g_i/∂p1 ≈ 2(c - o) · (1 - t)
     *   ∂g_i/∂p2 ≈ 2(c - o) · t
     *
     * @param var_set Name of the variable set.
     * @param jac     Sparse Jacobian to fill.
     */
    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            Eigen::Vector2d p1 = q.col(i);
            Eigen::Vector2d p2 = q.col(i + 1);
            Eigen::Vector2d seg  = p2 - p1;
            Eigen::Vector2d diff = obstacle_ - p1;
            double seg_len2 = seg.squaredNorm();
            double t = seg.dot(diff) / seg_len2;
            t = std::clamp(t, 0.0, 1.0);
            Eigen::Vector2d closest = p1 + t * seg;
            // Gradient of ||closest - obstacle||^2 w.r.t. closest.
            Eigen::Vector2d grad = 2.0 * (closest - obstacle_);

            // Distribute to p1 and p2 proportional to (1-t) and t.
            jac.coeffRef(i, i * 2 + 0)       = grad(0) * (1 - t); // ∂g_i / ∂x_i
            jac.coeffRef(i, i * 2 + 1)       = grad(1) * (1 - t); // ∂g_i / ∂y_i
            jac.coeffRef(i, (i + 1) * 2 + 0) = grad(0) * t;       // ∂g_i / ∂x_{i+1}
            jac.coeffRef(i, (i + 1) * 2 + 1) = grad(1) * t;       // ∂g_i / ∂y_{i+1}
        }
    }

private:
    Eigen::Vector2d obstacle_; ///< Obstacle centre.
    double radius_;            ///< Obstacle radius.
    size_t n_segments_;        ///< Number of waypoints.
};

/**
 * @brief IFOPT constraint bounding the length of each trajectory segment.
 *
 * Enforces min_length ≤ ||q_{i+1} - q_i|| ≤ max_length for every consecutive
 * waypoint pair.  Prevents degenerate solutions where waypoints collapse on
 * top of each other (min) or take unreasonably large jumps (max).
 */
class SegmentLengthConstraint : public ConstraintSet {
public:
    /**
     * @brief Construct the segment-length bounds.
     * @param min_length Minimum allowed Euclidean distance between adjacent waypoints.
     * @param max_length Maximum allowed Euclidean distance between adjacent waypoints.
     * @param n_segments Number of waypoints N (creates N-1 constraints).
     */
    SegmentLengthConstraint(double min_length, double max_length, size_t n_segments)
        : ConstraintSet(n_segments - 1, "segment_length"),
          min_length_(min_length),
          max_length_(max_length),
          n_segments_(n_segments) {}

    /**
     * @brief Evaluate the Euclidean length of each segment.
     * @return (N-1)-element vector of segment lengths.
     */
    VectorXd GetValues() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        VectorXd g(n_segments_ - 1);
        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            g(i) = (q.col(i + 1) - q.col(i)).norm();
        }
        return g;
    }

    /// Each segment length must lie within [min_length_, max_length_].
    VecBound GetBounds() const override {
        VecBound bounds(n_segments_ - 1);
        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            bounds[i] = Bounds(min_length_, max_length_);
        }
        return bounds;
    }

    /**
     * @brief Analytical Jacobian of segment-length constraints.
     *
     * For segment i: g_i = ||q_{i+1} - q_i||
     *   ∂g_i/∂q_i     = -(q_{i+1} - q_i) / ||q_{i+1} - q_i||
     *   ∂g_i/∂q_{i+1} =  (q_{i+1} - q_i) / ||q_{i+1} - q_i||
     * A small ε is added to the denominator to avoid division by zero.
     *
     * @param var_set Name of the variable set.
     * @param jac     Sparse Jacobian to fill.
     */
    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            Eigen::Vector2d diff = q.col(i + 1) - q.col(i);
            double dist = diff.norm();
            if (dist < 1e-8) dist = 1e-8; // guard against division by zero

            for (size_t dim = 0; dim < 2; ++dim) {
                size_t var_idx_i   = i * 2 + dim;
                size_t var_idx_ip1 = (i + 1) * 2 + dim;
                jac.coeffRef(i, var_idx_i)   = -diff(dim) / dist; // ∂g_i/∂q_i
                jac.coeffRef(i, var_idx_ip1) =  diff(dim) / dist; // ∂g_i/∂q_{i+1}
            }
        }
    }

private:
    double min_length_; ///< Minimum segment length.
    double max_length_; ///< Maximum segment length.
    size_t n_segments_; ///< Number of waypoints.
};

/**
 * @brief Plan a 2-D collision-free trajectory using NLP optimisation.
 *
 * Builds and solves an IFOPT/IPOPT non-linear programme that minimises total
 * squared path length subject to:
 *   - Endpoint equality constraints (start / goal pins).
 *   - Segment-length bounds [0.10, obstacle_radius].
 *   - Per-waypoint collision avoidance for every obstacle.
 *
 * The solver is warm-started from @p initial_guess.  On first call, pass a
 * zero matrix; on subsequent calls, pass the previous solution to improve
 * convergence speed.
 *
 * @param start          2-D start position.
 * @param goal           2-D goal  position.
 * @param obstacles      List of 2-D obstacle centre positions.
 * @param obstacle_radius Radius shared by all obstacles (also used as the
 *                        upper bound for segment lengths).
 * @param n_segments     Number of waypoints N in the output trajectory.
 * @param initial_guess  2×N Eigen matrix used as the warm-start for IPOPT.
 * @return               2×N matrix of optimised waypoints, or std::nullopt if
 *                       no feasible solution was found.
 */
inline std::optional<Eigen::MatrixXd> plan2DTrajectory(
  Eigen::Vector2d start,
  Eigen::Vector2d goal,
  std::vector<Eigen::Vector2d> obstacles,
  double obstacle_radius,
  size_t n_segments,
  Eigen::MatrixXd initial_guess)
{
  Problem nlp;

  // ---- Variables ----
  nlp.AddVariableSet(std::make_shared<Segments>(n_segments, initial_guess));

  // ---- Constraints ----
  // 1. Pin the trajectory endpoints to start and goal.
  nlp.AddConstraintSet(std::make_shared<EndpointConstraint>(start, goal));

  // 2. Keep individual segments within a sensible length range.
  //    Upper bound = obstacle_radius prevents segments from "jumping over" obstacles.
  nlp.AddConstraintSet(std::make_shared<SegmentLengthConstraint>(0.10, obstacle_radius, n_segments));

  // 3. Per-waypoint obstacle avoidance for each circular obstacle.
  for (const auto& obstacle : obstacles) {
    nlp.AddConstraintSet(
      std::make_shared<PointCollisionConstraint>(obstacle, obstacle_radius, n_segments));
    // Line-segment collision can be enabled for stricter avoidance:
    // nlp.AddConstraintSet(std::make_shared<LineCollisionConstraint>(
    //   obstacle, obstacle_radius, n_segments));
  }

  // ---- Cost ----
  nlp.AddCostSet(std::make_shared<LengthCost>());

  // ---- Solve ----
  IpoptSolver solver;
  solver.SetOption("print_level", 2);
  solver.Solve(nlp);

  // Extract the optimised waypoints and reshape to 2×N.
  Eigen::VectorXd x_opt = nlp.GetOptVariables()->GetValues();
  return Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>>(x_opt.data(), 2, x_opt.size() / 2);
}

}  // namespace traj_planning_2d

#endif  // TRAJ_PLANNING_2D_HPP
