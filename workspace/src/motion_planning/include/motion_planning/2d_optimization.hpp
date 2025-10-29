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

class Segments : public VariableSet {
public:
  Segments(size_t n_segments, Eigen::MatrixXd initial_guess)
  : VariableSet(2 * n_segments, "segments")
  {
    if (initial_guess.size() != 2 * n_segments) {
      std::cerr << "Initial guess size does not match number of segments, using zeroes instead." << std::endl;
      x_ = Eigen::MatrixXd::Zero(2 * n_segments, 1);
      return;
    }

    x_ = Eigen::Map<Eigen::VectorXd>(initial_guess.data(), 2 * n_segments);
  }

  void SetVariables(const VectorXd &x) override { x_ = x; }
  VectorXd GetValues() const override { return x_; }
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
  Eigen::VectorXd x_;  // X: vector form, size = 2 * N
  // q: matrix form, size = 2 x N
};

class LengthCost : public CostTerm {
public:
    LengthCost() : CostTerm("length_cost") {}

    double GetCost() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        double cost = 0.0;
        for (int i = 1; i < q.cols(); ++i) {
            cost += (q.col(i) - q.col(i - 1)).squaredNorm();
        }
        return cost;
    }

    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        const size_t N = q.cols();
        for (size_t i = 0; i < N; ++i) {
            for (size_t dim = 0; dim < 2; ++dim) {
                double grad = 0.0;
                if (i > 0) grad += 2.0 * (q(dim, i) - q(dim, i - 1));
                if (i < N - 1) grad += 2.0 * (q(dim, i) - q(dim, i + 1));
                jac.coeffRef(0, i * 2 + dim) = grad;
            }
        }
    }
};

class EndpointConstraint : public ConstraintSet {
public:
  EndpointConstraint(Eigen::Vector2d start, Eigen::Vector2d goal)
  : ConstraintSet(1, "endpoint_constraint"), start_(start), goal_(goal) {}

  VectorXd GetValues() const override {
    Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
    Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);
    VectorXd g(1);
    g(0) = (q.col(0) - start_).squaredNorm() + (q.col(q.cols() - 1) - goal_).squaredNorm();
    return g;
  }

  VecBound GetBounds() const override {
    VecBound b(GetRows());
    b[0] = Bounds(0.0, 0.0);  // equality constraint
    return b;
  }

  void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
    if (var_set == "segments") {
      Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
      Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

      const size_t N = q.cols();
      for (size_t dim = 0; dim < 2; ++dim) {
        // Derivative w.r.t. start point (first point)
        const size_t var_idx_start = 0 * 2 + dim;
        jac.coeffRef(0, var_idx_start) = 2.0 * (q(dim, 0) - start_(dim));

        // Derivative w.r.t. goal point (last point)
        const size_t var_idx_goal = (N - 1) * 2 + dim;
        jac.coeffRef(0, var_idx_goal) = 2.0 * (q(dim, N - 1) - goal_(dim));
      }
    }
  }

private:
  Eigen::Vector2d start_;
  Eigen::Vector2d goal_;
};

class PointCollisionConstraint : public ConstraintSet {
public:
    PointCollisionConstraint(const Eigen::Vector2d& obstacle, double radius, size_t n_segments)
        : ConstraintSet(n_segments, "collision_constraint"),
          obstacle_(obstacle),
          radius_(radius),
          n_segments_(n_segments) {}

    VectorXd GetValues() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        VectorXd g(n_segments_);
        for (size_t i = 0; i < n_segments_; ++i) {
            // Constraint: g_i >= 0 means outside obstacle
            g(i) = (q.col(i) - obstacle_).squaredNorm() - radius_ * radius_;
        }

        return g;
    }

    VecBound GetBounds() const override {
        VecBound bounds(n_segments_);
        for (size_t i = 0; i < n_segments_; ++i) {
            bounds[i] = Bounds(0.01, std::numeric_limits<double>::infinity()); 
            // g_i >= 0
        }
        return bounds;
    }

    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        for (size_t i = 0; i < n_segments_; ++i) {
            Eigen::Vector2d diff = q.col(i) - obstacle_;
            for (size_t dim = 0; dim < 2; ++dim) {
                size_t var_idx = i * 2 + dim;
                jac.coeffRef(i, var_idx) = 2.0 * diff(dim); // ∂/∂q_i (||q_i - o||^2 - r^2)
            }
        }
    }

private:
    Eigen::Vector2d obstacle_;
    double radius_;
    size_t n_segments_;
};

class LineCollisionConstraint : public ConstraintSet {
public:
    LineCollisionConstraint(const Eigen::Vector2d& obstacle, double radius, size_t n_segments)
        : ConstraintSet(n_segments - 1, "collision_constraint"), // N-1 segments
          obstacle_(obstacle),
          radius_(radius),
          n_segments_(n_segments) {}

    VectorXd GetValues() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        VectorXd g(n_segments_ - 1);

        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            Eigen::Vector2d p1 = q.col(i);
            Eigen::Vector2d p2 = q.col(i + 1);

            Eigen::Vector2d diff = obstacle_ - p1;
            Eigen::Vector2d seg = p2 - p1;
            double t = seg.dot(diff) / seg.squaredNorm();
            t = std::clamp(t, 0.0, 1.0); // project onto segment
            Eigen::Vector2d closest = p1 + t * seg;
            double dist2 = (obstacle_ - closest).squaredNorm();

            g(i) = dist2 - radius_ * radius_; // g_i >= 0
        }

        return g;
    }

    VecBound GetBounds() const override {
        VecBound bounds(n_segments_ - 1);
        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            bounds[i] = Bounds(0.01, std::numeric_limits<double>::infinity());
        }
        return bounds;
    }

    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            Eigen::Vector2d p1 = q.col(i);
            Eigen::Vector2d p2 = q.col(i + 1);
            Eigen::Vector2d seg = p2 - p1;
            Eigen::Vector2d diff = obstacle_ - p1;
            double seg_len2 = seg.squaredNorm();
            double t = seg.dot(diff) / seg_len2;
            t = std::clamp(t, 0.0, 1.0);
            Eigen::Vector2d closest = p1 + t * seg;
            Eigen::Vector2d grad = 2.0 * (closest - obstacle_); // ∂/∂closest ||closest - obstacle||^2

            // Chain rule: ∂closest/∂p1 = 1 - t + dt/dp1? Approximate with linear segment
            jac.coeffRef(i, i * 2 + 0) = grad(0) * (1 - t); // ∂g_i / ∂x_i
            jac.coeffRef(i, i * 2 + 1) = grad(1) * (1 - t); // ∂g_i / ∂y_i
            jac.coeffRef(i, (i + 1) * 2 + 0) = grad(0) * t; // ∂g_i / ∂x_{i+1}
            jac.coeffRef(i, (i + 1) * 2 + 1) = grad(1) * t; // ∂g_i / ∂y_{i+1}
        }
    }

private:
    Eigen::Vector2d obstacle_;
    double radius_;
    size_t n_segments_;
};

class SegmentLengthConstraint : public ConstraintSet {
public:
    SegmentLengthConstraint(double min_length, double max_length, size_t n_segments)
        : ConstraintSet(n_segments - 1, "segment_length"),
          min_length_(min_length),
          max_length_(max_length),
          n_segments_(n_segments) {}

    VectorXd GetValues() const override {
        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        VectorXd g(n_segments_ - 1);
        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            double seg_length = (q.col(i+1) - q.col(i)).norm();
            g(i) = seg_length;  // constraint will be bounded by [min_length, max_length]
        }
        return g;
    }

    VecBound GetBounds() const override {
        VecBound bounds(n_segments_ - 1);
        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            bounds[i] = Bounds(min_length_, max_length_);
        }
        return bounds;
    }

    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override {
        if (var_set != "segments") return;

        Eigen::VectorXd x = GetVariables()->GetComponent("segments")->GetValues();
        Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic>> q(x.data(), 2, x.size() / 2);

        for (size_t i = 0; i < n_segments_ - 1; ++i) {
            Eigen::Vector2d diff = q.col(i+1) - q.col(i);
            double dist = diff.norm();
            if (dist < 1e-8) dist = 1e-8; // avoid division by zero

            for (size_t dim = 0; dim < 2; ++dim) {
                size_t var_idx_i   = i*2 + dim;
                size_t var_idx_ip1 = (i+1)*2 + dim;
                jac.coeffRef(i, var_idx_i)   = -diff(dim) / dist;
                jac.coeffRef(i, var_idx_ip1) =  diff(dim) / dist;
            }
        }
    }

private:
    double min_length_;
    double max_length_;
    size_t n_segments_;
};

inline std::optional<Eigen::MatrixXd> plan2DTrajectory(
  Eigen::Vector2d start,
  Eigen::Vector2d goal,
  std::vector<Eigen::Vector2d> obstacles,
  double obstacle_radius,
  size_t n_segments,
  Eigen::MatrixXd initial_guess)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<Segments>(n_segments, initial_guess));
  nlp.AddConstraintSet(std::make_shared<EndpointConstraint>(start, goal));
  nlp.AddConstraintSet(std::make_shared<SegmentLengthConstraint>(0.10, obstacle_radius, n_segments));

  for (const auto& obstacle : obstacles) {
    nlp.AddConstraintSet(std::make_shared<PointCollisionConstraint>(obstacle, obstacle_radius, n_segments));
    // nlp.AddConstraintSet(std::make_shared<LineCollisionConstraint>(obstacle, obstacle_radius, n_segments));
  }
  nlp.AddCostSet(std::make_shared<LengthCost>());

  IpoptSolver solver;
  solver.SetOption("print_level", 2);
  solver.Solve(nlp);

  Eigen::VectorXd x_opt = nlp.GetOptVariables()->GetValues();

  return Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>>(x_opt.data(), 2, x_opt.size() / 2);
}

}  // namespace traj_planning_2d

#endif  // TRAJ_PLANNING_2D_HPP
