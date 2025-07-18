#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>

int main() {
  // Solve: minimize (1/2)xᵀP x + qᵀ x
  // Subject to l ≤ Ax ≤ u

  // 1) QP data
  Eigen::SparseMatrix<double> P(3,3);
  P.insert(0,0) = 2.0;
  P.insert(1,1) = 1.0;
  P.insert(2,2) = 4.0;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(3);

  Eigen::SparseMatrix<double> A(5,3);
  A.insert(0,0) = 1.0;
  A.insert(0,1) = 2.0;
  A.insert(0,2) = 3.0;

  A.insert(1,0) = 2.0;
  A.insert(1,1) = 7.0;
  A.insert(1,2) = 4.0;

  A.insert(2,0) = 1.0;
  A.insert(2,1) = 0.0;
  A.insert(2,2) = 0.0;

  A.insert(3,0) = 0.0;
  A.insert(3,1) = 1.0;
  A.insert(3,2) = 0.0;

  A.insert(4,0) = 0.0;
  A.insert(4,1) = 0.0;
  A.insert(4,2) = 1.0;

  Eigen::VectorXd l(5), u(5);
  l << 1, 1, -0.35, -0.35, -0.35;
  u << 1, 1, 0.35, 0.35, 0.35;

  // 2) Build solver
  OsqpEigen::Solver solver;

  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(3);
  solver.data()->setNumberOfConstraints(5);
  solver.data()->setHessianMatrix(P);
  solver.data()->setGradient(q);
  solver.data()->setLinearConstraintsMatrix(A);
  solver.data()->setLowerBound(l);
  solver.data()->setUpperBound(u);

  // 3) Initialize & solve
  if (!solver.initSolver()) {
    std::cerr << "Failed to initialize.\n";
    return 1;
  }

  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    std::cerr << "Solver failed.\n";
    return 1;
  }

  // 4) Extract results
  Eigen::VectorXd x = solver.getSolution();
  std::cout << "Solution x =\n" << x << "\n";
  return 0;
}
