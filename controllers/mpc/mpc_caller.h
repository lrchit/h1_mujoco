
#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Eigen>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <yaml-cpp/yaml.h>

#include "orientation_tools.h"
#include "state.h"

using namespace Eigen;

class QuadMpc {
public:
  QuadMpc(int _horizon);
  ~QuadMpc(){};

  void update_mpc(Matrix<double, 6, 2> foot_pos, VectorXd state_des,
                  Vector<double, 13> state_cur, Matrix<int, -1, 2> gait_table,
                  double x_drag, double dt);
  void solve_mpc();

  Vector<double, 12> get_solution();
  VectorXd get_solution_trajectory();

private:
  Matrix3d skew(Eigen::Vector3d vec);

  int horizon;
  int constraint_num;

  VectorXd weight_Q;
  VectorXd weight_R;
  DiagonalMatrix<double, -1> MpcQ;
  DiagonalMatrix<double, -1> MpcR;

  double mu;
  SparseMatrix<double> linear_constraints;

  Matrix<double, 13, 13> A_mat_c;
  Matrix<double, 13, 13> A_mat_d;
  Matrix<double, 13, 12> B_mat_c;
  Matrix<double, 13, 12> B_mat_d;
  Matrix<double, 13, 13> TempA;

  MatrixXd A_qp;
  MatrixXd B_qp;

  SparseMatrix<double> Hessian;
  VectorXd Gradient;

  OsqpEigen::Solver qp_solver;
  bool flag_qp_solver_init = false;

  VectorXd lb;
  VectorXd ub;

  double mass;
  Matrix3d inertia;

  Vector<double, 12> solution;
};
