#include "mpc_caller.h"
#include <chrono>

// 求反对称阵
Matrix3d H1Mpc::skew(Vector3d vec) {
  Matrix3d skew_mat;
  skew_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_mat;
}

H1Mpc::H1Mpc(int _horizon) {

  YAML::Node config = YAML::LoadFile("../controllers/mpc_config.yaml");

  horizon = _horizon;
  constraint_num = config["constraint_num"].as<int>();

  weight_Q.setZero(13 * horizon);
  weight_R.setZero(12 * horizon);

  for (int i = 0; i < horizon; ++i) {
    // Initialize weight matrix
    weight_Q(0 + 13 * i) = config["q1"].as<double>();
    weight_Q(1 + 13 * i) = config["q2"].as<double>();
    weight_Q(2 + 13 * i) = config["q3"].as<double>();
    weight_Q(3 + 13 * i) = config["q4"].as<double>();
    weight_Q(4 + 13 * i) = config["q5"].as<double>();
    weight_Q(5 + 13 * i) = config["q6"].as<double>();
    weight_Q(6 + 13 * i) = config["q7"].as<double>();
    weight_Q(7 + 13 * i) = config["q8"].as<double>();
    weight_Q(8 + 13 * i) = config["q9"].as<double>();
    weight_Q(9 + 13 * i) = config["q10"].as<double>();
    weight_Q(10 + 13 * i) = config["q11"].as<double>();
    weight_Q(11 + 13 * i) = config["q12"].as<double>();

    weight_R(0 + 12 * i) = config["r1"].as<double>();
    weight_R(1 + 12 * i) = config["r2"].as<double>();
    weight_R(2 + 12 * i) = config["r3"].as<double>();
    weight_R(3 + 12 * i) = config["r4"].as<double>();
    weight_R(4 + 12 * i) = config["r5"].as<double>();
    weight_R(5 + 12 * i) = config["r6"].as<double>();
    weight_R(6 + 12 * i) = config["r1"].as<double>();
    weight_R(7 + 12 * i) = config["r2"].as<double>();
    weight_R(8 + 12 * i) = config["r3"].as<double>();
    weight_R(9 + 12 * i) = config["r4"].as<double>();
    weight_R(10 + 12 * i) = config["r5"].as<double>();
    weight_R(11 + 12 * i) = config["r6"].as<double>();
  }
  MpcQ.resize(13 * horizon);
  MpcR.resize(12 * horizon);
  MpcQ.diagonal() = weight_Q;
  MpcR.diagonal() = weight_R;

  mu = 0.4;
  linear_constraints.resize(constraint_num * horizon, 12 * horizon);
  for (int i = 0; i < 2 * horizon; ++i) {
    linear_constraints.insert(0 + 8 * i, 0 + 6 * i) = 1;
    linear_constraints.insert(1 + 8 * i, 0 + 6 * i) = 1;
    linear_constraints.insert(2 + 8 * i, 1 + 6 * i) = 1;
    linear_constraints.insert(3 + 8 * i, 1 + 6 * i) = 1;
    linear_constraints.insert(4 + 8 * i, 2 + 6 * i) = 1;

    linear_constraints.insert(0 + 8 * i, 2 + 6 * i) = mu;
    linear_constraints.insert(1 + 8 * i, 2 + 6 * i) = -mu;
    linear_constraints.insert(2 + 8 * i, 2 + 6 * i) = mu;
    linear_constraints.insert(3 + 8 * i, 2 + 6 * i) = -mu;

    linear_constraints.insert(0 + 5 + 8 * i, 0 + 3 + 6 * i) = 1;
    linear_constraints.insert(1 + 5 + 8 * i, 1 + 3 + 6 * i) = 1;
    linear_constraints.insert(2 + 5 + 8 * i, 2 + 3 + 6 * i) = 1;
  }
  // std::cout << "linear_constraints = \n" << linear_constraints << std::endl;

  Hessian.resize(12 * horizon, 12 * horizon);

  A_mat_c.setZero();
  A_mat_d.setZero();
  B_mat_c.setZero();
  B_mat_d.setZero();
  TempA.setIdentity();
  A_qp.setZero(13 * horizon, 13);
  B_qp.setZero(13 * horizon, 12 * horizon);

  Gradient.setZero(12 * horizon);

  lb.setZero(constraint_num * horizon);
  ub.setZero(constraint_num * horizon);

  inertia << 4.6284, -0.0793959, -0.398517, -0.0793959, 4.13297, -0.58975,
      -0.398517, -0.58975, 1.46117;
  mass = 51.601;
}

void H1Mpc::update_inertia(H1State &state) {
  FBModelState floating_base_state;
  floating_base_state.bodyOrientation = ori::rpyToQuat(state.euler_angle);
  floating_base_state.bodyPosition = state.pos;
  Vector<double, 6> bodyVelocity;
  bodyVelocity.head<3>() = state.rot_mat * state.lin_vel;
  bodyVelocity.tail<3>() = state.rot_mat * state.omega;
  Vector<double, 18> q, qd;
  for (int i = 0; i < 2; ++i) {
    q.segment(5 * i, 5) = state.leg_qpos.col(i);
    qd.segment(5 * i, 5) = state.leg_qvel.col(i);

    q.segment(10 + 4 * i, 4) = state.arm_qpos.col(i);
    qd.segment(10 + 4 * i, 4) = state.arm_qvel.col(i);
  }

  floating_base_dyn.updateModel(floating_base_state);
  inertia = floating_base_dyn._A.block(3, 3, 3, 3);
}

// 更新mpc参数
void H1Mpc::update_mpc(Matrix<double, 3, 2> foot_pos, VectorXd &state_des,
                       Vector<double, 13> &state_cur,
                       Matrix<int, -1, 2> gait_table, double x_drag,
                       double dt) {

  // std::cout << "************* mpc_control *************" << std::endl;
  // std::cout << "state_cur\n" << state_cur.transpose() << std::endl;
  // std::cout << "state_des\n"
  //           << state_des.block(13, 0, 13, 1).transpose() << std::endl;
  // std::cout << "foot_pos\n" << foot_pos.col(0).transpose() << std::endl;
  // std::cout << "foot_pos\n" << foot_pos.col(1).transpose() << std::endl;
  // std::cout << "x_drag\n" << x_drag << std::endl;

  // 计算连续形式的A,B
  Matrix3d trans_mat;
  double cos_yaw = cos(state_cur(2));
  double sin_yaw = sin(state_cur(2));
  // trans_mat << cos_yaw, sin_yaw, 0, -sin_yaw, cos_yaw, 0, 0, 0, 1;

  trans_mat(0, 0) = cos(state_cur(2)) / cos(state_cur(1));
  trans_mat(0, 1) = sin(state_cur(2)) / cos(state_cur(1));
  trans_mat(0, 2) = 0;
  trans_mat(1, 0) = -sin(state_cur(2));
  trans_mat(1, 1) = cos(state_cur(2));
  trans_mat(1, 2) = 0;
  trans_mat(2, 0) = cos(state_cur(2)) * tan(state_cur(1));
  trans_mat(2, 1) = sin(state_cur(2)) * tan(state_cur(1));
  trans_mat(2, 2) = 1;

  A_mat_c.block(0, 6, 3, 3) = trans_mat;
  A_mat_c.block(3, 9, 3, 3) = Matrix3d::Identity();
  A_mat_c(11, 9) = x_drag;
  A_mat_c(11, 12) = 1;

  Matrix3d rot_mat;
  rot_mat = ori::rpyToRotMat(state_cur.block(0, 0, 3, 1)).transpose();
  Matrix3d inertia_world_inv;
  // inertia_world_inv = (rot_mat * inertia * rot_mat.transpose()).inverse();
  inertia_world_inv = inertia.inverse();
  Matrix3d L = Matrix3d::Identity();
  for (int i = 0; i < 2; ++i) {
    B_mat_c.block(6, 6 * i, 3, 3) = inertia_world_inv * skew(foot_pos.col(i));
    B_mat_c.block(6, 6 * i + 3, 3, 3) = L * inertia_world_inv;
    B_mat_c.block(9, 6 * i, 3, 3) = (1 / mass) * Matrix3d::Identity();
  }
  // std::cout << "B = \n" << B_mat_c << std::endl;
  // std::cout << "A = \n" << A_mat_c << std::endl;

  // 离散化A,B
  A_mat_d = Matrix<double, 13, 13>::Identity() + A_mat_c * dt;
  B_mat_d = B_mat_c * dt;

  // 摩擦力上下界
  double fz_min = 0;
  double fz_max = 1000;
  double tau_x_max = 0;
  double tau_y_max = 200;
  double tau_z_max = 200;
  VectorXd lb_one_horizon(constraint_num);
  VectorXd ub_one_horizon(constraint_num);
  for (int i = 0; i < horizon; ++i) {
    for (int j = 0; j < 2; ++j) {
      lb_one_horizon.segment(j * 8, 5) << 0, -OsqpEigen::INFTY, 0,
          -OsqpEigen::INFTY, fz_min * gait_table(i, j);
      ub_one_horizon.segment(j * 8, 5) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY,
          0, fz_max * gait_table(i, j);

      lb_one_horizon.segment(j * 8 + 5, 3) =
          Vector3d(-tau_x_max, -tau_y_max, -tau_z_max) * gait_table(i, j);
      ub_one_horizon.segment(j * 8 + 5, 3) =
          Vector3d(tau_x_max, tau_y_max, tau_z_max) * gait_table(i, j);
    }
    lb.segment(i * constraint_num, constraint_num) = lb_one_horizon;
    ub.segment(i * constraint_num, constraint_num) = ub_one_horizon;
    // std::cout << "gait_table = \n" << gait_table << std::endl;
    // std::cout << "ub_one_horizon = \n" << ub_one_horizon << std::endl;
  }
  // A_qp,B_qp
  TempA.setIdentity();
  for (int i = 0; i < horizon; ++i) {
    int ii = i * 13;
    int ii_last = (i - 1) * 13;
    int iu = i * 12;

    if (i == 0) {
      A_qp.block(ii, 0, 13, 13) = A_mat_d;
      B_qp.block(ii, 0, 13, 12) = B_mat_d;
    } else {
      A_qp.block(ii, 0, 13, 13) = A_qp.block(ii_last, 0, 13, 13) * A_mat_d;
      TempA = TempA * A_mat_d;
      B_qp.block(ii, 0, 13, 12) = TempA * B_mat_d;
      B_qp.block(ii, 12, 13, 12 * i) = B_qp.block(ii_last, 0, 13, 12 * i);
    }
  }

  // Calculate Hessian and gradient
  MatrixXd dense_hessian;
  dense_hessian.setZero(12 * horizon, 12 * horizon);
  SparseMatrix<double> B_qp_sparse;
  B_qp_sparse = B_qp.sparseView();
  SparseMatrix<double> Temp;
  Temp = B_qp_sparse.transpose() * MpcQ;
  dense_hessian = Temp * B_qp_sparse;
  dense_hessian += MpcR;
  Hessian = dense_hessian.sparseView();
  Gradient = Temp * (A_qp * state_cur - state_des);
}

// 解mpc
void H1Mpc::solve_mpc() {

  if (!qp_solver.isInitialized()) {
    qp_solver.settings()->setVerbosity(false);
    qp_solver.settings()->setWarmStart(false);
    qp_solver.data()->setNumberOfVariables(12 * horizon);
    qp_solver.data()->setNumberOfConstraints(constraint_num * horizon);
    qp_solver.data()->setLinearConstraintsMatrix(linear_constraints);
    qp_solver.data()->setHessianMatrix(Hessian);
    qp_solver.data()->setGradient(Gradient);
    qp_solver.data()->setLowerBound(lb);
    qp_solver.data()->setUpperBound(ub);
    qp_solver.initSolver();
  } else {
    qp_solver.data()->setNumberOfVariables(12 * horizon);
    qp_solver.data()->setNumberOfConstraints(constraint_num * horizon);
    qp_solver.updateHessianMatrix(Hessian);
    qp_solver.updateGradient(Gradient);
    qp_solver.updateLowerBound(lb);
    qp_solver.updateUpperBound(ub);
  }
  qp_solver.solveProblem();
  // std::cout << "return_value" << qp_solver.getObjValue() << std::endl;
}

Vector<double, 12> H1Mpc::get_solution() {

  Vector<double, 12> _solution = qp_solver.getSolution().segment(0, 12);
  if (!std::isnan(_solution.norm()))
    solution = _solution;

  return solution;
}
VectorXd H1Mpc::get_solution_trajectory() { return qp_solver.getSolution(); }
