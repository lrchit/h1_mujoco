
#include "FSM.h"

/**
 * @brief 初始化
 */
H1FSM::H1FSM() {

  YAML::Node config = YAML::LoadFile("../controllers/mpc_config.yaml");

  horizon = config["horizon"].as<int>();
  iteration_between_mpc = config["iteration_between_mpc"].as<int>();

  mpc_solver = new H1Mpc(horizon);

  gait_table.setZero(horizon, 2);

  use_wbc = config["use_wbc"].as<bool>();

  // limb_kin.resize(4);
  limb_kin.push_back(kinematics("../h1_description/urdf/h1_left_leg.urdf"));
  limb_kin.push_back(kinematics("../h1_description/urdf/h1_right_leg.urdf"));
  limb_kin.push_back(kinematics("../h1_description/urdf/h1_left_arm.urdf"));
  limb_kin.push_back(kinematics("../h1_description/urdf/h1_right_arm.urdf"));

  body_height_stand = config["body_height_stand"].as<double>();
  body_height_motion = config["body_height_motion"].as<double>();

  dt = 0.001;
  dtmpc = dt * iteration_between_mpc;

  foot_forces_kin.setZero();

  walking = new OffsetDurationGait(horizon, Vector<int, 2>(0, 0.5 * horizon),
                                   Vector<int, 2>(0.5 * horizon, 0.5 * horizon),
                                   "Walking");
  standing =
      new OffsetDurationGait(horizon, Vector<int, 2>(0, 0),
                             Vector<int, 2>(horizon, horizon), "Standing");

  counter = 0;
  iterationCounter = 0;
  mpc_update_needed = false;
  wbc_update_needed = false;

  estimater = new H1Estm(limb_kin);
  demo = new H1Demo(limb_kin);
  wbc_controller = new H1Wbc();
  motion_planning = new MotionPlanning(limb_kin);
  mpc_solver = new H1Mpc(horizon);
}

/**
 * @brief 更新规划
 */
void H1FSM::main_program() {

  counter++;

  if (counter >= 500) // don't change this
  {
    // std::cout << "***************** force_mode *****************" <<
    // std::endl;

    int mode;

    static int standing_counter;

    static bool first_stop = true;

    // 3种模式，站立，踏步，行走
    // 先切力控站立再运动
    if (counter < 2000) {
      // std::cout << "***************** standing *****************\n"
      //           << std::endl;
      mode = STANDING;
    } else if (counter < 5500000) {
      // std::cout << "***************** stepping *****************\n"
      //           << std::endl;
      mode = STEPPING;
    } else if (counter < 7500) {
      // std::cout << "***************** walking *****************\n" <<
      // std::endl;
      mode = WALKING;
    } else if (counter < 11500) {
      // std::cout << "***************** stepping *****************\n"
      //           << std::endl;
      mode = STEPPING;
      if (first_stop) {
        traj_integrate(3 + 0) = state_cur.pos(0);
        traj_integrate(3 + 1) = state_cur.pos(1);
      }
      first_stop = false;
    } else {
      if (fabs(state_cur.lin_vel(0)) < 0.1 &&
          fabs(state_cur.lin_vel(1)) < 0.1 &&
          fabs(state_cur.euler_angle_vel(2) < 0.1)) {
        // std::cout << "***************** standing *****************\n"
        //           << std::endl;
        mode = STANDING;
        standing_counter++;
      } else {
        // std::cout << "***************** stepping *****************\n"
        //           << std::endl;
        mode = STEPPING;
        standing_counter = 0;
      }
    }

    run(mode);
  }
}

void H1FSM::run(int mode) {

  iterationCounter++;

  for (int i = 2; i < 4; ++i)
    state_cur.contact_phase(i) = 0;

  // 站立模式
  if (mode == STANDING) {
    gait = standing;
    // 支撑相位要发给state_estimator
    state_cur.contact_phase << 0.5, 0.5, 0, 0;

    demo->standing_demo(state_cur, lin_vel_cmd, angle_vel_cmd, traj_integrate,
                        body_height_stand);
  }
  // walking运动
  else if (mode == STEPPING) {
    gait = walking;

    for (int i = 0; i < 2; ++i)
      state_cur.contact_phase(i) = gait->getContactState()(i);

    demo->stepping_demo(lin_vel_cmd, angle_vel_cmd, traj_integrate,
                        swing_height, body_height_motion);
  } else if (mode == WALKING) {
    gait = walking;

    for (int i = 0; i < 2; ++i)
      state_cur.contact_phase(i) = gait->getContactState()(i);

    demo->walking_demo(lin_vel_cmd, angle_vel_cmd, traj_integrate, swing_height,
                       body_height_motion);
  }

  // calc gait
  gait->setIterations(iteration_between_mpc, iterationCounter);

  int *mpcTable = gait->getMpcTable();
  // gait table，mpc要用
  for (int i = 0; i < horizon; ++i) {
    for (int j = 0; j < 2; j++) {
      gait_table(i, j) = mpcTable[i * 2 + j];
    }
  }

  // 更新指令
  motion_planning->update_command(lin_vel_cmd, angle_vel_cmd, state_cur,
                                  state_des, rpy_comp, traj_integrate);

  // 生成摆动腿控制
  motion_planning->generate_swing_ctrl(use_wbc, gait, state_cur, state_des,
                                       foot_forces_kin, swing_height);

  // 判断到达mpc更新周期
  if ((iterationCounter % iteration_between_mpc) == 0) {
    mpc_update_needed = true;
  }
  // 开启或关闭wbc
  if (use_wbc) {
    wbc_update_needed = true;
  } else {
    compute_joint_torques();
  }
}

// 状态估计
void H1FSM::state_estimate(mjData *d) {
  estimater->call_state_estimator(state_cur, d);
}

// 更新mpc需要的参数
void H1FSM::updateMpcData() {

  mpc_update_needed = false;

  // --- state_des_vec ---
  VectorXd state_des_vec;
  state_des_vec.setZero(13 * horizon);
  state_des_vec(0) = rpy_comp(0);
  state_des_vec(1) = rpy_comp(1);
  state_des_vec(2) = state_des.euler_angle(2);
  state_des_vec.segment(3, 3) = state_des.pos;
  state_des_vec.segment(6, 3) = state_des.euler_angle_vel;
  state_des_vec.segment(9, 3) = state_des.lin_vel;
  state_des_vec(12) = -9.81;
  for (int i = 1; i < horizon; ++i) {
    state_des_vec.segment(13 * i, 13) = state_des_vec.segment(13 * (i - 1), 13);
    state_des_vec.segment(13 * i + 0, 3) +=
        state_des_vec.segment(13 * (i - 1) + 6, 3) * dtmpc;
    state_des_vec.segment(13 * i + 3, 3) +=
        state_des_vec.segment(13 * (i - 1) + 9, 3) * dtmpc;
  }

  // --- update state_cur ---
  Vector<double, 13> state_cur_vec;
  state_cur_vec.segment(0, 3) = state_cur.euler_angle;
  state_cur_vec.segment(3, 3) = state_cur.pos;
  state_cur_vec.segment(6, 3) = state_cur.euler_angle_vel;
  state_cur_vec.segment(9, 3) = state_cur.lin_vel;
  state_cur_vec(12) = -9.81;

  // --- update r_foot_to_com ---
  Matrix<double, 3, 2> r_foot_to_com;
  for (int i = 0; i < 2; ++i)
    r_foot_to_com.col(i) =
        state_cur.foot_pos_world.block(0, i, 3, 1) - state_cur.pos;

  // --- update mpc solver ---
  // mpc_solver->update_inertia(state_cur);
  mpc_solver->update_mpc(r_foot_to_com, state_des_vec, state_cur_vec,
                         gait_table, x_drag, dtmpc);

  double pz_err = state_cur.pos(2) - state_des.pos(2);
  if (state_cur.lin_vel(0) > 0.3 || state_cur.lin_vel(0) < -0.3) {
    x_drag += 3 * pz_err * dtmpc / state_cur.lin_vel(0);
  }
}

// 求解mpc
void H1FSM::compute_mpc() {

  // update
  updateMpcData();

  // --- solve the qp ---
  mpc_solver->solve_mpc();

  // --- return grf ---
  Vector<double, 12> solution = mpc_solver->get_solution();
  for (int i = 0; i < 2; ++i) {
    state_cur.grf_ref.segment(6 * i, 6) =
        solution.segment(6 * i, 6) * gait_table(0, i);
  }
}

// 更新wbc需要的参数
void H1FSM::updateWbcData() {
  wbc_data.contact_state.resize(4);
  wbc_data.pEnd_des.resize(4);
  wbc_data.vEnd_des.resize(4);
  wbc_data.aEnd_des.resize(4);
  wbc_data.Fr_des.resize(4);

  // update for wbc
  wbc_data.state.bodyOrientation = ori::rpyToQuat(state_cur.euler_angle);
  wbc_data.state.bodyPosition = state_cur.pos;

  Matrix3d RotMat = state_cur.rot_mat;

  wbc_data.state.bodyVelocity.segment(0, 3) =
      RotMat * state_cur.euler_angle_vel;
  wbc_data.state.bodyVelocity.segment(3, 3) = RotMat * state_cur.lin_vel;

  for (int i = 0; i < 2; ++i) {
    wbc_data.state.q.segment(i * 5, 5) = state_cur.leg_qpos.col(i);
    wbc_data.state.qd.segment(i * 5, 5) = state_cur.leg_qvel.col(i);
  }
  wbc_data.state.q(10) = state_cur.torso_qpos;
  wbc_data.state.qd(10) = state_cur.torso_qvel;
  for (int i = 0; i < 2; ++i) {
    wbc_data.state.q.segment(i * 4 + 11, 4) =
        state_cur.arm_qpos.block(1, i, 4, 1);
    wbc_data.state.qd.segment(i * 4 + 11, 4) =
        state_cur.arm_qvel.block(1, i, 4, 1);
  }

  wbc_data.pBody_des = state_des.pos;
  // std::cout << "state_cur.pos\n" << state_cur.pos << std::endl;
  // std::cout << "state_des.pos\n" << state_des.pos << std::endl;
  wbc_data.vBody_des = state_des.lin_vel;
  wbc_data.aBody_des = state_des.lin_acc;
  wbc_data.pBodyOri_des = ori::rpyToQuat(state_des.euler_angle);
  wbc_data.vBodyOri_des = state_des.euler_angle_vel;

  for (int i = 0; i < 2; ++i) {
    wbc_data.pEnd_des[i] = state_des.foot_pos_world.col(i);

    wbc_data.vEnd_des[i] = state_des.foot_vel_world.col(i);
    wbc_data.aEnd_des[i] = Vec6::Zero();

    if (state_cur.contact_phase(i) > 0 && state_cur.contact_phase(i) <= 1) {
      wbc_data.contact_state[i] = 1;
    } else
      wbc_data.contact_state[i] = 0;

    wbc_data.Fr_des[i] = state_cur.grf_ref.segment(6 * i, 6);
  }

  // [todo]
  state_des.hand_pos_world << 0.0185, 0.0185, 0.21353, -0.21353, 0.911614,
      0.911614, 0, 0, 0, 0, 0, 0;
  state_des.hand_vel_world << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  for (int i = 2; i < 4; ++i) {
    wbc_data.pEnd_des[i] = state_des.hand_pos_world.col(i);

    wbc_data.vEnd_des[i] = state_des.hand_vel_world.col(i);
    wbc_data.aEnd_des[i] = Vec6::Zero();

    if (state_cur.contact_phase(i) > 0 && state_cur.contact_phase(i) <= 1) {
      wbc_data.contact_state[i] = 1;
    } else
      wbc_data.contact_state[i] = 0;

    wbc_data.Fr_des[i] = state_cur.grf_ref.segment(6 * i, 6);
  }
}

// 求解wbc
void H1FSM::compute_wbc() {
  // update
  updateWbcData();
  // --- solve the qp ---
  wbc_controller->run(wbc_data, state_cur.joint_torque);
}

// 算关节力矩，前馈+pd
void H1FSM::compute_joint_torques() {
  for (int i = 0; i < 2; ++i) {
    std::vector<std::string> frame_name;
    frame_name.push_back("left_foot_center");
    frame_name.push_back("right_foot_center");

    Matrix<double, 6, 5> jacobian =
        limb_kin[i].get_limb_jacobian(state_cur.leg_qpos.col(i), frame_name[i]);

    Vector<double, 6> grf_world = state_cur.grf_ref.segment(6 * i, 6);
    // std::cout << "grf_world = \n" << grf_world.transpose() << std::endl;
    Vector<double, 6> grf_base;
    grf_base.head<3>() = state_cur.rot_mat * grf_world.head<3>();
    grf_base.tail<3>() = state_cur.rot_mat * grf_world.tail<3>();

    state_cur.joint_torque.segment(5 * i, 5) =
        jacobian.transpose() * (foot_forces_kin.col(i) - grf_base);
  }
}

// 返回joint_torque发给电机控制器
Matrix<double, 19, 1> H1FSM::get_joint_torques() {
  // std::cout << "joint_torque = \n"
  //           << state_cur.joint_torque.transpose() << std::endl;
  return state_cur.joint_torque;
}
