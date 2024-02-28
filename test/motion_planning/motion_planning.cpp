
#include "motion_planning.h"

#define pi 3.1416

/**
 * @brief 初始化
 */
MotionPlanning::MotionPlanning(std::vector<kinematics> _limb_kin) {
  limb_kin = _limb_kin;

  YAML::Node config = YAML::LoadFile("../controllers/mpc_config.yaml");

  omniMode = config["omniMode"].as<bool>();
  kpCartesian.setZero();
  kdCartesian.setZero();
  kpCartesian(0, 0) = config["kp_cartesian_x"].as<double>();
  kpCartesian(1, 1) = config["kp_cartesian_y"].as<double>();
  kpCartesian(2, 2) = config["kp_cartesian_z"].as<double>();
  kpCartesian(3, 3) = config["kp_cartesian_roll"].as<double>();
  kpCartesian(4, 4) = config["kp_cartesian_pitch"].as<double>();
  kpCartesian(5, 5) = config["kp_cartesian_yaw"].as<double>();
  kdCartesian(0, 0) = config["kd_cartesian_x"].as<double>();
  kdCartesian(1, 1) = config["kd_cartesian_y"].as<double>();
  kdCartesian(2, 2) = config["kd_cartesian_z"].as<double>();
  kdCartesian(3, 3) = config["kd_cartesian_roll"].as<double>();
  kdCartesian(4, 4) = config["kd_cartesian_pitch"].as<double>();
  kdCartesian(5, 5) = config["kd_cartesian_yaw"].as<double>();

  dt = 0.001;
  dtmpc = dt * config["iteration_between_mpc"].as<int>();

  firstRun = true;
  for (int i = 0; i < 2; ++i) {
    firstSwing[i] = true;
  }
  swing_state.setZero();

  foot_hold.setZero();
  swing_state.setZero();
}

// imu得到的rpy是(-pi，pi)，state_des.euler_angle也需要控制在这范围内
void MotionPlanning::rpy_safety_check(Vector3d &angle_des, Vector3d angle_act) {
  for (int i = 0; i < 3; ++i) {
    if (angle_des(i) - angle_act(i) > pi * 3 / 4) {
      angle_des(i) -= 2 * pi;
    } else if (angle_des(i) - angle_act(i) < -pi * 3 / 4) {
      angle_des(i) += 2 * pi;
    }
  }
}

void MotionPlanning::generate_swing_ctrl(bool use_wbc, Gait *gait,
                                         const H1State &state_cur,
                                         H1State &state_des,
                                         Matrix<double, 6, 2> &foot_forces_kin,
                                         double swing_height) {

  // 摆动相位
  for (int i = 0; i < 2; ++i) {
    if (gait->getSwingState()(i) >= 0 && gait->getSwingState()(i) <= 1)
      swing_state(i) = gait->getSwingState()(i);
  }
  // std::cout << "swing_state\n" << swing_state.transpose() << std::endl;

  // some first time initialization
  if (firstRun) {
    for (int i = 0; i < 2; ++i) {
      footSwingTrajectories[i].set_lift_point(state_cur.foot_pos_world.col(i));
      footSwingTrajectories[i].set_hold_point(state_cur.foot_pos_world.col(i));
    }
    firstRun = false;
  }

  // 落足点配置
  config_foot_hold(state_cur, state_des, gait, swing_height);
  std::vector<std::string> frame_name;
  frame_name.push_back("left_foot_center");
  frame_name.push_back("right_foot_center");

  for (int i = 0; i < 2; ++i) {
    // 获取轨迹上的足点，世界系
    state_des.foot_pos_world.col(i) =
        footSwingTrajectories[i].get_swing_pos(swing_state(i));
    state_des.foot_vel_world.col(i) =
        footSwingTrajectories[i].get_swing_vel(swing_state(i));
    // 获取轨迹上的足点，机体系[todo]
    state_des.foot_pos_base.block(0, i, 3, 1) =
        state_cur.rot_mat *
        (state_des.foot_pos_world.block(0, i, 3, 1) - state_cur.pos);
    Matrix3d rot_world_to_foot =
        ori::rpyToRotMat(state_des.foot_pos_world.block(3, i, 3, 1));
    Matrix3d rot_base_to_foot =
        state_cur.rot_mat.transpose() * rot_world_to_foot;
    Quat quat_base_to_foot = ori::rotationMatrixToQuaternion(rot_base_to_foot);
    state_des.foot_pos_base.block(3, i, 3, 1) =
        ori::quatToRPY(quat_base_to_foot);

    state_des.foot_vel_base.block(0, i, 3, 1) =
        state_cur.rot_mat *
        (state_des.foot_vel_world.block(0, i, 3, 1) - state_cur.lin_vel);
    state_des.foot_vel_base.block(3, i, 3, 1) =
        state_cur.rot_mat * (state_des.foot_vel_world.block(0, i, 3, 1) +
                             state_cur.euler_angle_vel);
  }
  // std::cout << "state_des.foot_pos_base\n"
  //           << state_des.foot_pos_base << std::endl;
  // std::cout << "state_cur.foot_pos_base\n"
  //           << state_cur.foot_pos_base << std::endl;
  if (state_cur.foot_pos_base(1, 1) >= 0)
    exit(0);

  // 计算关节力矩
  if (!use_wbc) {
    Matrix<double, 6, 2> foot_pos_error;
    Matrix<double, 6, 2> foot_vel_error;
    for (int i = 0; i < 2; ++i) {
      // 摆动腿，在Cartesian下规划
      if (swing_state(i) > 0) {
        // 摆动腿完全用pd
        foot_pos_error.col(i) =
            state_des.foot_pos_base.col(i) - state_cur.foot_pos_base.col(i);
        foot_vel_error.col(i) =
            state_des.foot_vel_base.col(i) - state_cur.foot_vel_base.col(i);

      } else {
        // 支撑腿
        foot_pos_error.col(i) =
            (state_des.foot_pos_base.col(i) - state_cur.foot_pos_base.col(i)) *
            0;
        foot_vel_error.col(i) =
            (state_des.foot_vel_base.col(i) - state_cur.foot_vel_base.col(i));
      }
      // pd控制
      foot_forces_kin.col(i) = kpCartesian * foot_pos_error.col(i) +
                               kdCartesian * foot_vel_error.col(i);
    }
  }
}

// 配置落足点
void MotionPlanning::config_foot_hold(const H1State &state_cur,
                                      H1State &state_des, Gait *gait,
                                      double swing_height) {
  // 配置落足点
  Matrix<double, 3, 2> foot_location_offset;
  foot_location_offset << 0.039468, 0.039468, 0.20285, -0.20285, 0, 0;

  // 计算控制量
  for (int i = 0; i < 2; ++i) {

    // 摆动腿落足点，在Cartesian下规划
    if (swing_state(i) > 0) {

      // foot is in swing
      if (firstSwing[i]) {
        // 摆动高度
        footSwingTrajectories[i].set_swing_height(swing_height);

        firstSwing[i] = false;
        Vector<double, 6> foot_pos_world;
        foot_pos_world.segment(0, 3) =
            state_cur.foot_pos_world.block(0, i, 3, 1);
        foot_pos_world.segment(3, 3) << 0, 0, 0;
        footSwingTrajectories[i].set_lift_point(foot_pos_world);

        swingTimeRemaining[i] = gait->getCurrentSwingTime(dtmpc, i);

      } else {
        swingTimeRemaining[i] -= dt;
      }

      double stance_time = gait->getCurrentStanceTime(dtmpc, i);

      double cos_yaw = cos(-state_des.euler_angle_vel(2) * stance_time / 2);
      double sin_yaw = sin(-state_des.euler_angle_vel(2) * stance_time / 2);
      Matrix3d Rz;
      Rz << cos_yaw, sin_yaw, 0, -sin_yaw, cos_yaw, 0, 0, 0, 1;
      Vector3d pRobotFrame(foot_location_offset.col(i));
      Vector3d pYawCorrected(Rz * pRobotFrame);

      // 完全按照wbic的论文：p_shoulder + p_symmetry + p_centrifugal
      Vector3d p_shoulder =
          state_cur.pos +
          state_cur.rot_mat.transpose() *
              (pYawCorrected +
               Vector3d(xyz_vel_des(0), xyz_vel_des(1), xyz_vel_des(2)) *
                   swingTimeRemaining[i]);

      Vector3d p_symmetry;
      p_symmetry(0) = 0.5 * stance_time * state_cur.lin_vel(0) +
                      0.03 * (state_cur.lin_vel(0) - state_des.lin_vel(0));
      p_symmetry(1) = 0.5 * stance_time * state_cur.lin_vel(1) +
                      0.03 * (state_cur.lin_vel(1) - state_des.lin_vel(1));

      Vector3d p_centrifugal;
      p_centrifugal(0) = 0.5 * sqrt(fabs(state_cur.pos(2) / 9.81)) *
                         (state_cur.lin_vel(1) * state_des.euler_angle_vel(2) -
                          state_cur.lin_vel(2) * state_des.euler_angle_vel(1));
      p_centrifugal(1) = 0.5 * sqrt(fabs(state_cur.pos(2) / 9.81)) *
                         (state_cur.lin_vel(1) * state_des.euler_angle_vel(0) -
                          state_cur.lin_vel(0) * state_des.euler_angle_vel(2));

      // 落脚点
      foot_hold.block(0, i, 3, 1) = p_shoulder + p_symmetry + p_centrifugal;
      foot_hold.block(3, i, 3, 1) << 0, 0, 0;

      // 摆动腿控的不好，所以补偿z方向高度[todo]
      foot_hold(2, i) = -0.005;
      footSwingTrajectories[i].set_hold_point(foot_hold.col(i));
    } else {
      firstSwing[i] = true;
    }
  }
  // std::cout << "foot_hold\n" << foot_hold << std::endl;
}

// 更新指令
void MotionPlanning::update_command(Vector3d lin_vel_cmd,
                                    Vector3d angle_vel_cmd,
                                    const H1State &state_cur,
                                    H1State &state_des, Vector3d &rpy_comp,
                                    Matrix<double, 6, 1> &traj_integrate) {

  // 获取速度，位置指令
  double filter_acc = 0.01;
  double filter_dec = 0.005;

  state_des.euler_angle_vel(0) = angle_vel_cmd(0);
  state_des.euler_angle_vel(1) = angle_vel_cmd(1);
  // 加速和减速分别处理，只对yaw速度进行控制，另外2个方向只给角度指令
  for (int i = 0; i < 3; ++i) {
    if (angle_vel_cmd(i) >= state_des.euler_angle_vel(i))
      state_des.euler_angle_vel(i) =
          angle_vel_cmd(i) * filter_acc +
          (1 - filter_acc) * state_des.euler_angle_vel(i);
    else
      state_des.euler_angle_vel(i) =
          angle_vel_cmd(i) * filter_dec +
          (1 - filter_dec) * state_des.euler_angle_vel(i);
  }

  traj_integrate.segment(0, 3) += state_des.euler_angle_vel * dt;
  // traj_integrate.segment(0, 3) =
  //     state_cur.euler_angle + state_des.euler_angle_vel * dt;
  state_des.euler_angle = traj_integrate.segment(0, 3);
  rpy_safety_check(state_des.euler_angle, state_cur.euler_angle);
  traj_integrate.segment(0, 3) = state_des.euler_angle;
  // --- safety check ---
  const float max_yaw_error = .05;
  for (int i = 0; i < 3; ++i) {
    if (state_des.euler_angle(i) - state_cur.euler_angle(i) > max_yaw_error)
      state_des.euler_angle(i) = state_cur.euler_angle(i) + max_yaw_error;
    if (state_cur.euler_angle(i) - state_des.euler_angle(i) > max_yaw_error)
      state_des.euler_angle(i) = state_cur.euler_angle(i) - max_yaw_error;
  }

  // 加速和减速分别处理，只控制x,y方向速度，z方向只给位置指令
  for (int i = 0; i < 3; ++i) {
    if (lin_vel_cmd(i) >= xyz_vel_des(i))
      xyz_vel_des(i) =
          lin_vel_cmd(i) * filter_acc + (1 - filter_acc) * xyz_vel_des(i);
    else
      xyz_vel_des(i) =
          lin_vel_cmd(i) * filter_dec + (1 - filter_dec) * xyz_vel_des(i);
  }

  state_des.lin_vel =
      omniMode ? xyz_vel_des : state_cur.rot_mat.transpose() * xyz_vel_des;

  if (lin_vel_cmd(2) == 0)
    state_des.lin_vel(2) = 0;
  traj_integrate.segment(3, 3) += state_des.lin_vel * dt;
  state_des.pos = traj_integrate.segment(3, 3);
  // std::cout << traj_integrate.segment(3, 3).transpose() << std::endl;
  // --- safety check ---
  const float max_pos_error = .05;
  for (int i = 0; i < 3; ++i) {
    if (state_des.pos(i) - state_cur.pos(i) > max_pos_error)
      state_des.pos(i) = state_cur.pos(i) + max_pos_error;
    if (state_cur.pos(i) - state_des.pos(i) > max_pos_error)
      state_des.pos(i) = state_cur.pos(i) - max_pos_error;
  }
  state_des.lin_acc.setZero();

  // integral-esque pitch and roll compensation
  // turn off for pronking
  Vector3d lin_vel_robot = state_cur.rot_mat * state_cur.lin_vel;
  Vector3d rpy_int;
  if (fabs(lin_vel_robot(0)) > 0.01) // avoid dividing by zero
  {
    rpy_int(1) += dt * (state_des.euler_angle(1) - state_cur.euler_angle(1)) /
                  lin_vel_robot(0);
  }
  if (fabs(lin_vel_robot(1)) > 0.01) {
    rpy_int(0) += dt * (state_des.euler_angle(0) - state_cur.euler_angle(0)) /
                  lin_vel_robot(1);
  }
  rpy_int(0) = fminf(fmaxf(rpy_int(0), -0.5), 0.5);
  rpy_int(1) = fminf(fmaxf(rpy_int(1), -0.5), 0.5);
  rpy_comp(1) = lin_vel_robot(0) * rpy_int(1) + state_des.euler_angle(1);
  rpy_comp(0) = lin_vel_robot(1) * rpy_int(0) + state_des.euler_angle(0);
}
