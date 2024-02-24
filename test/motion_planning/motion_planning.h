
#pragma once

#include "Gait.h"
// #include "WbcCtrl.hpp"
#include "foot_swing_traj.h"
#include "kinematics.h"
#include "orientation_tools.h"
#include "state.h"
#include <yaml-cpp/yaml.h>

using namespace Eigen;

class MotionPlanning {
public:
  MotionPlanning();
  ~MotionPlanning(){};

  void generate_swing_ctrl(bool use_wbc, Gait *gait, const H1State &state_cur,
                           H1State &state_des,
                           Matrix<double, 6, 2> &foot_forces_kin,
                           double swing_height);

  void update_command(Vector3d lin_vel_cmd, Vector3d angle_vel_cmd,
                      const H1State &state_cur, H1State &state_des,
                      Vector3d &rpy_comp, Matrix<double, 6, 1> &traj_integrate);

private:
  void rpy_safety_check(Vector3d &angle_des, Vector3d angle_act);
  void config_foot_hold(const H1State &state_cur, H1State &state_des,
                        Gait *gait, double swing_height);

  bool omniMode;
  double dt;
  double dtmpc;

  Matrix<double, 6, 6> kpCartesian;
  Matrix<double, 6, 6> kdCartesian;
  Matrix<double, 5, 5> kpJoint;
  Matrix<double, 5, 5> kdJoint;

  Vector3d xyz_vel_des;

  bool firstRun;
  bool firstSwing[2];
  double swingTimeRemaining[2];
  Matrix<double, 2, 1> swing_state;
  Matrix<double, 6, 2> foot_hold;
  foot_swing_traj footSwingTrajectories[2];
};
