
#pragma once

#include <yaml-cpp/yaml.h>

#include "Gait.h"
#include "WbcCtrl.hpp"
#include "demo_container.h"
#include "estimate.h"
#include "kinematics.h"
#include "motion_planning.h"
#include "orientation_tools.h"
#include "state.h"

class H1FSM {
public:
  H1FSM();
  ~H1FSM(){};

  void main_program();
  void compute_mpc();
  void compute_wbc();
  void state_estimate(mjData *d);
  Matrix<double, 19, 1> get_joint_torques();

  bool use_wbc;
  bool mpc_update_needed;
  bool wbc_update_needed;

  H1Demo *demo;
  H1State state_cur, state_des;

private:
  void compute_joint_torques();
  void updateMpcData();
  void updateWbcData();
  void run(int mode);

  // 模块：状态估计，mpc，wbc，demo
  H1Estm *estimater;
  H1Wbc *wbc_controller;
  MotionPlanning *motion_planning;
  H1Mpc *mpc_solver;

  std::vector<kinematics> limb_kin;
  WbcData wbc_data;

  Matrix<double, 6, 2> foot_forces_kin;
  Matrix<double, 5, 2> leg_joint_torque_kin;

  int counter;

  double dt;
  double dtmpc;
  double x_drag;
  int horizon;
  int iteration_between_mpc;

  Vector3d lin_vel_cmd, angle_vel_cmd;

  double body_height_stand;
  double body_height_motion;

  Vector3d rpy_comp;

  double swing_height;

  Matrix<double, 6, 1> traj_integrate;

  // 步态
  Gait *gait;
  OffsetDurationGait *walking, *standing;
  int iterationCounter;
  Matrix<int, -1, 2> gait_table;
};
