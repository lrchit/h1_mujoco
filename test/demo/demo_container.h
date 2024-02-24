
#pragma once

// #include "WbcCtrl.hpp"
#include "cppTypes.h"
#include "kinematics.h"
#include "mpc_caller.h"
#include "state.h"
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#define WALKING 0
#define STEPPING 1
#define STANDING 2

class H1Demo {
public:
  H1Demo(std::vector<kinematics> _limb_kin);
  ~H1Demo(){};

  // demo

  void standing_demo(H1State &_state_cur, Vector3d &lin_vel_cmd,
                     Vector3d &angle_vel_cmd,
                     Eigen::Matrix<double, 6, 1> &_traj_integrate,
                     double body_height_stand);

  void stepping_demo(Vector3d &lin_vel_cmd, Vector3d &angle_vel_cmd,
                     Eigen::Matrix<double, 6, 1> &_traj_integrate,
                     double &_swing_height, double body_height_motion);

  void walking_demo(Vector3d &lin_vel_cmd, Vector3d &angle_vel_cmd,
                    Eigen::Matrix<double, 6, 1> &_traj_integrate,
                    double &_swing_height, double body_height_motion);

private:
  std::vector<kinematics> limb_kin;

  double body_height_stand;

  double swing_height_walking;
};
