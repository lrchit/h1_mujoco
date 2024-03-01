
#include "demo_container.h"

H1Demo::H1Demo() {
  YAML::Node config = YAML::LoadFile("../controllers/mpc_config.yaml");

  body_height_stand = config["body_height_stand"].as<double>();

  swing_height_walking = config["swing_height_walking"].as<double>();
}

// 力控站立，踏步，行走动作
void H1Demo::standing_demo(H1State &_state_cur, Vector3d &lin_vel_cmd,
                           Vector3d &angle_vel_cmd,
                           Eigen::Matrix<double, 6, 1> &_traj_integrate,
                           double body_height_stand) {
  lin_vel_cmd(0) = 0.;
  lin_vel_cmd(1) = 0.;
  lin_vel_cmd(2) = 0.;
  angle_vel_cmd(0) = 0.;
  angle_vel_cmd(1) = 0.;
  angle_vel_cmd(2) = 0.;

  _traj_integrate(0) = 0;
  _traj_integrate(1) = 0;
  _traj_integrate(3) =
      (_state_cur.foot_pos_world(0, 0) + _state_cur.foot_pos_world(0, 1)) / 2;
  _traj_integrate(4) =
      (_state_cur.foot_pos_world(1, 0) + _state_cur.foot_pos_world(1, 1)) / 2;
  _traj_integrate(5) = body_height_stand;
}

void H1Demo::stepping_demo(Vector3d &lin_vel_cmd, Vector3d &angle_vel_cmd,
                           Eigen::Matrix<double, 6, 1> &_traj_integrate,
                           double &_swing_height, double body_height_motion) {
  lin_vel_cmd(0) = 0.;
  lin_vel_cmd(1) = 0.;
  lin_vel_cmd(2) = 0.;
  angle_vel_cmd(0) = 0.;
  angle_vel_cmd(1) = 0.;
  angle_vel_cmd(2) = 0.;

  _traj_integrate(0) = 0;
  _traj_integrate(1) = 0;
  _traj_integrate(5) = body_height_motion;

  _swing_height = swing_height_walking;
}

void H1Demo::walking_demo(Vector3d &lin_vel_cmd, Vector3d &angle_vel_cmd,
                          Eigen::Matrix<double, 6, 1> &_traj_integrate,
                          double &_swing_height, double body_height_motion) {
  lin_vel_cmd(0) = 0.5;
  lin_vel_cmd(1) = 0.;
  lin_vel_cmd(2) = 0.;
  angle_vel_cmd(0) = 0.;
  angle_vel_cmd(1) = 0.;
  angle_vel_cmd(2) = 0.;

  _traj_integrate(0) = 0;
  _traj_integrate(1) = 0;
  _traj_integrate(5) = body_height_motion;

  _swing_height = swing_height_walking;
}