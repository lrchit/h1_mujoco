
#pragma once
#include <Eigen/Dense>

struct H1State {
  H1State() {
    euler_angle.setZero();
    pos.setZero();
    euler_angle_vel.setZero();
    lin_vel.setZero();
    lin_acc.setZero();
    rot_mat.setZero();

    foot_pos.setZero();
    foot_vel.setZero();
    foot_pos_world.setZero();
    foot_vel_world.setZero();
    hand_pos.setZero();
    hand_vel.setZero();
    hand_pos_world.setZero();
    hand_vel_world.setZero();

    leg_qpos.setZero();
    leg_qvel.setZero();
    arm_qpos.setZero();
    arm_qvel.setZero();

    torso_qpos = 0;
    torso_qvel = 0;

    contact_phase.setZero();
    grf_ref.setZero();
    joint_torque.setZero();
  }

  Eigen::Vector3d euler_angle;
  Eigen::Vector3d pos;
  Eigen::Vector3d euler_angle_vel;
  Eigen::Vector3d lin_vel;
  Eigen::Vector3d lin_acc;
  Eigen::Matrix3d rot_mat;

  Eigen::Matrix<double, 3, 2> foot_pos;
  Eigen::Matrix<double, 3, 2> foot_vel;
  Eigen::Matrix<double, 3, 2> foot_pos_world;
  Eigen::Matrix<double, 3, 2> foot_vel_world;
  Eigen::Matrix<double, 3, 2> hand_pos;
  Eigen::Matrix<double, 3, 2> hand_vel;
  Eigen::Matrix<double, 3, 2> hand_pos_world;
  Eigen::Matrix<double, 3, 2> hand_vel_world;

  Eigen::Matrix<double, 5, 2> leg_qpos;
  Eigen::Matrix<double, 5, 2> leg_qvel;
  // for convenience, the 1st one is torso_qpos
  Eigen::Matrix<double, 5, 2> arm_qpos;
  Eigen::Matrix<double, 5, 2> arm_qvel;

  double torso_qpos;
  double torso_qvel;

  Eigen::Matrix<double, 4, 1> contact_phase;

  Eigen::Matrix<double, 10, 1> grf_ref;
  Eigen::Matrix<double, 18, 1> joint_torque;
};
