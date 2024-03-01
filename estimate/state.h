
#pragma once
#include <Eigen/Dense>

struct H1State {
  H1State() {
    euler_angle.setZero();
    pos.setZero();
    euler_angle_vel.setZero();
    omega.setZero();
    lin_vel.setZero();
    lin_acc.setZero();
    rot_mat.setZero();

    foot_pos_world.setZero();
    foot_vel_world.setZero();
    hand_pos_world.setZero();
    hand_vel_world.setZero();

    leg_qpos.setZero();
    leg_qvel.setZero();
    arm_qpos.setZero();
    arm_qvel.setZero();

    contact_phase.setZero();
    grf_ref.setZero();
    joint_torque.setZero();
  }

  Eigen::Vector3d euler_angle;
  Eigen::Vector3d pos;
  Eigen::Vector3d euler_angle_vel;
  Eigen::Vector3d omega;
  Eigen::Vector3d lin_vel;
  Eigen::Vector3d lin_acc;
  Eigen::Matrix3d rot_mat;

  Eigen::Matrix<double, 6, 2> foot_pos_world;
  Eigen::Matrix<double, 6, 2> foot_vel_world;
  Eigen::Matrix<double, 6, 2> hand_pos_world;
  Eigen::Matrix<double, 6, 2> hand_vel_world;

  Eigen::Matrix<double, 5, 2> leg_qpos;
  Eigen::Matrix<double, 5, 2> leg_qvel;
  Eigen::Matrix<double, 4, 2> arm_qpos;
  Eigen::Matrix<double, 4, 2> arm_qvel;

  Eigen::Matrix<double, 4, 1> contact_phase;

  Eigen::Matrix<double, 24, 1> grf_ref;
  Eigen::Matrix<double, 18, 1> joint_torque;
};
