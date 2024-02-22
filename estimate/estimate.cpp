#include "estimate.h"
#include <chrono>
#include <iostream>

H1Estm::H1Estm() {
  YAML::Node config = YAML::LoadFile("../estimate/esti_config.yaml");
  dt = 0.0005;
  cheater_mode = config["cheater_mode"].as<bool>();

  qvel_filter.resize(16);

  std::vector<std::string> filename;
  filename.push_back("../h1_description/urdf/h1_left_leg.urdf");
  filename.push_back("../h1_description/urdf/h1_right_leg.urdf");
  filename.push_back("../h1_description/urdf/h1_left_arm.urdf");
  filename.push_back("../h1_description/urdf/h1_right_arm.urdf");
  limb_kin.push_back(kinematics(filename[0]));
  limb_kin.push_back(kinematics(filename[1]));
  limb_kin.push_back(kinematics(filename[2]));
  limb_kin.push_back(kinematics(filename[3]));
}

// real value, need to transform if you use imu for rpy,
// because pelvis is the center but imu is on torso
void H1Estm::cheater_compute_state(H1State &state, mjData *d) {

  // std::cout << "************* cheater_mode *************" << std::endl;

  // --- get pos ---
  for (int i = 0; i < 3; ++i) {
    state.pos(i) = d->qpos[i];
  }
  // std::cout << "pos = \n" << state.pos.transpose() << std::endl;

  // --- get rpy ---
  Vector<double, 4> q(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
  state.euler_angle = ori::quatToRPY(q);
  state.rot_mat = ori::quaternionToRotationMatrix(q);
  // std::cout << "euler_angle = \n" << state.euler_angle.transpose() <<
  // std::endl; std::cout << "q = \n" << q.transpose() << std::endl;

  // --- get linvel ---
  for (int i = 0; i < 3; ++i) {
    state.lin_vel(i) = d->qvel[i];
  }
  // std::cout << "lin_vel = \n" << state.lin_vel.transpose() << std::endl;

  // --- get angvel ---
  for (int i = 0; i < 3; ++i) {
    state.euler_angle_vel(i) = d->qvel[3 + i];
  }
  state.euler_angle_vel = state.euler_angle_vel;
  // std::cout << "euler_angle_vel = \n" << state.euler_angle_vel.transpose()
  //           << std::endl;

  // --- get leg_qpos and leg_qvel ---
  int leg_joint_num = 5;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < leg_joint_num; ++j) {
      state.leg_qpos(j, i) = d->qpos[leg_joint_num * i + j + 7];
      state.leg_qvel(j, i) = d->qvel[leg_joint_num * i + j + 6];
    }
  }

  // --- get torso_qpos and torso_qvel ---
  state.torso_qpos = d->qpos[7 + 2 * leg_joint_num];
  state.torso_qvel = d->qvel[6 + 2 * leg_joint_num];

  // --- get arm_qpos and arm_qvel ---
  state.arm_qpos(0, 0) = d->qpos[7 + 2 * leg_joint_num];
  state.arm_qvel(0, 0) = d->qvel[6 + 2 * leg_joint_num];
  state.arm_qpos(0, 1) = d->qpos[7 + 2 * leg_joint_num];
  state.arm_qvel(0, 1) = d->qvel[6 + 2 * leg_joint_num];
  int arm_joint_num = 4;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < arm_joint_num; ++j) {
      state.arm_qpos(j + 1, i) =
          d->qpos[arm_joint_num * i + j + 7 + 2 * leg_joint_num + 1];
      state.arm_qvel(j + 1, i) =
          d->qvel[arm_joint_num * i + j + 6 + 2 * leg_joint_num + 1];
    }
  }
  // std::cout << "arm_qpos = \n" << state.arm_qpos.transpose() << std::endl;

  // --- get foot_pos, foot_vel ---
  std::vector<std::string> frame_name;
  frame_name.push_back("left_foot_center");
  frame_name.push_back("right_foot_center");
  frame_name.push_back("left_elbow_link");
  frame_name.push_back("right_elbow_link");

  for (int i = 0; i < 2; ++i) {
    Vector3d p_rel, dp_rel, ddp_rel;
    limb_kin[i].forward_kin_frame(state.leg_qpos.col(i), state.leg_qvel.col(i),
                                  Vector<double, 5>::Zero(), p_rel, dp_rel,
                                  ddp_rel, frame_name[i]);
    state.foot_pos.col(i) = p_rel;
    state.foot_pos_world.col(i) =
        state.rot_mat.transpose() * state.foot_pos.col(i) + state.pos;
    state.foot_vel.col(i) = dp_rel;
    state.foot_vel_world.col(i) =
        state.rot_mat.transpose() * state.foot_vel.col(i) + state.lin_vel;
  }
  Vector<double, 5> qpos, qvel, qacc;
  Vector3d x, dx, ddx;
  x = state.foot_pos.col(0);
  dx = state.foot_vel.col(0);
  limb_kin[0].inverse_kin_frame(qpos, qvel, qacc, x, dx, ddx, frame_name[0],
                                last_state.leg_qpos.col(0));
  std::cout << "leg_qpos = \n" << state.leg_qpos.transpose() << std::endl;
  std::cout << "inverse q = \n" << qpos.transpose() << std::endl;
  std::cout << "leg_qvel = \n" << state.leg_qvel.transpose() << std::endl;
  std::cout << "inverse qvel = \n" << qvel.transpose() << std::endl;

  // --- get hand_pos, hand_vel ---
  for (int i = 0; i < 2; ++i) {
    Vector3d p_rel, dp_rel, ddp_rel;
    limb_kin[2 + i].forward_kin_frame(
        state.arm_qpos.col(i), state.arm_qvel.col(i), Vector<double, 5>::Zero(),
        p_rel, dp_rel, ddp_rel, frame_name[2 + i]);
    state.hand_pos.col(i) = p_rel;
    state.hand_pos_world.col(i) =
        state.rot_mat.transpose() * state.hand_pos.col(i) + state.pos;
    state.hand_vel.col(i) = dp_rel;
    state.hand_vel_world.col(i) =
        state.rot_mat.transpose() * state.hand_vel.col(i) + state.lin_vel;
  }
  // Vector<double, 5> qpos, qvel, qacc;
  // Vector3d x, dx, ddx;
  // x = state.hand_pos.col(0);
  // dx = state.hand_vel.col(0);
  // limb_kin[2].inverse_kin_frame(qpos, qvel, qacc, x, dx, ddx, frame_name[2],
  //                               last_state.arm_qpos.col(2));
  // std::cout << "arm_qpos = \n" << state.arm_qpos.transpose() << std::endl;
  // std::cout << "inverse q = \n" << qpos.transpose() << std::endl;
  // std::cout << "arm_qvel = \n" << state.arm_qvel.transpose() << std::endl;
  // std::cout << "inverse qvel = \n" << qvel.transpose() << std::endl;

  last_state = state;
}