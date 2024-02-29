#include "estimate.h"
#include <chrono>
#include <iostream>

H1Estm::H1Estm(std::vector<kinematics> _limb_kin) {
  YAML::Node config = YAML::LoadFile("../estimate/esti_config.yaml");
  dt = 0.001;
  cheater_mode = config["cheater_mode"].as<bool>();

  limb_kin = _limb_kin;
}

void H1Estm::base_to_world(Vector<double, 6> &p_end_effector_base,
                           Vector<double, 6> &v_end_effector_base,
                           Vector<double, 6> &p_end_effector_world,
                           Vector<double, 6> &v_end_effector_world,
                           Vector<double, 6> p_base_world,
                           Vector<double, 6> v_base_world) {

  const Vector3d p_end_effector_base_linear =
      p_end_effector_base.head<3>(); // 末端执行器在基座坐标系下的位置
  const Vector3d p_end_effector_base_angular =
      p_end_effector_base.tail<3>(); // 末端执行器在基座坐标系下的rpy
  const Vector3d v_end_effector_base_linear =
      v_end_effector_base.head<3>(); // 末端执行器相对于基座的线速度
  const Vector3d v_end_effector_base_angular =
      v_end_effector_base.tail<3>(); // 末端执行器相对于基座的角速度

  // 基座姿态数据
  const Vector3d p_base_world_linear =
      p_base_world.head<3>(); // 基座在世界坐标系下的位置
  const Vector3d p_base_world_angular =
      p_base_world.tail<3>(); // 基座在世界坐标系下的rpy
  const Vector3d v_base_world_linear =
      v_base_world.head<3>(); // 基座在世界坐标系下的线速度
  const Vector3d v_base_world_angular =
      v_base_world.tail<3>(); // 基座在世界坐标系下的角速度

  // 获取基座到世界坐标系的旋转矩阵 R_world_to_base
  const Matrix3d R_world_to_base =
      ori::rpyToRotMat(p_base_world_angular).transpose();
  // 获取end到base系的旋转矩阵 R_base_to_end
  const Matrix3d R_base_to_end =
      ori::rpyToRotMat(p_end_effector_base_angular).transpose();

  Vector3d p_end_effector_world_linear =
      R_world_to_base * p_end_effector_base_linear + p_base_world_linear;
  Matrix3d R_world_to_end = R_world_to_base * R_base_to_end;
  Quat quat_world_to_end =
      ori::rotationMatrixToQuaternion(R_world_to_end.transpose());

  // 合并pos和rpy得到完整的p_end_effector_world向量
  p_end_effector_world.head<3>() = p_end_effector_world_linear;
  p_end_effector_world.tail<3>() = ori::quatToRPY(quat_world_to_end);

  // 将末端执行器相对于基座的线速度转换到世界坐标系下
  Vector3d v_end_effector_world_linear =
      R_world_to_base * v_end_effector_base_linear.head<3>() +
      v_base_world_linear;

  // 角速度部分转换会更复杂一些，因为它涉及旋转向量的转动定律
  // 如果末端执行器的角速度v_end_effector_base_angular是在基座坐标系下的，则在世界坐标系下的角速度应为：
  Vector3d v_end_effector_world_angular =
      R_world_to_base * v_end_effector_base_angular +
      v_base_world_angular.cross(p_end_effector_base_linear);

  // 合并线速度和角速度得到完整的速度向量
  v_end_effector_world.head<3>() = v_end_effector_world_linear;
  v_end_effector_world.tail<3>() = v_end_effector_world_angular;
}

// real value, need to transform if you use imu for rpy,
// because pelvis is the center but imu is on torso
void H1Estm::cheater_compute_state(H1State &state, mjData *d) {

  // std::cout << "************* cheater_mode *************" << std::endl;

  // --- get pos ---
  for (int i = 0; i < 3; ++i) {
    state.pos(i) = d->qpos[i];
  }
  // std::cout << "state.pos = \n" << state.pos.transpose() << std::endl;

  // --- get rpy ---
  Vector<double, 4> q(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
  state.euler_angle = ori::quatToRPY(q);
  state.rot_mat = ori::quaternionToRotationMatrix(q);

  // --- get linvel ---
  for (int i = 0; i < 3; ++i) {
    state.lin_vel(i) = d->qvel[i];
  }

  // --- get angvel ---
  for (int i = 0; i < 3; ++i) {
    state.euler_angle_vel(i) = d->qvel[3 + i];
  }
  state.euler_angle_vel = state.euler_angle_vel;

  // --- get leg_qpos and leg_qvel ---
  int leg_joint_num = 5;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < leg_joint_num; ++j) {
      state.leg_qpos(j, i) = d->qpos[leg_joint_num * i + j + 7];
      state.leg_qvel(j, i) = d->qvel[leg_joint_num * i + j + 6];
    }
  }
  // std::cout << "leg_qpos = \n" << state.leg_qpos.transpose() << std::endl;

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
  frame_name.push_back("left_hand_center");
  frame_name.push_back("right_hand_center");

  for (int i = 0; i < 2; ++i) {
    Vector<double, 6> p_rel, dp_rel;
    limb_kin[i].forward_kin_frame(state.leg_qpos.col(i), state.leg_qvel.col(i),
                                  p_rel, dp_rel, frame_name[i]);
    Vector<double, 6> p_base_world, v_base_world;
    p_base_world.head<3>() = state.pos;
    p_base_world.tail<3>() = state.euler_angle;
    v_base_world.head<3>() = state.lin_vel;
    v_base_world.tail<3>() = state.euler_angle_vel;

    Vector<double, 6> p_rel_world, dp_rel_world;
    state.foot_pos_base.col(i) = p_rel;
    state.foot_vel_base.col(i) = dp_rel;
    base_to_world(p_rel, dp_rel, p_rel_world, dp_rel_world, p_base_world,
                  v_base_world);

    state.foot_pos_world.col(i) = p_rel_world;
    state.foot_vel_world.col(i) = dp_rel_world;
  }
  // std::cout << "foot_pos_base = \n"
  //           << state.foot_pos_base.col(0).transpose() << std::endl;
  // std::cout << "foot_pos_world = \n"
  //           << state.foot_pos_world.col(0).transpose() << std::endl;

  // Vector<double, 5> qpos, qvel;
  // Vector<double, 6> x, dx;
  // x << 0.039468, 0.20286, -0.801049, 0, 0, 0;
  // dx = state.foot_vel_base.col(0);
  // limb_kin[0].inverse_kin_frame(qpos, qvel, x, dx, frame_name[0],
  //                               Vector<double, 5>(0, 0, -0.4, 0.8, -0.4));
  // std::cout << "leg_qpos = \n" << state.leg_qpos.transpose() << std::endl;
  // std::cout << "inverse q = \n" << qpos.transpose() << std::endl;
  // std::cout << "leg_qvel = \n" << state.leg_qvel.transpose() << std::endl;
  // std::cout << "inverse qvel = \n" << qvel.transpose() << std::endl;

  // --- get hand_pos, hand_vel ---
  for (int i = 0; i < 2; ++i) {
    Vector<double, 6> p_rel, dp_rel;
    limb_kin[2 + i].forward_kin_frame(state.arm_qpos.col(i),
                                      state.arm_qvel.col(i), p_rel, dp_rel,
                                      frame_name[2 + i]);
    Vector<double, 6> p_base_world, v_base_world;
    p_base_world.head<3>() = state.pos;
    p_base_world.tail<3>() = state.euler_angle;
    v_base_world.head<3>() = state.lin_vel;
    v_base_world.tail<3>() = state.euler_angle_vel;

    Vector<double, 6> p_rel_world, dp_rel_world;
    state.hand_pos_base.col(i) = p_rel;
    state.hand_vel_base.col(i) = dp_rel;
    base_to_world(p_rel, dp_rel, p_rel_world, dp_rel_world, p_base_world,
                  v_base_world);

    state.hand_pos_world.col(i) = p_rel_world;
    state.hand_vel_world.col(i) = dp_rel_world;
  }
  // std::cout << "hand_pos_base = \n"
  //           << state.hand_pos_base.col(0).transpose() << std::endl;
  // std::cout << "hand_pos_world = \n"
  //           << state.hand_pos_world.col(0).transpose() << std::endl;
  // std::cout << "hand_vel_base = \n"
  //           << state.hand_vel_base.col(0).transpose() << std::endl;
  // std::cout << "hand_vel_world = \n"
  //           << state.hand_vel_world.col(0).transpose() << std::endl;

  // Vector<double, 5> qpos, qvel;
  // Vector<double, 6> x, dx;
  // x = state.hand_pos_base.col(0);
  // dx = state.hand_vel_base.col(0);
  // limb_kin[2].inverse_kin_frame(qpos, qvel, x, dx, frame_name[2],
  //                               Vector<double, 5>(0, 0, 0, 0, 0));
  // std::cout << "arm_qpos = \n" << state.arm_qpos.transpose() << std::endl;
  // std::cout << "inverse q = \n" << qpos.transpose() << std::endl;
  // std::cout << "arm_qvel = \n" << state.arm_qvel.transpose() << std::endl;
  // std::cout << "inverse qvel = \n" << qvel.transpose() << std::endl;
}

void H1Estm::call_state_estimator(H1State &state, mjData *d) {
  if (cheater_mode)
    cheater_compute_state(state, d);
}
