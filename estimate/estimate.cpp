#include "estimate.h"
#include <chrono>
#include <iostream>

H1Estm::H1Estm(kinematics *_limb_kin) {
  YAML::Node config = YAML::LoadFile("../estimate/esti_config.yaml");
  dt = 0.001;
  cheater_mode = config["cheater_mode"].as<bool>();

  limb_kin = _limb_kin;
}

// void H1Estm::base_to_world(Vector<double, 6> &p_end_effector_base,
//                            Vector<double, 6> &v_end_effector_base,
//                            Vector<double, 6> &p_end_effector_world,
//                            Vector<double, 6> &v_end_effector_world,
//                            Vector<double, 6> p_base_world,
//                            Vector<double, 6> v_base_world) {

//   const Vector3d p_end_effector_base_linear =
//       p_end_effector_base.head<3>(); // 末端执行器在基座坐标系下的位置
//   const Vector3d p_end_effector_base_angular =
//       p_end_effector_base.tail<3>(); // 末端执行器在基座坐标系下的rpy
//   const Vector3d v_end_effector_base_linear =
//       v_end_effector_base.head<3>(); // 末端执行器相对于基座的线速度
//   const Vector3d v_end_effector_base_angular =
//       v_end_effector_base.tail<3>(); // 末端执行器相对于基座的角速度

//   // 基座姿态数据
//   const Vector3d p_base_world_linear =
//       p_base_world.head<3>(); // 基座在世界坐标系下的位置
//   const Vector3d p_base_world_angular =
//       p_base_world.tail<3>(); // 基座在世界坐标系下的rpy
//   const Vector3d v_base_world_linear =
//       v_base_world.head<3>(); // 基座在世界坐标系下的线速度
//   const Vector3d v_base_world_angular =
//       v_base_world.tail<3>(); // 基座在世界坐标系下的角速度

//   // 获取基座到世界坐标系的旋转矩阵 R_world_to_base
//   const Matrix3d R_world_to_base =
//       ori::rpyToRotMat(p_base_world_angular).transpose();
//   // 获取end到base系的旋转矩阵 R_base_to_end
//   const Matrix3d R_base_to_end =
//       ori::rpyToRotMat(p_end_effector_base_angular).transpose();

//   Vector3d p_end_effector_world_linear =
//       R_world_to_base * p_end_effector_base_linear + p_base_world_linear;
//   Matrix3d R_world_to_end = R_world_to_base * R_base_to_end;
//   Quat quat_world_to_end =
//       ori::rotationMatrixToQuaternion(R_world_to_end.transpose());

//   // 合并pos和rpy得到完整的p_end_effector_world向量
//   p_end_effector_world.head<3>() = p_end_effector_world_linear;
//   p_end_effector_world.tail<3>() = ori::quatToRPY(quat_world_to_end);

//   //
//   如果末端执行器的角速度v_end_effector_base_angular是在基座坐标系下的，则在世界坐标系下的角速度应为：
//   Vector3d v_end_effector_world_angular =
//       R_world_to_base * v_end_effector_base_angular + v_base_world_angular;

//   // 将末端执行器相对于基座的线速度转换到世界坐标系下
//   Vector3d v_end_effector_world_linear =
//       R_world_to_base * v_end_effector_base_linear.head<3>() +
//       v_base_world_linear;

//   // 合并线速度和角速度得到完整的速度向量
//   v_end_effector_world.head<3>() = v_end_effector_world_linear;
//   v_end_effector_world.tail<3>() = v_end_effector_world_angular;
// }

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
  // std::cout << "state.rot_mat\n" << state.rot_mat << std::endl;
  // std::cout << "state.euler_angle\n"
  //           << ori::rpyToRotMat(state.euler_angle) << std::endl;

  // --- get linvel ---
  for (int i = 0; i < 3; ++i) {
    state.lin_vel(i) = d->qvel[i];
  }

  // --- get angvel ---
  Vector3d omegaBody;
  for (int i = 0; i < 3; ++i) {
    omegaBody(i) = d->sensordata[i];
  }
  state.omega = state.rot_mat.transpose() * omegaBody;

  double cos1, tan1, cos2, sin2, cos0, sin0;
  cos1 = cos(state.euler_angle(1));
  tan1 = tan(state.euler_angle(1));
  cos2 = cos(state.euler_angle(2));
  sin2 = sin(state.euler_angle(2));
  cos0 = cos(state.euler_angle(0));
  sin0 = sin(state.euler_angle(0));
  Matrix3d trans_mat;
  trans_mat << cos2 / cos1, sin2 / cos1, 0, -sin2, cos2, 0, cos2 * tan1,
      sin2 * tan1, 1;
  state.euler_angle_vel = trans_mat * state.omega;

  // std::cout << "state.euler_angle_vel\n"
  //           << state.euler_angle_vel.transpose() << std::endl;
  // std::cout << "state.euler_angle_vel_world\n"
  //           << (state.rot_mat * state.euler_angle_vel).transpose() <<
  //           std::endl;
  // std::cout << "state.omega\n"
  //           << state.omega.transpose() << std::endl;
  // std::cout << "omegaBody\n" << omegaBody.transpose() <<
  // std::endl; std::cout << "state.omega2euler\n"
  //           << (trans_mat * state.omega).transpose() << std::endl;
  // trans_mat.setZero();
  // trans_mat << 1, sin0 * tan1, cos0 * tan1, 0, cos0, -sin0, 0, sin0 / cos1,
  //     cos0 / cos1;
  // std::cout << "omegaBody2euler\n"
  //           << (trans_mat * omegaBody).transpose() << std::endl;

  // --- get leg_qpos and leg_qvel ---
  int leg_joint_num = 5;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < leg_joint_num; ++j) {
      state.leg_qpos(j, i) = d->qpos[leg_joint_num * i + j + 7];
      state.leg_qvel(j, i) = d->qvel[leg_joint_num * i + j + 6];
    }
  }
  // std::cout << "leg_qpos = \n" << state.leg_qpos.transpose() << std::endl;

  int arm_joint_num = 4;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < arm_joint_num; ++j) {
      state.arm_qpos(j, i) =
          d->qpos[arm_joint_num * i + j + 7 + 2 * leg_joint_num];
      state.arm_qvel(j, i) =
          d->qvel[arm_joint_num * i + j + 6 + 2 * leg_joint_num];
    }
  }
  // std::cout << "arm_qpos = \n" << state.arm_qpos.transpose() << std::endl;

  // --- get foot_pos, foot_vel ---
  std::vector<std::string> frame_name;
  frame_name.push_back("left_foot_center");
  frame_name.push_back("right_foot_center");
  frame_name.push_back("left_hand_center");
  frame_name.push_back("right_hand_center");

  limb_kin->forward_kin_frame(state, frame_name);

  // std::cout << '\n' << "x: " << state.pos.transpose() << std::endl;
  // std::cout << '\n' << "x: " << state.euler_angle.transpose() << std::endl;
  // std::cout << '\n'
  //           << "x: " << state.foot_pos_world.col(0).transpose() << std::endl;
  // std::cout << '\n'
  //           << "x: " << state.foot_pos_world.col(1).transpose() << std::endl;
  // std::cout << '\n'
  //           << "x: " << state.hand_pos_world.col(0).transpose() << std::endl;
  // std::cout << '\n'
  //           << "x: " << state.hand_pos_world.col(1).transpose() << std::endl;
}

void H1Estm::call_state_estimator(H1State &state, mjData *d) {
  if (cheater_mode)
    cheater_compute_state(state, d);
}
