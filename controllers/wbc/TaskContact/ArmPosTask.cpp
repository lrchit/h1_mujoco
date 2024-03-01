
#include "ArmPosTask.hpp"
#include <orientation_tools.h>

// (X, Y, Z, Roll, Pitch, Yaw)
ArmPosTask::ArmPosTask(const FBDynModel *robot, int link_idx,
                       bool virtual_depend)
    : Task(6), robot_sys_(robot), link_idx_(link_idx),
      virtual_depend_(virtual_depend) {
  TK::Jt_ = DMat::Zero(TK::dim_task_, 24); // configure space dim
  TK::JtDotQdot_ = DVec::Zero(TK::dim_task_);

  _Kp = DVec::Constant(TK::dim_task_, 100.);
  _Kd = DVec::Constant(TK::dim_task_, 5.);
  _Kp_kin = DVec::Constant(TK::dim_task_, 1.);
}

ArmPosTask::~ArmPosTask() {}

bool ArmPosTask::_UpdateCommand(const void *pos_des, const DVec &vel_des,
                                const DVec &acc_des) {
  Vec3 pos_cmd = ((Vec6 *)pos_des)->segment(0, 3);
  Vec3 link_pos(robot_sys_->_pGC[link_idx_].segment(0, 3));

  Vec3 ori_cmd_ = ((Vec6 *)pos_des)->segment(3, 3);
  Quat ori_cmd = ori::rpyToQuat(ori_cmd_);
  Vec3 link_ori_ = robot_sys_->_pGC[link_idx_].segment(3, 3);
  Quat link_ori = ori::rpyToQuat(link_ori_);

  Quat link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];
  // link_ori_inv /= link_ori.norm();

  // Explicit because operational space is in global frame
  Quat ori_err = ori::quatProduct(ori_cmd, link_ori_inv);

  // std::cout <<"[RPY]"<< ori::quatToRPY(link_ori).transpose() << std::endl;
  // std::cout <<"[RPY]"<< ori::quatToRPY(*ori_cmd).transpose() << std::endl;

  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3 ori_err_so3;
  ori::quaternionToso3(ori_err, ori_err_so3);

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ((pos_cmd)[i] - link_pos[i]);
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    // Op acceleration command
    TK::op_cmd_[i] =
        _Kp[i] * TK::pos_err_[i] +
        _Kd[i] * (TK::vel_des_[i] - robot_sys_->_vGC[link_idx_][i]) +
        TK::acc_des_[i];
  }

  // Rx, Ry, Rz
  for (int i(3); i < 6; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i - 3];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    // Op acceleration command
    TK::op_cmd_[i] =
        _Kp[i] * ori_err_so3[i - 3] +
        _Kd[i] * (TK::vel_des_[i] - robot_sys_->_vGC[link_idx_][i]) +
        TK::acc_des_[i];
  }

  // if (link_idx_ == 2) {
  //   std::cout << "ori_err_so3\n" << ori_err_so3.transpose() << std::endl;
  //   //   std::cout << "op_cmd_\n" << op_cmd_.transpose() << std::endl;
  //   //   std::cout << "pos_cmd\n" << pos_cmd->transpose() << std::endl;
  //   // std::cout << "pos_err_\n" << pos_err_.transpose() << std::endl;
  //   //   // std::cout << "vel_des_\n" << vel_des_.transpose() << std::endl;
  //   //   // std::cout << "curr_vel\n"
  //   //   //           << robot_sys_->_vGC[link_idx_].transpose() <<
  //   std::endl;
  //   //   // std::cout << "acc_des_\n" << acc_des_.transpose() << std::endl;
  // }

  // Vec6 *pos_cmd = (Vec6 *)pos_des;
  // Vec6 link_pos;

  // link_pos = robot_sys_->_pGC[link_idx_];

  // // X, Y, Z
  // for (int i(0); i < TK::dim_task_; ++i) {
  //   TK::pos_err_[i] = _Kp_kin[i] * ((*pos_cmd)[i] - link_pos[i]);
  //   TK::vel_des_[i] = vel_des[i];
  //   TK::acc_des_[i] = acc_des[i];
  // }

  // // Op acceleration command
  // for (size_t i(0); i < TK::dim_task_; ++i) {
  //   TK::op_cmd_[i] =
  //       _Kp[i] * TK::pos_err_[i] +
  //       _Kd[i] * (TK::vel_des_[i] - robot_sys_->_vGC[link_idx_][i]) +
  //       TK::acc_des_[i];
  // }

  return true;
}

bool ArmPosTask::_UpdateTaskJacobian() {
  TK::Jt_ = robot_sys_->_Jc[link_idx_];
  // std::cout << "TK::Jt_\n" << TK::Jt_ << std::endl;
  if (!virtual_depend_) {
    TK::Jt_.block(0, 0, 4, 6) = DMat::Zero(4, 6);
  }
  return true;
}

bool ArmPosTask::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_ = robot_sys_->_Jcdqd[link_idx_];
  return true;
}