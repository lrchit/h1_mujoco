
#include "WbcCtrl.hpp"
#include "cppTypes.h"
#include "orientation_tools.h"
#include <yaml-cpp/yaml.h>

H1Wbc::H1Wbc()
    : _full_config(25), // [error]?plus 7?
      _tau_ff(19), _des_jpos(19), _des_jvel(19)
// _model(urdf_filename)
{
  _iter = 0;
  _full_config.setZero();

  _kin_wbc = new KinWBC(25);

  _dyn_wbc = new DynWbc(25, &(_contact_list), &(_task_list));
  _dyn_wbc_data = new DynWbcExtraData();

  _body_ori_task = new BodyOriTask(&(_model));
  _body_pos_task = new BodyPosTask(&(_model));

  _end_contact[0] = new SingleContact(&(_model), 0);
  _end_contact[1] = new SingleContact(&(_model), 1);
  _end_contact[2] = new SingleContact(&(_model), 2);
  _end_contact[3] = new SingleContact(&(_model), 3);

  _end_task[0] = new LinkPosTask(&(_model), 0);
  _end_task[1] = new LinkPosTask(&(_model), 1);
  _end_task[2] = new LinkPosTask(&(_model), 2);
  _end_task[3] = new LinkPosTask(&(_model), 3);

  _Kp_joint.resize(10);
  _Kd_joint.resize(10);
  _ParameterSetup();
}

H1Wbc::~H1Wbc() {
  delete _kin_wbc;
  delete _dyn_wbc;
  delete _dyn_wbc_data;

  typename std::vector<Task *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

void H1Wbc::run(const WbcData &input_data, Vec19 &joint_tau) {
  ++_iter;

  _UpdateModel(input_data);

  _ContactTaskUpdate(input_data);

  _ComputeWBC();

  Vec19 joint_torques_temp;

  _UpdateLimbCMD(joint_torques_temp, input_data);

  std::vector<int> transLimb = {1, 0, 2, 3};
  for (int limb = 0; limb < 2; limb++) {
    joint_tau.segment(5 * limb, 5) =
        joint_torques_temp.segment(5 * transLimb[limb], 5);
  }
  joint_tau(10) = joint_torques_temp(10);
  for (int arm = 0; arm < 2; arm++) {
    joint_tau.segment(4 * arm + 11, 4) =
        joint_torques_temp.segment(4 * transLimb[arm + 2] + 11, 4);
  }
}

void H1Wbc::_UpdateModel(const WbcData &input_data) {
  _model.updateModel(input_data.state);
  _full_config = _model._full_config;
  _A = _model._A;
  _Ainv = _model._Ainv;
  _coriolis = _model._coriolis;
  _grav = _model._grav;
  // std::cout << "[A]" << _A.block(3, 3, 3, 3) << std::endl;
  // std::cout << _grav.transpose() <<std::endl;
}

void H1Wbc::_UpdateLimbCMD(Vec19 &joint_tau, const WbcData &input_data) {
  //[todo] the order of q qv
  //更新要发给腿部的指令 that is tau_ff + pd
  // now the joint tau is listed as the order of q in Mqdd+Cq+g = St + JF
  double err_jpos = 0;
  double err_jvel = 0;
  // std::cout<< std::endl <<"errjpos";
  for (int leg = 0; leg < 2; leg++) {
    for (int jt = 0; jt < 5; jt++) {
      err_jpos = _des_jpos(5 * leg + jt) - input_data.state.q(5 * leg + jt);
      err_jvel = _des_jvel(5 * leg + jt) - input_data.state.qd(5 * leg + jt);
      joint_tau(5 * leg + jt) = _tau_ff(5 * leg + jt) +
                                _Kp_joint[jt] * err_jpos +
                                _Kd_joint[jt] * err_jvel;
    }
  }
  err_jpos = _des_jpos(10) - input_data.state.q(10);
  err_jvel = _des_jvel(10) - input_data.state.qd(10);
  joint_tau(10) =
      _tau_ff(10) + _Kp_joint[6] * err_jpos + _Kd_joint[6] * err_jvel;
  for (int arm = 0; arm < 2; arm++) {
    for (int jt = 0; jt < 4; jt++) {
      err_jpos =
          _des_jpos(4 * arm + jt + 11) - input_data.state.q(4 * arm + jt + 11);
      err_jvel =
          _des_jvel(4 * arm + jt + 11) - input_data.state.qd(4 * arm + jt + 11);
      joint_tau(4 * arm + jt + 11) = _tau_ff(4 * arm + jt + 11) +
                                     _Kp_joint[jt + 7] * err_jpos +
                                     _Kd_joint[jt + 7] * err_jvel;
    }
  }

  // std::cout << _tau_ff.transpose() << std::endl;
  // std::cout << joint_tau.transpose() << std::endl;
  // std::cout <<"[desjpos]" <<_des_jpos.segment(0,3).transpose() <<
  // std::endl;//input_data.state.q.transpose() - std::cout <<"[realjpos]" <<
  // input_data.state.q.segment(0,3).transpose() << std::endl;//-
}

void H1Wbc::_ComputeWBC() {
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  // std::cout <<"[_full_config]"<< _full_config.transpose() <<std::endl;
  // std::cout <<"[_des_jpos]"<< _des_jpos.segment(6,3).transpose() <<std::endl;
  // std::cout <<"[_des_jvel]"<< _des_jvel.segment(6,3).transpose() <<std::endl;

  _dyn_wbc->UpdateSetting(_A, _Ainv, _coriolis, _grav);

  // std::cout << "_A:" << _A << std::endl;
  // std::cout << "grav" << _grav.transpose() <<std::endl;

  _dyn_wbc->MakeTorque(_tau_ff, _dyn_wbc_data);
  // std::cout <<"[TAU_FF]"<< _tau_ff.segment(0,3).transpose() <<std::endl;
}

void H1Wbc::_ContactTaskUpdate(const WbcData &input_data) {
  // update contact specialize and kinmatics task

  _CleanTaskContact();

  Vec3 zero_vec3;
  zero_vec3.setZero();

  _body_ori_task->UpdateTask(&(input_data.pBodyOri_des),
                             input_data.vBodyOri_des, zero_vec3); // error

  // std::cout <<"[RPY]"<< ori::quatToRPY(input_data.pBodyOri_des).transpose()
  // << std::endl;

  _body_pos_task->UpdateTask(&(input_data.pBody_des), input_data.vBody_des,
                             input_data.aBody_des);

  _task_list.push_back(_body_ori_task);
  _task_list.push_back(_body_pos_task);

  for (int limb = 0; limb < 4; limb++) {
    if (input_data.contact_state[limb] > 0) { // contact
      // std::cout << "[des_fr_contact]" <<input_data.Fr_des[limb].transpose()<<
      // std::endl;
      _end_contact[limb]->setRFDesired(input_data.Fr_des[limb]);
      _end_contact[limb]->UpdateContactSpec();
      _contact_list.push_back(_end_contact[limb]);
    } else { // no contact
      _end_task[limb]->UpdateTask(&(input_data.pEnd_des[limb]),
                                  input_data.vEnd_des[limb],
                                  input_data.aEnd_des[limb]);
      _task_list.push_back(_end_task[limb]);
    }
  }
}

void H1Wbc::_CleanTaskContact() {
  _contact_list.clear();
  _task_list.clear();
}

void H1Wbc::_ParameterSetup() {
  YAML::Node config = YAML::LoadFile("../controllers/wbc_config.yaml");

  _Kp_joint[0] = config["kp_joint_leg_1"].as<double>();
  _Kp_joint[1] = config["kp_joint_leg_2"].as<double>();
  _Kp_joint[2] = config["kp_joint_leg_3"].as<double>();
  _Kp_joint[3] = config["kp_joint_leg_4"].as<double>();
  _Kp_joint[4] = config["kp_joint_leg_5"].as<double>();
  _Kp_joint[5] = config["kp_joint_torso"].as<double>();
  _Kp_joint[6] = config["kp_joint_arm_1"].as<double>();
  _Kp_joint[7] = config["kp_joint_arm_2"].as<double>();
  _Kp_joint[8] = config["kp_joint_arm_3"].as<double>();
  _Kp_joint[9] = config["kp_joint_arm_4"].as<double>();

  _Kd_joint[0] = config["kd_joint_leg_1"].as<double>();
  _Kd_joint[1] = config["kd_joint_leg_2"].as<double>();
  _Kd_joint[2] = config["kd_joint_leg_3"].as<double>();
  _Kd_joint[3] = config["kd_joint_leg_4"].as<double>();
  _Kd_joint[4] = config["kd_joint_leg_5"].as<double>();
  _Kd_joint[5] = config["kd_joint_torso"].as<double>();
  _Kd_joint[6] = config["kd_joint_arm_1"].as<double>();
  _Kd_joint[7] = config["kd_joint_arm_2"].as<double>();
  _Kd_joint[8] = config["kd_joint_arm_3"].as<double>();
  _Kd_joint[9] = config["kd_joint_arm_4"].as<double>();

  ((BodyOriTask *)_body_ori_task)->_Kp[0] =
      config["kp_body_ori_x"].as<double>();
  ((BodyOriTask *)_body_ori_task)->_Kp[1] =
      config["kp_body_ori_y"].as<double>();
  ((BodyOriTask *)_body_ori_task)->_Kp[2] =
      config["kp_body_ori_z"].as<double>();
  ((BodyOriTask *)_body_ori_task)->_Kd[0] =
      config["kd_body_ori_x"].as<double>();
  ((BodyOriTask *)_body_ori_task)->_Kd[1] =
      config["kd_body_ori_y"].as<double>();
  ((BodyOriTask *)_body_ori_task)->_Kd[2] =
      config["kd_body_ori_z"].as<double>();

  ((BodyPosTask *)_body_pos_task)->_Kp[0] =
      config["kp_body_pos_x"].as<double>();
  ((BodyPosTask *)_body_pos_task)->_Kp[1] =
      config["kp_body_pos_y"].as<double>();
  ((BodyPosTask *)_body_pos_task)->_Kp[2] =
      config["kp_body_pos_z"].as<double>();
  ((BodyPosTask *)_body_pos_task)->_Kd[0] =
      config["kd_body_pos_x"].as<double>();
  ((BodyPosTask *)_body_pos_task)->_Kd[1] =
      config["kd_body_pos_y"].as<double>();
  ((BodyPosTask *)_body_pos_task)->_Kd[2] =
      config["kd_body_pos_z"].as<double>();

  for (int i = 0; i < 2; i++) {
    ((LinkPosTask *)_end_task[i])->_Kp[0] = config["kp_foot_x"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[1] = config["kp_foot_y"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[2] = config["kp_foot_z"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[3] = config["kp_foot_roll"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[4] =
        config["kp_foot_pitch"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[5] = config["kp_foot_yaw"].as<double>();

    ((LinkPosTask *)_end_task[i])->_Kd[0] = config["kd_foot_x"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[1] = config["kd_foot_y"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[2] = config["kd_foot_z"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[3] = config["kd_foot_roll"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[4] =
        config["kd_foot_pitch"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[5] = config["kd_foot_yaw"].as<double>();
  }
  for (int i = 2; i < 4; i++) {
    ((LinkPosTask *)_end_task[i])->_Kp[0] = config["kp_hand_x"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[1] = config["kp_hand_y"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[2] = config["kp_hand_z"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[3] = config["kp_hand_roll"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[4] =
        config["kp_hand_pitch"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kp[5] = config["kp_hand_yaw"].as<double>();

    ((LinkPosTask *)_end_task[i])->_Kd[0] = config["kd_hand_x"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[1] = config["kd_hand_y"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[2] = config["kd_hand_z"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[3] = config["kd_hand_roll"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[4] =
        config["kd_hand_pitch"].as<double>();
    ((LinkPosTask *)_end_task[i])->_Kd[5] = config["kd_hand_yaw"].as<double>();
  }

  _dyn_wbc_data->_W_floating =
      DVec::Constant(6, config["weight_floating"].as<double>());
  _dyn_wbc_data->_W_rf =
      DVec::Constant(24, config["weight_reaction_force"].as<double>());
}
