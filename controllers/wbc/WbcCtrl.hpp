
#pragma once

#include "BodyOriTask.hpp"
#include "BodyPosTask.hpp"
#include "DynWbc.hpp"
#include "KinWbc.hpp"
#include "LinkPosTask.hpp"
#include "SingleContact.hpp"
#include "cppTypes.h"
#include "dynamics.hpp"
#include "vector"

class WbcData {
public:
  FBModelState state;
  std::vector<int> contact_state;
  Vec3 pBody_des; // global
  Vec3 vBody_des; // global
  Vec3 aBody_des; //

  Quat pBodyOri_des; // global
  Vec3 vBodyOri_des; // local

  std::vector<Vec6> pEnd_des; // global
  std::vector<Vec6> vEnd_des; // global
  std::vector<Vec6> aEnd_des; // global

  std::vector<Vec6> Fr_des; // global
};

class H1Wbc {
public:
  H1Wbc();
  virtual ~H1Wbc();

  void run(const WbcData &input_data, Vec19 &joint_tau); // output joint_tau

protected:
  void _ContactTaskUpdate(const WbcData &input_data);
  void _UpdateModel(const WbcData &input_data);
  void _UpdateLimbCMD(Vec19 &joint_tau,
                      const WbcData &input_data); // output joint_tau
  void _ComputeWBC();
  void _CleanTaskContact();
  void _ParameterSetup();

  KinWBC *_kin_wbc;
  DynWbc *_dyn_wbc;
  DynWbcExtraData *_dyn_wbc_data;

  FBDynModel _model;
  std::vector<ContactSpec *> _contact_list;
  std::vector<Task *> _task_list;

  DMat _A;
  DMat _Ainv;
  DVec _grav;
  DVec _coriolis;

  DVec _full_config; // q
  DVec _tau_ff;
  DVec _des_jpos;
  DVec _des_jvel;

  Task *_body_pos_task;
  Task *_body_ori_task;

  Task *_end_task[4];
  ContactSpec *_end_contact[4];

  std::vector<double> _Kp_joint;
  std::vector<double> _Kd_joint;
  unsigned long long _iter;
};
