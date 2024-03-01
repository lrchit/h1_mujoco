
#pragma once

// (X, Y, Z, Roll, Pitch, Yaw)
#include "dynamics.hpp"
#include <Task.hpp>

class LegPosTask : public Task {
public:
  LegPosTask(const FBDynModel *, int link_idx, bool virtual_depend = true);
  virtual ~LegPosTask();

  DVec _Kp, _Kd, _Kp_kin;

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void *pos_des, const DVec &vel_des,
                              const DVec &acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }

  const FBDynModel *robot_sys_;
  int link_idx_;        // link index
  bool virtual_depend_; // needing virtual joint or not
};
