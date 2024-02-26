
#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include "dynamics.hpp"
#include <ContactSpec.hpp>

class SingleContact : public ContactSpec {
public:
  SingleContact(const FBDynModel *robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(double max_fz) { _max_Fz = max_fz; }

protected:
  double _max_Tx;
  double _max_Ty;
  double _max_Tz;
  double _max_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FBDynModel *robot_sys_;
};
