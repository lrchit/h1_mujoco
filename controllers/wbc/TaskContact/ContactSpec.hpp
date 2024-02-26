
#pragma once

#include <cppTypes.h>

#define Contact ContactSpec
//地面期望接触情况抽象类
class ContactSpec {
public:
  ContactSpec(int dim) : dim_contact_(dim), b_set_contact_(false) {
    idx_Fz_ =
        dim -
        4; // 地面反力z方向的索引 because normally (linear_x,y,z, tau_x,y,z)
    Fr_des_ = DVec::Zero(dim);
  }
  virtual ~ContactSpec() {}

  int getDim() const { return dim_contact_; }
  int getDimRFConstraint() const { return Uf_.rows(); }
  int getFzIndex() const { return idx_Fz_; }

  void getContactJacobian(DMat &Jc) { Jc = Jc_; } // contact jacobian
  void getJcDotQdot(DVec &JcDotQdot) { JcDotQdot = JcDotQdot_; }
  void UnsetContact() { b_set_contact_ = false; }

  void getRFConstraintMtx(DMat &Uf) {
    Uf = Uf_;
  } // reaction force constraint matrix
  void getRFConstraintVecLb(DVec &ieq_vec) {
    ieq_vec = ieq_vec_lb;
  } // reaction force constraint vector
  void getRFConstraintVecUb(DVec &ieq_vec) {
    ieq_vec = ieq_vec_ub;
  } // reaction force constraint vector
  const DVec &getRFDesired() { return Fr_des_; } // desired reaction force
  void setRFDesired(const DVec &Fr_des) {
    Fr_des_ = Fr_des;
  } // set desired reaction force

  bool UpdateContactSpec() {
    _UpdateJc();
    _UpdateJcDotQdot();
    _UpdateUf();
    _UpdateInequalityVector();
    b_set_contact_ = true;
    return true;
  }

protected:
  virtual bool _UpdateJc() = 0; // update contact jacobian
  virtual bool _UpdateJcDotQdot() = 0;
  virtual bool _UpdateUf() = 0;
  virtual bool
  _UpdateInequalityVector() = 0; // update reaction force constraint vector

  int idx_Fz_;
  DMat Uf_;
  DVec ieq_vec_lb; // reaction force constraint vector
  DVec ieq_vec_ub; // reaction force constraint vector
  DVec Fr_des_;

  DMat Jc_;
  DVec JcDotQdot_;
  int dim_contact_;
  bool b_set_contact_;
};