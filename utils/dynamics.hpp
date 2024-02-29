
#pragma once

#include "cppTypes.h"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include <pinocchio/fwd.hpp> // always include it before any other header
#include <string>
#include <vector>

using namespace Eigen;

struct FBModelState {
  Quat bodyOrientation; // world frame [wxyz]
  Vec3 bodyPosition;    // world frame
  SVec bodyVelocity;    // local frame [ang_vel, lin_vel]
  Vec19 q;  // q here is 19 dof and the order should be "left_leg", //
            // "torso_joint","right_leg", "left_arm", "right_arm"
  Vec19 qd; // here is 19 dof
};

class FBDynModel {
public:
  // vector<D6Mat<T>> _J;
  // vector<SVec<T>> _Jdqd;
  FBDynModel(
      const std::string urdf_filename = "../h1_description/urdf/h1.urdf") {

    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data temp_data(model);
    data = temp_data;

    _Jc.resize(4);
    _Jcdqd.resize(4);
    _pGC.resize(4);
    _vGC.resize(4);
  }

  ~FBDynModel() {}

  std::vector<Matrix<double, 6, 25>>
      _Jc;                   // contact jacobian list (limb point jacobian)
  Matrix<double, 6, 5> _Jcd; // contact jacobian time derivation
  std::vector<Vec6> _Jcdqd;
  FBModelState _state;
  std::vector<Vec6> _pGC; // limb point position (global)
  std::vector<Vec6> _vGC; // limb point velocity (global)

  pinocchio::Model model;
  pinocchio::Data data;

  DMat _A;
  DMat _Ainv;
  DVec _grav;
  DVec _coriolis;
  Vec25 _full_config;

  void updateModel(const FBModelState &state);

protected:
  Vec26
      q; // first 7 [global_base_position, global_base_quaternion] in pinocchio
  Vec25 v; // first 6 [local_base_velocity_linear, local_base_velocity_angular]
           // in pinocchio
  Vec25 qd;

  std::vector<std::string> limbs = {"left_foot_center", "right_foot_center",
                                    "left_hand_center", "right_hand_center"};

  void _updateMBkinmatics();
  void _updateMBdynamics();
};
