
/* related header files */
#pragma once
#include <pinocchio/fwd.hpp> // always include it before any other header

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "pseudoInverse.h"

using namespace Eigen;

class kinematics {
private:
  /* data */
public:
  kinematics(const std::string urdf_filename);
  ~kinematics();

  // forward kin
  void forward_kin_frame(Vector<double, 5> q, Vector<double, 5> dq,
                         Vector<double, 5> ddq, Vector3d &x, Vector3d &dx,
                         Vector3d &ddx, std::string frame_name);

  // inverse_kin
  void inverse_kin_frame(Vector<double, 5> &q, Vector<double, 5> &dq,
                         Vector<double, 5> &ddq, Vector3d x, Vector3d dx,
                         Vector3d ddx, std::string frame_name,
                         Vector<double, 5> q_init = Vector<double, 5>(0, 0, 0,
                                                                      0, 0));

  pinocchio::Model model;
  pinocchio::Data data;

  Matrix<double, 6, 5> get_leg_jacobian(Vector<double, 5> q,
                                        std::string frame_name);
};
