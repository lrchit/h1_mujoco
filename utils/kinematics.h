
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

#include "orientation_tools.h"
#include "pseudoInverse.h"

using namespace Eigen;

class kinematics {
private:
  /* data */
public:
  kinematics(const std::string urdf_filename);
  ~kinematics();

  // forward kin
  void leg_forward_kin_frame(Vector<double, 7> qbase, Vector<double, 6> qdbase,
                             Vector<double, 5> qlimb, Vector<double, 5> qdlimb,
                             Vector<double, 6> &x, Vector<double, 6> &dx,
                             std::string frame_name);
  void arm_forward_kin_frame(Vector<double, 7> qbase, Vector<double, 6> qdbase,
                             Vector<double, 4> qlimb, Vector<double, 4> qdlimb,
                             Vector<double, 6> &x, Vector<double, 6> &dx,
                             std::string frame_name);

  // inverse_kin
  void inverse_kin_frame(Vector<double, 5> &q, Vector<double, 5> &dq,
                         Vector<double, 6> x, Vector<double, 6> dx,
                         std::string frame_name,
                         Vector<double, 5> q_init = Vector<double, 5>(0, 0, 0,
                                                                      0, 0));

  pinocchio::Model model;
  pinocchio::Data data;
};
