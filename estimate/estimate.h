#ifndef QUAD_ESTM_H
#define QUAD_ESTM_H

#pragma once

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "fir_filter.h"
#include "kinematics.h"
#include "orientation_tools.h"
#include "state.h"

using namespace Eigen;

class H1Estm {
public:
  H1Estm();

  void cheater_compute_state(H1State &state, mjData *d);

private:
  void base2world(Vector<double, 6> &p_end_effector_base,
                  Vector<double, 6> &v_end_effector_base,
                  Vector<double, 6> &p_end_effector_world,
                  Vector<double, 6> &v_end_effector_world,
                  Vector<double, 6> p_base_world,
                  Vector<double, 6> v_base_world);

  bool cheater_mode;

  std::vector<FIRFilter> qvel_filter;
  double dt;

  std::vector<kinematics> limb_kin;
};
#endif