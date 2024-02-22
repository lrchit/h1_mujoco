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
  bool cheater_mode;

  H1State last_state;

  std::vector<FIRFilter> qvel_filter;
  double dt;

  std::vector<kinematics> limb_kin;
};
#endif