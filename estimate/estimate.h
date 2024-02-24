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
  H1Estm(std::vector<kinematics> _limb_kin);

  void call_state_estimator(H1State &state, mjData *d);

  void cheater_compute_state(H1State &state, mjData *d);
  void EKF_compute_state(H1State &state, mjData *d);

private:
  void base_to_world(Vector<double, 6> &p_end_effector_base,
                     Vector<double, 6> &v_end_effector_base,
                     Vector<double, 6> &p_end_effector_world,
                     Vector<double, 6> &v_end_effector_world,
                     Vector<double, 6> p_base_world,
                     Vector<double, 6> v_base_world);

  bool cheater_mode;

  double dt;

  std::vector<kinematics> limb_kin;
};
#endif