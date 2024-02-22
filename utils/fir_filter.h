/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: jingfan.zhang
 * Build Date: 01/05/2023
 * Modify Date:
 *************************************************************/

#pragma once

#include <iostream>

#include "eigen3/Eigen/Dense"

class FIRFilter {
public:
  /**
   * @brief constructor
   */
  FIRFilter();

  /**
   * @brief destructor
   */
  ~FIRFilter();

  double VelLowpassFirFilter(const double data_now);

  // same cuttoff frequency, higher order filter
  double VelLowpassFirFilterHighOrder(const double data_now);

private:
  int data_count;
  int length_fir;
  Eigen::VectorXd hfir;        // fir low pass filter coefficient
  Eigen::VectorXd data_last_n; // last n data
  Eigen::VectorXd data_last_temp;

  int data_count_ho;
  int length_fir_ho;              // ho: high order
  Eigen::VectorXd hfir_ho;        // fir low pass filter coefficient
  Eigen::VectorXd data_last_n_ho; // last n data
  Eigen::VectorXd data_last_temp_ho;
};
