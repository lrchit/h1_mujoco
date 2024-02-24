
#pragma once

#include <Eigen/Eigen>
#include <iostream>

using namespace Eigen;

class foot_swing_traj {
public:
  foot_swing_traj() {
    lift_point.setZero();
    hold_point.setZero();
  }

  void set_lift_point(Vector<double, 6> _lift_point) {
    lift_point = _lift_point;
  }
  void set_hold_point(Vector<double, 6> _hold_point) {
    hold_point = _hold_point;
  }
  void set_swing_height(double _swing_height) { swing_height = _swing_height; }

  Vector<double, 6> get_swing_pos(double phase) {

    Vector<double, 6> y;

    for (int i = 0; i < 6; ++i) {
      if (i == 2) {
        if (phase < 0.5) {
          phase *= 2;
          y(i) =
              Cubic_Bezier(phase, lift_point(i), swing_height + lift_point(i));
        } else {
          phase = 2 * phase - 1;
          y(i) =
              Cubic_Bezier(phase, swing_height + lift_point(i), hold_point(i));
        }
      } else {
        y(i) = Cubic_Bezier(phase, lift_point(i), hold_point(i));
      }
    }

    return y;
  }

  Vector<double, 6> get_swing_vel(double phase) {
    Vector<double, 6> y_dot;

    for (int i = 0; i < 6; ++i) {
      if (i == 2) {
        if (phase < 0.5) {
          phase *= 2;
          y_dot(i) = Cubic_Bezier_FirstDerivative(phase, lift_point(i),
                                                  swing_height + lift_point(i));
        } else {
          phase = 2 * phase - 1;
          y_dot(i) = Cubic_Bezier_FirstDerivative(
              phase, swing_height + lift_point(i), hold_point(i));
        }
      } else {
        y_dot(i) =
            Cubic_Bezier_FirstDerivative(phase, lift_point(i), hold_point(i));
      }
    }

    return y_dot;
  }

  Vector<double, 6> get_swing_acc(double phase) {
    Vector<double, 6> y_dotdot;

    for (int i = 0; i < 6; ++i) {
      if (i == 2) {
        if (phase < 0.5) {
          phase *= 2;
          y_dotdot(i) = Cubic_Bezier_SecondDerivative(
              phase, lift_point(i), swing_height + lift_point(i));
        } else {
          phase = 2 * phase - 1;
          y_dotdot(i) = Cubic_Bezier_SecondDerivative(
              phase, swing_height + lift_point(i), hold_point(i));
        }
      } else {
        y_dotdot(i) =
            Cubic_Bezier_SecondDerivative(phase, lift_point(i), hold_point(i));
      }

      return y_dotdot;
    }
  }

  Vector<double, 6> get_lift_point() { return lift_point; }
  Vector<double, 6> get_hold_point() { return hold_point; }

private:
  // 三次贝塞尔曲线
  double Cubic_Bezier(double phase, double x0, double x1) {
    return x0 + pow(phase, 2) * (3 - 2 * phase) * (x1 - x0);
  }
  double Cubic_Bezier_FirstDerivative(double phase, double x0, double x1) {
    return 6 * phase * (1 - phase) * (x1 - x0);
  }
  double Cubic_Bezier_SecondDerivative(double phase, double x0, double x1) {
    return (6 - 12 * phase) * (x1 - x0);
  }

  Vector<double, 6> lift_point;
  Vector<double, 6> hold_point;
  double swing_height;
};
