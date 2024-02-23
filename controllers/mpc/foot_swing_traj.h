#include <Eigen/Eigen>
#include <iostream>

using namespace Eigen;

class foot_swing_traj {
public:
  foot_swing_traj() {
    lift_point.setZero();
    hold_point.setZero();
  }

  void set_lift_point(Vector3d _lift_point) { lift_point = _lift_point; }
  void set_hold_point(Vector3d _hold_point) {
    hold_point = _hold_point;
    set_intermediate_point();
  }
  void set_swing_height(double _swing_height) { swing_height = _swing_height; }

  Vector3d get_swing_pos(double phase) {

    Vector3d y;

    y(0) = Quintic_Bezier(phase, lift_point(0), p1(0), p2(0), p3(0), p4(0),
                          hold_point(0));
    y(1) = Quintic_Bezier(phase, lift_point(1), p1(1), p2(1), p3(1), p4(1),
                          hold_point(1));
    if (phase < 0.5) {
      phase *= 2;
      y(2) = Quintic_Bezier(phase, lift_point(2), z1, z2, z3, z4,
                            swing_height + lift_point(2));
    } else {
      phase = 2 * phase - 1;
      y(2) = Quintic_Bezier(phase, swing_height + lift_point(2), z5, z6, z7, z8,
                            hold_point(2));
    }

    // y(0) = Cubic_Bezier(phase, lift_point(0), hold_point(0));
    // y(1) = Cubic_Bezier(phase, lift_point(1), hold_point(1));
    // if (phase < 0.5) {
    //   phase *= 2;
    //   y(2) = Cubic_Bezier(phase, lift_point(2), swing_height +
    //   lift_point(2));
    // } else {
    //   phase = 2 * phase - 1;
    //   y(2) = Cubic_Bezier(phase, swing_height + lift_point(2),
    //   hold_point(2));
    // }

    return y;
  }
  Vector3d get_swing_vel(double phase) {
    Vector3d y_dot;

    y_dot(0) = Quintic_Bezier_FirstDerivative(
        phase, lift_point(0), p1(0), p2(0), p3(0), p4(0), hold_point(0));
    y_dot(1) = Quintic_Bezier_FirstDerivative(
        phase, lift_point(1), p1(1), p2(1), p3(1), p4(1), hold_point(1));
    if (phase < 0.5) {
      phase *= 2;
      y_dot(2) = Quintic_Bezier_FirstDerivative(
          phase, lift_point(2), z1, z2, z3, z4, swing_height + lift_point(2));
    } else {
      phase = 2 * phase - 1;
      y_dot(2) = Quintic_Bezier_FirstDerivative(
          phase, swing_height + lift_point(2), z5, z6, z7, z8, hold_point(2));
    }

    // y_dot(0) =
    //     Cubic_Bezier_FirstDerivative(phase, lift_point(0), hold_point(0));
    // y_dot(1) =
    //     Cubic_Bezier_FirstDerivative(phase, lift_point(1), hold_point(1));
    // if (phase < 0.5) {
    //   phase *= 2;
    //   y_dot(2) = Cubic_Bezier_FirstDerivative(phase, lift_point(2),
    //                                           swing_height + lift_point(2));
    // } else {
    //   phase = 2 * phase - 1;
    //   y_dot(2) = Cubic_Bezier_FirstDerivative(
    //       phase, swing_height + lift_point(2), hold_point(2));
    // }

    return y_dot;
  }
  Vector3d get_swing_acc(double phase) {
    Vector3d y_dotdot;

    y_dotdot(0) = Quintic_Bezier_SecondDerivative(
        phase, lift_point(0), p1(0), p2(0), p3(0), p4(0), hold_point(0));
    y_dotdot(1) = Quintic_Bezier_SecondDerivative(
        phase, lift_point(1), p1(1), p2(1), p3(1), p4(1), hold_point(1));
    if (phase < 0.5) {
      phase *= 2;
      y_dotdot(2) = Quintic_Bezier_SecondDerivative(
          phase, lift_point(2), z1, z2, z3, z4, swing_height + lift_point(2));
    } else {
      phase = 2 * phase - 1;
      y_dotdot(2) = Quintic_Bezier_SecondDerivative(
          phase, swing_height + lift_point(2), z5, z6, z7, z8, hold_point(2));
    }

    // y_dotdot(0) =
    //     Cubic_Bezier_SecondDerivative(phase, lift_point(0), hold_point(0));
    // y_dotdot(1) =
    //     Cubic_Bezier_SecondDerivative(phase, lift_point(1), hold_point(1));
    // if (phase < 0.5) {
    //   phase *= 2;
    //   y_dotdot(2) = Cubic_Bezier_SecondDerivative(phase, lift_point(2),
    //                                               swing_height +
    //                                               lift_point(2));
    // } else {
    //   phase = 2 * phase - 1;
    //   y_dotdot(2) = Cubic_Bezier_SecondDerivative(
    //       phase, swing_height + lift_point(2), hold_point(2));
    // }

    return y_dotdot;
  }

  Vector3d get_lift_point() { return lift_point; }
  Vector3d get_hold_point() { return hold_point; }

private:
  // 设置中间点
  void set_intermediate_point() {

    // [todo] 应该调整插值点位置
    p1(0) = lift_point(0) - 1 * (hold_point(0) - lift_point(0)) / 16;
    p1(1) = lift_point(1) - 1 * (hold_point(1) - lift_point(1)) / 16;
    p2(0) = lift_point(0);
    p2(1) = lift_point(1);
    z1 = lift_point(2) + 9 * swing_height / 16;
    z2 = z1;
    z3 = lift_point(2) + 13 * swing_height / 16;
    z4 = lift_point(2) + swing_height;

    p3(0) = hold_point(0);
    p3(1) = hold_point(1);
    p4(0) = hold_point(0) + 1 * (hold_point(0) - lift_point(0)) / 16;
    p4(1) = hold_point(1) + 1 * (hold_point(1) - lift_point(1)) / 16;
    z5 = hold_point(2) +
         13 * (lift_point(2) + swing_height - hold_point(2)) / 16;
    z6 = z5;
    z7 =
        hold_point(2) + 9 * (lift_point(2) + swing_height - hold_point(2)) / 16;
    z8 = hold_point(2);
  }

  // 五次贝塞尔曲线
  double Quintic_Bezier(double phase, double x0, double x1, double x2,
                        double x3, double x4, double x5) {
    return pow(phase, 5) * x5 + 5 * pow(phase, 4) * (1 - phase) * x4 +
           10 * pow(phase, 3) * pow(1 - phase, 2) * x3 +
           10 * pow(phase, 2) * pow(1 - phase, 3) * x2 +
           5 * phase * pow(1 - phase, 4) * x1 + pow(1 - phase, 5) * x0;
  }
  double Quintic_Bezier_FirstDerivative(double phase, double x0, double x1,
                                        double x2, double x3, double x4,
                                        double x5) {
    return 5 * x5 * pow(phase, 4) - 5 * x4 * pow(phase, 4) -
           5 * x0 * pow(phase - 1, 4) + 5 * x1 * pow(phase - 1, 4) +
           20 * x1 * phase * pow(phase - 1, 3) -
           20 * x2 * phase * pow(phase - 1, 3) -
           20 * x4 * pow(phase, 3) * (phase - 1) +
           10 * x3 * pow(phase, 3) * (2 * phase - 2) -
           30 * x2 * pow(phase, 2) * pow(phase - 1, 2) +
           30 * x3 * pow(phase, 2) * pow(phase - 1, 2);
  }
  double Quintic_Bezier_SecondDerivative(double phase, double x0, double x1,
                                         double x2, double x3, double x4,
                                         double x5) {
    return 20 * x5 * pow(phase, 3) + 20 * x3 * pow(phase, 3) -
           40 * x4 * pow(phase, 3) - 20 * x0 * pow(phase - 1, 3) +
           40 * x1 * pow(phase - 1, 3) - 20 * x2 * pow(phase - 1, 3) +
           60 * x1 * phase * pow(phase - 1, 2) -
           120 * x2 * phase * pow(phase - 1, 2) +
           60 * x3 * phase * pow(phase - 1, 2) -
           60 * x4 * pow(phase, 2) * (phase - 1) -
           30 * x2 * pow(phase, 2) * (2 * phase - 2) +
           60 * x3 * pow(phase, 2) * (2 * phase - 2);
  }

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

  Vector3d lift_point;
  Vector3d hold_point;
  double swing_height;

  Vector3d p1;
  Vector3d p2;
  Vector3d p3;
  Vector3d p4;
  double z1, z2, z3, z4, z5, z6, z7, z8;
};
