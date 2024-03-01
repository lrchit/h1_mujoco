
#include "kinematics.h"

kinematics::kinematics(const std::string urdf_filename) {
  pinocchio::urdf::buildModel(urdf_filename, model);
  pinocchio::Data temp_data(model);

  data = temp_data;
}

kinematics::~kinematics() {}

void kinematics::forward_kin_frame(H1State &state,
                                   std::vector<std::string> frame_name) {

  Vector<double, 25> q;
  q.segment(0, 3) = state.pos;
  Quat quat_eigen = ori::rpyToQuat(state.euler_angle);
  Quat quat_pinocchio(quat_eigen[1], quat_eigen[2], quat_eigen[3],
                      quat_eigen[0]);
  q.segment(3, 4) = quat_pinocchio;
  q.segment(7, 5) = state.leg_qpos.col(0);
  q.segment(12, 5) = state.leg_qpos.col(1);
  q.segment(17, 4) = state.arm_qpos.col(0);
  q.segment(21, 4) = state.arm_qpos.col(1);

  Vector<double, 24> qd;
  qd.segment(0, 3) = state.lin_vel;
  qd.segment(3, 3) = state.euler_angle_vel;
  qd.segment(6, 5) = state.leg_qvel.col(0);
  qd.segment(11, 5) = state.leg_qvel.col(1);
  qd.segment(16, 4) = state.arm_qvel.col(0);
  qd.segment(20, 4) = state.arm_qvel.col(1);

  pinocchio::forwardKinematics(model, data, q, qd);
  pinocchio::updateFramePlacements(model, data);

  std::vector<Vector<double, 6>> x(4), xd(4);
  for (int i = 0; i < 4; ++i) {
    pinocchio::FrameIndex FRAME_ID = model.getFrameId(frame_name[i]);

    // 获取末端位姿
    const pinocchio::SE3 &end_effector_placement = data.oMf[FRAME_ID];
    x[i].segment(0, 3) = end_effector_placement.translation();
    Matrix3d rot_mat_base_to_end(end_effector_placement.rotation().transpose());
    Quat quat_base_to_end =
        ori::rotationMatrixToQuaternion(rot_mat_base_to_end);
    x[i].segment(3, 3) = ori::quatToRPY(quat_base_to_end);
    // 计算并获取末端执行器的速度向量（包含线速度和角速度）
    xd[i] = pinocchio::getFrameVelocity(model, data, FRAME_ID,
                                        pinocchio::LOCAL_WORLD_ALIGNED);
  }
  state.foot_pos_world.col(0) = x[0];
  state.foot_pos_world.col(1) = x[1];
  state.hand_pos_world.col(0) = x[2];
  state.hand_pos_world.col(1) = x[3];
  state.foot_vel_world.col(0) = xd[0];
  state.foot_vel_world.col(1) = xd[1];
  state.hand_vel_world.col(0) = xd[2];
  state.hand_vel_world.col(1) = xd[3];
}

// void kinematics::inverse_kin_frame(Vector<double, 5> &q, Vector<double, 5>
// &dq,
//                                    Vector<double, 6> x, Vector<double, 6> dx,
//                                    std::string frame_name,
//                                    Vector<double, 5> q_init) {

//   pinocchio::FrameIndex FRAME_ID = model.getFrameId(frame_name);

//   // use Newton's Method to solve a root finding method
//   Vector<double, 5> temp_q = q_init;
//   const double eps = 1e-3;       // convergence tol
//   const int IT_MAX = 1e4;        // max iter
//   const double DT = 1e-1;        // damped Newton param
//   const double threshold = 1e-3; // threshold for solving pseudo inverse

//   Matrix3d rot_mat(ori::rpyToRotMat(x.segment(3, 3)));
//   Vector3d l(x.segment(0, 3));
//   pinocchio::SE3 oMdes(rot_mat, l);
//   bool success = false;

//   Vector<double, 6> err;
//   pinocchio::Data::Matrix6x J(6, model.nv);
//   MatrixXd J_inverse(5, 6);
//   J.setZero();

//   // Newton's Method(damped Newton)
//   for (int i = 0; i < IT_MAX; i++) {
//     pinocchio::forwardKinematics(model, data, temp_q);
//     pinocchio::computeJointJacobians(model, data, temp_q);

//     pinocchio::updateFramePlacements(model, data);
//     pinocchio::getFrameJacobian(model, data, FRAME_ID,
//                                 pinocchio::LOCAL_WORLD_ALIGNED, J);

//     // err = log(Trans_new.inverse - Trans_old)
//     const pinocchio::SE3 iMd = data.oMf[FRAME_ID].actInv(oMdes);
//     err = pinocchio::log6(iMd).toVector();

//     if (err.norm() < eps) {
//       success = true;
//       break;
//     }
//     if (i >= IT_MAX) {
//       success = false;
//       break;
//     }

//     // delta_q = jacobian.inverse * err
//     pseudoInverse(J, threshold, J_inverse);
//     Vector<double, 5> delta_q(J_inverse * err);
//     // update q_new
//     temp_q = pinocchio::integrate(model, temp_q, delta_q * DT);
//   }

//   // if (success) {
//   //   std::cout << "Convergence achieved!" << std::endl;
//   // } else {
//   //   std::cout << "\nWarning: the iterative algorithm has not reached "
//   //                "convergence to the desired precision"
//   //             << std::endl;
//   // }

//   q = temp_q;
//   pinocchio::forwardKinematics(model, data, q);
//   pinocchio::computeJointJacobians(model, data, q);
//   pinocchio::updateFramePlacements(model, data);
//   pinocchio::getFrameJacobian(model, data, FRAME_ID,
//                               pinocchio::LOCAL_WORLD_ALIGNED, J);
//   pseudoInverse(J, threshold, J_inverse);
//   dq = J_inverse * dx;
// }
