
#include "kinematics.h"

kinematics::kinematics(const std::string urdf_filename) {
  pinocchio::urdf::buildModel(urdf_filename, model);
  pinocchio::Data temp_data(model);

  data = temp_data;
}

kinematics::~kinematics() {}

void kinematics::forward_kin_frame(Vector<double, 5> q, Vector<double, 5> dq,
                                   Vector<double, 6> &x, Vector<double, 6> &dx,
                                   std::string frame_name) {

  pinocchio::FrameIndex FRAME_ID = model.getFrameId(frame_name);

  pinocchio::forwardKinematics(model, data, q, dq);
  pinocchio::updateFramePlacements(model, data);

  // 获取末端位姿
  const pinocchio::SE3 &end_effector_placement = data.oMf[FRAME_ID];

  x.segment(0, 3) = end_effector_placement.translation();

  Matrix3d rot_mat(end_effector_placement.rotation());
  Quat quat_end_to_base = ori::rotationMatrixToQuaternion(rot_mat);
  x.segment(3, 3) = ori::quatToRPY(quat_end_to_base);

  // 计算并获取末端执行器的速度向量（包含线速度和角速度）
  dx = pinocchio::getFrameVelocity(model, data, FRAME_ID,
                                   pinocchio::LOCAL_WORLD_ALIGNED);

  // std::cout<<'\n'<<"x: "<<x.transpose()<< '\n'<<"dx "<< \
    // dx.transpose()<<'\n'<<"dx"<<dx.transpose()<<std::endl;
}

void kinematics::inverse_kin_frame(Vector<double, 5> &q, Vector<double, 5> &dq,
                                   Vector<double, 6> x, Vector<double, 6> dx,
                                   std::string frame_name,
                                   Vector<double, 5> q_init) {

  pinocchio::FrameIndex FRAME_ID = model.getFrameId(frame_name);

  // use Newton's Method to solve a root finding method
  Vector<double, 5> temp_q = q_init;
  const double eps = 1e-4;       // convergence tol
  const int IT_MAX = 1e4;        // max iter
  const double DT = 1e-1;        // damped Newton param
  const double threshold = 1e-4; // threshold for solving pseudo inverse

  Matrix3d rot_mat(ori::rpyToRotMat(x.segment(3, 3)));
  Vector3d l(x.segment(0, 3));
  pinocchio::SE3 oMdes(rot_mat, l);
  bool success = false;

  Vector<double, 6> err;
  pinocchio::Data::Matrix6x J(6, model.nv);
  MatrixXd J_inverse(5, 6);
  J.setZero();

  // Newton's Method(damped Newton)
  for (int i = 0; i < IT_MAX; i++) {
    pinocchio::forwardKinematics(model, data, temp_q);
    pinocchio::computeJointJacobians(model, data, temp_q);

    pinocchio::updateFramePlacements(model, data);
    pinocchio::getFrameJacobian(model, data, FRAME_ID,
                                pinocchio::LOCAL_WORLD_ALIGNED, J);

    // err = log(Trans_new.inverse - Trans_old)
    const pinocchio::SE3 iMd = data.oMf[FRAME_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();

    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= IT_MAX) {
      success = false;
      break;
    }

    // delta_q = jacobian.inverse * err
    pseudoInverse(J, threshold, J_inverse);
    Vector<double, 5> delta_q(J_inverse * err);
    // update q_new
    temp_q = pinocchio::integrate(model, temp_q, delta_q * DT);
  }

  // if (success) {
  //   std::cout << "Convergence achieved!" << std::endl;
  // } else {
  //   std::cout << "\nWarning: the iterative algorithm has not reached "
  //                "convergence to the desired precision"
  //             << std::endl;
  // }

  q = temp_q;
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::getFrameJacobian(model, data, FRAME_ID,
                              pinocchio::LOCAL_WORLD_ALIGNED, J);
  pseudoInverse(J, threshold, J_inverse);
  dq = J_inverse * dx;
}

Matrix<double, 6, 5> kinematics::get_limb_jacobian(Vector<double, 5> q,
                                                   std::string frame_name) {

  pinocchio::FrameIndex id = model.getFrameId(frame_name);

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  pinocchio::forwardKinematics(model, data, q);

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  pinocchio::getFrameJacobian(model, data, id, pinocchio::LOCAL_WORLD_ALIGNED,
                              J);

  return J;
}
