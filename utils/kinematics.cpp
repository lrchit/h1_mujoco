
#include "kinematics.h"

kinematics::kinematics(const std::string urdf_filename) {
  pinocchio::urdf::buildModel(urdf_filename, model);
  pinocchio::Data temp_data(model);

  data = temp_data;
}

kinematics::~kinematics() {}

void kinematics::forward_kin_frame(Vector<double, 5> q, Vector<double, 5> dq,
                                   Vector<double, 5> ddq, Vector3d &x,
                                   Vector3d &dx, Vector3d &ddx,
                                   std::string frame_name) {

  pinocchio::FrameIndex id = model.getFrameId(frame_name);

  pinocchio::forwardKinematics(model, data, q, dq, ddq);
  pinocchio::updateFramePlacements(model, data);
  // pinocchio::framesForwardKinematics(model, data, q);
  x = data.oMf[id].translation();
  dx = pinocchio::getFrameVelocity(model, data, id,
                                   pinocchio::LOCAL_WORLD_ALIGNED)
           .linear();

  ddx = pinocchio::getFrameAcceleration(model, data, id,
                                        pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();

  // std::cout<<'\n'<<"x: "<<x.transpose()<< '\n'<<"dx "<< \
    // dx.transpose()<<'\n'<<"ddx"<<ddx.transpose()<<std::endl;
}

void kinematics::inverse_kin_frame(Vector<double, 5> &q, Vector<double, 5> &dq,
                                   Vector<double, 5> &ddq, Vector3d x,
                                   Vector3d dx, Vector3d ddx,
                                   std::string frame_name,
                                   Vector<double, 5> q_init) {

  pinocchio::FrameIndex id = model.getFrameId(frame_name);

  Vector<double, 5> temp_q = q_init;
  const double eps = 1e-4;
  const int IT_MAX = 1e3;
  const double DT = 1e-1;
  const double damp = 1e-6;
  const double threshold = 0.001;

  pinocchio::SE3 oMdes(Matrix3d::Identity(), x);
  bool success = false;

  Vector3d err;
  pinocchio::Data::Matrix6x J(6, model.nv);
  pinocchio::Data::Matrix6x dJ(6, model.nv);
  MatrixXd J_(3, 5);
  MatrixXd J_inverse(5, 3);
  // Matrix<double,6,model.nv> dJ;
  J.setZero();
  dJ.setZero();
  int i = 0;
  for (i = 0; i < IT_MAX; i++) {
    pinocchio::forwardKinematics(model, data, temp_q);
    pinocchio::computeJointJacobians(
        model, data,
        temp_q); // if we use this ,we can get frame or joint jacobian

    pinocchio::updateFramePlacements(model, data);
    pinocchio::getFrameJacobian(model, data, id, pinocchio::LOCAL_WORLD_ALIGNED,
                                J); // cal this, you need forwardKinematics

    pinocchio::SE3 T_temp = data.oMf[id];
    err = oMdes.translation() -
          T_temp.translation(); // must compute the forward kinematics

    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= IT_MAX) {
      success = false;
      break;
    }

    J_ = J.block(0, 0, 3, 5);
    pseudoInverse(J_, threshold, J_inverse);
    dq = J_inverse * err;
    temp_q = pinocchio::integrate(model, temp_q, dq * DT);

    //  if(!(i%10))
    //  std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  // if (success) {
  //   std::cout << "Convergence achieved!" << std::endl;
  // } else {
  //   std::cout << "\nWarning: the iterative algorithm has not reached "
  //                "convergence to the desired precision"
  //             << std::endl;
  // }

  q = temp_q;
  J_ = J.block(0, 0, 3, 5);
  dq = J_inverse * dx;
  // pinocchio::computeJointJacobiansTimeVariation(
  //     model, data, q, dq); // if we use this .we can get frame or joint
  //                          // derviation of jacobians
  // pinocchio::getFrameJacobianTimeVariation(
  //     model, data, id, pinocchio::LOCAL_WORLD_ALIGNED,
  //     dJ); // this can replace getframejacobian

  // ddq = J.block(0, 0, 3, 5).inverse() * (ddx - (dJ * dq).block(0, 0, 3, 1));
  // std::cout << "dj" << dJ << std::endl;
  // std::cout << "\nresult: " << q.transpose() << dq.transpose() << "ddq"
  //           << ddq.transpose() << std::endl;
  // std::cout << "    i =   " << i << std::endl;
  // std::cout << "\nfinal error: " << err.transpose() << std::endl;
}

Matrix<double, 6, 5> kinematics::get_leg_jacobian(Vector<double, 5> q,
                                                  std::string frame_name) {

  pinocchio::FrameIndex id = model.getFrameId(frame_name);

  pinocchio::Data::Matrix6x J(6, model.nv);
  // pinocchio::Data::Matrix6x dJ(6,model.nv);
  // Matrix<double,6,6> dJ;
  J.setZero();
  pinocchio::forwardKinematics(model, data, q);

  pinocchio::computeJointJacobians(
      model, data, q); // if we use this ,we can get frame or joint jocobians
  pinocchio::updateFramePlacements(model, data);

  pinocchio::getFrameJacobian(model, data, id, pinocchio::LOCAL_WORLD_ALIGNED,
                              J); // cal this, you need forwardKinematics

  return J;
}
