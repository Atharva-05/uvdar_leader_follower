#include <uvdar_leader_follower/velocity_estimator.h>

VelocityEstimator::VelocityEstimator() {
}

/* VelocityEstimator //{ */
VelocityEstimator::VelocityEstimator(kalman3D::Q_t Q, kalman3D::R_t R, kalman3D::x_t x0, double dt) {
A << 
    1, 0, 0, dt, 0, 0,
    0, 1, 0, 0, dt, 0,
    0, 0, 1, 0, 0, dt,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
  // B matrix is zero in case of constant velocity model. Modified in case acceleration inputs become available in the future
  B << 
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    dt, 0, 0,
    0, dt, 0,
    0, 0, dt;

  // Measurements available are [x, y, z]
  H << 
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0;

  this->Q  = Q;
  this->R  = R;
  this->dt = dt;

  kalman3D::P_t P0;
  P0 << 
    10, 0, 0, 0, 0, 0,
    0, 10, 0, 0, 0, 0,
    0, 0, 10, 0, 0, 0,
    0, 0, 0, 1000, 0, 0,
    0, 0, 0, 0, 1000, 0,
    0, 0, 0, 0, 0, 1000;


  kalman3D::statecov_t sc0({x0, P0});

  velocity_estimator_ = std::make_unique<kalman3D>(A, B, H);
  sc                  = sc0;
}
//}

/* predict //{ */
VelocityEstimator::kalman3D::x_t VelocityEstimator::predict(Eigen::Vector3d velocity_estimate, double dt_since_last_prediction) {
  kalman3D::u_t u;
  u << 0.0, 0.0, 0.0;    // How is this valid? | Why is velocity an input to the system? (was feeding velocity as inputs earlier)
  kalman3D::A_t A_new;
  A_new << 
    1, 0, 0, dt, 0, 0,
    0, 1, 0, 0, dt, 0,
    0, 0, 1, 0, 0, dt,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;

  velocity_estimator_->A = A_new;
  dt                     = dt_since_last_prediction;
  sc                     = velocity_estimator_->predict(sc, u, Q, dt);
  return sc.x;
}
//}

/* fuse //{ */
// VelocityEstimator::kalman3D::x_t VelocityEstimator::fuse(Eigen::Vector3d position_measurement, Eigen::Vector3d velocity_measurement) {
VelocityEstimator::kalman3D::x_t VelocityEstimator::fuse(Eigen::Vector3d position_measurement) {
  // Updated to include velocity measurement
  kalman3D::z_t measurement;
  // measurement << position_measurement.x(), position_measurement.y(), position_measurement.z(), velocity_measurement.x(), velocity_measurement.y(), velocity_measurement.z();
  measurement << position_measurement.x(), position_measurement.y(), position_measurement.z();  // Updating latest measured pose
  // this->R = position_covariance;
  // kalman3D::R_t new_covariance;
  // new_covariance = position_covariance;                                                                 // Updating measurement covariance from previous KF
  sc = velocity_estimator_->correct(sc, measurement, this->R);
  return sc.x;
}
//}

/* getStates //{ */
VelocityEstimator::kalman3D::x_t VelocityEstimator::getStates() {
  return sc.x;
}
//}

/* getCovariance //{ */
VelocityEstimator::kalman3D::P_t VelocityEstimator::getCovariance() {
  return sc.P;
}
//}

/* getVelocity //{ */
Eigen::Vector3d VelocityEstimator::getVelocity() {
  return Eigen::Vector3d(sc.x(3), sc.x(4), sc.x(2));
}
//}

/* getPosition //{ */
Eigen::Vector3d VelocityEstimator::getPosition() {
  return Eigen::Vector3d(sc.x(0), sc.x(1), sc.x(2));
}
//}
