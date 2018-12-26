#include "FusionEKF.h"
#include <cmath>
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  Hj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // Lidar position component mapped directly, ignore velocity.
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /**
   * Initialization
   */
  if (!is_initialized_) {
    VectorXd x(4);
    const Eigen::VectorXd& measurements =
        measurement_pack.raw_measurements_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float x_comp = std::cos(measurements[1]);
      float y_comp = std::sin(measurements[1]);
      x << measurements[0] * x_comp,
           measurements[0] * y_comp,
           measurements[2] * x_comp,
           measurements[2] * y_comp;
    
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x << measurements[0],
           measurements[1],
           0,
           0;
    }

    MatrixXd P(4,4);
    P << 1000, 0,    0,    0,
         0, 1000,    0,    0,
         0,    0, 1000,    0,
         0,    0,    0, 1000;
    ekf_.Init(x, P, H_laser_);
    is_initialized_ = true;
  } else {
    /**
     * Prediction
     */
    ekf_.Predict(measurement_pack.timestamp_ - previous_timestamp_);
    previous_timestamp_ = measurement_pack.timestamp_;

    /**
     * Update
     */
    const VectorXd &z = measurement_pack.raw_measurements_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      MatrixXd Hj = tools.CalculateJacobian(z);
      ekf_.UpdateEKF(z, R_radar_, Hj);
    } else {
      ekf_.Update(z, R_laser_);
    }
  }
}
