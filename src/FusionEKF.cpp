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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  const Eigen::VectorXd& measurements =
    measurement_pack.raw_measurements_;
  /**
   * Initialization
   */
  if (!is_initialized_) {
    VectorXd x(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float x_comp = std::cos(measurements[1]);
      float y_comp = std::sin(measurements[1]);
      x << measurements[0] * x_comp,
           measurements[0] * y_comp,
           0,
           0;
    
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
    ekf_.Init(x, P);
    is_initialized_ = true;
  } else {
    /**
     * Prediction
     */
    //if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
      //return;
    float delta_T = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0E6;
    ekf_.Predict(delta_T);

    /**
     * Update
     */
    const VectorXd &z = measurements;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.RadarUpdate(z, R_radar_);
    } else {
      ekf_.LidarUpdate(z, R_laser_);
    }
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}
