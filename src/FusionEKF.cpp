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

  MatrixXd R_laser(2, 2);
  R_laser << 0.0225, 0,
              0, 0.0225;
  MatrixXd R_radar(3, 3);
  R_radar << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

    ekf_.Init(x, P, R_laser, R_radar);
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
      ekf_.RadarUpdate(z);
    } else {
      ekf_.LidarUpdate(z);
    }
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}
