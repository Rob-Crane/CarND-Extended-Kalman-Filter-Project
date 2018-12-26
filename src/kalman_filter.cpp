#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, // MatrixXd &F_in,
                        MatrixXd &H_in/*, MatrixXd &R_in, MatrixXd &Q_in*/) {
  x_ = x_in;
  P_ = P_in;
  //F_ = F_in;
  H_ = H_in;
  //R_ = R_in;
  //Q_ = Q_in;
}

void KalmanFilter::Predict(long long delta_T) {
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &R) {
  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
