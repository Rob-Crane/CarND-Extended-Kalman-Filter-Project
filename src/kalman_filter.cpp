#include "kalman_filter.h"

#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &H_in) {
  x_ = x_in;
  P_ = P_in;
  H_ = H_in;
}

void KalmanFilter::Predict(long long delta_T) {

  MatrixXd F(4,4);
  F << 1, 0, delta_T, 0,
       0, 1, 0,       delta_T,
       0, 0, 1,       0,
       0, 0, 0,       1;
  x_ = F * x_;

  MatrixXd Q(4,4);
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float d4_coef = pow(delta_T, 4) / 4.0;
  float d3_coef = pow(delta_T, 3) / 2.0;
  float d2_coef = pow(delta_T, 2);
  Q << d4_coef*noise_ax, 0,                d3_coef*noise_ax, 0,
       0,                d4_coef*noise_ay, 0,                d3_coef*noise_ay,
       d3_coef*noise_ax, 0,                d2_coef*noise_ax, 0,
       0,                d3_coef*noise_ay, 0,                d2_coef*noise_ay;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &R) {
    UpdateEKF(z, R, H_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z,
                             const MatrixXd &R,
                             const MatrixXd &H) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd y = z - H * x_;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(4,4) - K*H)*P_;
}
