#include "kalman_filter.h"
#include "tools.h"
#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in,
                        MatrixXd &P_in,
                        MatrixXd &R_laser_in,
                        MatrixXd &R_radar_in) {
  x_ = x_in;
  P_ = P_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  H_laser_ = MatrixXd(2,4);
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
}

void KalmanFilter::Predict(float delta_T) {

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

void KalmanFilter::LidarUpdate(const VectorXd &z) {
  VectorXd y = z - H_laser_ * x_;
  MatrixXd S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
  MatrixXd K = P_ * H_laser_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(4,4) - K*H_laser_)*P_;
}

void KalmanFilter::RadarUpdate(const VectorXd &z) {

  float phi = atan2(x_[1], x_[0]);
  float range = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
  float rangerate = (x_[0]*x_[2] + x_[1]*x_[3]) / range;
  VectorXd Hx(3);
  Hx << range,
        phi,
        rangerate;
  VectorXd y = z - Hx;
  if (y[1] > M_PI || y[1] < -M_PI) {
      y[1] = std::fmod(y[1], 2*M_PI);
      if (y[1] > M_PI)
          y[1] = y[1] - 2*M_PI;
  }
  Tools tools;
  const MatrixXd& H = tools.CalculateJacobian(x_);
  MatrixXd S = H * P_ * H.transpose() + R_radar_;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(4,4) - K*H)*P_;
}
