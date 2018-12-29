#include "tools.h"
#include <iostream>
#include <cassert>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  assert (estimations.size() == ground_truth.size());
  using v_sz = vector<VectorXd>::size_type;
  v_sz n = estimations.size();
  
  VectorXd ret(4);
  ret << 0, 0, 0, 0;
  for (v_sz i = 0; i < n; ++i) {
    VectorXd residual = ground_truth[i] - estimations[i];
    ret += VectorXd(residual.array() * residual.array());
  }

  ret /= n;
  ret = ret.array().sqrt();
  return ret;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  float px = x_state[0];
  float py = x_state[1];
              
  MatrixXd Hj(3,4);
  if (px == 0 && py == 0) {
      Hj<< 0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 0;
  } else {
    float px2 = px*px;
    float py2 = py*py;

    float vx = x_state[2];
    float vy = x_state[3];
    float term00 = px/sqrt(px2+py2);
    float term01 = py/sqrt(px2+py2);
    float term10 = -py/(px2+py2);
    float term11 = px/(px2+py2);
    float term20 = py*(vx*py-vy*px)/pow(px2+py2,1.5);
    float term21 = px*(vy*px-vx*py)/pow(px2+py2,1.5);
    float term22 = px/sqrt(px2+py2);
    float term23 = py/sqrt(px2+py2);

    Hj << term00, term01, 0, 0,
          term10, term11, 0, 0,
          term20, term21, term22, term23;
  }
  return Hj;
}
