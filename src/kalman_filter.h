#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param H_in Measurement matrix
   */
  void Init(Eigen::VectorXd &x_in,
            Eigen::MatrixXd &P_in,
            Eigen::MatrixXd &R_laser_in,
            Eigen::MatrixXd &R_radar_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float delta_T);

  //void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   * @param R The measurement covariance matrix
   * @param H The Jacobian matrix to use for EKF.
   */
  //void UpdateEKF(const Eigen::VectorXd &z,
                 //const Eigen::MatrixXd &R,
                 //const Eigen::MatrixXd &H);

  /**
   * Updates the state by using standard Kalman Filter equations
   * with fixed state-to-sensor space mapping matrix H.
   * @param z The measurement at k+1
   */
  void LidarUpdate(const Eigen::VectorXd &z);
  void RadarUpdate(const Eigen::VectorXd &z);
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // Measurement covariance matrix for laser
  Eigen::MatrixXd R_laser_;
  // Laser measurement function
  Eigen::MatrixXd H_laser_;
  // Measurement covariance matrix for radar
  Eigen::MatrixXd R_radar_;
};

#endif // KALMAN_FILTER_H_
