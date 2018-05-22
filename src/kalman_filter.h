#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
private:
  Tools tools;

public:
  // state vector
  Eigen::VectorXd x_;
  // state covariance matrix
  Eigen::MatrixXd P_;
  // state transition matrix
  Eigen::MatrixXd F_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // measurement covariance matrix
  Eigen::MatrixXd R_;

  KalmanFilter();
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  void Predict();
  void Update(const Eigen::VectorXd &z);
  void UpdateEKF(const Eigen::VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
