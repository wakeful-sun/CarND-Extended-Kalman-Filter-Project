#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "tools.h"

class KalmanFilter {
private:
  //process noise components
  int noise_ax;
  int noise_ay;
  Eigen::MatrixXd F;
  Eigen::MatrixXd Q;
  void UpdateStateTransition(const double &time_shift);
  void UpdateProcessNoise(const double &time_shift);
  Tools tools;

public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  KalmanFilter();

  /**
   * Initializes Kalman filter with first measurement
   * @param x_coordinate initial x coordinate of tracking target position
   * @param y_coordinate initial x coordinate of tracking target position
   */
  void Initialize(double x_coordinate, double y_coordinate);
  
  void Predict(const double time_shift);
  void Update(const Eigen::VectorXd &z, const MatrixXd &H, const MatrixXd &R);
};

#endif /* KALMAN_FILTER_H_ */
