#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "tools.h"
#include "sensor.h"

class KalmanFilter {
private:
  //process noise components
  int noise_ax;
  int noise_ay;
  Eigen::MatrixXd F;
  Eigen::MatrixXd Q;
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
	Eigen::MatrixXd I;

  void UpdateStateTransition(const float &time_shift);
  void UpdateProcessNoise(const float &time_shift);

public:
  KalmanFilter();
  /**
   * Initializes Kalman filter with first measurement
   * @param x_coordinate initial x coordinate of tracking target position
   * @param y_coordinate initial x coordinate of tracking target position
   */
  void Initialize(float x_coordinate, float y_coordinate);
  
  /**
  * Predicts the state and the state covariance
  * using the process model
  * @param time_shift Time between k and k+1 in s
  */
  Eigen::VectorXd Predict(const float time_shift);

  /**
  * Updates the state by using Extended Kalman Filter equations
  * @param measurement The measurement at k+1
  * @param sensor
  */
  void Update(const Eigen::VectorXd& measurement, Sensor& sensor);
  
};

#endif /* KALMAN_FILTER_H_ */
