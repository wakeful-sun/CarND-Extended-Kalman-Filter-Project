#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void KalmanFilter::UpdateStateTransition(const float& time_shift)
{
  F(0, 2) = time_shift;
  F(1, 3) = time_shift;
}

void KalmanFilter::UpdateProcessNoise(const float& time_shift)
{
  float time_shift_2 = time_shift * time_shift;
  float time_shift_3 = time_shift_2 * time_shift;
  float time_shift_4 = time_shift_3 * time_shift;
  
  Q << time_shift_4 / 4 * noise_ax, 0, time_shift_3 / 2 * noise_ax, 0,
    0, time_shift_4 / 4 * noise_ay, 0, time_shift_3 / 2 * noise_ay,
    time_shift_3 / 2 * noise_ax, 0, time_shift_2 * noise_ax, 0,
    0, time_shift_3 / 2 * noise_ay, 0, time_shift_2 * noise_ay;  
}

KalmanFilter::KalmanFilter()
  : noise_ax(27), noise_ay(27),
  F(MatrixXd(4, 4)),
  Q(MatrixXd(4, 4)), x(VectorXd(4)), P(MatrixXd(4, 4))
{
  // initial state transition matrix
  F << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
  
  // initial uncertainty
  P << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;

  long x_size = x.size();
  I = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Initialize(float x_coordinate, float y_coordinate)
{
  x << x_coordinate, y_coordinate, 0, 0;
}

VectorXd KalmanFilter::Predict(const float time_shift)
{
  UpdateStateTransition(time_shift);
  UpdateProcessNoise(time_shift);
  
  x = F * x;
  MatrixXd F_transposed = F.transpose();
  P = F * P * F_transposed + Q;

  VectorXd prediction = VectorXd(4);
  prediction << x(0), x(1), x(2), x(3);
  return prediction;
}

void KalmanFilter::Update(const VectorXd& measurement, Sensor& sensor)
{
  sensor.Update(x, measurement);
  
  MatrixXd H_transposed = sensor.H.transpose();
  MatrixXd S = sensor.H * P * H_transposed + sensor.R;
  MatrixXd S_inversed = S.inverse();
  MatrixXd K = P * H_transposed * S_inversed;
  
  x = x + K * sensor.prediction_error;
  P = (I - K * sensor.H) * P;
}
