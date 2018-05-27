#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void KalmanFilter::UpdateStateTransition(const double& time_shift)
{
  F(0, 2) = time_shift;
  F(1, 3) = time_shift;
}

void KalmanFilter::UpdateProcessNoise(const double& time_shift)
{
	double time_shift_2 = time_shift * time_shift;
	double time_shift_3 = time_shift_2 * time_shift;
	double time_shift_4 = time_shift_3 * time_shift;

	Q << time_shift_4 / 4 * noise_ax, 0, time_shift_3 / 2 * noise_ax, 0,
	  0, time_shift_4 / 4 * noise_ay, 0, time_shift_3 / 2 * noise_ay,
	  time_shift_3 / 2 * noise_ax, 0, time_shift_2 * noise_ax, 0,
	  0, time_shift_3 / 2 * noise_ay, 0, time_shift_2 * noise_ay;

}

KalmanFilter::KalmanFilter()
  :
  noise_ax(9), noise_ay(9),
  x(VectorXd(4)),
  F(MatrixXd(4, 4)), P(MatrixXd(4, 4)), Q(MatrixXd(4, 4))
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
}

void KalmanFilter::Initialize(double x_coordinate, double y_coordinate)
{
	x << x_coordinate, y_coordinate, 0, 0;
}

void KalmanFilter::Predict(const double time_shift)
{
  UpdateStateTransition(time_shift);
  UpdateProcessNoise(time_shift);
  
  x = F * x;
  MatrixXd F_transposed = F.transpose();
  P = F * P * F_transposed + Q;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const MatrixXd &R)
{
  VectorXd z_predicted = H * x;
  VectorXd y = z - z_predicted;
  MatrixXd H_transposed = H.transpose();
  MatrixXd S = H * P * H_transposed + R;
  MatrixXd S_inversed = S.inverse();
  MatrixXd KalmanGain = P * H_transposed * S_inversed;

  x = x + KalmanGain * y;
  long x_size = x.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P = (I - KalmanGain * H) * P;
}
