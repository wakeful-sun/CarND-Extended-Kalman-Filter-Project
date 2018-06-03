#include "sensor.h"
#include "lidar_sensor.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

LidarSensor::LidarSensor()
	: Sensor(MatrixXd(2, 4), MatrixXd(2, 2))
{
	H << 1, 0, 0, 0,
		0, 1, 0, 0;

	R << 0.0225, 0,
		0, 0.0225;
}

void LidarSensor::Update(const VectorXd& prediction, const VectorXd& measurement)
{
	VectorXd z_predicted = H * prediction;
	prediction_error = measurement - z_predicted;
}
