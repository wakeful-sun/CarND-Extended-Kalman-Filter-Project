#include "sensor.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

RadarSensor::RadarSensor()
	: Sensor(MatrixXd(3, 4), MatrixXd(3, 3))
{
	H << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0;

	R << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;
}

void RadarSensor::Update(const VectorXd& prediction, const VectorXd& measurement)
{
	float px = prediction(0);
	float py = prediction(1);
	float vx = prediction(2);
	float vy = prediction(3);

	float range = sqrt(px * px + py * py);
	float bearing = atan(py / px);
	float radial_velocity = (px * vx + py * vy) / range;

	VectorXd radarPredicted = VectorXd(3);
	radarPredicted << range, bearing, radial_velocity;

	prediction_error = measurement - radarPredicted;

	//TODO: set H
}

//double x = range * cos(bearing);
//double y = range * sin(bearing);
