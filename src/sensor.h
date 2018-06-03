#ifndef SENSORS_H_
#define SENSORS_H_
#include "Eigen/Dense"

class Sensor
{
public:
	Sensor(Eigen::MatrixXd H, Eigen::MatrixXd R);
	virtual ~Sensor() = default;

	Eigen::MatrixXd H;
	Eigen::MatrixXd R;
	Eigen::VectorXd prediction_error;
	virtual void Update(const Eigen::VectorXd& prediction, const Eigen::VectorXd& measurement) = 0;
};


class LidarSensor : public Sensor
{
public:
	LidarSensor();

	void Update(const Eigen::VectorXd& prediction, const Eigen::VectorXd& measurement) override;
};

class RadarSensor : public Sensor
{
public:
	RadarSensor();

	void Update(const Eigen::VectorXd& prediction, const Eigen::VectorXd& measurement) override;
};

#endif /* SENSORS_H_ */
