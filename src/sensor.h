#ifndef SENSOR_H_
#define SENSOR_H_
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

#endif /* SENSOR_H_ */
