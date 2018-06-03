#ifndef LIDAR_SENSOR_H_
#define LIDAR_SENSOR_H_
#include "sensor.h"

class LidarSensor : public Sensor
{
public:
  LidarSensor();
  void Update(const Eigen::VectorXd& prediction, const Eigen::VectorXd& measurement) override;
};

#endif /* LIDAR_SENSOR_H_ */
