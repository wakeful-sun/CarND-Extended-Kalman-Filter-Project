#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "kalman_filter.h"
#include "radar_sensor.h"
#include "lidar_sensor.h"

class FusionEKF {
public:
  FusionEKF();
  Eigen::VectorXd ProcessMeasurement(const MeasurementPackage &measurement_pack);

private:
  bool is_initialized_;
  long long previous_timestamp_;

  KalmanFilter ekf_;
  RadarSensor radar_sensor;
  LidarSensor lidar_sensor;
	
  void Initialize(const MeasurementPackage& measurement_pack);
};

#endif /* FusionEKF_H_ */
