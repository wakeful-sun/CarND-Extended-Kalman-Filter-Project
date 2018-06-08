#ifndef RADAR_SENSOR_H_
#define RADAR_SENSOR_H_
#include "sensor.h"

class RadarSensor : public Sensor
{
private:
  float px;
  float py;
  float vx;
  float vy;
  
  float m_2PI;

  void NormalizeBearing(float& bearing_error);
  void UpdateMeasurementMatrix(const float& range, const float& range_2);
public:
  RadarSensor();  
  void Update(const Eigen::VectorXd& prediction, const Eigen::VectorXd& measurement) override;
};

#endif /* RADAR_SENSOR_H_ */
