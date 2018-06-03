#include "sensor.h"
#include "radar_sensor.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

void RadarSensor::UpdateMeasurementMatrix()
{
  float range_3 = range_2 * range;
  
  float H_00 = px / range;
  float H_01 = py / range;
  
  float H_10 = -py / range_2;
  float H_11 = px / range_2;
  
  float H_20 = py * (vx * py - vy * px) / range_3;
  float H_21 = px * (vy * px - vx * py) / range_3;
  
  H << H_00, H_01, 0, 0,
    H_10, H_11, 0, 0,
    H_20, H_21, H_00, H_01;
}

RadarSensor::RadarSensor()
  : Sensor(MatrixXd(3, 4), MatrixXd(3, 3)),
  px(0), py(0), vx(0), vy(0), range(0), range_2(0), radar_prediction(VectorXd(3))
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
  px = prediction(0);
  py = prediction(1);
  vx = prediction(2);
  vy = prediction(3);
  
  if (px == 0 && py == 0)
  {
    throw std::invalid_argument("received zero state values");
  }
  
  range_2 = px * px + py * py;
  range = sqrt(range_2);
  float bearing = atan(py / px);
  float radial_velocity = (px * vx + py * vy) / range;
  
  radar_prediction << range, bearing, radial_velocity;
  prediction_error = measurement - radar_prediction;
  UpdateMeasurementMatrix();
}
