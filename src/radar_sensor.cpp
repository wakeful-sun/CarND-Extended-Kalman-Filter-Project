#include "sensor.h"
#include "radar_sensor.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;

void RadarSensor::NormalizeBearing(float &bearing_error)
{
  if(bearing_error < -M_PI)
  {
    bearing_error = bearing_error + m_2PI;
    NormalizeBearing(bearing_error);
  }
  if(bearing_error > M_PI)
  {
    bearing_error = bearing_error - m_2PI;
    NormalizeBearing(bearing_error);
  }
}

void RadarSensor::UpdateMeasurementMatrix(const float& range, const float& range_2)
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
  px(0), py(0), vx(0), vy(0), m_2PI(M_PI * 2)
{
  H << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0;
  
  R << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;

  prediction_error = VectorXd(3);
}

void RadarSensor::Update(const VectorXd& prediction, const VectorXd& m)
{
  px = prediction(0);
  py = prediction(1);
  vx = prediction(2);
  vy = prediction(3);

  if (px == 0 && py == 0)
  {
    throw std::invalid_argument("You hit an obstacle!!!");
  }

  float range_2 = px * px + py * py;
  
  float range = sqrt(range_2);
  float bearing = atan2(py, px);
  float radial_velocity = (px * vx + py * vy) / range;
  float bearing_error = m(1) - bearing;
  NormalizeBearing(bearing_error);
  
  prediction_error << m(0) - range, bearing_error, m(2) - radial_velocity;
  UpdateMeasurementMatrix(range, range_2);
}
