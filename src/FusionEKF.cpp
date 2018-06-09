#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include "radar_sensor.h"
#include "lidar_sensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEKF::FusionEKF()
  : is_initialized_(false),
  previous_timestamp_(0),
  radar_sensor(RadarSensor()),
  lidar_sensor(LidarSensor())
{
}

void FusionEKF::Initialize(const MeasurementPackage& measurement_pack)
{
  previous_timestamp_ = measurement_pack.timestamp_;
  
  switch (measurement_pack.sensor_type_)
  {
    case MeasurementPackage::RADAR:
    {
      float x = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float y = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      ekf_.Initialize(x, y);
    }
    break;
    case MeasurementPackage::LASER:
      ekf_.Initialize(measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1]);
      break;
    default:
      throw "Given sensor type in not supported.";
  }
}

VectorXd FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  if (!is_initialized_)
  {
    Initialize(measurement_pack);
    is_initialized_ = true;
    VectorXd v = VectorXd(4);
    v << 0, 0, 0, 0;
    return v;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  float time_shift = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  VectorXd prediction = ekf_.Predict(time_shift);
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  
  switch (measurement_pack.sensor_type_)
  {
    case MeasurementPackage::RADAR:
      ekf_.Update(measurement_pack.raw_measurements_, radar_sensor);
      break;
    case MeasurementPackage::LASER:
      ekf_.Update(measurement_pack.raw_measurements_, lidar_sensor);
      break;
    default:
      throw "Given sensor type in not supported.";
  }

  previous_timestamp_ = measurement_pack.timestamp_;

  return prediction;
}
