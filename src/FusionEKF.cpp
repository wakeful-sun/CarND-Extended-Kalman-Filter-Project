#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector; // used for error calculation

FusionEKF::FusionEKF()
{
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // measuremet noise covariance
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  if (!is_initialized_)
  {    
    std::cout << "EKF: " << std::endl;
	previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
	  double x = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
	  double y = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
	  
      ekf_.Initialize(x, y);
	}
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
	  ekf_.Initialize(measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1]);
    }
    
  	// done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
	  //TODO: calculate H matrices
	ekf_.Update(measurement_pack.raw_measurements_, Hj_, R_radar_);
  }
  else
  {
	ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
  }

  previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
  std::cout << "x = " << ekf_.x << std::endl;
  std::cout << "P = " << ekf_.P << std::endl;
}
