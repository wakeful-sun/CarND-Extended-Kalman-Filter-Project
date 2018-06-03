#include "sensor.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

Sensor::Sensor(MatrixXd H_in, MatrixXd R_in)
  : H(H_in), R(R_in)
{
}
