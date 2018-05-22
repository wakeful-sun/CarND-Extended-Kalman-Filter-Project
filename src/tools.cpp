#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  auto estimations_size = estimations.size();

  if (estimations_size != ground_truth.size() || estimations_size == 0)
  {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  for (unsigned int i = 0; i < estimations_size; ++i)
  {

    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations_size;
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if (px == 0 && py == 0)
  {
    throw std::invalid_argument("received zero state values");
  }

  MatrixXd Hj(3, 4);

  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  float H_00 = px / c2;
  float H_01 = py / c2;

  float H_10 = -py / c1;
  float H_11 = px / c1;

  float H_20 = py * (vx * py - vy * px) / c3;
  float H_21 = px * (vy * px - vx * py) / c3;

  Hj << H_00, H_01, 0, 0,
      H_10, H_11, 0, 0,
      H_20, H_21, H_00, H_01;

  return Hj;
}

VectorXd MapCartesianToPolar(const VectorXd &x_state)
{
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);  
  
  if (px == 0 && py == 0)
  {
    throw std::invalid_argument("received zero state values");
  }

  float range = sqrt(px * px + py * py);
  float bearing_radians = atan(py/px); //TODO: normalize: add or subtract 2π from ϕ until it is between −π and π.
  float range_rate = (px * vx + py * vy) / range;

  VectorXd h;
  h << range, bearing_radians, range_rate;
  return h;
}
