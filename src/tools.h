#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Tools {
public:
  Tools();
  virtual ~Tools();

  VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);
  MatrixXd CalculateJacobian(const VectorXd& x_state);
  VectorXd MapCartesianToPolar(const VectorXd& x_state);
};

#endif /* TOOLS_H_ */
