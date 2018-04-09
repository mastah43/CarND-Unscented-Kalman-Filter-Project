#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.empty()) {
    cerr << "Estimations are empty" << endl;
    return rmse;
  }
  if(estimations.size() != ground_truth.size()){
    cerr << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  double error_px = rmse(0);
  double error_py = rmse(1);
  double error_vx = rmse(2);
  double error_vy = rmse(3);

  if ((error_px > 0.09) || (error_py > 0.10) || (error_vx > 0.40) || (error_vy > 0.30)) {
      cerr << "rmse too high: x=" << error_px << "; y=" << error_py
           << "; vx=" << error_vx << "; vy=" << error_vy << endl;
  }

  return rmse;
}