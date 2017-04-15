#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

  if ((estimations.size() == 0) || (estimations.size() != ground_truth.size())) {
	    return rmse;
	}

	for(int i=0; i < estimations.size(); ++i){
		VectorXd err = estimations[i] - ground_truth[i];
    err = err.array() * err.array();
		rmse += err;
	}

	rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float pos_norm = pow(px, 2) + pow(py, 2);

  if (pos_norm <= 0.000001) {
	    return Hj;
	}

  float pxdp = px/pow(pos_norm, 0.5);
	float pydp = py/pow(pos_norm, 0.5);
	Hj << pxdp, pydp, 0, 0,
        -1*py/pos_norm, px/pos_norm, 0, 0,
        py*(vx*py-vy*px)/pow(pos_norm, 1.5), px*(vy*px-vx*py)/pow(pos_norm, 1.5), pxdp, pydp;
	
	return Hj;
}
