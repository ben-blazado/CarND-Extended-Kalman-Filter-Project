#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation of ground_truth data" << endl;
    return rmse;
  }  

  // TODO: accumulate squared residuals
  VectorXd sum(4);
  VectorXd error(4), sqerror(4);
  VectorXd mean(4);
  
  sum << 0, 0, 0, 0;
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    error = estimations[i] - ground_truth[i];
    sqerror = error.array().square();
    sum += sqerror;
  }

  // TODO: calculate the mean
  mean = sum / estimations.size();

  // TODO: calculate the squared root
  rmse = mean.array().sqrt();

  // return the result
  return rmse;
}

const float ALMOST_ZERO = 0.000001;

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   
  MatrixXd Hj(3,4);
  // unpack position and velocities
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if ((abs(px) < ALMOST_ZERO) && (abs(py) < ALMOST_ZERO)) {
    // set px and py to almost zero
    // output error notification but continue calculations
    cout << "CalculateJacobian () - Error - Division by Zero";
    px = ALMOST_ZERO;
    py = ALMOST_ZERO;
  }
  
  // compute the Jacobian matrix
  float g2 = px*px + py*py;
  float g  = sqrt(g2);
  float g232 = sqrt (g2*g2*g2);
  
  Hj <<                    px/g,                      py/g,    0,    0,
                       -(py/g2),                     px/g2,    0,    0,
        py*(vx*py - vy*px)/g232, px * (vy*px - vx*py)/g232, px/g, py/g;

  return Hj;   
}
