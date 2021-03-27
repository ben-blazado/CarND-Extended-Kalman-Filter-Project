#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in; 
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  cout << "KalmanFilter::Predict()" << endl;
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  cout << "KalmanFilter::Update()" << endl;
  MatrixXd Ht = H_.transpose();
  
  VectorXd y = z - H_*x_;;
   
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht*S.inverse();
  
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
     cout << "KalmanFilter::UpdateEKF()" << endl;
}
