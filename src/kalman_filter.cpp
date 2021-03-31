#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using std::cout;
using std::endl;

#include <math.h>

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
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void dims(const MatrixXd &m) {
  cout << m.rows() << "x" << m.cols() << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd y = z - H_*x_;
  
  MatrixXd Ht = H_.transpose();
   
  MatrixXd S = H_*P_*Ht + R_;
  
  MatrixXd K = P_*Ht*S.inverse();
  
  x_ = x_ + K*y;
  P_ = P_ - K*H_*P_;
}

const float TWO_PI = 2*M_PI;  // 360-degrees; used to normalize phi error

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // unpack predicted pos and vel from x_ 
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // h() function to convert predictions from cartesion to polar
  float rho   = sqrt(px*px + py*py);
  float phi   = atan2(py,px);
  float phi_d = (px*vx + py*vy) / rho;

  // pack predicted polar measurement to pred_z
  VectorXd pred_z = VectorXd(3);
  pred_z << rho, phi, phi_d;
  
  // measure error, y
  VectorXd y = z - pred_z;
  // normalize phi error, y(1), between -PI and PI
  while (y(1) > M_PI)  y(1) -= TWO_PI;
  while (y(1) < -M_PI) y(1) += TWO_PI;
  
  // jacobian matrix, Hj, is assigned to H_ in FusionEKF
  // radar meausrement noise, R_Radar, is assigned to R_ in FusionEKF  
  MatrixXd Ht = H_.transpose();  
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht*S.inverse();
   
  x_ = x_ + K*y;
  P_ = P_ - K*H_*P_;
}


