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
  cout << "KalmanFilter::Predict()" << endl;
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
  cout << "KalmanFilter::Update()" << endl;
  MatrixXd Ht = H_.transpose();

  cout << "dims z :"; dims(z);
  cout << "dims H_:"; dims(H_);
  cout << "dims x_:"; dims(x_);  
  VectorXd y = z - H_*x_;
  
  if (y.size() == 3) {
      cout << "z_pred " << endl;
    cout << y << endl;
  }
  
  cout << "dims y:"; dims(y);

  cout << "dims H_:"; dims(H_);
  cout << "dims P_:"; dims(P_);
  cout << "dims Ht:"; dims(Ht);  
  cout << "dims R_:"; dims(R_);
   
  MatrixXd S = H_*P_*Ht + R_;
  cout << "dims S:";
  dims(S);
  
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
  MatrixXd Ht = H_.transpose();  
  
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho   = sqrt(px*px + py*py);
  cout << "px " << px << endl;
  cout << "py " << py << endl;
  float phi   = atan2(py,px);
  float phi_d = (px*vx + py*vy) / rho;
  
  cout << "rho " << rho << endl;
  cout << "phi_deg " << phi * (180/M_PI) << endl;
  cout << "phi_d " << phi_d << endl;
  //assert (py >= 0);

  VectorXd pred_z = VectorXd(3);
  pred_z << rho, phi, phi_d;
  
  VectorXd y = z - pred_z;
  while (y(1) > M_PI)  y(1) -= 2*M_PI;
  while (y(1) < -M_PI) y(1) += 2*M_PI;
  
  /*
  cout << "dims y:"; dims(y);

  cout << "dims H_:"; dims(H_);
  cout << "dims P_:"; dims(P_);
  cout << "dims Ht:"; dims(Ht);  
  cout << "dims R_:"; dims(R_);
  */
   
  MatrixXd S = H_*P_*Ht + R_;
  
  // cout << "dims S:"; dims(S);
  
  MatrixXd K = P_*Ht*S.inverse();
  
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
}


