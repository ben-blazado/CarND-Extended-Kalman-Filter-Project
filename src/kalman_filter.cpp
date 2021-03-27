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
  
  VectorXd y = z - H_*x_;
     
  float rho   = z(0);
  float phi   = - z(1);
  float phi_d = z(2);
  
  cout << "rho " << rho << endl;
  cout << "phi " << phi << endl;
  cout << "phi " << phi_d << endl;

  float px = rho * cos(phi);
  float py = -(rho * sin(phi));
  
  VectorXd p = VectorXd(2);
  p << px, py;
  VectorXd p_inverse = p.array() / pow(rho, 2);
  
  //https://www.euclideanspace.com/maths/algebra/vectors/vecAlgebra/inverse/index.htm
  VectorXd v = phi_d*rho*p_inverse.array();
  
  //VectorXd z = VectorXd(4);
  //z << p(0), p(1), v(0), v(1);
  
  //Update(z);
}


