#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
              
  VectorXd x_in = VectorXd(4);
  x_in << 1, 1, 1, 1;
              
  MatrixXd P_in = MatrixXd(4, 4);
  P_in << 1,    0,    0,    0,
          0,    1,    0,    0,
          0,    0, 1000,    0,    // initial high uncertaintany values
          0,    0,    0, 1000;    // for velocity components
         
  MatrixXd F_in = MatrixXd(4, 4);
  F_in << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
         
  MatrixXd H_in = H_laser_;
  MatrixXd R_in = R_laser_;
  MatrixXd Q_in = MatrixXd::Zero(4, 4);
 
  ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    
    // first measurement
    
    /*
     * covariance matrix P is initialized in FusionEKF constructor
     * this only initializes the state, ekf_.x_, with raw_measurements_.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      
      ekf_.x_ << rho * cos(phi),
                 rho * sin(phi),
                 0,
                 0;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      
      ekf_.x_ << measurement_pack.raw_measurements_(0),
                 measurement_pack.raw_measurements_(1),
                 0,
                 0;
    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  const float noise_ax = 9;
  const float noise_ay = 9;
    
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
             
  float dt2 = dt*dt;
  float dt3 = (dt2*dt)/2;
  float dt4 = (dt2*dt2)/4;
  
  float dt3_xnoise = dt3*noise_ax;
  float dt3_ynoise = dt3*noise_ay;
  
  ekf_.Q_ << dt4*noise_ax,            0,   dt3_xnoise,            0,
                        0, dt4*noise_ay,            0,   dt3_ynoise,
               dt3_xnoise,            0, dt2*noise_ax,            0, 
                        0,   dt3_ynoise,            0, dt2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
