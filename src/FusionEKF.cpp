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
     
    VectorXd x_in = VectorXd(4);
    
    // first measurement
    cout << "EKF: " << endl;
    // ekf_.x_ = VectorXd(4);
    // ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      cout << "Radar.rho   " << measurement_pack.raw_measurements_(0) << endl;
      cout << "Radar.phi   " << measurement_pack.raw_measurements_(1) << endl;
      cout << "Radar.phi_d " << measurement_pack.raw_measurements_(2) << endl;
      cout << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      
      x_in << measurement_pack.raw_measurements_(0),
              measurement_pack.raw_measurements_(1),
              0,
              0;
      
    }
   
    MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1,    0,    0,    0,
            0,    1,    0,    0,
            0,    0, 1000,    0,
            0,    0,    0, 1000;
           
    MatrixXd F_in = MatrixXd(4, 4);
    F_in << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
           
    MatrixXd H_in = H_laser_;
    MatrixXd R_in = R_laser_;
    MatrixXd Q_in = MatrixXd::Zero(4, 4);
   
    ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
    
    previous_timestamp_ = measurement_pack.timestamp_;
    
    cout << "Lidar.x " << ekf_.x_(0) << endl;
    cout << "Lidar.y " << ekf_.x_(1) << endl;
    cout << "t_stamp " << previous_timestamp_ << endl;
    
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
  
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
             
  float dt4, dt3, dt2;
            
  dt4 = (dt*dt*dt*dt)/4;
  dt3 = (dt*dt*dt)/2;
  dt2 = dt*dt;
  ekf_.Q_ << dt4*noise_ax,            0, dt3*noise_ax,            0,
                        0, dt4*noise_ay,            0, dt3*noise_ay,
             dt3*noise_ax,            0, dt2*noise_ax,            0, 
                        0, dt3*noise_ay,            0, dt2*noise_ay;

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
    ekf_.Update(measurement_pack.raw_measurements_);
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
