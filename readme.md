# Extended Kalman Filter
This is a Udacity Self-Driving Car NanoDegree project submission that uses a kalman filter to estimate the state of a moving object of interest with noisy simulated lidar and radar measurements. 

![](Screenshot.png)

## Installation
* Clone or fork this repository. 
* Install [Udacity's Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) which allows the user to visualize the moving object, lidar and radar sensor data, and the kalman filter estimated positions.
* Install [uWebSocketIO](https://github.com/uNetworking/uWebSockets) which allows the project to send data to the Term 2 Simulator.
  * Linux: run the script `install-ubuntu.sh`.
  * Mac: run the script `install-mac.sh`.
  * Windows: install Ubuntu Bash enviroment, clone repo in Ubuntu Bash environment, then run the linux install script `install-ubuntu.sh`.
* Troubleshooting tips can be found on this [Udacity knowledge post](https://knowledge.udacity.com/questions/5184).

## Usage
Intended user is the Udacity evaluator for this project. 

1. Run the following from the project directory:
   * `cd build`
   * `./ExtendedKF`
2. Run the Term 2 Simulator.
   * In the main menu screen select Project 1/2 EKF and UKF.
   * Click the START button to observe the object moving and the trailing measurement markers.
     * The green measurement markers indicate the kalman filter's estimated position of the moving object.
     * RMSE values for position and velocity components are also displayed.

## Main Project Files
The C++ code and headers can be found in the `src` folder.
* `FusionEKF.cpp`: manages the kalman filter and associated matrices.
* `kalman_filter.cpp`: performs kalman filter algorithm for lidar and radar measurements.
* `tools.cpp`: calculates RMSE and jacobian for H matrix.

To compile the source code, run the following from the main project directory:
* `cd build`
* `cmake ..`
* `make`