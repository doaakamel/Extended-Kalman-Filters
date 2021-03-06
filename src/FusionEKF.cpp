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
  
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 10,  0,   0,   0,
             0,   10,  0,   0,
             0,   0,   100, 0,
             0,   0,   0,   100;
  
  
  

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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
     double Rho = measurement_pack.raw_measurements_[0];
     double Phi = measurement_pack.raw_measurements_[1];
     double Rho_dot= measurement_pack.raw_measurements_[2];
     
    double car_1= Rho*cos(Phi);
    double car_2= Rho*sin(Phi);
//     double dot_1= Rho_dot*cos(Phi);
//     double dot_2= Rho_dot*sin(Phi);
      ekf_.x_<< car_1, car_2, 0.f, 0.f;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
       ekf_.x_<< measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.f, 0.f;

    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_ ;
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
  double  dt = (measurement_pack.timestamp_ - previous_timestamp_);
  dt /= 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  
  
  double dt2 = dt*dt;
  double dt3 = dt2*dt;
  double dt4 = dt3*dt;
  double dt4_divided_by_4 =dt4/4;
  double dt3_divided_by_2 =dt3/2;
    
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_<< dt4_divided_by_4*noise_ax,0,dt3_divided_by_2*noise_ax,0,
        0,dt4_divided_by_4*noise_ay,0,dt3_divided_by_2*noise_ay,
        dt3_divided_by_2*noise_ax,0,dt2*noise_ax,0,
        0,dt3_divided_by_2*noise_ay,0,dt2*noise_ay;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;


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
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
