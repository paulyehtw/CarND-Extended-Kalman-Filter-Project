#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
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

  // Initialize state transition matrix
  ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  // initial state vector
  ekf_.x_ = VectorXd(4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::InitializeStates(const MeasurementPackage &measurement_pack)
{
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // TODO: Convert radar from polar to cartesian coordinates
    // and initialize state.
    double rho = measurement_pack.raw_measurements_(0);
    double phi = measurement_pack.raw_measurements_(1);
    double rho_d = measurement_pack.raw_measurements_(2);

    double pos_x = rho * std::sin(phi);
    double pos_y = rho * std::cos(phi);
    double vel_x = rho_d * std::sin(phi);
    double vel_y = rho_d * std::cos(phi);

    ekf_.x_ << pos_x, pos_y, vel_x, vel_y;

    ekf_.P_ << 10, 0, 0, 0,
        0, 10, 0, 0,
        0, 0, 10, 0,
        0, 0, 0, 10;
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  {
    // TODO: Initialize state.
    double pos_x = measurement_pack.raw_measurements_(0);
    double pos_y = measurement_pack.raw_measurements_(1);

    ekf_.x_ << pos_x, pos_y, 0.0, 0.0;

    // Initialize state covariance
    ekf_.P_ << 10, 0, 0, 0,
        0, 10, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  }
  is_initialized_ = true;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    InitializeStates(measurement_pack);
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

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // TODO: Radar updates
  }
  else
  {
    // TODO: Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
