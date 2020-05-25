#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::UpdateProcessCovarianMatrix(const double &dt)
{
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  Q_ << dt4 / 4 * noise_ax_, 0, dt3 / 2 * noise_ax_, 0,
      0, dt4 / 4 * noise_ay_, 0, dt3 / 2 * noise_ay_,
      dt3 / 2 * noise_ax_, 0, dt2 * noise_ax_, 0,
      0, dt3 / 2 * noise_ay_, 0, dt2 * noise_ay_;
}

void KalmanFilter::UpdateTransitionMatrix(const double &dt)
{
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void KalmanFilter::Predict(const double &dt)
{
  /**
   * TODO: predict the state
   */
  UpdateTransitionMatrix(dt);
  UpdateProcessCovarianMatrix(dt);

  x_ = F_ * x_;                       // Predict state
  P_ = F_ * P_ * F_.transpose() + Q_; // Predict covariance
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_ * H_.transpose() * S_inv; // Kalman gain

  x_ = x_ + (K * y); // Update state
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; // Update covariance matrix
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
