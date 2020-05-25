#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

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
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
