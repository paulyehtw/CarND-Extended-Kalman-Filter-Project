#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * UpdateProcessCovarianMatrix updates process covariance matrix Q_
   * @param dt Time between k and k+1 in s
   */
  void UpdateProcessCovarianMatrix(const double &dt);

  /**
   * UpdateTransitionMatrix updates transition matrix F_
   * @param dt Time between k and k+1 in s
   */
  void UpdateTransitionMatrix(const double &dt);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(const double &dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // jacobian matrix
  Eigen::MatrixXd Hj_;

  // Lidar measurement covariance matrix
  Eigen::MatrixXd R_lidar_;

  // Radar measurement covariance matrix
  Eigen::MatrixXd R_radar_;

  // proecess noise on x direction
  double noise_ax_;

  // proecess noise on y direction
  double noise_ay_;
};

#endif // KALMAN_FILTER_H_
