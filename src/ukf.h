#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* NIS of radar
  MatrixXd NIS_radar_;

  ///* NIS of radar
  MatrixXd NIS_lidar_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param measurementPackage The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage measurementPackage);


private:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  //* Number of sigma points used for prediction and measurement update
  int n_sig_points_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* radar measurement noise covariance matrix
  MatrixXd R_radar_;

  ///* lidar measurement noise covariance matrix
  MatrixXd R_lidar_;

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param measurementPackage The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage measurementPackage);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param measurementPackage The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage measurementPackage);

  /**
   * Updates the state and covariance matrix from the given measurement.
   *
   * @param measurementPackage measurement
   * @param n_z dimensions in measurement space
   * @param Zsig sigma points from measurement
   * @param R measurement noise covariance matrix
   */
  MatrixXd UpdateUKF(MeasurementPackage measurementPackage, int n_z, MatrixXd Zsig, MatrixXd R);

  /**
   * Initialize the kalman filter with the first measurement.
   * @param measurementPackage first measurement. could be from lidar or radar.
   */
  void Init(MeasurementPackage measurementPackage);

  /**
   * Generate sigma points for the unscented kalman filter into the given matrix
   * including process noise sigma points.
   * @param Xsig_out
   */
  void AugmentedSigmaPoints(MatrixXd* Xsig_out);

  /**
   * Predict sigma points.
   * @param Xsig_aug augmented sigma points as input for prediction
   * @param Xsig_out predicted sigma points output
   * @param dt time delta in seconds.
   */
  void SigmaPointPrediction(MatrixXd Xsig_aug, MatrixXd *Xsig_out, double dt);

  /**
   * Predict the mean state vector and the covariance matrix based on the given predicted sigma points.
   * @param Xsig_pred predicted sigma points
   * @param z_out output of predicted state vector
   * @param S_out output of predicted covariance matrix
   */
  void PredictMeanAndCovariance(MatrixXd Xsig_pred, VectorXd* z_out, MatrixXd* S_out);

  /**
   * time in seconds of the last measurement
   */
  double previous_timestamp_;

  /**
   * Normalizes an angle in radians for range -PI to PI.
   * @param rad angle unnormalized
   * @return angle normalized
   */
  double NormalizeAngle(double rad);

};

#endif /* UKF_H */
