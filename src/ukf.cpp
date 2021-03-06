#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

#define EPS 0.001
#define COUT if (0) cout

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;

  ///* State dimension
  n_x_ = x_.size();

  ///* Augmented state dimension
  n_aug_ = n_x_ + 2;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  //* Number of sigma points
  n_sig_points_ = 2 * n_aug_ + 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2.
  // A bicycle is tracked - so acceleration is much lower than a vehicle.
  // I followed the arguments in this discussion whereas the maximum acceleration is considered 6 m/s*s due
  // to the friction limits by tires (see https://www.bikeradar.com/forums/viewtopic.php?t=12964394).
  // Standard deviation (not squared) should be half of maximum expected acceleration.
  // The actual values must be in 95% of all cases below abs(2*std_a_) for a gaussian distribution
  // which should be fulfilled since 2*std_a_ is the maximum acceleration to be expected for a bicycle.
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2.
  // Imagine the bicycle is traveling in a circle with a constant yaw rate (angular velocity)
  // PI/8 m/s. That means the bicycle would complete a full circle in 16 seconds.
  // That seems reasonable for an average bike rider traveling in a circle with a radius of maybe 16 meters.
  // The bike rider would have also have a tangential velocity of 6.28 meters per second because
  // PI/8*16m = 6.28 m/s.
  // If the angular acceleration were now -PI/8 rad instead of zero
  // then in 2 seconds the biker would go in a circle of the opposite direction.
  // Standard deviation (not squared) should be half of maximum expected acceleration.
  std_yawdd_ = M_PI / 16.;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_points_);

  ///* Weights of sigma points
  weights_ = VectorXd(n_sig_points_);
  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  double weight = 0.5 / (n_aug_ + lambda_);
  for (int i = 1; i < n_sig_points_; i++) {  //2n+1 weights
    weights_(i) = weight;
  }

  ///* Measurement noise covariance matrices
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;
}

UKF::~UKF() {}

void UKF::Init(MeasurementPackage measurementPackage) {
  COUT << "# initializing ukf" << endl;

  // x is [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  if (measurementPackage.sensor_type_ == MeasurementPackage::RADAR) {
    // Initialization with radar can be done for position and velocity.
    // Convert radar from polar to cartesian coordinates and initialize state.
    double range = measurementPackage.raw_measurements_(0);
    double angle = measurementPackage.raw_measurements_(1);
    double rangeRate = measurementPackage.raw_measurements_(2);
    double px = range * cos(angle);
    double py = range * sin(angle);
    double vx = rangeRate * cos(angle);
    double vy = rangeRate * sin(angle);
    double v  = sqrt(vx * vx + vy * vy);
    double yaw_rate = 0; // unknown
    double yaw_rate_acceleration = 0; // unknown
    x_ << px, py, v, yaw_rate, yaw_rate_acceleration;
  } else if (measurementPackage.sensor_type_ == MeasurementPackage::LASER) {
    // Initialization with lidar can only be done for position
    double px = measurementPackage.raw_measurements_(0);
    double py = measurementPackage.raw_measurements_(1);
    double v = 0; // unknown with lidar
    double yaw_rate = 0; // unknown with lidar
    double yaw_rate_acceleration = 0; // unknown with lidar
    x_ << px, py, v, yaw_rate, yaw_rate_acceleration;
  }

  if (fabs(x_(0)) < EPS) {
    x_(0) = EPS;
  }
  if (fabs(x_(1)) < EPS) {
    x_(1) = EPS;
  }

  previous_timestamp_ = measurementPackage.timestamp_;

  is_initialized_ = true;
}

double UKF::NormalizeAngle(double rad) {
  while (rad > M_PI) rad -= 2. * M_PI;
  while (rad < -M_PI) rad += 2. * M_PI;
  return rad;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurementPackage) {
  if (!is_initialized_) {
    Init(measurementPackage);
    is_initialized_ = true;

  } else {

    double dt = (measurementPackage.timestamp_ - previous_timestamp_) / 1000000.;
    previous_timestamp_ = measurementPackage.timestamp_;

    Prediction(dt);

    // Use the sensor type to perform the update step.
    // Update the state and covariance matrices.
    if (measurementPackage.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
      //COUT << "radar measurement" << endl;
      UpdateRadar(measurementPackage);
    } else {
      // Laser updates
      //COUT << "lidar measurement" << endl;
      UpdateLidar(measurementPackage);
    }
  }

  // print the output
  // P should converge to small values (since uncertainty should reduce over time)
  COUT << "x_ = " << x_ << endl;
  COUT << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  COUT << "## predicting state after " << delta_t << "ms" << endl;

  MatrixXd Xsig_points_aug = MatrixXd(n_x_, n_sig_points_);
  AugmentedSigmaPoints(&Xsig_points_aug);
  SigmaPointPrediction(Xsig_points_aug, &Xsig_pred_, delta_t);
  PredictMeanAndCovariance(Xsig_pred_, &x_, &P_);

  // print the output
  // P should converge to small values (since uncertainty should reduce over time)
  COUT << "x_ = " << x_ << endl;
  COUT << "P_ = " << P_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurementPackage) {
  COUT << "## updating from lidar measurement" << endl;

  //set measurement dimension - lidar can measure px, py
  int n_z = 2;

  // Create matrix for sigma points in measurement space.
  // Simply use sigma points of last prediction.
  // Transform sigma points from state space into measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_points_);
  for (int i = 0; i < n_sig_points_; i++) {
    // extract values for better readability
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    Zsig(0, i) = px;
    Zsig(1, i) = py;
  }

  NIS_lidar_ = UpdateUKF(measurementPackage, n_z, Zsig, R_lidar_);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * Modifie the state vector x_ and the covariance matrix P_.
 * @param {MeasurementPackage} measurementPackage
 */
void UKF::UpdateRadar(MeasurementPackage measurementPackage) {
  COUT << "## updating from radar measurement" << endl;

  //set measurement dimension - radar can measure r, phi, and r_dot
  int n_z = 3;

  // Create matrix for sigma points in measurement space.
  // Simply use sigma points of last prediction.
  // Transform sigma points from state space into measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_points_);
  for (int i = 0; i < n_sig_points_; i++) {
    // extract values for better readability
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    double range = sqrt(px * px + py * py); // r
    double angle_rad = atan2(py, px); // phi, angle normalization not needed since this is done for z diff later
    double range_velocity = (px * v1 + py * v2) / sqrt(px * px + py * py); //r_dot

    Zsig(0, i) = range;
    Zsig(1, i) = angle_rad;
    Zsig(2, i) = range_velocity;
  }

  NIS_radar_ = UpdateUKF(measurementPackage, n_z, Zsig, R_radar_);
}

MatrixXd UKF::UpdateUKF(MeasurementPackage measurementPackage, int n_z, MatrixXd Zsig, MatrixXd R) {

  COUT << "# update uk" << endl;

  // mean of measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_points_; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R;

  //print result
  COUT << "z_pred: " << endl << z_pred << endl;
  COUT << "S: " << endl << S << endl;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_points_; i++) {

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = measurementPackage.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  z_diff(1) = NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  //print result
  COUT << "# updated state using ukf:" << endl;
  COUT << "Updated state x: " << endl << x_ << endl;
  COUT << "Updated state covariance P: " << endl << P_ << endl;

  MatrixXd nis = z.transpose() * S.inverse() * z;
  return nis;
}

void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out) {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_points_);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  double d = sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; i++) {
    VectorXd dL = d * L.col(i);
    Xsig_aug.col(i + 1) = x_aug + dL;
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - dL;
  }

  //write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, MatrixXd *Xsig_out, double dt) {
  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, n_sig_points_);

  //predict sigma points
  for (int i = 0; i < n_sig_points_; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > EPS) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
    } else {
      px_p = p_x + v * dt * cos(yaw);
      py_p = p_y + v * dt * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * dt;
    double yawd_p = yawd;

    //add noise
    double nu_a_dt = 0.5 * nu_a * dt * dt;
    px_p = px_p + nu_a_dt * cos(yaw);
    py_p = py_p + nu_a_dt * dt * dt * sin(yaw);
    v_p = v_p + nu_a * dt;

    yaw_p = yaw_p + 0.5 * nu_yawdd * dt * dt;
    yawd_p = yawd_p + nu_yawdd * dt;

    //write predicted sigma point into right column
    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = yaw_p;
    Xsig_pred(4, i) = yawd_p;
  }

  // write result
  *Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(MatrixXd Xsig_pred, VectorXd *x_out, MatrixXd *P_out) {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < n_sig_points_; i++) {  //iterate over sigma points
    x = x + weights_(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < n_sig_points_; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  //write result
  *x_out = x;
  *P_out = P;
}
