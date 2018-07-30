#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  cout << "Predict: " << endl;
  x_ = F_ * x_ ;
  cout << "Predict: x_: " << x_ << endl;
  MatrixXd Ft = F_.transpose();
  cout << "Predict: Ft: " << Ft << endl;
  cout << "Predict: P_: " << P_ << endl;
  cout << "Predict: Q_: " << Q_ << endl;
  P_ = F_ * P_ * Ft + Q_;
  cout << "Predict: P_: " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  cout << "Update: " << endl;
  cout << "Update: z: " << z << endl;
  cout << "Update: H_: " << H_ << endl;
  cout << "Update: x_: " << x_ << endl;
  VectorXd y = z - H_ * x_;
  cout << "Update: y: " << y << endl;

  UpdateCommons(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout << "UpdateEKF: " << endl;
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  cout << "UpdateEKF: x_: " << x_ << endl;
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;
  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;
  cout << "UpdateEKF: h: " << h << endl;
  VectorXd y = z - h;
  cout << "UpdateEKF: y: " << y << endl;
  // Remove laps
  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
  UpdateCommons(y);
}
void KalmanFilter::UpdateCommons(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  cout << "UpdateCommons: Ht: " << Ht << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << "UpdateCommons: S: " << S << endl;
  cout << "UpdateCommons: P_: " << P_ << endl;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  cout << "UpdateCommons: K: " << K << endl;
  // New state
  x_ = x_ + (K * y);
  cout << "UpdateCommons: x_: " << x_ << endl;
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout << "UpdateCommons: P_: " << P_ << endl;
}
