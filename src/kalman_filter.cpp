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
   * TODO: predict the state
   */
  x_ =  F_ * x_;
  P_ = F_*P_*F_.transpose()+ Q_ ;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  MatrixXd y = z - H_ * x_ ; 
  MatrixXd s = H_ * P_ * H_.transpose() + R_;
  MatrixXd sInver = s.inverse();
  MatrixXd K = P_ * H_.transpose() * sInver ;
  x_ = x_ + K*y;
  long long xSize = x_.size();
  MatrixXd I = MatrixXd::Identity(xSize, xSize);
  P_ = (I-K*H_) * P_ ; 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  VectorXd h = VectorXd(3);
  h(0) = sqrt( x_(0) *  x_(0) + x_(1) * x_(1) );
  h(1) = atan2( x_(1) , x_(0) );
  if (h(0)==0)
    h(2)=0;
  else
    h(2) = (x_(0) * x_(2) + x_(1) * x_(3)) / h(0);
  
  VectorXd y =  z -  h;
  
  y(1) = atan2(sin(y(1)),cos(y(1)));
  
  MatrixXd s = H_ * P_ * H_.transpose() + R_;
  MatrixXd sInver = s.inverse();
  MatrixXd K = P_ * H_.transpose() * sInver ;
  x_ = x_ + K*y;
  long long xSize = x_.size();
  MatrixXd I = MatrixXd::Identity(xSize, xSize);
  P_ = (I-K*H_) * P_ ; 
}
