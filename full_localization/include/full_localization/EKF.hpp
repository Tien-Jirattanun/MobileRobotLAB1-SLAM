#ifndef EKF_HPP
#define EKF_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

class EKF {

private:

  Eigen::Vector<double, 5> prediction_state_;
  Eigen::Matrix<double, 5, 5> prediction_covariance_;

  Eigen::Vector<double, 5> state_;
  Eigen::Matrix<double, 5, 5> covariance_;
  Eigen::Matrix<double, 5, 5> kalman_gain_;

public:
  EKF();
  ~EKF();

  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 5, 5> R;

  void Prediction(double dt_);
  void Correction(double v, double theta, double omega);
  void Correction(double x_pos, double y_pos, double v, double theta, double omega);

  void q_setter(double x_var, double y_var, double v_var,double theta_var,double omega_var);
  void r_setter(double x_var, double y_var, double v_var,double theta_var,double omega_var);
  std::vector<double> state_getter();
};

#endif

//EOF