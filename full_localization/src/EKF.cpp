#include <full_localization/EKF.hpp>

EKF::EKF()
{
    // Initial variable
    prediction_state_ = Eigen::Vector<double, 5>::Zero();
    prediction_covariance_ = Eigen::Matrix<double, 5, 5>::Identity();

    state_ = Eigen::Vector<double, 5>::Zero();
    covariance_ = Eigen::Matrix<double, 5, 5>::Identity();
    kalman_gain_ = Eigen::Matrix<double, 5, 5>::Identity();

    r_setter(0.01, 0.01, 0.01, 0.001, 0.08);

    q_setter(0.1, 0.1, 0.06, 0.0001, 0.001); 
}

EKF::~EKF() {}

void EKF::Prediction(double dt_)
{
    // Estimate
    double x = state_(0);
    double y = state_(1);
    double v = state_(2);
    double theta = state_(3);
    double omega = state_(4);

    prediction_state_(0) = x + v * dt_ * std::cos(theta);
    prediction_state_(1) = y + v * dt_ * std::sin(theta);
    prediction_state_(2) = v;
    prediction_state_(3) = theta + dt_ * omega;
    prediction_state_(3) = std::atan2(std::sin(prediction_state_(3)), std::cos(prediction_state_(3)));
    prediction_state_(4) = omega;

    // Prediction Covarience Update

    // Linearlize the model
    Eigen::Matrix<double, 5, 5> F = Eigen::Matrix<double, 5, 5>::Identity();
    F(0, 2) = std::cos(theta) * dt_;
    F(0, 3) = -v * std::sin(theta) * dt_;
    F(1, 2) = std::sin(theta) * dt_;
    F(1, 3) = v * std::cos(theta) * dt_;
    F(3, 4) = dt_;

    // Covarience Update
    prediction_covariance_ = (F * covariance_ * F.transpose()) + Q;
}

void EKF::Correction(double v, double theta, double omega)
{
    // 1. Measurement vector (3x1)
    Eigen::Vector3d y_meas;
    y_meas << v, theta, omega;

    // 2. Mapping matrix H (3x5)
    // We only map the 3rd, 4th, and 5th states (indices 2, 3, 4)
    Eigen::Matrix<double, 3, 5> H = Eigen::Matrix<double, 3, 5>::Zero();
    H(0, 2) = 1.0; // velocity
    H(1, 3) = 1.0; // theta
    H(2, 4) = 1.0; // omega

    // 3. Noise sub-matrix (3x3)
    // Pulling the variances for v, theta, and omega from your R matrix
    Eigen::Matrix3d R_sub = Eigen::Matrix3d::Zero();
    R_sub(0, 0) = R(2, 2);
    R_sub(1, 1) = R(3, 3);
    R_sub(2, 2) = R(4, 4);

    // 4. Kalman Gain calculation (5x3)
    // K = P * H' * (H * P * H' + R)^-1
    Eigen::Matrix<double, 5, 3> K;
    K = prediction_covariance_ * H.transpose() * (H * prediction_covariance_ * H.transpose() + R_sub).inverse();

    // 5. Innovation (Measurement Residual)
    Eigen::Vector3d h = H * prediction_state_; // Predicted measurement
    Eigen::Vector3d innovation = y_meas - h;

    // Normalize angle in innovation
    innovation(1) = atan2(std::sin(innovation(1)), std::cos(innovation(1)));

    // 6. Update State
    state_ = prediction_state_ + K * innovation;
    state_(3) = atan2(std::sin(state_(3)), std::cos(state_(3))); // Final normalization

    // 7. Update Covariance
    Eigen::Matrix<double, 5, 5> I = Eigen::Matrix<double, 5, 5>::Identity();
    covariance_ = (I - K * H) * prediction_covariance_;
}

void EKF::Correction(double x_pos, double y_pos,double v, double theta, double omega)
{

    Eigen::Vector<double, 5> y = Eigen::Vector<double, 5>::Zero(5); // measurement
    y(0) = x_pos;
    y(1) = y_pos;
    y(2) = v;
    y(3) = theta;
    y(4) = omega;

    Eigen::Vector<double, 5> h = prediction_state_; // measurement

    Eigen::Matrix<double, 5, 5> H = Eigen::Matrix<double, 5, 5>::Identity();

    // Kalman gain update
    kalman_gain_ = prediction_covariance_ * H.transpose() *
                   (H * prediction_covariance_ * H.transpose() + R).inverse();

    // State update
    Eigen::Vector<double, 5> innovation = y - h;

    innovation(3) = atan2(std::sin(innovation(3)), std::cos(innovation(3)));

    state_ = prediction_state_ + kalman_gain_ * innovation;

    state_(3) = atan2(std::sin(state_(3)), std::cos(state_(3)));

    // Covarence update
    covariance_ =
        (Eigen::Matrix<double, 5, 5>::Identity() - kalman_gain_ * H) * prediction_covariance_;

    // covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

std::vector<double> EKF::state_getter()
{
    std::vector<double> state;
    state.reserve(5);

    for (int i = 0; i < 5; i++)
    {
        state.push_back(state_(i));
    }
    return state;
}


void EKF::q_setter(double x_var, double y_var, double v_var,double theta_var,double omega_var)
{
    Q = Eigen::Matrix<double, 5,5>::Identity(5,5);
    Q(0, 0) = x_var;
    Q(1, 1) = y_var;
    Q(2, 2) = v_var;
    Q(3, 3) = theta_var;
    Q(4, 4) = omega_var;
}
void EKF::r_setter(double x_var, double y_var, double v_var,double theta_var,double omega_var)
{
    R = Eigen::Matrix<double, 5, 5>::Identity(5, 5);
    R(0, 0) = x_var;
    R(1, 1) = y_var;
    R(2, 2) = v_var;
    R(3, 3) = theta_var;
    R(4, 4) = omega_var;
}

// EOF