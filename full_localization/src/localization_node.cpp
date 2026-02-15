#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp" // Added Pose header
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp" // Added Path header
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp" // Helper for PC2
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// TF2 for RViz Visualization
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "full_localization/EKF.hpp"
#include "full_localization/ICP.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

class Localization : public rclcpp::Node
{
  public:
    Localization() : Node("localization_node")
    {
        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        pure_ekf.q_setter(0.01, 0.01, 0.005, 0.001, 0.001);
        pure_ekf.r_setter(0.05, 0.05, 0.02, 0.08, 0.02);

        icp_ekf.q_setter(0.15, 0.15, 0.008, 0.002, 0.002);
        icp_ekf.r_setter(0.03, 0.03, 0.02, 0.08, 0.02);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&Localization::ImuCallback, this, _1));

        wheel_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Localization::WheelCallback, this, _1));

        icp_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "slam_icp_odom", 10, std::bind(&Localization::IcpOdomCallback, this, _1));

        ekf_measurement_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("ekf_measurement", 10);
        pure_ekf_state_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("pure_ekf_state", 10);

        wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
        ekf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 10);
        icp_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 10);

        // Path publishers
        wheel_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("wheel_path", 10);
        ekf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
        icp_ekf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("icp_ekf_path", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&Localization::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {

        rclcpp::Time current_time = this->now();

        // ------ wheel odom calculate ------
        WheelOdomCal(0.05);

        // ------ pure ekf odom calculation ------
        EfkOdomCal(0.05);

        // ------ ICP EKF prediction (runs at 20Hz) ------
        if (!first_icp_) {
            IcpEkfPrediction(0.05);
        }

        // ------ wheel odom publisher ------
        auto wheel_odom_msg = nav_msgs::msg::Odometry();
        wheel_odom_msg.header.stamp = current_time;
        wheel_odom_msg.header.frame_id = "odom";
        wheel_odom_msg.child_frame_id = "base_link";  // Changed to base_link for slam_toolbox
        wheel_odom_msg.pose.pose.position.x = wheel_odom_[0];
        wheel_odom_msg.pose.pose.position.y = wheel_odom_[1];
        wheel_odom_msg.pose.pose.orientation.w = cos(wheel_odom_[2] * 0.5);
        wheel_odom_msg.pose.pose.orientation.z = sin(wheel_odom_[2] * 0.5);
        wheel_odom_pub_->publish(wheel_odom_msg);

        // Publish TF: odom → base_link
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = wheel_odom_[0];
        transform.transform.translation.y = wheel_odom_[1];
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = cos(wheel_odom_[2] * 0.5);
        transform.transform.rotation.z = sin(wheel_odom_[2] * 0.5);
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        tf_broadcaster_->sendTransform(transform);

        // Wheel Path Update
        geometry_msgs::msg::PoseStamped wheel_pose;
        wheel_pose.header = wheel_odom_msg.header;
        wheel_pose.pose = wheel_odom_msg.pose.pose;
        wheel_path_.header.stamp = current_time;
        wheel_path_.header.frame_id = "odom";
        wheel_path_.poses.push_back(wheel_pose);
        wheel_path_pub_->publish(wheel_path_);

        // ------ pure EKF measurement -------
        auto ekf_measurement_msg = std_msgs::msg::Float64MultiArray();
        ekf_measurement_msg.data = ekf_measurement_;
        ekf_measurement_pub_->publish(ekf_measurement_msg);

        // ------ pure EKF state ------
        auto pure_ekf_state_msg = std_msgs::msg::Float64MultiArray();
        pure_ekf_state_msg.data = pure_ekf_state_;
        pure_ekf_state_pub_->publish(pure_ekf_state_msg);

        // ------ pure EKF odom calculation ------
        auto ekf_odom_msg = nav_msgs::msg::Odometry();
        ekf_odom_msg.header.stamp = current_time;
        ekf_odom_msg.header.frame_id = "odom";
        ekf_odom_msg.child_frame_id = "ekf_base_link";
        ekf_odom_msg.pose.pose.position.x = pure_ekf_state_[0];
        ekf_odom_msg.pose.pose.position.y = pure_ekf_state_[1];
        ekf_odom_msg.pose.pose.orientation.w = cos(pure_ekf_state_[3] * 0.5);
        ekf_odom_msg.pose.pose.orientation.z = sin(pure_ekf_state_[3] * 0.5);
        ekf_odom_pub_->publish(ekf_odom_msg);

        // EKF Path Update
        geometry_msgs::msg::PoseStamped ekf_pose;
        ekf_pose.header = ekf_odom_msg.header;
        ekf_pose.pose = ekf_odom_msg.pose.pose;
        ekf_path_.header.stamp = current_time;
        ekf_path_.header.frame_id = "odom";
        ekf_path_.poses.push_back(ekf_pose);
        ekf_path_pub_->publish(ekf_path_);

        // ------ ICP EKF odom publisher (20Hz) ------
        if (!first_icp_) {
            auto icp_ekf_msg = nav_msgs::msg::Odometry();
            icp_ekf_msg.header.stamp = current_time;
            icp_ekf_msg.header.frame_id = "odom";
            icp_ekf_msg.child_frame_id = "icp_ekf_frame";
            icp_ekf_msg.pose.pose.position.x = icp_ekf_state_[0];
            icp_ekf_msg.pose.pose.position.y = icp_ekf_state_[1];
            icp_ekf_msg.pose.pose.orientation.w = cos(icp_ekf_state_[3] * 0.5);
            icp_ekf_msg.pose.pose.orientation.z = sin(icp_ekf_state_[3] * 0.5);
            icp_odom_pub_->publish(icp_ekf_msg);
            
            // ICP EKF Path Update
            geometry_msgs::msg::PoseStamped icp_ekf_pose;
            icp_ekf_pose.header = icp_ekf_msg.header;
            icp_ekf_pose.pose = icp_ekf_msg.pose.pose;
            icp_ekf_path_.header.stamp = current_time;
            icp_ekf_path_.header.frame_id = "odom";
            icp_ekf_path_.poses.push_back(icp_ekf_pose);
            icp_ekf_path_pub_->publish(icp_ekf_path_);
        }
    }

    void ImuCallback(const sensor_msgs::msg::Imu &msg)
    {
        double current_yaw = std::atan2(
            2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
            1.0 - 2.0 * (msg.orientation.y * msg.orientation.y +
                         msg.orientation.z * msg.orientation.z));
        
        if (first_imu_) {
            if (first_imu_reading_count_ == 0) {
                first_imu_yaw_ = current_yaw;
                first_imu_reading_count_++;
            } else {
                double yaw_change = std::abs(current_yaw - first_imu_yaw_);
                
                if (yaw_change > 0.01 || first_imu_reading_count_ > 20) {
                    imu_yaw_offset_ = (std::abs(current_yaw) > 0.01) ? current_yaw : first_imu_yaw_;
                    first_imu_ = false;
                    
                    RCLCPP_INFO(this->get_logger(), 
                               "IMU yaw offset calibrated: %.2f rad (%.1f deg) after %d readings", 
                               imu_yaw_offset_, imu_yaw_offset_ * 180.0 / M_PI, first_imu_reading_count_);
                    RCLCPP_INFO(this->get_logger(), 
                               "Robot will start at yaw=0 in odometry frame");
                } else {
                    first_imu_reading_count_++;
                }
            }
        }
        
        // Apply offset - robot starts at yaw=0, IMU bias removed
        double calibrated_yaw = current_yaw - imu_yaw_offset_;
        
        // Normalize to [-pi, pi]
        while (calibrated_yaw > M_PI) calibrated_yaw -= 2.0 * M_PI;
        while (calibrated_yaw < -M_PI) calibrated_yaw += 2.0 * M_PI;
        
        ekf_measurement_[1] = calibrated_yaw;
        ekf_measurement_[2] = msg.angular_velocity.z;
    }

    void WheelCallback(const sensor_msgs::msg::JointState &msg)
    {

        if (first_wheel_update_) {
            prev_wheel_pos_[0] = msg.position[0];
            prev_wheel_pos_[1] = msg.position[1];
            first_wheel_update_ = false;
            return; 
        }   

        wheel_v_[0] = ((msg.position[0] - prev_wheel_pos_[0]) * 0.033)/0.05;
        wheel_v_[1] = ((msg.position[1] - prev_wheel_pos_[1]) * 0.033)/0.05;

        prev_wheel_pos_[0] = msg.position[0];
        prev_wheel_pos_[1] = msg.position[1];

        ekf_measurement_[0] = ((wheel_v_[0] + wheel_v_[1]) / 2);
    }

    void IcpOdomCallback(const nav_msgs::msg::Odometry &msg)
    {   
        if (first_icp_) {
            first_icp_ = false;
            // Initialize ICP EKF state with first ICP measurement
            icp_ekf_state_[0] = msg.pose.pose.position.x;
            icp_ekf_state_[1] = msg.pose.pose.position.y;
            double qz = msg.pose.pose.orientation.z;
            double qw = msg.pose.pose.orientation.w;
            double qx = msg.pose.pose.orientation.x;
            double qy = msg.pose.pose.orientation.y;
            icp_ekf_state_[3] = std::atan2(2.0 * (qw * qz + qx * qy), 
                                           1.0 - 2.0 * (qy * qy + qz * qz));
            return;
        }
        
        // ICP measurement arrived - do correction step only
        icp_ekf_measurement_[0] = msg.pose.pose.position.x;
        icp_ekf_measurement_[1] = msg.pose.pose.position.y;
        
        // Correction step (measurement update)
        icp_ekf.Correction(icp_ekf_measurement_[0], icp_ekf_measurement_[1], 
                          ekf_measurement_[0], ekf_measurement_[1], ekf_measurement_[2]);
        icp_ekf_state_ = icp_ekf.state_getter();
        
        RCLCPP_DEBUG(this->get_logger(), "ICP correction applied at x=%.2f, y=%.2f", 
                     icp_ekf_measurement_[0], icp_ekf_measurement_[1]);
    }

    void WheelOdomCal(double dt)
    {

        double v_l = wheel_v_[0];
        double v_r = wheel_v_[1];

        double v = (v_r + v_l) / 2.0;
        double omega = (v_r - v_l) / 0.16;

        // Use the average heading during the step for much better accuracy
        double delta_theta = omega * dt;
        double avg_theta = wheel_odom_[2] + (delta_theta / 2.0);

        wheel_odom_[0] += v * std::cos(avg_theta) * dt;
        wheel_odom_[1] += v * std::sin(avg_theta) * dt;
        wheel_odom_[2] += delta_theta;
    
        // Normalize heading
        wheel_odom_[2] = std::atan2(std::sin(wheel_odom_[2]), std::cos(wheel_odom_[2]));
    }

    void EfkOdomCal(double dt)
    {
        pure_ekf.Prediction(dt);
        pure_ekf.Correction(ekf_measurement_[0], ekf_measurement_[1], ekf_measurement_[2]);
        pure_ekf_state_ = pure_ekf.state_getter();
    }

    void IcpEkfPrediction(double dt)
    {
        // Only prediction step - correction happens in IcpOdomCallback
        icp_ekf.Prediction(dt);
        icp_ekf_state_ = icp_ekf.state_getter();
    }

    rclcpp::Time current_time;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<double> wheel_v_ = {0.0, 0.0};
    std::vector<double> prev_wheel_pos_ = {0.0, 0.0};

    std::vector<double> wheel_odom_ = {0.0, 0.0, 0.0};

    EKF pure_ekf;
    std::vector<double> pure_ekf_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    EKF icp_ekf;
    std::vector<double> icp_ekf_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> icp_ekf_measurement_ = {0.0, 0.0};

    std::vector<double> ekf_measurement_ = {0.0, 0.0, 0.0};

    // Path publishers and storage
    nav_msgs::msg::Path wheel_path_;
    nav_msgs::msg::Path ekf_path_;
    nav_msgs::msg::Path icp_ekf_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wheel_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr icp_ekf_path_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ekf_measurement_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pure_ekf_state_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr icp_ekf_measurement_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr icp_ekf_state_pub_;

    bool first_wheel_update_= true;
    bool first_imu_ = true;
    int first_imu_reading_count_ = 0;
    double first_imu_yaw_ = 0.0;
    double imu_yaw_offset_ = 0.0;
    bool first_icp_ = true;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr icp_odom_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_odom_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localization>());
    rclcpp::shutdown();
    return 0;
}