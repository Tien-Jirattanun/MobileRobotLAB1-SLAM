#include <chrono>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <memory>
#include <string>

#include "full_localization/ICP.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" // For the map
#include "sensor_msgs/point_cloud2_iterator.hpp" // Crucial for easy packing

using namespace std::chrono_literals;
using std::placeholders::_1;

class Slam : public rclcpp::Node
{
  public:
    Slam() : Node("slam_icp_node")
    {

        auto lidar_qos = rclcpp::SensorDataQoS();

        icp_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_icp_odom", 10);

        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ekf_odom", 10, std::bind(&Slam::OdomCallback, this, _1));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", lidar_qos, std::bind(&Slam::ScanCallback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("icp_path", 10);
        

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 10);
    }

  private:
    void OdomCallback(const nav_msgs::msg::Odometry &msg)
    {
        wheel_odom_[0] = msg.pose.pose.position.x;
        wheel_odom_[1] = msg.pose.pose.position.y;

        double qz = msg.pose.pose.orientation.z;
        double qw = msg.pose.pose.orientation.w;
        double qx = msg.pose.pose.orientation.x;
        double qy = msg.pose.pose.orientation.y;

        // Proper quaternion to yaw conversion
        double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 
                                1.0 - 2.0 * (qy * qy + qz * qz));

        wheel_odom_[2] = yaw;
    }

    void ScanCallback(const sensor_msgs::msg::LaserScan &msg)
    {
        std::vector<Eigen::Vector2d> raw_local_pt = lib_icp.range_to_pos(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max);

        lib_icp.iterative_closet_point(raw_local_pt, wheel_odom_);

        map_ = lib_icp.debug_map_;

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom"; // ICP usually provides map-level correction
        odom_msg.child_frame_id = "icp_frame";

        odom_msg.pose.pose.position.x = lib_icp.icp_odom[0];
        odom_msg.pose.pose.position.y = lib_icp.icp_odom[1];

        double half_yaw = lib_icp.icp_odom[2] * 0.5;
        odom_msg.pose.pose.orientation.z = std::sin(half_yaw);
        odom_msg.pose.pose.orientation.w = std::cos(half_yaw);

        icp_odom_pub_->publish(odom_msg);

        geometry_msgs::msg::PoseStamped curr_pose;
        curr_pose.header = odom_msg.header;
        curr_pose.pose = odom_msg.pose.pose;
        path_msg_.header.frame_id = "odom";

        path_msg_.header.stamp = odom_msg.header.stamp;
        path_msg_.poses.push_back(curr_pose);

        path_pub_->publish(path_msg_);
        publish_map(odom_msg.header);
    }

    void publish_map(const std_msgs::msg::Header &header)
    {
        if (map_.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = header;
        cloud_msg.height = 1;
        cloud_msg.width = map_.size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(map_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto &pt : map_)
        {
            *iter_x = static_cast<float>(pt.x());
            *iter_y = static_cast<float>(pt.y());
            *iter_z = 0.0f;
            ++iter_x; ++iter_y; ++iter_z;
        }

        map_pub_->publish(cloud_msg);
    } 

    ICP lib_icp;

    std::vector<double> wheel_odom_ = {0.0, 0.0, 0.0};
    nav_msgs::msg::Path path_msg_;

    std::vector<Eigen::Vector2d> map_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr icp_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Slam>());
    rclcpp::shutdown();
    return 0;
}