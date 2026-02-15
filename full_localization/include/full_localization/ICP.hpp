#ifndef ICP_HPP
#define ICP_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

class ICP {

private:

  Eigen::Isometry2d curr_tf;
  Eigen::Isometry2d odomToTF(const std::vector<double> &input_odom);
  void updateMap(const std::vector<Eigen::Vector2d> &icp_pt);

  std::vector<double> prev_odom_ = {0.0, 0.0, 0.0};
  bool first_run_ = true;
  Eigen::Isometry2d last_map_update_pose_ = Eigen::Isometry2d::Identity();
  // Add this to your class
  std::vector<Eigen::Vector2d> target_map;

public:
  ICP();
  ~ICP();

  int recovery_cooldown_;  // Add this member variable
  std::vector<double> icp_odom;
  std::vector<Eigen::Vector2d> global_map_;
  std::vector<Eigen::Vector2d> debug_map_;

  std::vector<Eigen::Vector2d> range_to_pos(const std::vector<float>& ranges,double angle_min,
    double angle_increment, double range_min, double range_max);

  void iterative_closet_point(const std::vector<Eigen::Vector2d> &raw_local_scan ,const std::vector<double> &input_odom);
  
};

#endif

//EOF