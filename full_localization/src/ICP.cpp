#include "full_localization/ICP.hpp"
#include <limits>
#include <iostream>
#include <unordered_map>
#include <cmath>

ICP::ICP() : recovery_cooldown_(0) {
    curr_tf = Eigen::Isometry2d::Identity();
    icp_odom = {0.0, 0.0, 0.0};
}

ICP::~ICP() {
    // Cleanup if needed
}

Eigen::Isometry2d ICP::odomToTF(const std::vector<double> &input_odom) {
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    
    // Create rotation matrix from yaw angle
    double cos_yaw = std::cos(input_odom[2]);
    double sin_yaw = std::sin(input_odom[2]);
    
    tf.linear() << cos_yaw, -sin_yaw,
                   sin_yaw,  cos_yaw;
    
    tf.translation() = Eigen::Vector2d(input_odom[0], input_odom[1]);
    
    return tf;
}

std::vector<Eigen::Vector2d> ICP::range_to_pos(
    const std::vector<float>& ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max) {
    
    std::vector<Eigen::Vector2d> points;
    points.reserve(ranges.size());
    
    for (size_t i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        
        if (std::isnan(range) || std::isinf(range) || 
            range < range_min || range > range_max) {
            continue;
        }
        
        double angle = angle_min + i * angle_increment;
        
        Eigen::Vector2d point;
        point.x() = range * std::cos(angle);
        point.y() = range * std::sin(angle);
        
        points.push_back(point);
    }
    
    return points;
}

void ICP::updateMap(const std::vector<Eigen::Vector2d> &icp_pt) {
    // Pre-allocate space
    size_t current_size = global_map_.size();
    global_map_.reserve(current_size + icp_pt.size());
    debug_map_.reserve(current_size + icp_pt.size());
    
    // Transform points to global frame and add to both maps
    for (const auto& pt : icp_pt) {
        Eigen::Vector2d global_pt = curr_tf * pt;
        global_map_.push_back(global_pt);
        debug_map_.push_back(global_pt);
    }
    
    last_map_update_pose_ = curr_tf;
    
    // Voxel filter to prevent map from growing too large
    const double voxel_size = 0.05; // 5cm resolution
    if (global_map_.size() > 80000) {
        std::unordered_map<int64_t, Eigen::Vector2d> voxel_map;
        
        for (const auto& pt : global_map_) {
            int64_t vx = static_cast<int64_t>(std::floor(pt.x() / voxel_size));
            int64_t vy = static_cast<int64_t>(std::floor(pt.y() / voxel_size));
            int64_t key = (vx << 32) | (vy & 0xFFFFFFFF);
            
            voxel_map[key] = pt;
        }
        
        global_map_.clear();
        global_map_.reserve(voxel_map.size());
        for (const auto& pair : voxel_map) {
            global_map_.push_back(pair.second);
        }
        std::cout << "Map downsampled from 80000 to " << global_map_.size() << " points" << std::endl;
    }
}

// Simple 2D grid-based spatial hash for fast nearest neighbor lookup
class SpatialHash {
private:
    std::unordered_map<int64_t, std::vector<Eigen::Vector2d>> grid_;
    double cell_size_;
    
    int64_t hash(double x, double y) const {
        int64_t ix = static_cast<int64_t>(std::floor(x / cell_size_));
        int64_t iy = static_cast<int64_t>(std::floor(y / cell_size_));
        return (ix << 32) | (iy & 0xFFFFFFFF);
    }
    
public:
    SpatialHash(double cell_size) : cell_size_(cell_size) {}
    
    void insert(const Eigen::Vector2d& pt) {
        grid_[hash(pt.x(), pt.y())].push_back(pt);
    }
    
    void build(const std::vector<Eigen::Vector2d>& points) {
        grid_.clear();
        for (const auto& pt : points) {
            insert(pt);
        }
    }
    
    Eigen::Vector2d findNearest(const Eigen::Vector2d& query, double max_dist, bool& found) const {
        int64_t cx = static_cast<int64_t>(std::floor(query.x() / cell_size_));
        int64_t cy = static_cast<int64_t>(std::floor(query.y() / cell_size_));
        
        double min_dist_sq = max_dist * max_dist;
        Eigen::Vector2d nearest;
        found = false;
        
        // Search in 3x3 grid around query point
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                int64_t key = ((cx + dx) << 32) | ((cy + dy) & 0xFFFFFFFF);
                auto it = grid_.find(key);
                
                if (it != grid_.end()) {
                    for (const auto& pt : it->second) {
                        double dist_sq = (query - pt).squaredNorm();
                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            nearest = pt;
                            found = true;
                        }
                    }
                }
            }
        }
        
        return nearest;
    }
};

// Normalize angle to [-pi, pi]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Compute quality metrics for ICP match
struct MatchQuality {
    double mean_error;
    double std_error;
    double inlier_ratio;
    int num_matches;
    
    bool isGood() const {
        // Relaxed quality thresholds for better tracking
        return inlier_ratio > 0.3 && num_matches > 15 && mean_error < 0.5;
    }
    
    bool isReasonable() const {
        // Even more relaxed for hallways
        return inlier_ratio > 0.2 && num_matches > 10;
    }
};

MatchQuality computeMatchQuality(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& correspondences) {
    MatchQuality quality;
    quality.num_matches = correspondences.size();
    
    if (correspondences.empty()) {
        quality.mean_error = 1000.0;
        quality.std_error = 1000.0;
        quality.inlier_ratio = 0.0;
        return quality;
    }
    
    // Compute errors
    std::vector<double> errors;
    errors.reserve(correspondences.size());
    double sum_error = 0.0;
    
    for (const auto& corr : correspondences) {
        double error = (corr.first - corr.second).norm();
        errors.push_back(error);
        sum_error += error;
    }
    
    quality.mean_error = sum_error / correspondences.size();
    
    // Compute standard deviation
    double sum_sq_diff = 0.0;
    for (double error : errors) {
        double diff = error - quality.mean_error;
        sum_sq_diff += diff * diff;
    }
    quality.std_error = std::sqrt(sum_sq_diff / correspondences.size());
    
    // Count inliers (within 2 std devs)
    double inlier_threshold = quality.mean_error + 2.0 * quality.std_error;
    int inliers = 0;
    for (double error : errors) {
        if (error < inlier_threshold) {
            inliers++;
        }
    }
    quality.inlier_ratio = static_cast<double>(inliers) / correspondences.size();
    
    return quality;
}

void ICP::iterative_closet_point(
    const std::vector<Eigen::Vector2d> &raw_local_scan,
    const std::vector<double> &input_odom) {
    
    // Handle first run - initialize map
    if (first_run_) {
        curr_tf = odomToTF(input_odom);
        updateMap(raw_local_scan);
        prev_odom_ = input_odom;
        icp_odom = input_odom;
        first_run_ = false;
        std::cout << "ICP: First run initialized at (" << input_odom[0] << ", " << input_odom[1] << ")" << std::endl;
        return;
    }
    
    // Calculate odometry delta
    Eigen::Isometry2d prev_tf = odomToTF(prev_odom_);
    Eigen::Isometry2d curr_odom_tf = odomToTF(input_odom);
    Eigen::Isometry2d delta_odom = prev_tf.inverse() * curr_odom_tf;
    
    // Extract delta for drift detection
    Eigen::Vector2d delta_trans = delta_odom.translation();
    double delta_dist = delta_trans.norm();
    double delta_yaw = std::atan2(delta_odom.linear()(1, 0), delta_odom.linear()(0, 0));
    
    // Initial guess using odometry
    Eigen::Isometry2d icp_initial_guess = curr_tf * delta_odom;
    
    // If map is too small, just update with odometry
    if (global_map_.size() < 20) {
        curr_tf = icp_initial_guess;
        updateMap(raw_local_scan);
        prev_odom_ = input_odom;
        
        Eigen::Vector2d trans = curr_tf.translation();
        double yaw = std::atan2(curr_tf.linear()(1, 0), curr_tf.linear()(0, 0));
        icp_odom = {trans.x(), trans.y(), yaw};
        return;
    }
    
    // Downsample scan for faster processing but keep more points for accuracy
    std::vector<Eigen::Vector2d> source;
    int skip = std::max(1, static_cast<int>(raw_local_scan.size() / 250));
    source.reserve(raw_local_scan.size() / skip);
    for (size_t i = 0; i < raw_local_scan.size(); i += skip) {
        source.push_back(raw_local_scan[i]);
    }
    
    // Build spatial hash for fast nearest neighbor queries
    const double cell_size = 0.5; // 50cm cells
    SpatialHash spatial_hash(cell_size);
    spatial_hash.build(global_map_);
    
    // ICP parameters
    const int max_iterations = 25;
    const double convergence_threshold = 1e-5;
    const double max_correspondence_distance = 0.5; // meters
    
    // Pre-allocate vectors
    std::vector<Eigen::Vector2d> transformed_source;
    transformed_source.reserve(source.size());
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences;
    correspondences.reserve(source.size());
    
    // Start with odometry estimate
    Eigen::Isometry2d working_tf = icp_initial_guess;
    
    // ICP main loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Transform source points to global frame
        transformed_source.clear();
        Eigen::Matrix2d R = working_tf.linear();
        Eigen::Vector2d t = working_tf.translation();
        
        for (const auto& pt : source) {
            transformed_source.push_back(R * pt + t);
        }
        
        // Find correspondences using spatial hash
        correspondences.clear();
        for (const auto& src_pt : transformed_source) {
            bool found = false;
            Eigen::Vector2d nearest_pt = spatial_hash.findNearest(src_pt, max_correspondence_distance, found);
            
            if (found) {
                correspondences.push_back({src_pt, nearest_pt});
            }
        }
        
        // Need minimum correspondences to proceed
        if (correspondences.size() < 15) {
            break;
        }
        
        // Outlier rejection using RANSAC-like approach (only after first iteration)
        if (iter > 0 && correspondences.size() > 30) {
            std::vector<double> errors;
            errors.reserve(correspondences.size());
            for (const auto& corr : correspondences) {
                errors.push_back((corr.first - corr.second).squaredNorm());
            }
            std::sort(errors.begin(), errors.end());
            double median_error = errors[errors.size() / 2];
            double threshold = 2.5 * std::sqrt(median_error);
            
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> inliers;
            for (const auto& corr : correspondences) {
                if ((corr.first - corr.second).norm() < threshold) {
                    inliers.push_back(corr);
                }
            }
            correspondences = inliers;
        }
        
        if (correspondences.size() < 15) {
            break;
        }
        
        // Compute centroids
        Eigen::Vector2d centroid_src = Eigen::Vector2d::Zero();
        Eigen::Vector2d centroid_tgt = Eigen::Vector2d::Zero();
        
        for (const auto& corr : correspondences) {
            centroid_src += corr.first;
            centroid_tgt += corr.second;
        }
        double inv_size = 1.0 / correspondences.size();
        centroid_src *= inv_size;
        centroid_tgt *= inv_size;
        
        // Compute covariance matrix
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (const auto& corr : correspondences) {
            Eigen::Vector2d src_centered = corr.first - centroid_src;
            Eigen::Vector2d tgt_centered = corr.second - centroid_tgt;
            H.noalias() += src_centered * tgt_centered.transpose();
        }
        
        // SVD for rotation
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R_corr = svd.matrixV() * svd.matrixU().transpose();
        
        // Ensure proper rotation (det = 1)
        if (R_corr.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1;
            R_corr = V * svd.matrixU().transpose();
        }
        
        // Compute translation
        Eigen::Vector2d t_corr = centroid_tgt - R_corr * centroid_src;
        
        // Create correction transform
        Eigen::Isometry2d correction = Eigen::Isometry2d::Identity();
        correction.linear() = R_corr;
        correction.translation() = t_corr;
        
        // Apply correction
        Eigen::Isometry2d new_tf = correction * working_tf;
        
        // Check convergence
        double delta_translation_sq = (new_tf.translation() - working_tf.translation()).squaredNorm();
        double delta_rotation = std::atan2(
            (new_tf.linear() * working_tf.linear().transpose())(1, 0),
            (new_tf.linear() * working_tf.linear().transpose())(0, 0)
        );
        
        working_tf = new_tf;
        
        if (delta_translation_sq < convergence_threshold * convergence_threshold &&
            std::abs(delta_rotation) < convergence_threshold) {
            break;
        }
    }
    
    // Compute match quality
    MatchQuality quality = computeMatchQuality(correspondences);
    
    // RELAXED HALLWAY ALIASING PROTECTION
    // Check if ICP result is unreasonable compared to odometry
    Eigen::Vector2d icp_delta = working_tf.translation() - curr_tf.translation();
    double icp_delta_dist = icp_delta.norm();
    
    double curr_yaw = std::atan2(curr_tf.linear()(1, 0), curr_tf.linear()(0, 0));
    double working_yaw = std::atan2(working_tf.linear()(1, 0), working_tf.linear()(0, 0));
    double icp_delta_yaw = normalizeAngle(working_yaw - curr_yaw);
    
    // More permissive thresholds - allow bigger deviations
    double max_allowed_dist_diff = std::max(0.8, delta_dist * 5.0); // Allow 5x odometry or 0.8m
    double max_allowed_rot_diff = std::max(0.5, std::abs(delta_yaw) * 5.0); // Allow 5x odometry or 0.5 rad
    
    bool icp_reasonable = (std::abs(icp_delta_dist - delta_dist) < max_allowed_dist_diff) &&
                          (std::abs(icp_delta_yaw - delta_yaw) < max_allowed_rot_diff);
    
    // Decide whether to trust ICP or fall back to odometry
    // Use ICP if quality is reasonable OR if it's at least somewhat reasonable and motion is small
    bool use_icp = (quality.isGood() && icp_reasonable) || 
                   (quality.isReasonable() && icp_reasonable && delta_dist < 0.2);
    
    if (use_icp) {
        // Trust ICP result
        curr_tf = working_tf;
        if (recovery_cooldown_ > 0) {
            recovery_cooldown_--;
        }
    } else {
        // Fall back to odometry (hallway or poor match detected)
        curr_tf = icp_initial_guess;
        recovery_cooldown_ = std::min(5, recovery_cooldown_ + 1);
        
        if (recovery_cooldown_ >= 3) {
            std::cout << "ICP: Using odometry (quality=" << quality.inlier_ratio 
                      << ", matches=" << quality.num_matches 
                      << ", error=" << quality.mean_error << ")" << std::endl;
        }
    }
    
    // Update map more frequently but only when quality is good
    double distance_from_last_update = 
        (curr_tf.translation() - last_map_update_pose_.translation()).norm();
    double rotation_from_last_update = std::abs(normalizeAngle(
        std::atan2(curr_tf.linear()(1, 0), curr_tf.linear()(0, 0)) -
        std::atan2(last_map_update_pose_.linear()(1, 0), last_map_update_pose_.linear()(0, 0))
    ));
    
    // More frequent map updates when quality is good, less when using odometry fallback
    double update_dist_threshold = recovery_cooldown_ > 0 ? 0.5 : 0.25;
    double update_rot_threshold = recovery_cooldown_ > 0 ? 0.4 : 0.2;
    
    if ((distance_from_last_update > update_dist_threshold || 
         rotation_from_last_update > update_rot_threshold) && 
        quality.isReasonable()) {
        updateMap(raw_local_scan);
    }
    
    // Update previous odometry
    prev_odom_ = input_odom;
    
    // Extract pose from transform
    Eigen::Vector2d trans = curr_tf.translation();
    double yaw = std::atan2(curr_tf.linear()(1, 0), curr_tf.linear()(0, 0));
    icp_odom = {trans.x(), trans.y(), yaw};
}

// EOF