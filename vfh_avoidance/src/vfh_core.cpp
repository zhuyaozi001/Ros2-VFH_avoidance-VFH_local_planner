#include "vfh_avoidance/vfh_core.hpp"
#include <angles/angles.h>
#include <cmath>
#include <algorithm>

namespace vfh_avoidance {

void VFHPlanner::Initialize(nav2_costmap_2d::Costmap2D*) {
    if (!initialized_) {
        vfh_histogram_.resize(72, 0.0);
        initialized_ = true;
    }
}

void VFHPlanner::Reconfigure(VFHConfig& config) {
    config_ = &config;
    vfh_histogram_.resize(config_->config.vfh_sections_number, 0.0);
}

bool VFHPlanner::UpdateHistogram(nav2_costmap_2d::Costmap2D* costmap) {
    int num_sectors = config_->config.vfh_sections_number;
    std::fill(vfh_histogram_.begin(), vfh_histogram_.end(), 0.0);
    double sector_res = 360.0 / num_sectors;

    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
        for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
            unsigned char cost = costmap->getCost(mx, my);
            if (cost < 50) continue; 

            double wx, wy;
            costmap->mapToWorld(mx, my, wx, wy);
            double dist = std::hypot(wx, wy); 
            if (dist > config_->config.vhf_detection_range) continue;

            double angle_deg = angles::to_degrees(angles::normalize_angle_positive(std::atan2(wy, wx)));
            int sector = std::clamp((int)std::floor(angle_deg / sector_res), 0, num_sectors - 1);
            vfh_histogram_[sector] += (double)cost * (config_->config.vhf_detection_range - dist);
        }
    }
    SmoothHistogram();
    return FindCandidateValleys();
}

void VFHPlanner::SmoothHistogram() {
    int n = vfh_histogram_.size();
    std::vector<double> smoothed(n);
    for (int i = 0; i < n; ++i) {
        smoothed[i] = (vfh_histogram_[(i-1+n)%n] + 2.0*vfh_histogram_[i] + vfh_histogram_[(i+1)%n]) / 4.0;
    }
    vfh_histogram_ = smoothed;
}

bool VFHPlanner::FindCandidateValleys() {
    candidate_valleys_.clear();
    bool in_valley = false;
    Valley v;
    for (int i = 0; i < (int)vfh_histogram_.size(); ++i) {
        if (vfh_histogram_[i] < config_->config.vfh_threshold_high) {
            if (!in_valley) { v.start_index = i; in_valley = true; }
        } else if (in_valley) {
            v.end_index = i - 1; candidate_valleys_.push_back(v); in_valley = false;
        }
    }
    if (in_valley) { v.end_index = (int)vfh_histogram_.size()-1; candidate_valleys_.push_back(v); }
    return !candidate_valleys_.empty();
}

double VFHPlanner::GetNewDirection(double target, double, double prev) {
    double best = target, min_c = 1e10;
    double res = 360.0 / vfh_histogram_.size();
    for (auto& v : candidate_valleys_) {
        std::vector<int> indices = {v.start_index, (v.start_index + v.end_index) / 2, v.end_index};
        for (int i : indices) {
            double cand = angles::from_degrees(i * res);
            double cost = config_->config.goal_weight * std::abs(angles::shortest_angular_distance(cand, target)) +
                          config_->config.prev_direction_weight * std::abs(angles::shortest_angular_distance(cand, prev));
            if (cost < min_c) { min_c = cost; best = cand; }
        }
    }
    return best;
}

void VFHPlanner::RotateToGoal(const geometry_msgs::msg::PoseStamped& pose, double goal_th, geometry_msgs::msg::TwistStamped& cmd) {
    double diff = angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), goal_th);
    cmd.twist.angular.z = std::clamp(2.0 * diff, -config_->config.max_vel_th, config_->config.max_vel_th);
}

void VFHPlanner::DriveToward(double diff, geometry_msgs::msg::TwistStamped& cmd) {
    cmd.twist.linear.x = std::clamp(config_->config.max_vel_x * std::cos(diff), 0.0, config_->config.max_vel_x);
    cmd.twist.angular.z = std::clamp(2.5 * diff, -config_->config.max_vel_th, config_->config.max_vel_th);
}

bool VFHPlanner::DirectionIsClear(double angle) {
    int s = std::clamp((int)(angles::to_degrees(angles::normalize_angle_positive(angle)) / (360.0/vfh_histogram_.size())), 0, (int)vfh_histogram_.size()-1);
    return vfh_histogram_[s] < config_->config.vfh_threshold_high;
}

}