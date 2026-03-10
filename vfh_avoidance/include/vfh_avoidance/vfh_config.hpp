#ifndef VFH_AVOIDANCE_VFH_CONFIG_HPP_
#define VFH_AVOIDANCE_VFH_CONFIG_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace vfh_avoidance {
struct VFHConfig {
    struct Config {
        int vfh_sections_number = 72;
        double vfh_threshold_high = 20.0;
        double vhf_detection_range = 1.5;
        double goal_weight = 2.0;
        double prev_direction_weight = 0.5;
        double max_vel_x = 0.5;
        double max_vel_th = 1.0;
        double xy_goal_tolerance = 0.2;
        double yaw_goal_tolerance = 0.1;
    } config;

    void declareParameters(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string name) {
        node->declare_parameter(name + ".vfh_sections_number", 72);
        node->declare_parameter(name + ".vfh_threshold_high", 20.0);
    }
    
    void loadRosParamFromNodeHandle(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string name) {
        node->get_parameter(name + ".vfh_sections_number", config.vfh_sections_number);
        node->get_parameter(name + ".vfh_threshold_high", config.vfh_threshold_high);
    }
};
}
#endif