#include "vfh_avoidance/vfh_local_planner_ros.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>
#include <tf2/utils.h>

namespace vfh_avoidance {

void VFHPlannerRos::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    config_ = std::make_shared<VFHConfig>();
    config_->declareParameters(node, name);
    config_->loadRosParamFromNodeHandle(node, name);
    vfh_planner_.Reconfigure(*config_);
    vfh_planner_.Initialize(costmap_ros_->getCostmap());
}

geometry_msgs::msg::TwistStamped VFHPlannerRos::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = rclcpp::Clock().now();
    cmd.header.frame_id = "base_link";
    
    vfh_planner_.UpdateHistogram(costmap_ros_->getCostmap());
    
    double cur_yaw = tf2::getYaw(pose.pose.orientation);
    double target_yaw = cur_yaw; 

    if (!global_plan_.poses.empty()) {
        auto goal = global_plan_.poses.back();
        target_yaw = std::atan2(goal.pose.position.y - pose.pose.position.y, goal.pose.position.x - pose.pose.position.x);
    }
    
    double final_dir = vfh_planner_.GetNewDirection(target_yaw, cur_yaw, previews_direction_);
    previews_direction_ = final_dir;
    
    vfh_planner_.DriveToward(angles::shortest_angular_distance(cur_yaw, final_dir), cmd);
    return cmd;
}

void VFHPlannerRos::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

}
PLUGINLIB_EXPORT_CLASS(vfh_avoidance::VFHPlannerRos, nav2_core::Controller)