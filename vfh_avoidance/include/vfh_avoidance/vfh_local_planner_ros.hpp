#ifndef VFH_AVOIDANCE_VFH_LOCAL_PLANNER_ROS_HPP_
#define VFH_AVOIDANCE_VFH_LOCAL_PLANNER_ROS_HPP_

#include "nav2_core/controller.hpp"
#include "vfh_avoidance/vfh_core.hpp"
#include "vfh_avoidance/vfh_config.hpp"
#include "tf2_ros/buffer.h"

namespace vfh_avoidance {

class VFHPlannerRos : public nav2_core::Controller {
public:
    VFHPlannerRos() = default;
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void activate() override {}
    void deactivate() override {}
    void cleanup() override {}
    void setPlan(const nav_msgs::msg::Path & path) override;
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override {(void)speed_limit; (void)percentage;}

private:
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav_msgs::msg::Path global_plan_;
    std::shared_ptr<VFHConfig> config_;
    VFHPlanner vfh_planner_;
    double previews_direction_{0.0};
    bool finding_alternative_way_{false};
};
}
#endif