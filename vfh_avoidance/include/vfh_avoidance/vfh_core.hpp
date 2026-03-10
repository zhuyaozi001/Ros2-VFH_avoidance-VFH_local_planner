#ifndef VFH_AVOIDANCE_VFH_CORE_HPP_
#define VFH_AVOIDANCE_VFH_CORE_HPP_

#include <vector>
#include "vfh_avoidance/vfh_config.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "tf2/utils.h"

namespace vfh_avoidance {

struct Valley { int start_index; int end_index; };

class VFHPlanner {
public:
    VFHPlanner();
    void Initialize(nav2_costmap_2d::Costmap2D* costmap);
    void Reconfigure(VFHConfig& config);
    bool UpdateHistogram(nav2_costmap_2d::Costmap2D* costmap);
    double GetNewDirection(double target_angle, double current_yaw, double prev_direction);
    bool DirectionIsClear(double angle);
    void RotateToGoal(const geometry_msgs::msg::PoseStamped& pose, double goal_th, geometry_msgs::msg::TwistStamped& cmd_vel);
    void DriveToward(double angle_diff, geometry_msgs::msg::TwistStamped& cmd_vel);

private:
    void SmoothHistogram();
    bool FindCandidateValleys();
    VFHConfig* config_;
    std::vector<double> vfh_histogram_;
    std::vector<Valley> candidate_valleys_;
    bool initialized_;
};
}
#endif