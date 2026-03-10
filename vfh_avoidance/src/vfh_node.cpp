#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"

namespace vfh_avoidance {

struct Valley { int start_index; int end_index; };

class VFHNode : public rclcpp::Node {
public:
    VFHNode() : Node("vfh_avoidance_node") {
        this->declare_parameter("sections", 72);
        this->declare_parameter("detect_range", 1.5);
        this->declare_parameter("threshold_high", 4.0);
        this->declare_parameter("robot_radius", 0.3);
        this->declare_parameter("max_speed", 0.3);
        this->declare_parameter("angle_gain", 1.8);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&VFHNode::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&VFHNode::odomCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&VFHNode::goalCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        histogram_.assign(this->get_parameter("sections").as_int(), 0.0);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!has_goal_) return;

        int n = histogram_.size();
        std::fill(histogram_.begin(), histogram_.end(), 0.0);
        double res = 2.0 * M_PI / n;
        double d_max = this->get_parameter("detect_range").as_double();
        double r_safe = this->get_parameter("robot_radius").as_double();

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            if (std::isnan(r) || r < 0.1 || r > d_max) continue;

            // 逻辑修正 1：统一使用 [-PI, PI] 进行计算，最后映射到索引
            double alpha = msg->angle_min + i * msg->angle_increment;
            double gamma = (r > r_safe) ? std::asin(r_safe / r) : M_PI_2;
            
            // 填充受阻扇区
            for (double a = alpha - gamma; a <= alpha + gamma; a += 0.05) {
                double norm_a = angles::normalize_angle_positive(a);
                int idx = std::clamp(static_cast<int>(norm_a / res), 0, n - 1);
                histogram_[idx] += (d_max - r);
            }
        }
        executeControl();
    }

    void executeControl() {
        geometry_msgs::msg::Twist cmd;
        double goal_angle = std::atan2(goal_y_ - curr_y_, goal_x_ - curr_x_);
        // 逻辑修正 2：使用 shortest_angular_distance 确保转向最短路径
        double angle_to_goal = angles::shortest_angular_distance(curr_yaw_, goal_angle);
        
        double res = 2.0 * M_PI / histogram_.size();
        double th = this->get_parameter("threshold_high").as_double();

        // 逻辑修正 3：动态搜索最接近目标方向的“生路”
        double best_steer = angle_to_goal;
        bool blocked = true;

        // 检查目标方向是否被堵
        double norm_goal = angles::normalize_angle_positive(angle_to_goal);
        int goal_idx = static_cast<int>(norm_goal / res) % histogram_.size();
        
        if (histogram_[goal_idx] < th) {
            blocked = false; // 直指目标的方向是安全的
        } else {
            // 搜索最近的空旷方向
            double min_delta = 1e10;
            for (int i = 0; i < (int)histogram_.size(); ++i) {
                if (histogram_[i] < th) {
                    double cand_angle = angles::normalize_angle(i * res);
                    double delta = std::abs(angles::shortest_angular_distance(cand_angle, angle_to_goal));
                    if (delta < min_delta) {
                        min_delta = delta;
                        best_steer = cand_angle;
                        blocked = false;
                    }
                }
            }
        }

        if (blocked) {
            // 逻辑修正 4：如果全屏被堵，停止线速度，只进行小幅度原地探测
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.3; 
        } else {
            double steer_diff = angles::shortest_angular_distance(0.0, best_steer);
            // 逻辑修正 5：线速度与转向角挂钩。角度越大，速度越慢，防止画龙
            double v_scale = std::max(0.0, std::cos(steer_diff));
            cmd.linear.x = this->get_parameter("max_speed").as_double() * v_scale;
            cmd.angular.z = this->get_parameter("angle_gain").as_double() * steer_diff;
        }

        if (std::hypot(goal_x_ - curr_x_, goal_y_ - curr_y_) < 0.4) {
            cmd.linear.x = 0.0; cmd.angular.z = 0.0; has_goal_ = false;
        }
        cmd_pub_->publish(cmd);
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_x_ = msg->pose.position.x; goal_y_ = msg->pose.position.y; has_goal_ = true;
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        curr_x_ = msg->pose.pose.position.x; curr_y_ = msg->pose.pose.position.y;
        curr_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::vector<double> histogram_;
    double curr_x_=0, curr_y_=0, curr_yaw_=0, goal_x_=0, goal_y_=0;
    bool has_goal_ = false;
};
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vfh_avoidance::VFHNode>());
    rclcpp::shutdown();
    return 0;
}