#ifndef PATH_TRACKER_PATH_TRACKER_HPP_
#define PATH_TRACKER_PATH_TRACKER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>

namespace path_tracker {
    struct TrackerParam {
        double linear_x_P{0.05};
        double linear_y_P{0.05};
        double angular_z_P{0.05};
    };

    struct TrackerTwistLimit {
        double max_linear_x{1.0};
        double max_linear_y{1.0};
        double max_angular_z{0.5};
        double min_linear_x{-1.0};
        double min_linear_y{-1.0};
        double min_angular_z{-0.5};
    };

    struct TrackerTargetTolerance
    {
        double linear_x {0.01};
        double linear_y {0.01};
        double angular_z {0.01};
    };

    using RPY_T = std::array<double, 3>;

    class PathTracker : public rclcpp::Node {
    public:
        PathTracker();

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::TimerBase::SharedPtr watchdog_twist_zero_pub_timer_;

        geometry_msgs::msg::Pose target_pose_;
        RPY_T target_rpy_;

        nav_msgs::msg::Path current_target_path_;

        TrackerParam tracker_param_;
        TrackerTwistLimit tracker_twist_limit_;
        TrackerTargetTolerance tracker_target_tolerance_;

        static constexpr uint32_t watchdog_timeout_ns_{500'000};
        uint32_t odom_last_stamped_{0};

        void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pathCb(const nav_msgs::msg::Path::SharedPtr msg);
        void WatchDogTwistZeroPubCb();
        void computeTwist(const geometry_msgs::msg::Pose& current_pose);
        void clampTwist(const geometry_msgs::msg::Twist& computed_twist);
        bool isTargetReached(const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& target_pose);
        static RPY_T computeYawFromQuaternion(const geometry_msgs::msg::Pose& msg);
    };
} // namespace path_tracker

#endif