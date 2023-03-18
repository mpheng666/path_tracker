#ifndef PATH_TRACKER_PATH_TRACKER_HPP_
#define PATH_TRACKER_PATH_TRACKER_HPP_

#include "path_tracker/path_generator.hpp"
#include "path_tracker/path_math.hpp"
#include "path_tracker/array_parser.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <queue>

namespace path_tracker {
    struct TrackerParam {
        double linear_x_P{0.6};
        double linear_y_P{0.6};
        double angular_z_P{0.6};
    };

    struct TrackerTwistLimit {
        double max_linear_x{1.0};
        double max_linear_y{1.0};
        double max_angular_z{0.5};
        double min_linear_x{-1.0};
        double min_linear_y{-1.0};
        double min_angular_z{-0.5};
    };

    struct TrackerTargetTolerance {
        double linear_x{0.2};
        double linear_y{0.2};
        double angular_z{0.3};
    };

    enum class TrackerStatus : int {
        IDLE = 0,
        RUNNING = 1,
        PAUSED = 2,
        STOPPED = 3,
        ERRORED = 4,
        COMPLETED = 5
    };

    enum class TrackerEvent : int { START = 0, STOP = 1, PAUSE = 2 };


    class PathTracker : public rclcpp::Node {
    public:
        PathTracker();

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::TimerBase::SharedPtr watchdog_twist_zero_pub_timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
        rclcpp::TimerBase::SharedPtr status_pub_timer_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_event_sub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr blade_rpm_pub_;

        geometry_msgs::msg::Pose target_pose_;
        RPY_T target_rpy_;

        nav_msgs::msg::Path current_target_path_;
        std::queue<geometry_msgs::msg::Pose> current_path_queue_;
        std::vector<std::vector<float>> loaded_points_;

        TrackerParam tracker_param_;
        TrackerTwistLimit tracker_twist_limit_;
        TrackerTargetTolerance tracker_target_tolerance_;
        TrackerStatus tracker_status_{TrackerStatus::IDLE};

        static constexpr uint32_t watchdog_timeout_ns_{500'000};
        uint32_t odom_last_stamped_{0};

        geometry_msgs::msg::Pose initial_pose_;

        bool is_pose_initialised_ {false};

        void loadParams();
        void loadPath();
        void initPath();
        void start();
        void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pathCb(const nav_msgs::msg::Path::SharedPtr msg);
        void WatchDogTwistZeroPubCb();
        void computeTwist(const geometry_msgs::msg::Pose& current_pose);
        void clampTwist(const geometry_msgs::msg::Twist& computed_twist);
        bool isTargetReached(const geometry_msgs::msg::Pose& current_pose,
                             const geometry_msgs::msg::Pose& target_pose);
        template <typename T>
        bool isWithinTolerance(const T& target, const T& current, T tolerance);

        void updateNextPose();
        void statusPubCb();
        void statusEventCb(const std_msgs::msg::Int32::SharedPtr msg);

        void printPose(const geometry_msgs::msg::Pose& pose, const std::string& name);

        void reloadPathBasedOnOdom();
    };
} // namespace path_tracker

#endif
