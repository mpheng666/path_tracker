#include "path_tracker/path_tracker.hpp"

namespace path_tracker {
    PathTracker::PathTracker()
        : Node("path_tracker_node")
        , twist_pub_(this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10))
        , odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&PathTracker::odomCb, this, std::placeholders::_1)))
        , watchdog_twist_zero_pub_timer_(
          this->create_wall_timer(std::chrono::duration(std::chrono::milliseconds(100)),
                                  std::bind(&PathTracker::WatchDogTwistZeroPubCb, this)))
        , status_pub_(
          this->create_publisher<std_msgs::msg::Int32>("~/tracker_status", 10))
        , status_pub_timer_(
          this->create_wall_timer(std::chrono::duration(std::chrono::milliseconds(100)),
                                  std::bind(&PathTracker::statusPubCb, this)))
        , status_event_sub_(this->create_subscription<std_msgs::msg::Int32>(
          "tracker_command",
          10,
          std::bind(&PathTracker::statusEventCb, this, std::placeholders::_1)))
    {
    }

    void PathTracker::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        computeTwist(msg->pose.pose);
        odom_last_stamped_ = msg->header.stamp.nanosec;
    }

    void PathTracker::pathCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if ((*msg).header.stamp != current_target_path_.header.stamp) {
            current_target_path_ = *msg;
            if (current_target_path_.poses.size()) {
                while (current_path_queue_.size()) {
                    current_path_queue_.pop();
                }
                for (const auto& pose : msg->poses) {
                    current_path_queue_.push(pose.pose);
                }
            }
        }
    }

    void PathTracker::WatchDogTwistZeroPubCb()
    {
        if (odom_last_stamped_ - this->now().nanoseconds() >= watchdog_timeout_ns_) {
            twist_pub_->publish(geometry_msgs::msg::Twist());
            RCLCPP_WARN_STREAM(this->get_logger(), "No odom feedback!");
            tracker_status_ = TrackerStatus::ERRORED;
        }
        else {
            tracker_status_ = TrackerStatus::IDLE;
        }
    }

    void PathTracker::computeTwist(const geometry_msgs::msg::Pose& current_pose)
    {
        geometry_msgs::msg::Twist computed_twist_msg;

        if (isTargetReached(current_pose, target_pose_)) {
            updateNextPose();
        }

        auto current_rpy = computeYawFromQuaternion(current_pose);
        double x_error = target_pose_.position.x - current_pose.position.x;
        double y_error = target_pose_.position.y - current_pose.position.y;
        double yaw_error = current_rpy.at(2);

        computed_twist_msg.linear.x = x_error * tracker_param_.linear_x_P;
        computed_twist_msg.linear.y = y_error * tracker_param_.linear_y_P;
        computed_twist_msg.angular.z = yaw_error * tracker_param_.angular_z_P;

        clampTwist(computed_twist_msg);
    }

    void PathTracker::clampTwist(const geometry_msgs::msg::Twist& msg)
    {
        geometry_msgs::msg::Twist target_twist_msg;

        target_twist_msg.linear.x =
        std::clamp(msg.linear.x, tracker_twist_limit_.min_linear_x,
                   tracker_twist_limit_.max_linear_x);
        target_twist_msg.linear.y =
        std::clamp(msg.linear.y, tracker_twist_limit_.min_linear_y,
                   tracker_twist_limit_.max_linear_y);
        target_twist_msg.angular.z =
        std::clamp(msg.angular.z, tracker_twist_limit_.min_angular_z,
                   tracker_twist_limit_.max_angular_z);

        twist_pub_->publish(target_twist_msg);
    }

    bool PathTracker::isTargetReached(const geometry_msgs::msg::Pose& current_pose,
                                      const geometry_msgs::msg::Pose& target_pose)
    {
        if (abs(target_pose.position.x - target_pose_.position.x) >
            tracker_target_tolerance_.linear_x) {
            return false;
        }
        if (abs(target_pose.position.y - target_pose_.position.y) >
            tracker_target_tolerance_.linear_y) {
            return false;
        }
        auto current_rpy = computeYawFromQuaternion(current_pose);
        if (abs(target_rpy_.at(2) - current_rpy.at(2)) >
            tracker_target_tolerance_.angular_z) {
            return false;
        }
        return true;
    }

    void PathTracker::updateNextPose()
    {
        current_path_queue_.pop();
        if (current_path_queue_.size() && tracker_status_ == TrackerStatus::RUNNING) {
            target_pose_ = current_path_queue_.front();
        }
        else {
            tracker_status_ = TrackerStatus::COMPLETED;
        }
    }

    void PathTracker::statusPubCb()
    {
        std_msgs::msg::Int32 status_msg;
        status_msg.data = static_cast<int32_t>(tracker_status_);
        status_pub_->publish(status_msg);
    }

    void PathTracker::statusEventCb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        switch (static_cast<TrackerEvent>(msg->data)) {
            case TrackerEvent::START:
                tracker_status_ = TrackerStatus::RUNNING;
                break;
            case TrackerEvent::STOP:
                tracker_status_ = TrackerStatus::STOPPED;
                while (current_path_queue_.size()) {
                    current_path_queue_.pop();
                }
                break;
            case TrackerEvent::PAUSE:
                tracker_status_ = TrackerStatus::PAUSED;
                break;
            default:
                RCLCPP_WARN_STREAM(this->get_logger(), "INVALID EVNET");
                break;
        }
    }

    RPY_T PathTracker::computeYawFromQuaternion(const geometry_msgs::msg::Pose& msg)
    {
        tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z,
                          msg.orientation.w);
        tf2::Matrix3x3 rpy(q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        rpy.getRPY(roll, pitch, yaw);
        return RPY_T(roll, pitch, yaw);
    }

} // namespace path_tracker