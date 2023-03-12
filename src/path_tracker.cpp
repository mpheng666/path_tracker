#include "path_tracker/path_tracker.hpp"

namespace path_tracker {
    PathTracker::PathTracker()
        : Node("path_tracker_node")
        , twist_pub_(this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10))
        , odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&PathTracker::odomCb, this, std::placeholders::_1)))
        , path_sub_(this->create_subscription<nav_msgs::msg::Path>(
          "tracker_new_path",
          10,
          std::bind(&PathTracker::pathCb, this, std::placeholders::_1)))
        , path_pub_(this->create_publisher<nav_msgs::msg::Path>("tracker_path", 10))
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
        loadPath();
        // initPath();
        start();
    }

    void PathTracker::start()
    {
        current_target_path_.header.frame_id = "odom";
        tracker_status_ = TrackerStatus::RUNNING;
    }

    void PathTracker::loadPath()
    {
        std::string points = this->declare_parameter("waypoints", "");
        std::string error;

        RCLCPP_INFO_STREAM(this->get_logger(), "waypoints: " << points);

        auto loaded_points_ = ArrayParser::parseVVF(points, error);
        RCLCPP_INFO_STREAM(this->get_logger(), "path size: " << loaded_points_.size());

        if (error.empty()) {
            for (const auto& p : loaded_points_) {
                assert(p.size() == 3);
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = p.at(0);
                pose.pose.position.y = p.at(1);
                auto q = PathMath::EulerToQuaternion({0, 0, PathMath::degToRad(p.at(2))});
                pose.pose.orientation.x = q.getX();
                pose.pose.orientation.y = q.getY();
                pose.pose.orientation.z = q.getZ();
                pose.pose.orientation.w = q.getW();
                printPose(pose.pose, "path_pose");
                current_path_queue_.push(pose.pose);
                current_target_path_.poses.push_back(pose);
                current_target_path_.header.frame_id = "odom";
            }
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), error);
        }
    }

    void PathTracker::initPath()
    {
        current_target_path_ = PathGenerator::getCircularline(10, 0.6, 0.0);
        current_target_path_ = PathGenerator::getStraightline(5, 1.0, 0.0);
        for (const auto& p : current_target_path_.poses) {
            printPose(p.pose, "path_pose");
            current_path_queue_.push(p.pose);
        }
    }

    void PathTracker::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Getting odom");
        odom_last_stamped_ = msg->header.stamp.nanosec;
        path_pub_->publish(current_target_path_);
        // if (tracker_status_ == TrackerStatus::RUNNING) {
        computeTwist(msg->pose.pose);
        // }
    }

    void PathTracker::pathCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // if ((*msg).header.stamp != current_target_path_.header.stamp) {
        //     current_target_path_ = *msg;
        //     if (current_target_path_.poses.size()) {
        //         while (current_path_queue_.size()) {
        //             current_path_queue_.pop();
        //         }
        //         for (const auto& pose : msg->poses) {
        //             current_path_queue_.push(pose.pose);
        //         }
        //     }
        // }
    }

    void PathTracker::WatchDogTwistZeroPubCb()
    {
        // if (odom_last_stamped_ - this->now().nanoseconds() >= watchdog_timeout_ns_) {
        //     twist_pub_->publish(geometry_msgs::msg::Twist());
        //     RCLCPP_WARN_STREAM(this->get_logger(), "No odom feedback!");
        //     tracker_status_ = TrackerStatus::ERRORED;
        // }
        // else {
        //     tracker_status_ = TrackerStatus::IDLE;
        // }
    }

    void PathTracker::computeTwist(const geometry_msgs::msg::Pose& current_pose)
    {
        geometry_msgs::msg::Twist computed_twist_msg;

        bool res = isTargetReached(current_pose, target_pose_);
        if (res) {
            updateNextPose();
        }

        auto current_rpy = PathMath::quaternionToEulerRad(current_pose);
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
        return isWithinTolerance(target_pose.position.x, current_pose.position.x,
                                 tracker_target_tolerance_.linear_x) &&
               isWithinTolerance(target_pose.position.y, current_pose.position.y,
                                 tracker_target_tolerance_.linear_y) &&
               isWithinTolerance(target_rpy_.at(2),
                                 PathMath::quaternionToEulerRad(current_pose).at(2),
                                 tracker_target_tolerance_.angular_z);
    }

    template <typename T>
    bool PathTracker::isWithinTolerance(const T& target, const T& current, T tolerance)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Target: " << target);
        // RCLCPP_INFO_STREAM(this->get_logger(), "current: " << current);
        T result = target - current;
        // RCLCPP_INFO_STREAM(this->get_logger(), "Result: " << result);
        return std::abs(result) <= tolerance;
    }

    void PathTracker::updateNextPose()
    {
        if (current_path_queue_.size() && tracker_status_ == TrackerStatus::RUNNING) {
            current_path_queue_.pop();
            target_pose_ = current_path_queue_.front();
            printPose(target_pose_, "target_pose");
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

    void PathTracker::printPose(const geometry_msgs::msg::Pose& pose,
                                const std::string& name)
    {
        RCLCPP_INFO_STREAM(
        this->get_logger(),
        name << " x: " << pose.position.x << " y: " << pose.position.y
             << " z: " << PathMath::radToDeg(PathMath::quaternionToEulerRad(pose).at(2)));
    }

} // namespace path_tracker