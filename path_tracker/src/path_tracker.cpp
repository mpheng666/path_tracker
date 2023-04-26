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
        , blade_rpm_pub_(this->create_publisher<std_msgs::msg::Int32>(
          "/cfr/auto_client/blade_speed", 10))
    {
        loadParams();
        loadPath();
        // initPath();
        start();
    }

    void PathTracker::start()
    {
        current_target_path_.header.frame_id = "odom";
        target_pose_ = current_path_queue_.front();
        tracker_status_ = TrackerStatus::RUNNING;
    }

    void PathTracker::loadParams()
    {
        this->declare_parameter("tracker_param.linear_x_P");
        this->declare_parameter("tracker_param.linear_y_P");
        this->declare_parameter("tracker_param.angular_z_P");
        this->declare_parameter("tracker_param.linear_x_I");
        this->declare_parameter("tracker_param.linear_y_I");
        this->declare_parameter("tracker_param.angular_z_I");
        this->declare_parameter("tracker_param.linear_x_D");
        this->declare_parameter("tracker_param.linear_y_D");
        this->declare_parameter("tracker_param.angular_z_D");
        this->declare_parameter("tracker_behaviour.align_heading_first");

        this->declare_parameter("tracker_twist_limit.max_linear_x");
        this->declare_parameter("tracker_twist_limit.max_linear_y");
        this->declare_parameter("tracker_twist_limit.max_angular_z");
        this->declare_parameter("tracker_twist_limit.min_linear_x");
        this->declare_parameter("tracker_twist_limit.min_linear_y");
        this->declare_parameter("tracker_twist_limit.min_angular_z");

        this->declare_parameter("tracker_target_tolerance.linear_x");
        this->declare_parameter("tracker_target_tolerance.linear_y");
        this->declare_parameter("tracker_target_tolerance.angular_z");

        this->get_parameter("tracker_param.linear_x_P", tracker_param_.linear_x_P);
        this->get_parameter("tracker_param.linear_y_P", tracker_param_.linear_y_P);
        this->get_parameter("tracker_param.angular_z_P", tracker_param_.angular_z_P);
        this->get_parameter("tracker_param.linear_x_I", tracker_param_.linear_x_I);
        this->get_parameter("tracker_param.linear_y_I", tracker_param_.linear_y_I);
        this->get_parameter("tracker_param.angular_z_I", tracker_param_.angular_z_I);
        this->get_parameter("tracker_param.linear_x_D", tracker_param_.linear_x_D);
        this->get_parameter("tracker_param.linear_y_D", tracker_param_.linear_y_D);
        this->get_parameter("tracker_param.angular_z_D", tracker_param_.angular_z_D);

        this->get_parameter("tracker_twist_limit.max_linear_x",
                            tracker_twist_limit_.max_linear_x);
        this->get_parameter("tracker_twist_limit.max_linear_y",
                            tracker_twist_limit_.max_linear_y);
        this->get_parameter("tracker_twist_limit.max_angular_z",
                            tracker_twist_limit_.max_angular_z);
        this->get_parameter("tracker_twist_limit.min_linear_x",
                            tracker_twist_limit_.min_linear_x);
        this->get_parameter("tracker_twist_limit.min_linear_y",
                            tracker_twist_limit_.min_linear_y);
        this->get_parameter("tracker_twist_limit.min_angular_z",
                            tracker_twist_limit_.min_angular_z);

        this->get_parameter("tracker_target_tolerance.linear_x",
                            tracker_target_tolerance_.linear_x);
        this->get_parameter("tracker_target_tolerance.linear_y",
                            tracker_target_tolerance_.linear_y);
        this->get_parameter("tracker_target_tolerance.angular_z",
                            tracker_target_tolerance_.angular_z);
        this->get_parameter("tracker_behaviour.align_heading_first",
                            use_align_heading_first_);
    }

    void PathTracker::loadPath()
    {
        this->declare_parameter("waypoints.path_complex");
        this->declare_parameter("waypoints.path_forward_backward");
        this->declare_parameter("waypoints.path_left_right");
        this->declare_parameter("waypoints.path_s");
        this->declare_parameter("waypoints.path_s_alternate");
        this->declare_parameter("waypoints.path_45_deg");
        this->declare_parameter("waypoints.path_L");
        this->declare_parameter("waypoints.use_id");

        std::string path_complex;
        std::string path_forward_backward;
        std::string path_left_right;
        std::string path_s;
        std::string path_s_alternate;
        std::string path_45_deg;
        std::string path_L;
        int use_id;

        this->get_parameter("waypoints.path_complex", path_complex);
        this->get_parameter("waypoints.path_forward_backward", path_forward_backward);
        this->get_parameter("waypoints.path_left_right", path_left_right);
        this->get_parameter("waypoints.path_s", path_s);
        this->get_parameter("waypoints.path_s_alternate", path_s_alternate);
        this->get_parameter("waypoints.path_45_deg", path_45_deg);
        this->get_parameter("waypoints.path_L", path_L);
        this->get_parameter("waypoints.use_id", use_id);

        std::vector<std::string> waypoints{
        {path_complex},     {path_forward_backward}, {path_left_right}, {path_s},
        {path_s_alternate}, {path_45_deg},           {path_L}};

        std::string chosen_waypoints = path_s;
        try {
            chosen_waypoints = waypoints.at(use_id);
        }
        catch (const std::exception& e) {
            std::cerr << "Please use the correct path id: " << e.what() << '\n';
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Chosen waypoints: " << chosen_waypoints);

        std::string error;
        loaded_points_ = ArrayParser::parseVVF(chosen_waypoints, error);
        RCLCPP_INFO_STREAM(this->get_logger(), "Path size: " << loaded_points_.size());

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
        current_target_path_ = PathGenerator::getStraightline(10, 1.0, 0.0);
        for (const auto& p : current_target_path_.poses) {
            printPose(p.pose, "path_pose");
            current_path_queue_.push(p.pose);
        }
    }

    void PathTracker::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!is_pose_initialised_) {
            initial_pose_ = msg->pose.pose;
            // reloadPathBasedOnOdom();
            printPose(initial_pose_, "initial_pose");
            is_pose_initialised_ = true;
        }
        odom_last_stamped_ = msg->header.stamp.nanosec;
        path_pub_->publish(current_target_path_);
        if (tracker_status_ == TrackerStatus::RUNNING) {
            computeTwist(msg->pose.pose);
        }
        else if (tracker_status_ == TrackerStatus::COMPLETED) {
            geometry_msgs::msg::Twist target_twist_msg;
            twist_pub_->publish(target_twist_msg);
        }
    }

    void PathTracker::pathCb([[maybe_unused]] const nav_msgs::msg::Path::SharedPtr msg)
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
        if (odom_last_stamped_ - this->now().nanoseconds() >= watchdog_timeout_ns_) {
            twist_pub_->publish(geometry_msgs::msg::Twist());
            RCLCPP_WARN_STREAM(this->get_logger(), "No odom feedback!");
            tracker_status_ = TrackerStatus::ERRORED;
        }
        else {
            tracker_status_ = TrackerStatus::RUNNING;
        }
    }

    void PathTracker::computeTwist(const geometry_msgs::msg::Pose& current_pose)
    {
        geometry_msgs::msg::Twist computed_twist_msg;

        bool res = isTargetReached(current_pose, target_pose_);
        if (res) {
            updateNextPose();
        }

        auto current_rpy = PathMath::quaternionToEulerRad(current_pose);
        auto target_theta =
        PathMath::normalizePi(PathMath::quaternionToEulerRad(target_pose_).at(2));

        TrackerErrors current_errors_global;
        current_errors_global.linear_x =
        target_pose_.position.x - current_pose.position.x;
        current_errors_global.linear_y =
        target_pose_.position.y - current_pose.position.y;
        current_errors_global.angular_z = target_theta - current_rpy.at(2);

        TrackerErrors current_errors;
        current_errors.linear_x =
        current_errors_global.linear_x * cos(current_rpy.at(2)) +
        current_errors_global.linear_y * sin(current_rpy.at(2));
        current_errors.linear_y =
        current_errors_global.linear_y * cos(current_rpy.at(2)) -
        current_errors_global.linear_x * sin(current_rpy.at(2));
        current_errors.angular_z = current_errors_global.angular_z;

        RCLCPP_INFO_STREAM(this->get_logger(),
                           "x_error: " << current_errors_global.linear_x << " = " << target_pose_.position.x
                                       << " - " << current_pose.position.x);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "y_error: " << current_errors_global.linear_y << " = " << target_pose_.position.y
                                       << " - " << current_pose.position.y);
        RCLCPP_INFO_STREAM(this->get_logger(), "yaw_error: " << current_errors_global.angular_z << " = "
                                                             << target_theta << " - "
                                                             << current_rpy.at(2));

        if (isWithinTolerance(target_theta, current_rpy.at(2),
                              tracker_target_tolerance_.angular_z) || !use_align_heading_first_) 
        {
            computed_twist_msg.linear.x =
            current_errors.linear_x * tracker_param_.linear_x_P;
            computed_twist_msg.linear.y =
            current_errors.linear_y * tracker_param_.linear_y_P;

            computed_twist_msg.linear.x +=
            (current_errors.linear_x - previous_errors_.linear_x) *
            tracker_param_.linear_x_D;
            computed_twist_msg.linear.y +=
            (current_errors.linear_y - previous_errors_.linear_y) *
            tracker_param_.linear_y_D;

            computed_twist_msg.linear.x +=
            cumulative_I_errors_.linear_x * tracker_param_.linear_x_I;
            computed_twist_msg.linear.y +=
            cumulative_I_errors_.linear_y * tracker_param_.linear_x_I;
        }
        computed_twist_msg.angular.z =
        current_errors.angular_z * tracker_param_.angular_z_P;

        computed_twist_msg.angular.z +=
        (current_errors.angular_z - previous_errors_.angular_z) *
        tracker_param_.angular_z_D;

        computed_twist_msg.angular.z +=
        cumulative_I_errors_.angular_z * tracker_param_.angular_z_I;

        previous_errors_ = current_errors;
        cumulative_I_errors_.linear_x =
        cumulative_I_errors_.linear_x * 0.9 + (current_errors.linear_x);
        cumulative_I_errors_.linear_y =
        cumulative_I_errors_.linear_y * 0.9 + (current_errors.linear_y);
        cumulative_I_errors_.angular_z =
        cumulative_I_errors_.angular_z * 0.9 + (current_errors.angular_z);

        if (!std::isnan(computed_twist_msg.angular.z)) {
            clampTwist(computed_twist_msg);
        }
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
        // RCLCPP_INFO_STREAM(
        // this->get_logger(),
        // "target: "
        // << PathMath::normalizePi(PathMath::quaternionToEulerRad(target_pose).at(2)) <<
        // "|"
        // << "current: "
        // << PathMath::normalizePi(PathMath::quaternionToEulerRad(current_pose).at(2)));
        return isWithinTolerance(target_pose.position.x, current_pose.position.x,
                                 tracker_target_tolerance_.linear_x) &&
               isWithinTolerance(target_pose.position.y, current_pose.position.y,
                                 tracker_target_tolerance_.linear_y) &&
               isWithinTolerance(
               PathMath::normalizePi(PathMath::quaternionToEulerRad(target_pose).at(2)),
               PathMath::normalizePi(PathMath::quaternionToEulerRad(current_pose).at(2)),
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
            if (current_path_queue_.empty()) {
                std_msgs::msg::Int32 blade_rpm_msg;
                blade_rpm_msg.data = 0;
                blade_rpm_pub_->publish(blade_rpm_msg);
                geometry_msgs::msg::Twist target_twist_msg;
                twist_pub_->publish(target_twist_msg);
                RCLCPP_INFO_STREAM(this->get_logger(), "Done with path tracking!");
            }
            else {
                target_pose_ = current_path_queue_.front();
                printPose(target_pose_, "target_pose");
            }
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

    void PathTracker::reloadPathBasedOnOdom()
    {
        while (!current_path_queue_.empty()) {
            current_path_queue_.pop();
        }
        current_target_path_.poses.clear();
        for (auto& p : loaded_points_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.at(0) + initial_pose_.position.x;
            pose.pose.position.y = p.at(1) + initial_pose_.position.y;
            double initial_theta =
            p.at(2) +
            PathMath::radToDeg(PathMath::quaternionToEulerRad(initial_pose_).at(2));
            auto q =
            PathMath::EulerToQuaternion({0, 0, PathMath::degToRad(initial_theta)});
            pose.pose.orientation.x = q.getX();
            pose.pose.orientation.y = q.getY();
            pose.pose.orientation.z = q.getZ();
            pose.pose.orientation.w = q.getW();
            printPose(pose.pose, "New path_pose");
            current_path_queue_.push(pose.pose);
            current_target_path_.poses.push_back(pose);
            current_target_path_.header.frame_id = "odom";
        }
        start();
    }

} // namespace path_tracker
