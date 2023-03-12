#ifndef PATH_TRACKER_PATH_GENERATOR_HPP_
#define PATH_TRACKER_PATH_GENERATOR_HPP_

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace path_tracker {

    class PathGenerator {
    public:
        static nav_msgs::msg::Path
        getCircularline(int nums, double distance, double start)
        {
            nav_msgs::msg::Path generated_path;
            for (int i = 0; i < nums; ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = i * sin(i * 3.142 / 100.0) * distance + start;
                pose.pose.position.y = i * 0.5 * sin(i * 3.142 / 100.0) * distance + start;
                generated_path.poses.push_back(pose);
            }
            return generated_path;
        }

        static nav_msgs::msg::Path
        getStraightline(int nums, double distance, double start)
        {
            nav_msgs::msg::Path generated_path;
            for (int i = 0; i < nums; ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = i * distance + start;
                generated_path.poses.push_back(pose);
            }
            return generated_path;
        }
    };
} // namespace path_tracker

#endif