#ifndef PATH_TRACKER_CONTROLLER_HPP_
#define PATH_TRACKER_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>

namespace path_tracker
{
    class Controller
    {
        public:
        Controller()                                     = default;
        virtual ~Controller()                            = default;
        virtual geometry_msgs::msg::Twist computeTwist() = 0;
    };
}  // namespace path_tracker

#endif