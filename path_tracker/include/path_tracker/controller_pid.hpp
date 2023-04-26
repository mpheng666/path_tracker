#ifndef PATH_TRACKER_PID_CONTROLLER_HPP_
#define PATH_TRACKER_PID_CONTROLLER_HPP_

#include "path_tracker/controller.hpp"

namespace path_tracker
{
    class PIDController : public Controller
    {
        public:
        PIDController() { loadParam(); }

        void loadParam() { }

        virtual geometry_msgs::msg::Twist computeTwist()
        {
            
        }
    };
}  // namespace path_tracker

#endif