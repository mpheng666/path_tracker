#ifndef PATH_TRACKER_PATH_MATH_HPP_
#define PATH_TRACKER_PATH_MATH_HPP_

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose.hpp"

#include <cmath>

namespace path_tracker {
    using RPY_T = std::array<double, 3>;

    class PathMath {
    public:
        static RPY_T quaternionToEulerRad(const geometry_msgs::msg::Pose& msg)
        {
            tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z,
                              msg.orientation.w);
            tf2::Matrix3x3 rpy(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            rpy.getRPY(roll, pitch, yaw);
            return RPY_T{roll, pitch, yaw};
        }

        static tf2::Quaternion EulerToQuaternion(const RPY_T& rpy)
        {
            tf2::Quaternion q;
            q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
            return q;
        }

        static double degToRad(double val)
        {
            return (val / 180.0) * M_PI;
        }

        static double radToDeg(double val)
        {
            return (val / M_PI) * 180.0;
        }
    };
} // namespace path_tracker

#endif