
#include "path_tracker_interfaces/action/track_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "path_tracker/path_math.hpp"

#include <functional>
#include <memory>
#include <thread>

namespace path_tracker {
    class TrackerActionServer : public rclcpp::Node {
    public:
        using TrackerPath = path_tracker_interfaces::action::TrackPath;
        using GoalHandleTracker = rclcpp_action::ServerGoalHandle<TrackerPath>;

        explicit TrackerActionServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node("tracker_action_server", options)
        {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<TrackerPath>(
            this, "tracker_aserver",
            std::bind(&TrackerActionServer::handle_goal, this, _1, _2),
            std::bind(&TrackerActionServer::handle_cancel, this, _1),
            std::bind(&TrackerActionServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<TrackerPath>::SharedPtr action_server_;

        rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID& uuid,
                    std::shared_ptr<const TrackerPath::Goal> goal)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Receive goal path: ");
            for (const auto& pose : goal->target_path.poses) {
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Pose: "
                                   << "x: " << pose.pose.position.x
                                   << " y: " << pose.pose.position.y << "theta: "
                                   << PathMath::radToDeg(
                                      PathMath::quaternionToEulerRad(pose.pose).at(2)));
            }
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse
        handle_cancel(const std::shared_ptr<GoalHandleTracker> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTracker> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a
            // new thread
            std::thread{std::bind(&TrackerActionServer::execute, this, _1), goal_handle}
            .detach();
        }

        void execute(const std::shared_ptr<GoalHandleTracker> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TrackerPath::Feedback>();
            auto& completed_wp = feedback->completed_waypoints.poses;
            completed_wp.push_back(geometry_msgs::msg::PoseStamped());
            auto result = std::make_shared<TrackerPath::Result>();

            for (std::size_t i = 0; (i <= goal->target_path.poses.size()) && rclcpp::ok(); ++i) {
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    result->tracking_result = TrackerPath::Result::TRACKING_CANCELED;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                completed_wp.push_back(goal->target_path.poses.at(i));
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback");
                loop_rate.sleep();
            }

            // Check if goal is done
            if (rclcpp::ok()) {
                result->tracking_result = TrackerPath::Result::TRACKING_SUCCESS;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
    }; // class TrackerActionServer

} // namespace path_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(path_tracker::TrackerActionServer)