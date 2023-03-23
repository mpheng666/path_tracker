#include "path_tracker/path_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto tracker_node = std::make_shared<path_tracker::PathTracker>();
  rclcpp::spin(tracker_node);
  rclcpp::shutdown();

  return 0;
}
