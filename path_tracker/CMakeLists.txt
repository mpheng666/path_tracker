cmake_minimum_required(VERSION 3.5)
project(path_tracker)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_tracker_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(path_tracker_node src/path_tracker_node.cpp src/path_tracker.cpp)
target_include_directories(path_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_tracker_node "rclcpp" "std_msgs" "geometry_msgs" "nav_msgs")


add_library(tracker_aserver SHARED src/tracker_action_server.cpp)
target_include_directories(tracker_aserver PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(tracker_aserver
  PRIVATE "PATH_TRACKER_BUILDING_DLL")
ament_target_dependencies(tracker_aserver
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "path_tracker_interfaces")
rclcpp_components_register_node(tracker_aserver PLUGIN "path_tracker::TrackerActionServer" EXECUTABLE tracker_action_server)

install(TARGETS
  tracker_aserver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(TARGETS path_tracker_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
