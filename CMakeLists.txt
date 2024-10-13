cmake_minimum_required (VERSION 3.8)
project (ros2_odom_to_tf)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (std_msgs REQUIRED)
find_package (nav_msgs REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (tf2_ros REQUIRED)
find_package (builtin_interfaces REQUIRED)

add_executable (the_node src/the_node.cpp)
ament_target_dependencies (the_node
  rclcpp
  std_msgs
  builtin_interfaces
  nav_msgs
  geometry_msgs
  tf2_ros
)

install (TARGETS the_node DESTINATION lib/${PROJECT_NAME})

ament_package()
