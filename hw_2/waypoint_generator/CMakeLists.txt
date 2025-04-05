cmake_minimum_required(VERSION 3.5)
project(waypoint_generator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(waypoint_generator src/waypoint_generator.cpp src/main.cpp)
ament_target_dependencies(waypoint_generator
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)

install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
