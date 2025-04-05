cmake_minimum_required(VERSION 3.5)
project(path_finder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(occ_grid REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_executable(${PROJECT_NAME}
  src/test_planner.cpp
  src/kdtree.c
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  occ_grid
  pcl_ros
  pcl_conversions
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install  directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
