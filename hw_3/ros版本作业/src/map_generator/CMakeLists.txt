cmake_minimum_required(VERSION 3.5)
project(map_generator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(random_forest src/random_forest_sensing.cpp)

ament_target_dependencies(random_forest
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

install(TARGETS random_forest
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
