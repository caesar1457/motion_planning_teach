cmake_minimum_required(VERSION 3.5)
project(trajectory_optimization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)

# click_gen node
add_executable(click_gen
  src/click_gen.cpp
)

ament_target_dependencies(click_gen
  rclcpp
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

install(TARGETS click_gen
  DESTINATION lib/${PROJECT_NAME}
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
