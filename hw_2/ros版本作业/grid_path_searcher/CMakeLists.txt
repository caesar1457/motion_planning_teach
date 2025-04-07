cmake_minimum_required(VERSION 3.5)
project(grid_path_searcher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
)

add_executable(demo_node
  src/astar_path_finder.cpp
  src/jps3d_neib.cpp
  src/jps_path_finder.cpp
  src/demo_node.cpp
  src/demo_node_main.cpp
)

ament_target_dependencies(demo_node
  rclcpp
  nav_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

add_executable(random_complex src/random_complex_generator.cpp)

ament_target_dependencies(random_complex
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

install(TARGETS demo_node random_complex
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
