cmake_minimum_required(VERSION 3.5)
project(car_simulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(car_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# car_simulator_component
add_library(car_simulator_component SHARED
  src/car_simulator.cpp
)

ament_target_dependencies(car_simulator_component
  rclcpp
  rclcpp_components
  car_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

rclcpp_components_register_node(car_simulator_component
  PLUGIN "car_simulator::CarSimulator"
  EXECUTABLE car_simulator)

## Install
install(TARGETS ${PROJECT_NAME} car_simulator_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
