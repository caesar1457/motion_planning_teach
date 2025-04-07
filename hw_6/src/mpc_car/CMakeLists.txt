cmake_minimum_required(VERSION 3.5)
project(mpc_car)


list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../third_party/osqp/build/install")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(car_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp CONFIG REQUIRED)

# mpc_car_component
add_library(mpc_car_component SHARED
  src/mpc_car.cpp
)


target_include_directories(mpc_car_component PRIVATE
  include
  ${CMAKE_SOURCE_DIR}/../../third_party/osqp/build/install/include
)

ament_target_dependencies(mpc_car_component
  rclcpp
  rclcpp_components
  car_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(mpc_car_component
  osqp::osqp
)

rclcpp_components_register_node(mpc_car_component
  PLUGIN "mpc_car::MpcCarNode"
  EXECUTABLE mpc_car)

## Install
install(TARGETS ${PROJECT_NAME} mpc_car_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
