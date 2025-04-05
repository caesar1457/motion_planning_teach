cmake_minimum_required(VERSION 3.5)
project(occ_grid)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
)

add_library(occ_grid SHARED
    src/occ_map.cpp 
    src/raycast.cpp
)

ament_target_dependencies(occ_grid
  rclcpp
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS occ_grid
    EXPORT occ_grid
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_targets(occ_grid HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
