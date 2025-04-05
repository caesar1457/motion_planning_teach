
#ifndef DEMO2_NODE_HPP_
#define DEMO2_NODE_HPP_

#include <iostream>
#include <fstream>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "grid_path_searcher/hw_tool.hpp"

namespace grid_path_searcher
{

class Demo2Node : public rclcpp::Node
{
public:
  explicit Demo2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Demo2Node(){};

private:
  void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void pointcloud_callBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void vis_traLibrary(TrajectoryStatePtr *** tralibrary);

  void trajectory_library(const Eigen::Vector3d start_pt, const Eigen::Vector3d start_velocity, const Eigen::Vector3d target_pt);

private:
  // pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_vis_pub_;
  // sub
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pts_sub_;

  // simulation param from launch file
  double _resolution{0.2}, _inv_resolution, _cloud_margin{0.0};
  double _x_size{50.0};
  double _y_size{50.0};
  double _z_size{5.0};    

  bool _has_map{false};

  Eigen::Vector3d _start_pt{0,0,0};
  Eigen::Vector3d _start_velocity{0,0,0};
  Eigen::Vector3d _map_lower;
  Eigen::Vector3d _map_upper;
  int _max_x_id;
  int _max_y_id;
  int _max_z_id;
  // Integral parameter
  double _max_input_acc{1.0};
  int _discretize_step{2};
  double _time_interval{1.25};
  int _time_step{50};
  std::shared_ptr<Homeworktool> homework_tool_;
  TrajectoryStatePtr*** tralibrary_;
  // std::string test_case_{"astar"};
};

} // namespace grid_path_searcher

#endif  // DEMO2_NODE_HPP_
