
#ifndef DEMO_NODE_HPP_
#define DEMO_NODE_HPP_

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

#include "grid_path_searcher/graph_searcher.hpp"

namespace grid_path_searcher
{

class DemoNode : public rclcpp::Node
{
public:
  explicit DemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DemoNode(){};

private:
  void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void pointcloud_callBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  // pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_path_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_nodes_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closed_nodes_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr open_nodes_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr close_nodes_sequence_vis_pub_;
  // sub
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pts_sub_;

  // simulation param from launch file
  double _resolution, _inv_resolution, _cloud_margin;
  double _x_size, _y_size, _z_size;    

  bool _has_map{false};

  Eigen::Vector3d _start_pt;
  Eigen::Vector3d _map_lower;
  Eigen::Vector3d _map_upper;
  int _max_x_id;
  int _max_y_id;
  int _max_z_id;
  std::shared_ptr<GridPathFinder> _path_finder;
};

} // namespace grid_path_searcher

#endif  // DEMO_NODE_HPP_
