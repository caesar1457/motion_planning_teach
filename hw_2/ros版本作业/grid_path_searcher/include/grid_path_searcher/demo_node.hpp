
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

#include "grid_path_searcher/astar_path_finder.hpp"
#include "grid_path_searcher/jps_path_finder.hpp"

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

  void vis_grid_path(std::vector<Eigen::Vector3d> nodes, bool is_use_jps);
  void vis_visited_node(std::vector<Eigen::Vector3d> nodes );
  void find_path(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt, bool use_jps);

private:
  // pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_path_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visited_nodes_vis_pub_;
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
  // search algo
  // std::shared_ptr<GridPathFinder> _path_finder;
  std::shared_ptr<AstarPathFinder> astar_path_finder_;
  std::shared_ptr<JPSPathFinder> jps_path_finder_;
  bool use_jps_{false};
  std::string test_case_{"astar"};
};

} // namespace grid_path_searcher

#endif  // DEMO_NODE_HPP_
