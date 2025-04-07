
#ifndef WAYPOINT_GENERATOR_HPP_
#define WAYPOINT_GENERATOR_HPP_

#include <iostream>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// #include <eigen3/Eigen/Dense>

namespace waypoint_generator
{

class WaypointGenerator : public rclcpp::Node
{
public:
  explicit WaypointGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WaypointGenerator(){};

private:
  void load_seg(int segid, const rclcpp::Time& time_base);
  void load_waypoints(const rclcpp::Time& time_base);
  void publish_waypoints();
  void publish_waypoints_vis();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_vis_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_sub_;
  //
  std::string waypoint_type_{"manual"};
  bool is_odom_ready_;
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path waypoints_;
  // series waypoint needed
  std::deque<nav_msgs::msg::Path> waypoint_segments_;
  rclcpp::Time trigged_time_;
};

} // namespace waypoint_generator

#endif  // WAYPOINT_GENERATOR_HPP_
