#include "waypoint_generator/waypoint_generator.hpp"

#include "waypoint_generator/sample_waypoints.hpp"
#include <Eigen/Eigen>

namespace waypoint_generator
{
WaypointGenerator::WaypointGenerator(const rclcpp::NodeOptions & options)
: rclcpp::Node("waypoint_generator", options)
{
  // param
  declare_parameter("waypoint_type", waypoint_type_);
  declare_parameter("segment_cnt", 0);
  get_parameter("waypoint_type", waypoint_type_);
  // pub
  waypoints_pub_ = create_publisher<nav_msgs::msg::Path>("waypoints", 50);
  waypoints_vis_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
  // sub
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom_", 10, std::bind(&WaypointGenerator::odom_callback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal", 10, std::bind(&WaypointGenerator::goal_callback, this, std::placeholders::_1));
  traj_start_trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "traj_start_trigger", 10, std::bind(&WaypointGenerator::traj_start_trigger_callback, this, std::placeholders::_1));
}

void WaypointGenerator::load_seg(int segid, const rclcpp::Time& time_base){
    RCLCPP_FATAL(get_logger(), "TODO: WaypointGenerator::load_seg");
    std::string seg_str = "seg" + std::to_string(segid) + ".";
    double yaw;
    double time_from_start;
    RCLCPP_INFO(get_logger(), "Getting segment %d", segid);
    // ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
    // ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
    // ROS_ASSERT(nh.getParam(seg_str + "time_from_start", time_from_start));
    // ROS_ASSERT(time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    declare_parameter(seg_str + "x", ptx);
    declare_parameter(seg_str + "y", pty);
    declare_parameter(seg_str + "z", ptz);
    get_parameter(seg_str + "x", ptx);
    get_parameter(seg_str + "y", pty);
    get_parameter(seg_str + "z", ptz);

    assert(ptx.size());
    assert(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::msg::Path path_msg;

    path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);

    double baseyaw = getYaw(odom_.pose.pose.orientation);
    
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::msg::PoseStamped pt;
        pt.pose.orientation = createQuaternionMsgFromYaw(baseyaw + yaw);
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
        pt.pose.position.x = rdp.x() + odom_.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom_.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom_.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }
    waypoint_segments_.push_back(path_msg);
}
void WaypointGenerator::load_waypoints(const rclcpp::Time& time_base){
    int seg_cnt = 0;
    waypoint_segments_.clear();
    get_parameter("segment_cnt", seg_cnt);
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(i, time_base);
        if (i > 0) {
          assert(rclcpp::Time(waypoint_segments_[i - 1].header.stamp)
            < rclcpp::Time(waypoint_segments_[i].header.stamp));
        }
    }
    RCLCPP_INFO(get_logger(), "Overall load %zu segments", waypoint_segments_.size());
}

void WaypointGenerator::publish_waypoints(){
    waypoints_.header.frame_id = std::string("world");
    waypoints_.header.stamp = get_clock()->now();
    waypoints_pub_->publish(waypoints_);
    //
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom_.header;
    init_pose.pose = odom_.pose.pose;
    waypoints_.poses.insert(waypoints_.poses.begin(), init_pose);
    waypoints_.poses.clear();
}

void WaypointGenerator::publish_waypoints_vis(){
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = get_clock()->now();
    {
        geometry_msgs::msg::Pose init_pose;
        init_pose = odom_.pose.pose;
        poseArray.poses.push_back(init_pose);
    }
    for (auto it = waypoints_.poses.begin(); it != waypoints_.poses.end(); ++it) {
        geometry_msgs::msg::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }
    waypoints_vis_pub_->publish(poseArray);
}

void WaypointGenerator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    is_odom_ready_ = true;
    odom_ = *msg;
    if (waypoint_segments_.size()) {
        rclcpp::Time expected_time = waypoint_segments_.front().header.stamp;
        if (rclcpp::Time(odom_.header.stamp) >= expected_time) {
            waypoints_ = waypoint_segments_.front();
            RCLCPP_INFO(get_logger(), "Series send %.3f from start:\n", trigged_time_.seconds());
            for (auto& pose_stamped : waypoints_.poses) {
                RCLCPP_INFO(get_logger(), "P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f) \n",
                    pose_stamped.pose.position.x, pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z, pose_stamped.pose.orientation.w,
                    pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z);
            }
            publish_waypoints_vis();
            publish_waypoints();
            waypoint_segments_.pop_front();
        }
    }
}

void WaypointGenerator::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    trigged_time_ = get_clock()->now(); //odom_.header.stamp;
    if (waypoint_type_ == std::string("circle")) {
        waypoints_ = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type_ == std::string("eight")) {
        waypoints_ = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type_ == std::string("point")) {
        waypoints_ = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type_ == std::string("series")) {
        // load_waypoints(n, trigged_time);
    } else if (waypoint_type_ == std::string("manual-lonely-waypoint")) {
        if (msg->pose.position.z >= 0) {
            // if height >= 0, it's a valid goal;
            geometry_msgs::msg::PoseStamped pt = *msg;
            waypoints_.poses.clear();
            waypoints_.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            RCLCPP_WARN(get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        if (msg->pose.position.z > 0) {
            // if height > 0, it's a normal goal;
            geometry_msgs::msg::PoseStamped pt = *msg;
            if (waypoint_type_ == std::string("noyaw")) {
                double yaw = getYaw(odom_.pose.pose.orientation);
                pt.pose.orientation = createQuaternionMsgFromYaw(yaw);
            }
            waypoints_.poses.push_back(pt);
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            // if 0 > height > -1.0, remove last goal;
            if (waypoints_.poses.size() >= 1) {
                waypoints_.poses.erase(std::prev(waypoints_.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            // if -1.0 > height, end of input
            if (waypoints_.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

void WaypointGenerator::traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    if (!is_odom_ready_) {
        RCLCPP_ERROR(get_logger(), "[waypoint_generator] No odom!");
        return;
    }
    RCLCPP_WARN(get_logger(), "[waypoint_generator] Trigger!");
    trigged_time_ = odom_.header.stamp;
    // assert(trigged_time_ > ros::Time(0));
    RCLCPP_ERROR(get_logger(), "Pattern %s generated!", waypoint_type_.c_str());
    if (waypoint_type_ == std::string("free")) {
        waypoints_ = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type_ == std::string("circle")) {
        waypoints_ = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type_ == std::string("eight")) {
        waypoints_ = eight();
        publish_waypoints_vis();
        publish_waypoints();
   } else if (waypoint_type_ == std::string("point")) {
        waypoints_ = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type_ == std::string("series")) {
        load_waypoints(trigged_time_);
    }
}

} // namespace waypoint_generator
