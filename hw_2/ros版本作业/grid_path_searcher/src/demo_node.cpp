#include "grid_path_searcher/demo_node.hpp"

namespace grid_path_searcher
{
DemoNode::DemoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("grid_path_searcher_demo", options)
{
  // param
  declare_parameter("test_case", test_case_);
  declare_parameter("map.cloud_margin", _cloud_margin);
  declare_parameter("map.resolution", _resolution);
  declare_parameter("map.x_size", _x_size);
  declare_parameter("map.y_size", _y_size);
  declare_parameter("map.z_size", _z_size);
  declare_parameter("planning.x_size", _start_pt(0));
  declare_parameter("planning.y_size", _start_pt(1));
  declare_parameter("planning.z_size", _start_pt(2));
  get_parameter("test_case", test_case_);
  get_parameter("map.cloud_margin", _cloud_margin);
  get_parameter("map.resolution", _resolution);
  get_parameter("map.x_size", _x_size);
  get_parameter("map.y_size", _y_size);
  get_parameter("map.z_size", _z_size);
  get_parameter("planning.x_size", _start_pt(0));
  get_parameter("planning.y_size", _start_pt(1));
  get_parameter("planning.z_size", _start_pt(2));
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
  // pub
  grid_map_vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", 1);
  grid_path_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("grid_path_vis", 1);
  visited_nodes_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("visited_nodes_vis", 1);
  closed_nodes_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("closed_nodes_vis", 1);
  open_nodes_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("open_nodes_vis", 1);
  close_nodes_sequence_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("close_nodes_sequence_vis", 10);
  // sub
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", 1, std::bind(&DemoNode::pointcloud_callBack, this, std::placeholders::_1));
  pts_sub_ = create_subscription<nav_msgs::msg::Path>(
    "waypoints", 1,std::bind(&DemoNode::waypoints_callback, this, std::placeholders::_1));
  //
  astar_path_finder_  = std::make_shared<AstarPathFinder>();
  jps_path_finder_  = std::make_shared<JPSPathFinder>();

  astar_path_finder_->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
  jps_path_finder_->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
}

void DemoNode::waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg){
    if (msg->poses[0].pose.position.z < 0.0 || _has_map == false)
        return;

    Eigen::Vector3d target_pt;
    target_pt << msg->poses[0].pose.position.x,
                 msg->poses[0].pose.position.y,
                 msg->poses[0].pose.position.z;
  RCLCPP_INFO(get_logger(), "receive the way-points");
  if (test_case_ == "astar") {
    RCLCPP_INFO(get_logger(), "A* algorithm:");
    find_path(_start_pt, target_pt, false);
  } else if (test_case_ == "astar_heuristic_function") {
    RCLCPP_INFO(get_logger(), "A* algorithm (Manhattan):");
    astar_path_finder_->setHeuType(HeuType::Manhattan);
    find_path(_start_pt, target_pt, false);
    RCLCPP_INFO(get_logger(), "A* algorithm (Euclidean):");
    astar_path_finder_->setHeuType(HeuType::Euclidean);
    find_path(_start_pt, target_pt, false);
    RCLCPP_INFO(get_logger(), "A* algorithm (Diagonal):");
    astar_path_finder_->setHeuType(HeuType::Diagonal);
    find_path(_start_pt, target_pt, false);
    RCLCPP_INFO(get_logger(), "A* algorithm (None):");
    astar_path_finder_->setHeuType(HeuType::None);
    find_path(_start_pt, target_pt, false);
  } else if (test_case_ == "astar_tie_breaker") {
    RCLCPP_INFO(get_logger(), "A* algorithm (without Tie Breaker):");
    astar_path_finder_->setTieBreaker(false);
    find_path(_start_pt, target_pt, false);
    RCLCPP_INFO(get_logger(), "A* algorithm (with Tie Breaker):");
    astar_path_finder_->setTieBreaker(true);
    find_path(_start_pt, target_pt, false);
    astar_path_finder_->setTieBreaker(false);
  } else if (test_case_ == "astar_jps") {
    RCLCPP_INFO(get_logger(), "A* algorithm:");
    find_path(_start_pt, target_pt, false);
    RCLCPP_INFO(get_logger(), "JPS algorithm:");
    find_path(_start_pt, target_pt, true); 
  }

}

void DemoNode::pointcloud_callBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    if(_has_map) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::msg::PointCloud2 map_vis;

    pcl::fromROSMsg(*(msg.get()), cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        // set obstalces into grid map for path planning
        astar_path_finder_->setObs(pt.x, pt.y, pt.z);
        jps_path_finder_->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        Eigen::Vector3d cor_round = astar_path_finder_->coordRounding(Eigen::Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    grid_map_vis_pub_->publish(map_vis);

    _has_map = true;
}

void DemoNode::vis_grid_path(std::vector<Eigen::Vector3d> nodes, bool is_use_jps)
{
    visualization_msgs::msg::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = get_clock()->now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::msg::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::msg::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::msg::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    grid_path_vis_pub_->publish(node_vis);
}

void DemoNode::vis_visited_node(std::vector<Eigen::Vector3d> nodes)
{
    visualization_msgs::msg::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = get_clock()->now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::msg::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::msg::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::msg::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    visited_nodes_vis_pub_->publish(node_vis);
}

void DemoNode::find_path(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt, bool use_jps)
{
    //Call A* to search for a path
    auto time1 = get_clock()->now();
    if(!use_jps) {
      astar_path_finder_->AstarGraphSearch(start_pt, target_pt);
    } else {
      jps_path_finder_ -> JPSGraphSearch(start_pt, target_pt);
    }
    auto time2 = get_clock()->now();

    // Retrieve the path
    std::vector<Eigen::Vector3d> grid_path;
    std::vector<Eigen::Vector3d> visited_nodes;
    double cost;
    if(!use_jps) {
      grid_path     = astar_path_finder_->getPath();
      visited_nodes = astar_path_finder_->getVisitedNodes();
      cost = astar_path_finder_->getCost();
    } else {
      grid_path     = jps_path_finder_->getPath();
      visited_nodes = jps_path_finder_->getVisitedNodes();
      cost = jps_path_finder_->getCost();
    }
    RCLCPP_INFO(get_logger(), "path nodes size: %ld, visited nodes size: %ld", grid_path.size(), visited_nodes.size());
    RCLCPP_INFO(get_logger(), "time is %f ms, path cost if %f m", (time2 - time1).seconds() * 1000.0, cost); 
    //Visualize the result
    vis_grid_path(grid_path, use_jps);
    vis_visited_node(visited_nodes);

    //Reset map for next call
    if(!use_jps){
      astar_path_finder_->resetUsedGrids();
    }else {
      jps_path_finder_->resetUsedGrids();
    }
}

} // namespace grid_path_searcher
