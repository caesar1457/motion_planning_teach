#include "grid_path_searcher/demo_node.hpp"

namespace grid_path_searcher
{
DemoNode::DemoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("grid_path_searcher_demo", options)
{
  // param
  declare_parameter("map.cloud_margin", _cloud_margin);
  declare_parameter("map.resolution", _resolution);
  declare_parameter("map.x_size", _x_size);
  declare_parameter("map.y_size", _y_size);
  declare_parameter("map.z_size", _z_size);
  declare_parameter("planning.x_size", _start_pt(0));
  declare_parameter("planning.y_size", _start_pt(1));
  declare_parameter("planning.z_size", _start_pt(2));
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
  debug_nodes_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("debug_nodes_vis", 1);
  closed_nodes_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("closed_nodes_vis", 1);
  open_nodes_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("open_nodes_vis", 1);
  close_nodes_sequence_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("close_nodes_sequence_vis", 10);
  // sub
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", 1, std::bind(&DemoNode::pointcloud_callBack, this, std::placeholders::_1));
  pts_sub_ = create_subscription<nav_msgs::msg::Path>(
    "waypoints", 1,std::bind(&DemoNode::waypoints_callback, this, std::placeholders::_1));
  //
  _path_finder  = std::make_shared<GridPathFinder>();
  _path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
}

void DemoNode::waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg){
    if( msg->poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Eigen::Vector3d target_pt;
    target_pt << msg->poses[0].pose.position.x,
                 msg->poses[0].pose.position.y,
                 msg->poses[0].pose.position.z;
  RCLCPP_INFO(get_logger(), "[jps_node] receive the way-points");
}

void DemoNode::pointcloud_callBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::msg::PointCloud2 map_vis;

    pcl::fromROSMsg(*(msg.get()), cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt, pt_inf;
    int inf_step   = round(_cloud_margin * _inv_resolution);
    int inf_step_z = std::max(1, inf_step / 2);
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
        {
            for(int y = -inf_step ; y <= inf_step; y ++ )
            {
                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;
                    _path_finder->setObs(inf_x, inf_y, inf_z);

                    Eigen::Vector3d cor_inf = _path_finder->coordRounding(Eigen::Vector3d(inf_x, inf_y, inf_z));

                    pt_inf.x = cor_inf(0);
                    pt_inf.y = cor_inf(1);
                    pt_inf.z = cor_inf(2);
                    cloud_vis.points.push_back(pt_inf);

                }
            }
        }
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    grid_map_vis_pub_->publish(map_vis);

    _has_map = true;
}

} // namespace grid_path_searcher
