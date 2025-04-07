#include "grid_path_searcher/demo_node.hpp"

namespace grid_path_searcher
{
DemoNode::DemoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("demo_node", options)
{
  // param
  declare_parameter("map.cloud_margin", _cloud_margin);
  declare_parameter("map.resolution", _resolution);
  declare_parameter("map.x_size", _x_size);
  declare_parameter("map.y_size", _y_size);
  declare_parameter("map.z_size", _z_size);
  declare_parameter("planning.start_x", _start_pt(0));
  declare_parameter("planning.start_y", _start_pt(1));
  declare_parameter("planning.start_z", _start_pt(2));
  declare_parameter("planning.start_vx", _start_velocity(0));
  declare_parameter("planning.start_vy", _start_velocity(1));
  declare_parameter("planning.start_vz", _start_velocity(2));
  get_parameter("map.cloud_margin", _cloud_margin);
  get_parameter("map.resolution", _resolution);
  get_parameter("map.x_size", _x_size);
  get_parameter("map.y_size", _y_size);
  get_parameter("map.z_size", _z_size);
  get_parameter("planning.start_x", _start_pt(0));
  get_parameter("planning.start_y", _start_pt(1));
  get_parameter("planning.start_z", _start_pt(2));
  get_parameter("planning.start_vx", _start_velocity(0));
  get_parameter("planning.start_vy", _start_velocity(1));
  get_parameter("planning.start_vz", _start_velocity(2));
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
  // pub
  grid_map_vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", 1);
  path_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("path_vis", 1);
  // sub
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", 1, std::bind(&DemoNode::pointcloud_callBack, this, std::placeholders::_1));
  pts_sub_ = create_subscription<nav_msgs::msg::Path>(
    "waypoints", 1,std::bind(&DemoNode::waypoints_callback, this, std::placeholders::_1));
  // initialize tralibrary_, recored all trajectories after input
  tralibrary_  = new TrajectoryStatePtr ** [_discretize_step + 1]; 
  for(int i=0; i <= _discretize_step; i++){ 
    tralibrary_[i] = new TrajectoryStatePtr * [_discretize_step + 1];
    for(int j=0;j <= _discretize_step; j++){
      tralibrary_[i][j] = new TrajectoryStatePtr [_discretize_step + 1];
      for(int k=0; k <= _discretize_step; k++){
        tralibrary_[i][j][k] = new TrajectoryState();
      }
    }
  }
  //
  homework_tool_  = std::make_shared<Homeworktool>();
  homework_tool_  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
}

void DemoNode::waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg){
    if (msg->poses[0].pose.position.z < 0.0 || _has_map == false)
        return;

    Eigen::Vector3d target_pt;
    target_pt << msg->poses[0].pose.position.x,
                 msg->poses[0].pose.position.y,
                 msg->poses[0].pose.position.z;
  RCLCPP_INFO(get_logger(), "receive the way-points");
  trajectory_library(_start_pt, _start_velocity, target_pt);
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
        homework_tool_->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        Eigen::Vector3d cor_round = homework_tool_->coordRounding(Eigen::Vector3d(pt.x, pt.y, pt.z));
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

void DemoNode::vis_traLibrary(TrajectoryStatePtr *** tralibrary)
{
    double _resolution = 0.2;
    visualization_msgs::msg::MarkerArray  LineArray;
    visualization_msgs::msg::Marker       Line;

    Line.header.frame_id = "world";
    Line.header.stamp    = get_clock()->now();
    Line.ns              = "demo_node/tralibrary";
    Line.action          = visualization_msgs::msg::Marker::ADD;
    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    Line.scale.x         = _resolution/5;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 1.0;

    int marker_id = 0;

    for(int i = 0; i <= _discretize_step; i++){
        for(int j = 0; j<= _discretize_step;j++){  
            for(int k = 0; k<= _discretize_step;k++){
                if(tralibrary[i][j][k]->collision_check == false){
                    if(tralibrary[i][j][k]->optimal_flag == true){
                        Line.color.r         = 0.0;
                        Line.color.g         = 1.0;
                        Line.color.b         = 0.0;
                        Line.color.a         = 1.0;
                    }else{
                        Line.color.r         = 0.0;
                        Line.color.g         = 0.0;
                        Line.color.b         = 1.0;
                        Line.color.a         = 1.0;
                    }
                }else{
                    Line.color.r         = 1.0;
                    Line.color.g         = 0.0;
                    Line.color.b         = 0.0;
                    Line.color.a         = 1.0;
                }
                   Line.points.clear();
                    geometry_msgs::msg::Point pt;
                    Line.id = marker_id;
                    for(int index = 0; index < int(tralibrary[i][j][k]->Position.size());index++){
                        auto coord = tralibrary[i][j][k]->Position[index];
                        pt.x = coord(0);
                        pt.y = coord(1);
                        pt.z = coord(2);
                        Line.points.push_back(pt);
                    }
                    LineArray.markers.push_back(Line);
                    path_vis_pub_->publish(LineArray);
                    ++marker_id; 
            }
        }
    }
    
}

void DemoNode::trajectory_library(const Eigen::Vector3d start_pt, const Eigen::Vector3d start_velocity, const Eigen::Vector3d target_pt)
{
    Eigen::Vector3d acc_input;
    Eigen::Vector3d pos,vel;
    int a =0 ;
    int b =0 ;
    int c =0 ;

    double min_cost = 100000.0;
    double trajctory_cost;

    for(int i=0; i <= _discretize_step; i++){           //acc_input_ax
        for(int j=0;j <= _discretize_step; j++){        //acc_input_ay
            for(int k=0; k <= _discretize_step; k++){   //acc_input_az
                std::vector<Eigen::Vector3d> Position;
                std::vector<Eigen::Vector3d> Velocity;
                acc_input(0) = double(-_max_input_acc + i * (2 * _max_input_acc / double(_discretize_step)) );
                acc_input(1) = double(-_max_input_acc + j * (2 * _max_input_acc / double(_discretize_step)) );
                acc_input(2) = double( k * (2 * _max_input_acc / double(_discretize_step) ) + 0.1);                          //acc_input_az >0.1
                
                pos(0) = start_pt(0);
                pos(1) = start_pt(1);
                pos(2) = start_pt(2);
                vel(0) = start_velocity(0);
                vel(1) = start_velocity(1);
                vel(2) = start_velocity(2);
                Position.push_back(pos);
                Velocity.push_back(vel);

                bool collision = false;
                double delta_time;
                delta_time = _time_interval / double(_time_step);
                
                for(int step=0 ; step<=_time_step ; step ++){

                    /*
                    



                    STEP 1: finish the forward integration, the modelling has been given in the document
                    the parameter of forward integration: _max_input_acc|_discretize_step|_time_interval|_time_step   all have been given
                    use the pos and vel to recored the steps in the trakectory



                    */
                    pos = pos + vel * delta_time + 0.5 * acc_input * delta_time * delta_time;
                    vel = vel + acc_input * delta_time;
                    Position.push_back(pos);
                    Velocity.push_back(vel);
                    double coord_x = pos(0);
                    double coord_y = pos(1);
                    double coord_z = pos(2);
                    //check if if the trajectory face the obstacle
                    if(homework_tool_->isObsFree(coord_x,coord_y,coord_z) != 1){
                        collision = true;
                    }
                }
                /*
                    



                    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
                    the solving process has been given in the document

                    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP

                    after finish Homeworktool::OptimalBVP, the trajctory_cost will record the optimal cost of this trajectory


                */
                trajctory_cost = homework_tool_ -> OptimalBVP(pos,vel,target_pt);

                //input the trajetory in the trajectory library
                tralibrary_[i][j][k]->init(Position,Velocity,trajctory_cost);
                
                //if there is not any obstacle in the trajectory we need to set 'collision_check = true', so this trajectory is useable
                if(collision)
                    tralibrary_[i][j][k]->setCollisionfree();
                
                //record the min_cost in the trajectory Library, and this is the part pf selecting the best trajectory cloest to the planning traget
                if(trajctory_cost<min_cost && tralibrary_[i][j][k]->collision_check == false){
                    a = i;
                    b = j;
                    c = k;
                    min_cost = trajctory_cost;
                }
            }
        }
    }
    tralibrary_[a][b][c] -> setOptimal();
    vis_traLibrary(tralibrary_);
}

} // namespace grid_path_searcher
