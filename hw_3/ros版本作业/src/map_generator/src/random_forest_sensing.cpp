#include <iostream>
#include <random>
#include <cmath>

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace Eigen;

int _obs_num{30}, _cir_num{30};
double _x_size{50}, _y_size{50}, _z_size{5}, _init_x{0}, _init_y{0}, _resolution{0.2}, _sense_rate{1.0};
double _x_l, _x_h, _y_l, _y_h, _w_l{0.3}, _w_h{0.8}, _h_l{3.0}, _h_h{7.0}, _w_c_l{0.3}, _w_c_h{0.8};

bool _has_map{false};

sensor_msgs::msg::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;      

void RandomMapGenerate()
{  
   random_device rd;
   default_random_engine eng(rd());

   uniform_real_distribution<double> rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

   uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h);
   uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h);
   uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
   uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

   uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(_x_l + 1.0, _x_h - 1.0);
   uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(_y_l + 1.0, _y_h - 1.0);
   uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(_w_c_l, _w_c_h);

   uniform_real_distribution<double> rand_roll = uniform_real_distribution<double>(-M_PI, +M_PI);
   uniform_real_distribution<double> rand_pitch = uniform_real_distribution<double>(+M_PI / 4.0, +M_PI / 2.0);
   uniform_real_distribution<double> rand_yaw = uniform_real_distribution<double>(+M_PI / 4.0, +M_PI / 2.0);
   uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
   uniform_real_distribution<double> rand_num = uniform_real_distribution<double>(0.0, 1.0);

   pcl::PointXYZ pt_random;

   int base2(2), base3(3), base4(4); //Halton base
   // firstly, we put some circles
   for (int i = 0; i < _cir_num; i++)
   {
      double x0, y0, z0, R;
      std::vector<Vector3d> circle_set;

      // x0 = rand_x_circle(eng);
      // y0 = rand_y_circle(eng);
      z0 = rand_h(eng);

      //Halton sequence for x(0, 1)
      double f = 1;
      x0 = 0;
      int ii = i;
      while (ii > 0)
      {
         f = f / base2;
         x0 = x0 + f * (ii % base2);
         ii = floor(ii / base2);
      }
      x0 *= _x_size;
      x0 -= _x_size / 2;

      //Halton sequence for y(0, 1)
      f = 1;
      y0 = 0;
      ii = i;
      while (ii > 0)
      {
         f = f / base3;
         y0 = y0 + f * (ii % base3);
         ii = floor(ii / base3);
      }
      y0 *= _y_size;
      y0 -= _y_size / 2;

      R = rand_r_circle(eng);

      if (sqrt(pow(x0 - _init_x, 2) + pow(y0 - _init_y, 2)) < 1.5)
         continue;

      double a, b;
      a = rand_ellipse_c(eng);
      b = rand_ellipse_c(eng);

      double x, y, z;
      Vector3d pt3, pt3_rot;
      for (double theta = -M_PI; theta < M_PI; theta += 0.025)
      {
         x = a * cos(theta) * R;
         y = b * sin(theta) * R;
         z = 0;
         pt3 << x, y, z;
         circle_set.push_back(pt3);
      }
      // Define a random 3d rotation matrix
      Matrix3d Rot;
      double roll, pitch, yaw;
      double alpha, beta, gama;
      roll = rand_roll(eng);   // alpha
      pitch = rand_pitch(eng); // beta
      yaw = rand_yaw(eng);     // gama

      alpha = roll;
      beta = pitch;
      gama = yaw;

      double p = rand_num(eng);
      if (p < 0.5)
      {
         beta = M_PI / 2.0;
         gama = M_PI / 2.0;
      }

      Rot << cos(alpha) * cos(gama) - cos(beta) * sin(alpha) * sin(gama), -cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama), sin(alpha) * sin(beta),
          cos(gama) * sin(alpha) + cos(alpha) * cos(beta) * sin(gama), cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), -cos(alpha) * sin(beta),
          sin(beta) * sin(gama), cos(gama) * sin(beta), cos(beta);

      for (auto pt : circle_set)
      {
         pt3_rot = Rot * pt;
         pt_random.x = pt3_rot(0) + x0 + 0.001;
         pt_random.y = pt3_rot(1) + y0 + 0.001;
         pt_random.z = pt3_rot(2) + z0 + 0.001 - 1;

         if (pt_random.z >= 0.0)
            cloudMap.points.push_back(pt_random);
      }
   }

   bool is_kdtree_empty = false;
   if (cloudMap.points.size() > 0)
      kdtreeMap.setInputCloud(cloudMap.makeShared());
   else
      is_kdtree_empty = true;

   // then, we put some pilar
   for (int i = 0; i < _obs_num; i++)
   {
      double x, y, w, h;
      // x    = rand_x(eng);
      // y    = rand_y(eng);
      w = rand_w(eng);

      //Halton sequence for x(0, 1)
      double f = 1;
      x = 0;
      int ii = i;
      while (ii > 0)
      {
         f = f / base2;
         x = x + f * (ii % base2);
         ii = floor(ii / base2);
      }
      x *= _x_size;
      x -= _x_size / 2;

      //Halton sequence for y(0, 1)
      f = 1;
      y = 0;
      ii = i;
      while (ii > 0)
      {
         f = f / base3;
         y = y + f * (ii % base3);
         ii = floor(ii / base3);
      }
      y *= _y_size;
      y -= _y_size / 2;

      double d_theta = rand_theta(eng);

      if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0)
         continue;

      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h) / 2.0);
      pointIdxSearch.clear();
      pointSquaredDistance.clear();

      if (is_kdtree_empty == false)
      {
         if (kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0)
         {
            if (sqrt(pointSquaredDistance[0]) < 1.0)
               continue;
         }
      }

      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w / _resolution);
      int halfWidNum = widNum / 2.0;
      for (int r = -halfWidNum; r < halfWidNum; r++)
      {
         for (int s = -halfWidNum; s < halfWidNum; s++)
         {
            // make pilars hollow
            if (r > -halfWidNum + 2 && r < (halfWidNum - 3))
            {
               if (s > -halfWidNum + 2 && s < (halfWidNum - 3))
               {
                  continue;
               }
            }
            // rotate
            double th = atan2((double)s, (double)r);
            int len = sqrt(s * s + r * r);
            th += d_theta;
            int rr = cos(th) * len;
            int ss = sin(th) * len;

            h = rand_h(eng);
            int heiNum = 2.0 * ceil(h / _resolution);
            for (int t = 0; t < heiNum; t++)
            {
               pt_random.x = x + (rr + 0.0) * _resolution + 0.001;
               pt_random.y = y + (ss + 0.0) * _resolution + 0.001;
               pt_random.z = (t + 0.0) * _resolution * 0.5 - 1.0 + 0.001;
               cloudMap.points.push_back(pt_random);
            }
         }
      }
   }

   cloudMap.width = cloudMap.points.size();
   cloudMap.height = 1;
   cloudMap.is_dense = true;

   _has_map = true;

   pcl::toROSMsg(cloudMap, globalMap_pcd);
   globalMap_pcd.header.frame_id = "map";
}


int main (int argc, char** argv) 
{        
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("random_complex_scene");
  // param
  node->declare_parameter("init_state_x", _init_x);
  node->declare_parameter("init_state_y", _init_y);
  node->declare_parameter("map.x_size", _x_size);
  node->declare_parameter("map.y_size", _y_size);
  node->declare_parameter("map.z_size", _z_size);
  node->declare_parameter("map.obs_num", _obs_num);
  node->declare_parameter("map.circle_num", _cir_num);
  node->declare_parameter("map.resolution", _resolution);
  node->declare_parameter("ObstacleShape.lower_rad", _w_l);
  node->declare_parameter("ObstacleShape.upper_rad", _w_h);
  node->declare_parameter("ObstacleShape.lower_hei", _h_l);
  node->declare_parameter("ObstacleShape.upper_hei", _h_h);
  node->declare_parameter("CircleShape.lower_circle_rad", _w_c_l);
  node->declare_parameter("CircleShape.upper_circle_rad", _w_c_h);
  node->declare_parameter("sensing.rate", _sense_rate);
  node->get_parameter("init_state_x", _init_x);
  node->get_parameter("init_state_y", _init_y);
  node->get_parameter("map.x_size", _x_size);
  node->get_parameter("map.y_size", _y_size);
  node->get_parameter("map.z_size", _z_size);
  node->get_parameter("map.obs_num", _obs_num);
  node->get_parameter("map.circle_num", _cir_num);
  node->get_parameter("map.resolution", _resolution);
  node->get_parameter("ObstacleShape.lower_rad", _w_l);
  node->get_parameter("ObstacleShape.upper_rad", _w_h);
  node->get_parameter("ObstacleShape.lower_hei", _h_l);
  node->get_parameter("ObstacleShape.upper_hei", _h_h);
  node->get_parameter("CircleShape.lower_circle_rad", _w_c_l);
  node->get_parameter("CircleShape.upper_circle_rad", _w_c_h);
  node->get_parameter("sensing.rate", _sense_rate);
   _x_l = - _x_size / 2.0;
   _x_h = + _x_size / 2.0;

   _y_l = - _y_size / 2.0;
   _y_h = + _y_size / 2.0;
  // pub
  auto _all_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);
  //
  RandomMapGenerate();
  // 
  rclcpp::Rate loop_rate(_sense_rate);
  while (rclcpp::ok())
  {
    if(_has_map ) {
      _all_map_pub->publish(globalMap_pcd);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
}