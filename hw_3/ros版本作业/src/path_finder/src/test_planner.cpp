/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "occ_grid/occ_map.h"
#include "path_finder/rrt_star.h"
#include "path_finder/visualization.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TesterPathFinder
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;
    std::shared_ptr<path_plan::RRTStar> rrt_star_ptr_;

    Eigen::Vector3d start_, goal_;
    std::string test_case_{"rrt_star"};

public:
    TesterPathFinder(rclcpp::Node::SharedPtr node) : node_(node)
    {
        //
        node_->declare_parameter("test_case", test_case_);
        node_->get_parameter("test_case", test_case_);

        env_ptr_ = std::make_shared<env::OccMap>();
        env_ptr_->init(node_);

        vis_ptr_ = std::make_shared<visualization::Visualization>(node_);

        rrt_star_ptr_ = std::make_shared<path_plan::RRTStar>(node_, env_ptr_);
        rrt_star_ptr_->setVisualizer(vis_ptr_);

        using namespace std::placeholders;
        goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 1, std::bind(&TesterPathFinder::goalCallback, this, _1));
        using namespace std::literals;
        execution_timer_ = node_->create_wall_timer(1s, std::bind(&TesterPathFinder::executionCallback, this));

        start_.setZero();
    }
    ~TesterPathFinder(){};

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
    {
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        goal_[2] = goal_msg->pose.position.z;
        //RCLCPP_INFO("\n-----------------------------\n goal rcved at " << goal_.transpose());
        vis_ptr_->visualize_a_ball(start_, 0.3, "path_finder/start", visualization::Color::pink);
        vis_ptr_->visualize_a_ball(goal_, 0.3, "path_finder/goal", visualization::Color::steelblue);
        //
        if (test_case_ == "rrt_star") {
            rrt_star_ptr_->enable_informed_sampling(false);
            RCLCPP_INFO(node_->get_logger(), "*********************************");
            RCLCPP_INFO(node_->get_logger(), "RRT* algorithm:");
            find_path();
        } else if(test_case_ == "informed_rrt_star") {
            rrt_star_ptr_->enable_informed_sampling(true);
            RCLCPP_INFO(node_->get_logger(), "*********************************");
            RCLCPP_INFO(node_->get_logger(), "Informed RRT* algorithm:");
            find_path();
        } else if(test_case_ == "comparison") {
            rrt_star_ptr_->enable_informed_sampling(false);
            RCLCPP_INFO(node_->get_logger(), "*********************************");
            RCLCPP_INFO(node_->get_logger(), "RRT* algorithm:");
            find_path();
            rrt_star_ptr_->enable_informed_sampling(true);
            RCLCPP_INFO(node_->get_logger(), "*********************************");
            RCLCPP_INFO(node_->get_logger(), "Informed RRT* algorithm:");
            find_path();
        } else {
            RCLCPP_INFO(node_->get_logger(), "Unknown test case:%s", test_case_.c_str());
        }
        start_ = goal_;
    }

    bool find_path(){
        bool rrt_star_res = rrt_star_ptr_->plan(start_, goal_);
        if (rrt_star_res)
        {
            vector<vector<Eigen::Vector3d>> routes = rrt_star_ptr_->getAllPaths();
            vis_ptr_->visualize_path_list(routes, "path_finder/rrt_star_paths", visualization::blue);
            vector<Eigen::Vector3d> final_path = rrt_star_ptr_->getPath();
            vis_ptr_->visualize_path(final_path, "path_finder/rrt_star_final_path");
            vis_ptr_->visualize_pointcloud(final_path, "path_finder/rrt_star_final_wpts");
            vector<std::pair<double, double>> slns = rrt_star_ptr_->getSolutions();
            RCLCPP_INFO(node_->get_logger(), "find the best path length: %lf, use_time: %lf", slns.back().first, slns.back().second);
            for(size_t i = 0; i < slns.size(); i++){
                RCLCPP_INFO(node_->get_logger(), "solution[%ld]: path length: %lf, use_time: %lf", i, slns[i].first, slns[i].second);
            }
        }
        return rrt_star_res;
    }

    void executionCallback()
    {
        if (!env_ptr_->mapValid())
        {
            RCLCPP_INFO(node_->get_logger(), "no map rcved yet.");     
        }
        else
        {
            execution_timer_->reset();
        }
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_path_finder_node");
    auto tester = std::make_shared<TesterPathFinder>(node);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}