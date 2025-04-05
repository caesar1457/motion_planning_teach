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
#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <utility>
#include <queue>

#include "rclcpp/rclcpp.hpp"

#include "occ_grid/occ_map.h"
#include "path_finder/visualization.hpp"
#include "path_finder/sampler.h"
#include "path_finder/node.h"
#include "path_finder/kdtree.h"

namespace path_plan
{
  class RRTStar
  {
  public:
    RRTStar(){};
    RRTStar(rclcpp::Node::SharedPtr node, const env::OccMap::Ptr &mapPtr) : node_(node), map_ptr_(mapPtr)
    {
      node_->declare_parameter("RRT_Star.steer_length", steer_length_);
      node_->declare_parameter("RRT_Star.search_radius", search_radius_);
      node_->declare_parameter("RRT_Star.search_time", search_time_);
      node_->declare_parameter("RRT_Star.max_tree_node_nums", max_tree_node_nums_);
      node_->get_parameter("RRT_Star.steer_length", steer_length_);
      node_->get_parameter("RRT_Star.search_radius", search_radius_);
      node_->get_parameter("RRT_Star.search_time", search_time_);
      node_->get_parameter("RRT_Star.max_tree_node_nums", max_tree_node_nums_);
      RCLCPP_INFO(node_->get_logger(), "[RRT*] param: steer_length: %lf", steer_length_);
      RCLCPP_INFO(node_->get_logger(), "[RRT*] param: search_radius: %lf", search_radius_);
      RCLCPP_INFO(node_->get_logger(), "[RRT*] param: search_time: %lf", search_time_);
      RCLCPP_INFO(node_->get_logger(), "[RRT*] param: max_tree_node_nums: %d", max_tree_node_nums_);

      sampler_.setSamplingRange(mapPtr->getOrigin(), mapPtr->getMapSize());

      valid_tree_node_nums_ = 0;
      nodes_pool_.resize(max_tree_node_nums_);
      for (int i = 0; i < max_tree_node_nums_; ++i)
      {
        nodes_pool_[i] = new TreeNode;
      }
    }
    ~RRTStar(){};

    bool plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      reset();
      if (!map_ptr_->isStateValid(s))
      {
        RCLCPP_ERROR(node_->get_logger(), "[RRT*]: Start pos collide or out of bound");
        return false;
      }
      if (!map_ptr_->isStateValid(g))
      {
        RCLCPP_ERROR(node_->get_logger(), "[RRT*]: Goal pos collide or out of bound");
        return false;
      }
      /* construct start and goal nodes */
      start_node_ = nodes_pool_[1];
      start_node_->x = s;
      start_node_->cost_from_start = 0.0;
      goal_node_ = nodes_pool_[0];
      goal_node_->x = g;
      goal_node_->cost_from_start = DBL_MAX; // important
      valid_tree_node_nums_ = 2;             // put start and goal in tree
      RCLCPP_INFO(node_->get_logger(), "[RRT*]: RRT starts planning a path");
      return rrt_star(s, g);
    }

    vector<Eigen::Vector3d> getPath()
    {
      return final_path_;
    }

    vector<vector<Eigen::Vector3d>> getAllPaths()
    {
      return path_list_;
    }

    vector<std::pair<double, double>> getSolutions()
    {
      return solution_cost_time_pair_list_;
    }

    void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr)
    {
      vis_ptr_ = visPtr;
    };

    void enable_informed_sampling(bool value) {enable_informed_sampling_ = value;}

  private:
    // nodehandle params
    rclcpp::Node::SharedPtr node_;

    BiasSampler sampler_;

    // for informed RRT
    InformedSampler informed_sampler_;
    bool enable_informed_sampling_{false};

    double steer_length_{0};
    double search_radius_{0};
    double search_time_{0};
    int max_tree_node_nums_{0};
    int valid_tree_node_nums_;
    double first_path_use_time_;
    double final_path_use_time_;

    std::vector<TreeNode *> nodes_pool_;
    TreeNode *start_node_;
    TreeNode *goal_node_;
    vector<Eigen::Vector3d> final_path_;
    vector<vector<Eigen::Vector3d>> path_list_;
    vector<std::pair<double, double>> solution_cost_time_pair_list_;

    // environment
    env::OccMap::Ptr map_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;

    void reset()
    {
      final_path_.clear();
      path_list_.clear();
      solution_cost_time_pair_list_.clear();
      for (int i = 0; i < valid_tree_node_nums_; i++)
      {
        nodes_pool_[i]->parent = nullptr;
        nodes_pool_[i]->children.clear();
      }
      valid_tree_node_nums_ = 0;
    }

    double calDist(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
      return (p1 - p2).norm();
    }

    Eigen::Vector3d steer(const Eigen::Vector3d &nearest_node_p, const Eigen::Vector3d &rand_node_p, double len)
    {
      Eigen::Vector3d diff_vec = rand_node_p - nearest_node_p;
      double dist = diff_vec.norm();
      if (diff_vec.norm() <= len)
        return rand_node_p;
      else
        return nearest_node_p + diff_vec * len / dist;
    }

    RRTNode3DPtr addTreeNode(RRTNode3DPtr &parent, const Eigen::Vector3d &state,
                             const double &cost_from_start, const double &cost_from_parent)
    {
      RRTNode3DPtr new_node_ptr = nodes_pool_[valid_tree_node_nums_];
      valid_tree_node_nums_++;
      new_node_ptr->parent = parent;
      parent->children.push_back(new_node_ptr);
      new_node_ptr->x = state;
      new_node_ptr->cost_from_start = cost_from_start;
      new_node_ptr->cost_from_parent = cost_from_parent;
      return new_node_ptr;
    }

    void changeNodeParent(RRTNode3DPtr &node, RRTNode3DPtr &parent, const double &cost_from_parent)
    {
      if (node->parent)
        node->parent->children.remove(node); //DON'T FORGET THIS, remove it form its parent's children list
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);

      // for all its descedants, change the cost_from_start and tau_from_start;
      RRTNode3DPtr descendant(node);
      std::queue<RRTNode3DPtr> Q;
      Q.push(descendant);
      while (!Q.empty())
      {
        descendant = Q.front();
        Q.pop();
        for (const auto &leafptr : descendant->children)
        {
          leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
          Q.push(leafptr);
        }
      }
    }

    void fillPath(const RRTNode3DPtr &n, vector<Eigen::Vector3d> &path)
    {
      path.clear();
      RRTNode3DPtr node_ptr = n;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(start_node_->x);
      std::reverse(std::begin(path), std::end(path));
    }

    bool rrt_star(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      rclcpp::Time rrt_start_time = node_->get_clock()->now();
      bool goal_found = false;
      bool better_path_found = false;
      /* kd tree init */
      kdtree *kd_tree = kd_create(3);
      //Add start and goal nodes to kd tree
      kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
      if (enable_informed_sampling_) {
        informed_sampler_.setFocus(s, g);
      }
      /* main loop */
      int idx = 0;
      for (idx = 0; (node_->get_clock()->now() - rrt_start_time).seconds() < search_time_ && valid_tree_node_nums_ < max_tree_node_nums_; ++idx)
      {
        better_path_found = false;
        /* biased random sampling */
        Eigen::Vector3d x_rand;
        if (enable_informed_sampling_ && goal_found) {
          informed_sampler_.samplingOnce(x_rand);
        } else {
          sampler_.samplingOnce(x_rand);
        }
        // samplingOnce(x_rand);
        if (!map_ptr_->isStateValid(x_rand))
        {
          continue;
        }

        struct kdres *p_nearest = kd_nearest3(kd_tree, x_rand[0], x_rand[1], x_rand[2]);
        if (p_nearest == nullptr)
        {
          RCLCPP_ERROR(node_->get_logger(), "nearest query error");
          continue;
        }
        RRTNode3DPtr nearest_node = (RRTNode3DPtr)kd_res_item_data(p_nearest);
        kd_res_free(p_nearest);

        Eigen::Vector3d x_new = steer(nearest_node->x, x_rand, steer_length_);
        if (!map_ptr_->isSegmentValid(nearest_node->x, x_new))
        {
          continue;
        }

        /* 1. find parent */
        /* kd_tree bounds search for parent */
        vector<RRTNode3DPtr> neighbour_nodes;
        struct kdres *nbr_set;
        nbr_set = kd_nearest_range3(kd_tree, x_new[0], x_new[1], x_new[2], search_radius_);
        if (nbr_set == nullptr)
        {
          RCLCPP_ERROR(node_->get_logger(), "bkwd kd range query error");
          break;
        }
        while (!kd_res_end(nbr_set))
        {
          RRTNode3DPtr curr_node = (RRTNode3DPtr)kd_res_item_data(nbr_set);
          neighbour_nodes.emplace_back(curr_node);
          // store range query result so that we dont need to query again for rewire;
          kd_res_next(nbr_set); //go to next in kd tree range query result
        }
        kd_res_free(nbr_set); //reset kd tree range query

        /* choose parent from kd tree range query result*/
        double dist2nearest = calDist(nearest_node->x, x_new);
        double min_dist_from_start = nearest_node->cost_from_start + dist2nearest;
        double cost_from_p = dist2nearest;
        RRTNode3DPtr min_node = nearest_node; //set the nearest_node as the default parent

        // TODO Choose a parent according to potential cost-from-start values
        // ! Hints:
        // !  1. Use map_ptr_->isSegmentValid(p1, p2) to check line edge validity;
        // !  2. Default parent is [nearest_node];
        // !  3. Store your chosen parent-node-pointer, the according cost-from-parent and cost-from-start
        // !     in [min_node], [cost_from_p], and [min_dist_from_start], respectively;
        // !  4. [Optional] You can sort the potential parents first in increasing order by cost-from-start value;
        // !  5. [Optional] You can store the collison-checking results for later usage in the Rewire procedure.
        // ! Implement your own code inside the following loop
        for (auto &curr_node : neighbour_nodes)
        {
          if (!map_ptr_->isSegmentValid(curr_node->x, x_new))
          {
            continue;
          }
          double cur_dist2nearest = calDist(curr_node->x, x_new);
          double cur_min_dist_from_start = curr_node->cost_from_start + cur_dist2nearest;
          if (cur_min_dist_from_start < min_dist_from_start) {
            min_dist_from_start = cur_min_dist_from_start;
            cost_from_p = cur_dist2nearest;
            min_node = curr_node;
          }
        }
        // ! Implement your own code inside the above loop

        /* parent found within radius, then add a node to rrt and kd_tree */
        /* 1.1 add the randomly sampled node to rrt_tree */
        RRTNode3DPtr new_node(nullptr);
        new_node = addTreeNode(min_node, x_new, min_dist_from_start, cost_from_p);

        /* 1.2 add the randomly sampled node to kd_tree */
        kd_insert3(kd_tree, x_new[0], x_new[1], x_new[2], new_node);
        // end of find parent

        /* 2. try to connect to goal if possible */
        double dist_to_goal = calDist(x_new, goal_node_->x);
        if (dist_to_goal <= search_radius_)
        {
          bool is_connected2goal = map_ptr_->isSegmentValid(x_new, goal_node_->x);
          // this test can be omitted if sample-rejction is applied
          bool is_better_path = goal_node_->cost_from_start > dist_to_goal + new_node->cost_from_start;
          if (is_connected2goal && is_better_path)
          {
            if (!goal_found)
            {
              first_path_use_time_ = (node_->get_clock()->now() - rrt_start_time).seconds();
            }
            goal_found = true;
            changeNodeParent(goal_node_, new_node, dist_to_goal);
            vector<Eigen::Vector3d> curr_best_path;
            fillPath(goal_node_, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, (node_->get_clock()->now() - rrt_start_time).seconds());
            better_path_found = true;
          }
        }

        /* 3.rewire */
        // TODO Rewire according to potential cost-from-start values
        // ! Hints:
        // !  1. Use map_ptr_->isSegmentValid(p1, p2) to check line edge validity;
        // !  2. Use changeNodeParent(node, parent, cost_from_parent) to change a node's parent;
        // !  3. the variable [new_node] is the pointer of X_new;
        // !  4. [Optional] You can test whether the node is promising before checking edge collison.
        // ! Implement your own code between the dash lines [--------------] in the following loop
        for (auto &curr_node : neighbour_nodes)
        {
          double best_cost_before_rewire = goal_node_->cost_from_start;
          // ! -------------------------------------
          if (!map_ptr_->isSegmentValid(curr_node->x, new_node->x))
          {
            continue;
          };
          double new_cost_from_parent = calDist(curr_node->x, new_node->x);
          if(new_node->cost_from_start + new_cost_from_parent < curr_node->cost_from_start) {
            changeNodeParent(curr_node, new_node, new_cost_from_parent);
          }
          // ! -------------------------------------
          if (best_cost_before_rewire > goal_node_->cost_from_start)
          {
            vector<Eigen::Vector3d> curr_best_path;
            fillPath(goal_node_, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, (node_->get_clock()->now() - rrt_start_time).seconds());
            better_path_found = true;
          }
        }
        /* end of rewire */
        if (enable_informed_sampling_ && better_path_found) {
          // update informed sampler
          informed_sampler_.setCost(goal_node_->cost_from_start);
        }
      }
      /* end of sample once */

      vector<Eigen::Vector3d> vertice;
      vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
      sampleWholeTree(start_node_, vertice, edges);
      std::vector<visualization::BALL> balls;
      balls.reserve(vertice.size());
      visualization::BALL node_p;
      node_p.radius = 0.06;
      for (size_t i = 0; i < vertice.size(); ++i)
      {
        node_p.center = vertice[i];
        balls.push_back(node_p);
      }
      vis_ptr_->visualize_balls(balls, "path_finder/tree_vertice", visualization::Color::blue, 1.0);
      vis_ptr_->visualize_pairline(edges, "path_finder/tree_edges", visualization::Color::red, 0.04);

      if (goal_found)
      {
        final_path_use_time_ = (node_->get_clock()->now() - rrt_start_time).seconds();
        fillPath(goal_node_, final_path_);
      }
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        RCLCPP_ERROR(node_->get_logger(), "[RRT*]: NOT CONNECTED TO GOAL after %d nodes added to rrt-tree", max_tree_node_nums_);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "[RRT*]: NOT CONNECTED TO GOAL after %lf seconds", (node_->get_clock()->now() - rrt_start_time).seconds());
      }
      return goal_found;
    }

    void sampleWholeTree(const RRTNode3DPtr &root, vector<Eigen::Vector3d> &vertice, vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges)
    {
      if (root == nullptr)
        return;

      // whatever dfs or bfs
      RRTNode3DPtr node = root;
      std::queue<RRTNode3DPtr> Q;
      Q.push(node);
      while (!Q.empty())
      {
        node = Q.front();
        Q.pop();
        for (const auto &leafptr : node->children)
        {
          vertice.push_back(leafptr->x);
          edges.emplace_back(std::make_pair(node->x, leafptr->x));
          Q.push(leafptr);
        }
      }
    }
  };

} // namespace path_plan
#endif