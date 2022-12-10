/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef HYBRID_ASTAR_PLANNER_H_
#define HYBRID_ASTAR_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <algorithm>
#include <chrono>

//动态维诺图头文件
#include "dynamicvoronoi.h"

//节点定义头文件
#include "node2d.h"
#include "node3d.h"

//算法实现头文件
#include "algorithm.h"

//路径平滑头文件
#include "smoother.h"

//可视化路径头文件
#include "visualize_path.h"

//计时器
#include "timer.h"
namespace hybrid_astar_planner
{
  /**
   * @class Hybrid_astarPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class Hybrid_astarPlanner : public nav_core::BaseGlobalPlanner
  {
  public:
    /**
     * @brief  Constructor for the Hybrid_astarPlanner
     */
    Hybrid_astarPlanner();
    /**
     * @brief  Constructor for the Hybrid_astarPlanner
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    Hybrid_astarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     * @brief  Initialization function for the Hybrid_astarPlanner
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) override;

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan) override;

  private:
    costmap_2d::Costmap2DROS *costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D *costmap_;
    base_local_planner::WorldModel *world_model_; /// ros自带的世界模型，用于检测footprint在costmap中的碰撞情况

    bool initialized_;
    DynamicVoronoi voronoi_diagram; //维诺图，用于检测障碍物碰撞
    int costmap_width;
    int costmap_height;

    std::vector<Node3D> final_path; //最后输出的路径
    ros::Publisher vis_plan_pub;    //可视化路径publisher

    Smoother smoother;                   //路径平滑类对象
    Path initial_path_vis;               //初始路径可视化类对象
    Path smoothed_path_vis = Path(true); //平滑路径可视化类对象

  private:
    void recursive_path(const Node3D *node, std::vector<Node3D> &path);
    bool transfer_to_ros_path(std::vector<Node3D> &native_path, std::vector<geometry_msgs::PoseStamped> &ros_pat);
    // void visualize_plan(const std::vector<geometry_msgs::PoseStamped> &path);
  };
};
#endif
