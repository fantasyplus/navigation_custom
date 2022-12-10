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
 * Authors: Eitan Marder-Eppstein, Sachin Chitta
 *********************************************************************/
#include <hybrid_astar_planner/hybrid_astar_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::Hybrid_astarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner
{

  Hybrid_astarPlanner::Hybrid_astarPlanner()
      : costmap_ros_(NULL), initialized_(false) {}

  Hybrid_astarPlanner::Hybrid_astarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
      : costmap_ros_(NULL), initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  void Hybrid_astarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      //获得costmap网格宽度和高度
      costmap_width = costmap_->getSizeInCellsX();
      costmap_height = costmap_->getSizeInCellsY();

      ROS_INFO("hybrid_astar_planner.cpp: costmap_width:%d", costmap_width);
      ROS_INFO("hybrid_astar_planner.cpp: costmap_height:%d", costmap_height);

      //将costmap转化为二值地图传入维诺图中
      bool **bin_map;
      bin_map = new bool *[costmap_width];
      for (int x = 0; x < costmap_width; x++)
      {
        bin_map[x] = new bool[costmap_height];
      }

      for (int x = 0; x < costmap_width; x++)
      {
        for (int y = 0; y < costmap_height; y++)
        {
          bin_map[x][y] = costmap_->getCharMap()[y * costmap_width + x] ? true : false;
        }
      }

      voronoi_diagram.initializeMap(costmap_width, costmap_height, bin_map);
      voronoi_diagram.update();

      ros::NodeHandle private_nh("~/" + name);
      vis_plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); //用于检测在costmap中的碰撞

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool Hybrid_astarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {

    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_INFO("Got a start: %f, %f, and a goal: %f, %f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    final_path.clear();
    initial_path_vis.clear();
    smoothed_path_vis.clear();
    costmap_ = costmap_ros_->getCostmap();

    //判断目标和costmap是不是在同一个frame下
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }
    else
    {
      ROS_INFO("This planner's goals are in the %s frame, and costmap is in the %s frame.",
               costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    }

    Node2D *nodes_2d = new Node2D[costmap_width * costmap_height]();
    Node3D *nodes_3d = new Node3D[costmap_width * costmap_height * Constants::headings]();

    //定义Node3D形式的开始和目标节点
    float x_start = start.pose.position.x / Constants::cellSize;
    float y_start = start.pose.position.y / Constants::cellSize;
    float theta_start = tf2::getYaw(start.pose.orientation);
    theta_start = tf2NormalizeAngle(theta_start);
    Node3D node_start(x_start, y_start, theta_start, 0, 0, nullptr);

    float x_goal = goal.pose.position.x / Constants::cellSize;
    float y_goal = goal.pose.position.y / Constants::cellSize;
    float theta_goal = tf2::getYaw(goal.pose.orientation);
    theta_goal = tf2NormalizeAngle(theta_goal);
    Node3D node_goal(x_goal, y_goal, theta_goal, 0, 0, nullptr);

    TimerClock t0;

    Algorithm planner;
    Node3D *final_node = planner.hybridAStar(node_start, node_goal, nodes_3d, nodes_2d, costmap_width, costmap_height, costmap_ros_, world_model_);

    ROS_INFO("-----------hybrid astar time in %f ms-----------",t0.getTimerMilliSec());

    TimerClock t1;
    //从最后一个节点获取最终路径
    recursive_path(final_node, final_path);
    initial_path_vis.updatePath(final_path);

    //平滑路径
    smoother.smoothPath(voronoi_diagram, final_path);
    smoothed_path_vis.updatePath(final_path);

    if (transfer_to_ros_path(final_path, plan))
    {
      ROS_INFO("-----------get plan,plan size:%d", (int)plan.size());
      ROS_INFO("-----------visualize path-----------");
      
      //可视化未平滑的路径
      initial_path_vis.publishPath();
      initial_path_vis.publishPathNodes();
      initial_path_vis.publishPathVehicles();
      //可视化平滑后的路径
      smoothed_path_vis.publishPath();
      smoothed_path_vis.publishPathNodes();
      smoothed_path_vis.publishPathVehicles();


      ROS_INFO("-----------post process and visual time in %f ms-----------",t1.getTimerMilliSec());
      return true;
    }

    return false;
  }

  void Hybrid_astarPlanner::recursive_path(const Node3D *node, std::vector<Node3D> &path)
  {
    if (node == nullptr)
    {
      std::reverse(path.begin(), path.end());
      return;
    }

    path.push_back(*node);
    recursive_path(node->getPred(), path);
  }

  bool Hybrid_astarPlanner::transfer_to_ros_path(std::vector<Node3D> &native_path, std::vector<geometry_msgs::PoseStamped> &ros_path)
  {
    std::vector<hybrid_astar_planner::Node3D>::iterator it;
    for (it = native_path.begin(); it != native_path.end(); it++)
    {
      geometry_msgs::PoseStamped temp_pose;
      temp_pose.pose.position.x = (*it).getX();
      temp_pose.pose.position.y = (*it).getY();

      tf2::Quaternion q_tf2;
      q_tf2.setRPY(0, 0, (*it).getT());

      geometry_msgs::Quaternion q = tf2::toMsg(q_tf2);
      temp_pose.pose.orientation = q;

      temp_pose.header.frame_id = costmap_ros_->getGlobalFrameID();

      ros_path.push_back(temp_pose);
    }

    if (native_path.size() == ros_path.size())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // void Hybrid_astarPlanner::visualize_plan(const std::vector<geometry_msgs::PoseStamped> &path)
  // {
  //   if (!initialized_)
  //   {
  //     ROS_ERROR(
  //         "This planner has not been initialized yet, but it is being used, please call initialize() before use");
  //     return;
  //   }

  //   // create a message for the plan
  //   nav_msgs::Path gui_path;
  //   gui_path.poses.resize(path.size());

  //   gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  //   gui_path.header.stamp = ros::Time::now();

  //   for (unsigned int i = 0; i < path.size(); i++)
  //   {
  //     gui_path.poses[i] = path[i];
  //   }

  //   vis_plan_pub.publish(gui_path);
  // }

}
