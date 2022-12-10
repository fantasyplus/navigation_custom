/**
 * @file algorithm.h
 * @brief Hybrid A* 算法的核心过程函数，只有一个函数hybridAStar()
 * 输入：
 *      始点、
 *      目标点、
 *      配置空间的3维和2维表示（2D用来A*，3D用于hybrid A*）、
 *      搜索网格的宽度及高度、
 *      配置空间的查找表、
 *      Dubins查找表（程序实际上没有使用该表，而是直接调用OMPL库计算）、
 *      RVIZ可视化类(用于显示结果)
 * 返回：
 *      满足约束条件的节点（数据结构用指针表示）
 *
 * @date 2019-11-20
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "hybrid_astar_planner/node3d.h"
#include "hybrid_astar_planner/node2d.h"
#include "hybrid_astar_planner/dubins.h"

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "timer.h"

namespace hybrid_astar_planner
{
   class Node3D;
   class Node2D;

   /*!
    * \brief A class that encompasses the functions central to the search.
    */
   class Algorithm
   {
   public:
      /// The deault constructor
      Algorithm() {}

      // HYBRID A* ALGORITHM
      /*!
         \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

         \param start the start pose
         \param goal the goal pose
         \param nodes3D the array of 3D nodes representing the configuration space C in R^3
         \param nodes2D the array of 2D nodes representing the configuration space C in R^2
         \param width the width of the grid in number of cells 网格的宽度（以单元格为单位）
         \param height the height of the grid in number of cells 网格的高度（以单元格为单位）
         \return the pointer to the node satisfying the goal condition 返回满足目标条件的节点指针
      */
      Node3D *hybridAStar(Node3D &start,
                                 const Node3D &goal,
                                 Node3D *nodes3D,
                                 Node2D *nodes2D,
                                 int width,
                                 int height,
                                 costmap_2d::Costmap2DROS *costmap_ros,
                                 base_local_planner::WorldModel *world_model);

   private:
      //原始A*算法，用来搜索计算 holonomic-with-obstacles heuristic
      float aStar(Node2D &start, Node2D &goal, Node2D *nodes2D, int width, int height);
      //计算start到目标点goal的启发式代价(即：cost-to-go)
      void updateH(Node3D &start, const Node3D &goal, Node2D *nodes2D, int width, int height);
      // Dubins shot的路径
      Node3D *dubinsShot(Node3D &start, const Node3D &goal);
      //判断是否和障碍物相撞
      bool isTraversable(Node3D *node);
      bool isTraversable(Node2D *node);

   private:
      costmap_2d::Costmap2DROS *costmap_ros_global;
      base_local_planner::WorldModel *world_model_global;
   };
}
#endif // ALGORITHM_H
