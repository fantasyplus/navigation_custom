/**
 * @file smoother.h
 * @brief 用于将路径进行平滑处理的类
 *
 */

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace hybrid_astar_planner
{
   /*!
      \brief This class takes a path object and smoothes the nodes of the path.

      It also uses the Voronoi diagram as well as the configuration space.
   */
   class Smoother
   {
   public:
      Smoother() {}

      /*!
         \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.
         核心函数，将由节点组成的路径采用梯度下降方法进行平滑
         During the different interations the following cost are being calculated
         不同的迭代阶段采用计算下面的代价值作为指标
         obstacleCost：障碍物代价
         curvatureCost：曲率代价
         smoothnessCost：平滑代价
         voronoiCost: Voronoi代价 (实际计算时没有这一项)
      */
      void smoothPath(DynamicVoronoi &voronoi,std::vector<Node3D> &smoothed_path);

      /// obstacleCost - pushes the path away from obstacles
      //障碍物项，用于约束路径远离障碍物
      Vector2D obstacleTerm(Vector2D xi);

      /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
      //曲率项，用于保证可转弯性及通行性
      Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);

      /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
      //平滑项，用于将节点等距分布并尽量保持同一个方向
      Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

      /// voronoiCost - trade off between path length and closeness to obstaclesg
      //   Vector2D voronoiTerm(Vector2D xi);

      /// a boolean test, whether vector is on the grid or not
      bool isOnGrid(Vector2D vec)
      {
         if (vec.getX() >= 0 && vec.getX() < width &&
             vec.getY() >= 0 && vec.getY() < height)
         {
            return true;
         }
         return false;
      }

   private:
      /// maximum possible curvature of the non-holonomic vehicle
      float kappaMax = 1.f / (Constants::r * 1.1);
      /// maximum distance to obstacles that is penalized
      float obsDMax = Constants::minRoadWidth;
      /// maximum distance for obstacles to influence the voronoi field
      float vorObsDMax = Constants::minRoadWidth;

      //权重系数
      /// falloff rate for the voronoi field
      float alpha = 0.1;
      /// weight for the obstacle term
      float wObstacle = 0.2;
      /// weight for the voronoi term
      float wVoronoi = 0;
      /// weight for the curvature term
      float wCurvature = 0;
      /// weight for the smoothness term
      float wSmoothness = 0.2;

      // 描述地图中拓扑结构的voronoi diagram
      /// voronoi diagram describing the topology of the map
      DynamicVoronoi voronoi;

      //地图宽度与高度
      /// width of the map
      int width;
      /// height of the map
      int height;

      //待平滑的路径
      /// path to be smoothed
      std::vector<Node3D> path;
   };
}
#endif // SMOOTHER_H
