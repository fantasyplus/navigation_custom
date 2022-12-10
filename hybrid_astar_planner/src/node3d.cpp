/**
 * @file node3d.cpp
 * @brief 实现了前向行驶和逆向行驶的代价计算
 *
 */
#include <hybrid_astar_planner/node3d.h>
#include <iostream>
using namespace hybrid_astar_planner;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3; //三个方向。注意2D Node为8个
// possible movements
// const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
// const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
// const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

// R = 6, 6.75 DEG: 当转弯半径R=6时，每个方向的变化量
/*
  见链接https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
  R=L/tan（φ），这里φ是车轮转向角。
  在度数较小的情况下，正切值和弧度值几乎一致。

  在https://github.com/karlkurzer/path_planner/issues/27 中，
  原作者说了，Δθ=l/r，r肯定是转弯半径，但l不确定是否为轴距，虽然上一个链接中的L是轴距，公式也差不多，估计是近似。

*/
// const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};//R*cos(6.75*M_PI/180)-R
// const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};//R*sin(6.75*M_PI/180)
// const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};//delta_t=6.75*π/180

// R = 3, 6.75 DEG
// const float Node3D::dy[] = { 0,        -0.0207946, 0.0207946};
// const float Node3D::dx[] = { 0.35342917352,   0.352612,  0.352612};
// const float Node3D::dt[] = { 0,         0.11780972451,   -0.11780972451};

// const float Node3D::dy[] = { 0,       -0.16578, 0.16578};
// const float Node3D::dx[] = { 1.41372, 1.40067, 1.40067};
// const float Node3D::dt[] = { 0,       0.2356194,   -0.2356194};

// R = 1.8, 6.75 DEG,l=0.8
// const float Node3D::dy[] = { 0,        -0.01247678, 0.01247678};
// const float Node3D::dx[] = { 0.21205750412,   0.211567316,  0.211567316};
// const float Node3D::dt[] = { 0,         0.11780972451,   -0.11780972451};

// R = 1.6, 6.75 DEG,l=0.65
const float Node3D::dy[] = {0, -0.01109047, 0.01109047};
const float Node3D::dx[] = {0.18849552, 0.1880598, 0.1880598};
const float Node3D::dt[] = {0, 0.1178097, -0.1178097};

//###################################################
//                                         IS ON GRID
//###################################################
//判断是否在3D网格上， x \in [0, width], y \in [0, height], t/deltaHeadingRad \in [0, headings]
bool Node3D::isOnGrid(const int width, const int height) const
{
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}

//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D &goal) const
{
  int random = rand() % 10 + 1; //产生位于[1, 10]的随机数
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance; //距离的平方和在100以内则认为可达
}

//###################################################
//                                   CREATE SUCCESSOR
// 根据dx, dy, dt产生Successor,
// 当前源码的dx, dy, dt为人为指定的值，可以根据实际需要进行修改
//###################################################
Node3D *Node3D::createSuccessor(const int i)
{
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor p ositions forward
  if (i < 3)
  { //前向 Successor
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
    // std::cout<<"t:"<<t<<std::endl;
    // std::cout<<xSucc<<" "<<ySucc<<std::endl;
  }
  // backwards
  else
  { //后向 Successor
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

//###################################################
//                                      MOVEMENT COST
//###################################################
//更新代价函数：cost-so-far
void Node3D::updateG()
{
  // forward driving
  if (prim < 3)
  { //前进情况
    // penalize turning
    if (pred->prim != prim)
    { //方向发生改变时
      // penalize change of direction
      if (pred->prim > 2)
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD; //改变方向的惩罚
      }
      else
      {
        g += dx[0] * Constants::penaltyTurning; //没有改变方向
      }
    }
    else
    { //方向没有发生变化
      g += dx[0];
    }
  }
  // reverse driving
  else
  { //后退
    // penalize turning and reversing
    if (pred->prim != prim)
    {
      // penalize change of direction
      if (pred->prim < 3)
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      }
      else
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    }
    else
    {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}

//###################################################
//                                 3D NODE COMPARISON
//###################################################
// 3d节点的比较函数：x和y同时相同、并且theta在一个阈值范围内时可以认为是同一个Node
bool Node3D::operator==(const Node3D &rhs) const
{
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
