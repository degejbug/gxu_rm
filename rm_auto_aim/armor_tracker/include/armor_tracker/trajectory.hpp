// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

// Eigen
// #include <Eigen/Eigen>

// ROS
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <geometry_msgs/msg/vector3.hpp>

// STD
// #include <memory>
// #include <string>
#include <cmath>
#include "armor_tracker/tracker.hpp"
// #include "auto_aim_interfaces/msg/armors.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
constexpr float GRAVITY = 9.8066f;
constexpr float PI = 3.1415926535f;
class Trajectory
{
public:
  Trajectory(float v, float k);
  void initSolver();
  void autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg);
  

private:
  float k ;//弹道系数
  float v ;//子弹速度
  float s_bias;         //枪口前推的距离
  float z_bias;         //yaw轴电机到枪口水平面的垂直距离
  int bias_time;        //偏置时间
  float tempdz;
  //float newtonUpdate(float s, float v, float angle);
  //float pitchSolve(float s, float y, float v);
  struct tar_pos
  {
    float x = 0.0;           //装甲板在世界坐标系下的x
    float y = 0.0;           //装甲板在世界坐标系下的y
    float z = 0.0;           //装甲板在世界坐标系下的z
    float yaw = 0.0;         //装甲板坐标系相对于世界坐标系的yaw角
  };
  tar_pos tar_position[4];
  //float (float s, float y, float v);
  
};

}  // namespace rm_auto_aim

#endif  // TRAJECTORY_HPP_
