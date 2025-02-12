// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <geometry_msgs/msg/vector3.hpp>

// STD
// #include <memory>
// #include <string>
#include <cmath>
#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
constexpr double GRAVITY = 9.8066f;
constexpr double PI = 3.1415926535f;
class Trajectory
{
public:
  Trajectory(double v, double k);
  void initSolver();
  void autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg
  , auto_aim_interfaces::msg::TrackerInfo & info_msg);
  

private:
  double k ;//弹道系数
  double v ;//子弹速度
  double s_bias;         //枪口前推的距离
  double z_bias;         //yaw轴电机到枪口水平面的垂直距离
  double bias_time;        //偏置时间
  double tempdz;
  struct tar_pos
  {
    double x = 0.0;           //装甲板在世界坐标系下的x
    double y = 0.0;           //装甲板在世界坐标系下的y
    double z = 0.0;           //装甲板在世界坐标系下的z
    double yaw = 0.0;         //装甲板坐标系相对于世界坐标系的yaw角
  };
  tar_pos tar_position[4];
  double pitchSolve(double s, double z, double v);
  double newtonUpdate(double s, double v, double angle);
  //double (double s, double y, double v);
  
};

}  // namespace rm_auto_aim

#endif  // TRAJECTORY_HPP_
