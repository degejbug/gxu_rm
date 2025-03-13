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

//Ceres
#include <ceres/ceres.h>
#include <iostream>

namespace rm_auto_aim
{
const double GRAVITY = 9.7877;
const double FIRE_ERROR = 1.0;
class ResistanceFuncLinear {
  private:
      const double g{9.7877}; 
      const double horizontal_distance; 
      const double z;         
      const double v0;  
      const double k;         
  
  public:
      ResistanceFuncLinear(const double& horizontal_distance, const double& z, const double& v0, const double& k)
          : horizontal_distance(horizontal_distance), z(z), v0(v0), k(k) {}
  
      template <typename T>
      bool operator()(const T* const angle, T* residual) const {
          T cos_theta = ceres::cos(angle[0]);
          T v0x = v0 * cos_theta;
  
          // 修正飞行时间计算
          T arg = k * horizontal_distance / v0x;
          if (ceres::abs(arg) >= 1.0) {  // 处理无解情况
              residual[0] = T{1e10};     // 返回大残差
              return true;
          }
          T fly_time = -ceres::log(1.0 - arg) / k;
  
          // 计算垂直位移残差
          T v0y = v0 * ceres::sin(angle[0]);
          T term = (v0y + g / k) / k * (1.0 - ceres::exp(-k * fly_time));
          T y_pred = term - (g / k) * fly_time;
          residual[0] = y_pred - z;  // 残差应为预测值减目标值
          return true;
      }
  };


class Trajectory
{
public:
  Trajectory(double v, double k);
  void autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg
    , auto_aim_interfaces::msg::TrackerInfo & info_msg
    , const double & gimbal_now_yaw ,const double & gimbal_now_pitch);
  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state = TRACKING_ARMOR; //定义枚举变量决定装甲板跟踪状态
  

private: 
  const double r_k = 0.01; //空气阻力系数
  const double side_angle_ = 10; // 跳转到下一块装甲板的角度阈值
  const double min_switching_v_yaw_ = 1.0; // 如果目标角速度比较慢，则不需要角度阈值 这里设置为1.0rad/s
  const double shooting_range_w_ = 0.135;
  const double shooting_range_h_ = 0.145;
  const double max_tracking_v_yaw_ = 5.5; //最大追踪角速度
  const double transfer_thresh_ = 5; //检测如果超过5次则追踪中心
  double k;//弹道系数
  double v;//子弹速度
  double s_bias;         //枪口前推的距离
  double z_bias;         //yaw轴电机到枪口水平面的垂直距离
  double bias_time;        //偏置时间
  double predict_time;      //预测时间
  double aim_r;           //目标装甲板半径
  bool is_current_pair = true; // 相同高度装甲板配对
  double tempdz;
  int overflow_count_; //溢出计数
  struct tar_pos
  {
    double x = 0.0;           //装甲板在世界坐标系下的x
    double y = 0.0;           //装甲板在世界坐标系下的y
    double z = 0.0;           //装甲板在世界坐标系下的z
    double yaw = 0.0;         //装甲板坐标系相对于世界坐标系的yaw角
  };
  tar_pos tar_position[4];
  double pitchSolve(double s, double z, double v, double k);
  double getYaw(double fire_yaw, double tar_yaw);
  double calculateAngle(double x1, double y1, double x2, double y2);
  bool firecontrol(const double gimbal_yaw, const double gimbal_pitch, const double target_yaw, const double target_pitch, const double distance);
  void getarmorposition(
    auto_aim_interfaces::msg::Target & target_msg,
    tar_pos *tar_position,
    const double target_yaw,
    const double r1,
    const double r2,
    const double d_za,
    const size_t armors_num);
  int selectbestarmor(auto_aim_interfaces::msg::Target & target_msg,
    const double target_yaw,
    const double target_v_yaw,
    const size_t armors_num);
};


}  // namespace rm_auto_aim

#endif  // TRAJECTORY_HPP_
