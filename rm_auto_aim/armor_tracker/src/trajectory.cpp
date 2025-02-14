#include "armor_tracker/trajectory.hpp"

// STD
#include <cmath>
#include <memory>
#include <string>
namespace rm_auto_aim
{
Trajectory::Trajectory(double k, double v)
: k(k),
  v(v),
  s_bias(0.20),
  z_bias(0.02),
  bias_time(10.0),
  tempdz(0.0)
{
}

void Trajectory::initSolver()
{
  k = 0.038;//弹道系数
  v = 25.0;
  s_bias = 0.0;         
  z_bias = 0.0;
  bias_time = 100.0;
  tempdz = 0.0; 
}

void Trajectory::autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg
, auto_aim_interfaces::msg::TrackerInfo & info_msg
, double fire_yaw)
{
  //计算四块装甲板的位置
  //装甲板id顺序，以四块装甲板为例，逆时针编号
  //      2
  //   3     1
  //      0
  //double current_yaw = 0;
  double estimate_distance = 
  std::sqrt(target_msg.position.x *target_msg.position.x + target_msg.position.y *target_msg.position.y) > 1e-4 ?
  std::sqrt(target_msg.position.x *target_msg.position.x + target_msg.position.y *target_msg.position.y) : 1e-4;
  
  double estimate_t = estimate_distance / v;
  // target_msg.radius_1 = 0.3;
  // target_msg.radius_2 = 0.3;
  //info_msg.position_diff = estimate_distance;
  //test
  //pnp在远距离时出现高度解算误差，用线性函数强行拟合到正确坐标
  //tvec误差可能是标定问题
  //
  // 线性预测
  double timeDelay = bias_time/1000.0 + estimate_t;
  double aim_yaw = target_msg.yaw + target_msg.v_yaw * timeDelay;
  double car_center_yaw = std::atan2(target_msg.position.y, target_msg.position.x);
  //test
  // bool is_rotate = false;
  // if (target_msg.v_yaw > 0.1) is_rotate = true;
  //

  int use_1 = 1;
  int idx = 0; // 选择的装甲板
  if (target_msg.armors_num == 9) {  //前哨站test
      for (int i = 0; i<3; i++) {
          double tmp_yaw = aim_yaw + i * 2.0 * PI/3.0;  // 2/3PI
          double r =  (target_msg.radius_1 + target_msg.radius_2)/2;   //理论上r1=r2 这里取个平均值
          //
          tar_position[i].x = target_msg.position.x - r*std::cos(tmp_yaw);
          tar_position[i].y = target_msg.position.y - r*std::sin(tmp_yaw);
          //
          tar_position[i].z = target_msg.position.z;
          tar_position[i].yaw = tmp_yaw;
      }

      //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用
  } else {
    for (int i = 0; i<4; i++) {
        double tmp_yaw = aim_yaw + i * PI/2.0;
        double r = use_1 ? target_msg.radius_1 : target_msg.radius_2;
        tar_position[i].x = target_msg.position.x - r*std::cos(tmp_yaw);
        tar_position[i].y = target_msg.position.y - r*std::sin(tmp_yaw);
        tar_position[i].z = use_1 ? target_msg.position.z : target_msg.position.z + tempdz;
        tar_position[i].yaw = tmp_yaw;
        use_1 = !use_1;
    }

    //尽量打击接近yaw=0的装甲板
    // double yaw_diff_min = fabsf(std::atan2(tar_position[0].y, tar_position[0].x) - car_center_yaw);
    double yaw_diff_min = fabsf(std::atan2(tar_position[0].y, tar_position[0].x) - car_center_yaw);
    for (int i = 0; i<4; i++) {
        double temp_yaw_diff = fabsf(std::atan2(tar_position[i].y, tar_position[i].x) - car_center_yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = i;
        }
    }
  }
  //打击前方装甲板
  if(idx == 1) idx = 3;
  if(idx == 2) idx = 0;
  if(idx == 0){
    target_msg.is_fire = true;
  }
  // if(std::fabs(target_msg.v_yaw < 0.1)){
  //   //idx = 0;
  //   target_msg.is_fire = true;
  // }
  //
  auto aim_z = tar_position[idx].z + target_msg.velocity.z * timeDelay;//test
  auto aim_x = tar_position[idx].x + target_msg.velocity.x * timeDelay;
  auto aim_y = tar_position[idx].y + target_msg.velocity.y * timeDelay;
  double distance = std::sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - s_bias;
  //手动函数补偿高度
  info_msg.yaw = std::atan2(tar_position[idx].y, tar_position[idx].x) - car_center_yaw;
  info_msg.yaw_diff = idx;
  z_bias = distance * 0.028 + 0.08;
  //
  double pitch = 0.0;
  double yaw = 0.0;
  double temp_pitch = pitchSolve(distance, aim_z + z_bias, v);
  double temp_yaw = (double)(std::atan2(aim_y, aim_x));
  //temp_yaw = (double)(std::atan2(target_msg.position.y, target_msg.position.x));
  //纠正2025赛季全向轮步由于c板倒置出现的问题
  //纠正2025赛季全向轮步摄像头位置误差问题
  temp_yaw = -temp_yaw-0.005;
  temp_pitch = -temp_pitch;
  //

  if(temp_pitch)
    pitch = temp_pitch ;//* 1.5;
  if(aim_x || aim_y)
    yaw = temp_yaw;
  target_msg.position.x = yaw;
  target_msg.position.y = pitch;
    
}

double Trajectory::newtonUpdate(double s, double v, double angle)
{
  double z;
  //t为给定v与angle时的飞行时间
  double t = (double)((std::exp(k * s) - 1) / (k * v * std::cos(angle)));
  if(t < 0)
  {
      //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
      //printf("[WRAN]: Exceeding the maximum range!\n");
      //重置t，防止下次调用会出现nan
      t = 0;
      return 0;
  }
  //z为给定v与angle时的高度
  //z = v * std::sin(angle) * t / std::cos(angle) + 0.5 * GRAVITY * t * t / std::cos(angle) / std::cos(angle);//wu
  z = (v * std::sin(angle) * t - GRAVITY * t * t / 2);          
                
  return z;
}

double Trajectory::pitchSolve(double s, double z, double v)
{
  double z_temp, z_actual, dz;
  double angle_pitch;
  //int i = 0;
  z_temp = z;
  // iteration
  for (int i = 0; i < 40; i++)
  {
    angle_pitch = std::atan2(z_temp, s); // rad
    z_actual = newtonUpdate(s, v, angle_pitch);
    if(z_actual == 0)
    {
        angle_pitch = 0;
        break;
    }
    dz = 0.3*(z - z_actual);
    z_temp = z_temp + dz;

    if (fabsf(dz) < 0.00001){
      break;
    }   
  }

  return angle_pitch;
}

double Trajectory::getYaw(double fire_yaw, double tar_yaw)
{ 
  if(fire_yaw < 0){
    fire_yaw = 2*M_PI + fire_yaw;
  }
  if(tar_yaw < 0){
    tar_yaw = 2*M_PI + tar_yaw;
  }
  double return_yaw = std::fabs(fire_yaw + M_PI -tar_yaw);
  if(return_yaw > 2*M_PI){
    return_yaw -= 2*M_PI;
  }
  return return_yaw;
}

}//namespace rm_auto_aim