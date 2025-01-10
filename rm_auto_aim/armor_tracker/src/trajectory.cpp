#include "armor_tracker/trajectory.hpp"

// STD
#include <cfloat>
#include <cmath>
#include <memory>
#include <string>
namespace rm_auto_aim
{
Trajectory::Trajectory(float k, float v)
: k(k),
  v(v),
  s_bias(0.03),
  z_bias(0.02),
  bias_time(10),
  tempdz(0)
{
}

void Trajectory::initSolver()
{
  k = 0.038;//弹道系数
  v = 25;
  s_bias = 0;         
  z_bias = 0;
  bias_time = 100;
  tempdz = 0; 
}

void Trajectory::autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg)
{
  //计算四块装甲板的位置
  //装甲板id顺序，以四块装甲板为例，逆时针编号
  //      2
  //   3     1
  //      0
  //float current_yaw = 0;
  // float estimate_t = sqrt(target_msg.position.x *target_msg.position.x + 
  //                         target_msg.position.y *target_msg.position.y)/v;
  
  //test
  //pnp在远距离时出现高度解算误差，用线性函数强行拟合到正确坐标
  //tvec误差可能是标定问题
  z_bias = 0.10;
  //
  float estimate_t = 5/25;
  // 线性预测
  float timeDelay = bias_time/1000.0 + estimate_t;
  float aim_yaw = target_msg.yaw + target_msg.v_yaw * timeDelay;
  
  //test
  // bool is_rotate = false;
  // if (target_msg.v_yaw > 0.1) is_rotate = true;
  //
  float pitch = 0;
  float yaw = 0;
  int use_1 = 1;
  //int i = 0;
  int idx = 0; // 选择的装甲板
  if (target_msg.armors_num == 9) {  //前哨站test
      for (int i = 0; i<3; i++) {
          float tmp_yaw = aim_yaw + i * 2.0 * PI/3.0;  // 2/3PI
          float r =  (target_msg.radius_1 + target_msg.radius_2)/2;   //理论上r1=r2 这里取个平均值
          tar_position[i].x = target_msg.position.x - r*cos(tmp_yaw);
          tar_position[i].y = target_msg.position.y - r*sin(tmp_yaw);
          tar_position[i].z = target_msg.position.z;
          tar_position[i].yaw = tmp_yaw;
      }

      //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用
  } else {
    for (int i = 0; i<4; i++) {
        float tmp_yaw = aim_yaw + i * PI/2.0;
        float r = use_1 ? target_msg.radius_1 : target_msg.radius_2;
        tar_position[i].x = target_msg.position.x - r*cos(tmp_yaw);
        tar_position[i].y = target_msg.position.y - r*sin(tmp_yaw);
        tar_position[i].z = use_1 ? target_msg.position.z : target_msg.position.z + tempdz;
        tar_position[i].yaw = tmp_yaw;
        use_1 = !use_1;
    }

    //计算枪管到目标装甲板yaw最小的那个装甲板//rv origin
    // float yaw_diff_min = fabsf(current_yaw - tar_position[0].yaw);
    // for (int i = 1; i<4; i++) {
    //     float temp_yaw_diff = fabsf(current_yaw - tar_position[i].yaw);
    //     if (temp_yaw_diff < yaw_diff_min)
    //     {
    //         yaw_diff_min = temp_yaw_diff;
    //         idx = i;
    //     }

    //尽量打击接近yaw=0的装甲板gxu
    float yaw_diff_min = fabsf(tar_position[0].yaw);
    for (int i = 0; i<4; i++) {
        float temp_yaw_diff = fabsf(tar_position[i].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = i;
        }
    }
  }
  auto aim_z = tar_position[idx].z + target_msg.velocity.z * timeDelay;//test
  auto aim_x = tar_position[idx].x + target_msg.velocity.x * timeDelay;
  auto aim_y = tar_position[idx].y + target_msg.velocity.y * timeDelay;
  float distance = sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - s_bias;
  float fly_t = 0;
  //这里符号存疑
  float temp_pitch = -pitchSolve(distance, aim_z + z_bias, v, fly_t);
  if(temp_pitch)
    pitch = temp_pitch;
  if(aim_x || aim_y)
    yaw = (float)(std::atan2(aim_y, aim_x));
  
  target_msg.position.x = yaw;
  target_msg.position.y = pitch;
    
}

float Trajectory::newtonUpdate(float s, float v, float angle, float & final_t)
{
  float z;
  //t为给定v与angle时的飞行时间
  float t = (float)((exp(k * s) - 1) / (k * v * cos(angle)));
  if(t < 0)
  {
      //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
      //printf("[WRAN]: Exceeding the maximum range!\n");
      //重置t，防止下次调用会出现nan
      t = 0;
      return 0;
  }
  //z为给定v与angle时的高度
  z = v * sin(angle) * t / cos(angle) + 0.5 * GRAVITY * t * t / cos(angle) / cos(angle);//wu
      //(float)(v * sin(angle) * t - GRAVITY * t * t / 2);//rv          
                
  //printf("model %f %f\n", t, z);
  final_t = t;
  return z;
}

float Trajectory::pitchSolve(float s, float z, float v, float & final_t)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  //int i = 0;
  float fly_t = 0;
  int count = 0;
  z_temp = z;
  // iteration
  do//for (i = 0; i < 20; i++)
  {
      angle_pitch = std::atan2(z_temp, s); // rad
      z_actual = newtonUpdate(s, v, angle_pitch, fly_t);
      if(z_actual == 0)
      {
          angle_pitch = 0;
          break;
      }
      dz = 0.6*(z - z_actual);
      z_temp = z_temp + dz;

      count++; if (count >= 80)break;
      
  }while (fabsf(dz) < 0.00001);

  final_t = fly_t;
  return angle_pitch;
}
}//namespace rm_auto_aim