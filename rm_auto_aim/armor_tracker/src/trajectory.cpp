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
  z_bias(0.21),
  bias_time(0),
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

float Trajectory::autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg)
{ 
  using namespace std;
  float tx = target_msg.position.x;
  float ty = target_msg.position.y;
  float tz = target_msg.position.z;
  tz += z_bias;
  float distance_ = sqrt(tx * tx 
  + ty * ty) - s_bias; // 此处为枪口距离敌方中心的直线距离
  float pitch = atan(tz /distance_);//用arctan计算未解算的pitch轴角度
  float a;
  float theta = pitch;
  float delta_z;
  float aim_x,aim_y;
  float k1 = 0.038;//0.47 * 1.169 * (2 * PI * 0.0105 * 0.0105) / 2 / 0.021;//小弹丸
    //float k1 = 0.47 * 1.169 * (2 * M_PI * 0.02125 * 0.02125) / 2 / 0.041;//大弹丸
  float center_distance = distance_;    // 距离
  float flyTime;   //计算子弹飞行时间

  for (int i = 0; i < 80; i++) //最多迭代20次
  {
              
      // 计算炮弹的飞行时间
      a = (cos(theta) == 0 ? 1e-5 : cos(theta));//防止分母为0
      //flyTime = (pow(2.718281828, k1 * center_distance) - 1) / (k1 * v * a);//牛顿迭代求飞行时间
      flyTime = (std::exp(k1 * center_distance) - 1) / (k1 * v * a);
      delta_z = tz - v * sin(theta) * flyTime / cos(theta) +
                0.5 * GRAVITY * flyTime * flyTime / cos(theta) / cos(theta);
      if (fabs(delta_z) < 0.001)//如果误差小于一定值，则退出循环
          break;
      theta -= delta_z / (-(v * flyTime) / pow(cos(theta), 2) +
                          GRAVITY * flyTime * flyTime / (v * v) * sin(theta) / pow(cos(theta), 3));
  }
  aim_x = tx - target_msg.radius_1 * cos(target_msg.yaw + target_msg.v_yaw * flyTime);//装甲板动，车不动时，而且只考虑了一种装甲板模式，还有另一种需要改
  aim_y = ty - target_msg.radius_1 * sin(target_msg.yaw + target_msg.v_yaw * flyTime);
  aim_y = (aim_y == 0 ? 1e-5 : aim_y);//防止分母为0，用一个很小的值代替
  
  target_msg.position.x = (float)(atan2(aim_y, aim_x));//用arctan计算yaw轴取值
  //target_msg.position.y = -(float)theta;
  return -(float)theta;
}


}//namespace rm_auto_aim