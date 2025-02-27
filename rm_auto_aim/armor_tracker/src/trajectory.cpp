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
  bias_time(167.5),
  tempdz(0.0)
{
}

void Trajectory::initSolver()
{
  k = 0.038;//小弹丸空气阻力系数
  v = 25.0;//小弹丸初速度
  s_bias = 0.0;
  z_bias = 0.0;
  bias_time = 100.0;
  tempdz = 0.0;
}

//弹道解算函数666
void Trajectory::autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg
, auto_aim_interfaces::msg::TrackerInfo & info_msg
, const double & gimbal_now_yaw ,const double & gimbal_now_pitch)
{
  //计算四块装甲板的位置
  //装甲板id顺序，以四块装甲板为例，逆时针编号
  //      2
  //   3     1
  //      0
  //double current_yaw = 0;
  //使用原始数据估算的枪口到目标车中心距离?
  double estimate_distance_center =
  (std::sqrt(target_msg.position.x *target_msg.position.x + target_msg.position.y *target_msg.position.y) > 1e-4 ?
  std::sqrt(target_msg.position.x *target_msg.position.x + target_msg.position.y *target_msg.position.y) : 1e-4);
  
  //double estimate_t_init = estimate_distance / v;
  double estimate_t_init = fabsf(estimate_distance_center / v / std::cos(gimbal_now_pitch)); //计算获得一个估计的初预测飞行时间，考虑到pitch有正负值，故飞行时间取绝对值?
  //target_msg.radius_1 = 0.15;
  //target_msg.radius_2 = 0.24;
  //test
  //pnp在远距离时出现高度解算误差，用线性函数强行拟合到正确坐标
  //tvec误差可能是标定问题
  //
  // 线性预测
  double timeDelay = bias_time/1000.0 + estimate_t_init;  //偏置时间加上子弹飞行时间获得总预测时间
  double aim_yaw = target_msg.yaw + target_msg.v_yaw * timeDelay; //粗略获得预测后的目标装甲板的yaw角度
  double car_center_yaw = std::atan2(target_msg.position.y, target_msg.position.x); //计算目标中心和自身中心的相差角度，以自身X轴为0边
  //test
  target_msg.v_yaw = 2.6;
  if(target_msg.radius_1 < target_msg.radius_2){
    target_msg.radius_1 = 0.15;
    target_msg.radius_2 = 0.24;
  }else{
    target_msg.radius_1 = 0.24;
    target_msg.radius_2 = 0.15;
  }

/*选板逻辑*/
  int use_1 = 1;// 装甲板高度计算标志位
  int idx = 0; // 选择的装甲板

  //前哨站预测
  if (target_msg.armors_num == 9) {
      for (int i = 0; i<3; i++) {
          double tmp_yaw = aim_yaw + i * 2.0 * PI/3.0;  // 2/3PI
          aim_r =  (target_msg.radius_1 + target_msg.radius_2)/2;   //理论上r1=r2 这里取个平均值
          //
          tar_position[i].x = target_msg.position.x - aim_r*std::cos(tmp_yaw);
          tar_position[i].y = target_msg.position.y - aim_r*std::sin(tmp_yaw);
          //
          tar_position[i].z = target_msg.position.z;
          tar_position[i].yaw = tmp_yaw;
      }

     
  } 
  //计算四块装甲板的坐标位置
  else {
    for (int i = 0; i<4; i++) {
        double tmp_yaw = aim_yaw + i * PI/2.0;
        aim_r = use_1 ? target_msg.radius_1 : target_msg.radius_2;
        tar_position[i].x = target_msg.position.x - aim_r*std::cos(tmp_yaw);
        tar_position[i].y = target_msg.position.y - aim_r*std::sin(tmp_yaw);
        tar_position[i].z = use_1 ? target_msg.position.z : target_msg.position.z + tempdz;
        tar_position[i].yaw = tmp_yaw - aim_yaw;
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
  //打击前方装甲板？
  if(idx == 1) idx = 3;
  if(idx == 2) idx = 0;

  //dif_center_yaw是装甲板距离敌方中心线的相对角度
  double dif_center_yaw = calculateAngle(target_msg.position.x, target_msg.position.y, 
                             tar_position[idx].x, tar_position[idx].y);
  if(dif_center_yaw < 0.2){
    target_msg.is_fire = true;
  }

//选板完成后利用迭代法计算pitch并且得到实际子弹飞行速度
  //迭代法计算pitch
  auto aim_z = tar_position[idx].z + target_msg.velocity.z * timeDelay;
  //获得中心到对面装甲板的距离用于求pitch
  double estimate_distance_armor =
  (std::sqrt(tar_position[idx].x * tar_position[idx].x + tar_position[idx].y * tar_position[idx].y) > 1e-4 ?
  std::sqrt(tar_position[idx].x * tar_position[idx].x + tar_position[idx].y * tar_position[idx].y) : 1e-4) - s_bias;
  //手动补偿高度
  z_bias = estimate_distance_armor * 0.024 - 0.03;
  double temp_pitch = pitchSolve(estimate_distance_armor, aim_z + z_bias, v);
  //计算实际延迟
  timeDelay = bias_time/1000.0 + predict_time;

  //用实际延迟计算v_yaw下第一层运动下目标装甲板的位置
  tar_position[idx].x = target_msg.position.x - aim_r*std::cos(tar_position[idx].yaw + target_msg.yaw + target_msg.v_yaw * timeDelay);
  tar_position[idx].y = target_msg.position.y - aim_r*std::sin(tar_position[idx].yaw + target_msg.yaw + target_msg.v_yaw * timeDelay);
  //带入车移动速度，计算第二层运动下目标装甲板位置
  auto aim_x = tar_position[idx].x + target_msg.velocity.x * timeDelay;
  auto aim_y = tar_position[idx].y + target_msg.velocity.y * timeDelay;
  //计算预测后云台的目标yaw角
  info_msg.yaw = calculateAngle(target_msg.position.x, target_msg.position.y, 
                                tar_position[idx].x, tar_position[idx].y);
  info_msg.yaw_diff = idx;
  //
  double pitch = 0.0;
  double yaw = 0.0;
  double temp_yaw = (double)(std::atan2(aim_y, aim_x));
  //temp_yaw = (double)(std::atan2(target_msg.position.y, target_msg.position.x));
  //纠正2025赛季全向轮步由于c板倒置出现的问题
  //纠正2025赛季全向轮步摄像头位置误差问题
  temp_yaw = -temp_yaw-0.02;
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
  //更新预测时间    
  predict_time = t;      
                
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

double Trajectory::calculateAngle(double x1, double y1, double x2, double y2) {
  // 确定较远点P
  double OA = sqrt(x1*x1 + y1*y1);
  double OB = sqrt(x2*x2 + y2*y2);
  double xp, yp;
  if (OA > OB) {
      xp = x1;
      yp = y1;
  } else {
      xp = x2;
      yp = y2;
  }

  // 计算向量AB的模长d
  double dx = x2 - x1;
  double dy = y2 - y1;
  double d = sqrt(dx*dx + dy*dy);

  // 计算点积
  double ab_dot_op = dx * xp + dy * yp;

  // 计算OP的模长
  double op_norm = sqrt(xp*xp + yp*yp);

  // 计算夹角（弧度）
  double cos_theta = ab_dot_op / (d * op_norm);
  double theta = acos(cos_theta);

  if (theta > M_PI/2) {
    theta = M_PI - theta;
  }

  return theta; // 返回弧度值，转换为角度需乘以 (180 / π)
}
}//namespace rm_auto_aim