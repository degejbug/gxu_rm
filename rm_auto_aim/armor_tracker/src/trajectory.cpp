#include "armor_tracker/trajectory.hpp"

// STD
#include <cmath>
#include <memory>
#include <string>


namespace rm_auto_aim
{
Trajectory::Trajectory(double k, double v)
: k(k),
  v(23.9),
  s_bias(0.13),
  z_bias(0.1),
  bias_time(160.0),
  // 开始逐个计算装甲板坐标
  tempdz(0.0)
{
}

//弹道解算函数
void Trajectory::autoSolveTrajectory(auto_aim_interfaces::msg::Target & target_msg
, auto_aim_interfaces::msg::TrackerInfo & info_msg
, const double & gimbal_now_yaw ,const double & gimbal_now_pitch)
{
  /*参数初始化*/
  size_t armors_num = 4; // 装甲板数量4
  //获取装甲板世界坐标位置
  double yaw_armor = target_msg.yaw;
  double x_armor = target_msg.position.x - target_msg.radius_1*std::cos(yaw_armor);
  double y_armor = target_msg.position.y - target_msg.radius_1*std::sin(yaw_armor);
  double z_armor = target_msg.position.z;
  //空气阻力系数为0.01
  double resistance = r_k < 1e-4 ? 1e-4 : r_k;

  /*计算数据*/
  //计算到目标装甲板的直线距离，并且避免除0错误
  double estimate_distance_armor =
  (std::sqrt(x_armor * x_armor + y_armor * y_armor) > 1e-4 ?
  std::sqrt(x_armor * x_armor + y_armor * y_armor) : 1e-4);
  //计算仰角,用于计算子弹飞行时间
  double angle = atan2(target_msg.position.z, estimate_distance_armor);
  //计算子弹飞行时间，（以后考虑把这段计算直线距离，仰角，飞行时间放在getFlyingTime函数里面一起计算，赶时间先这样吧）
  double flying_time = (exp(resistance * estimate_distance_armor) - 1) / (resistance * v * cos(angle));
  if(state == TRACKING_CENTER){
    bias_time = 80;
  }
  else if(state == TRACKING_ARMOR){
    bias_time = 160;
  }
  //偏置时间加上子弹飞行时间获得总预测时间
  double timeDelay = bias_time/1000.0 + flying_time ;
  //预测正在观察的敌方装甲板在延迟后的位置
  x_armor += timeDelay * target_msg.velocity.x;
  y_armor += timeDelay * target_msg.velocity.y;
  z_armor += timeDelay * target_msg.velocity.z;
  yaw_armor += timeDelay * target_msg.v_yaw;

  /*开始进入选板逻辑*/
  //推算四块装甲板的坐标位置
  getarmorposition(target_msg, tar_position, yaw_armor ,target_msg.radius_1, target_msg.radius_2, target_msg.dz , armors_num);
  //选择装甲板
  int idx = selectbestarmor(target_msg, yaw_armor, target_msg.v_yaw, armors_num);
  //获取选择的目标装甲板位置
  auto aim_x = tar_position[idx].x;
  auto aim_y = tar_position[idx].y;
  auto aim_z = tar_position[idx].z;
  double aim_distance_armor = 
  (std::sqrt(aim_x * aim_x + aim_y * aim_y) > 1e-4 ?
  std::sqrt(aim_x * aim_x + aim_y * aim_y) : 1e-4);
  double yaw = -(double)(std::atan2(aim_y, aim_x));
  double pitch = pitchSolve(aim_distance_armor, aim_z, v, 0.038);

  /*火控逻辑*/
  target_msg.is_fire = firecontrol(gimbal_now_yaw, gimbal_now_pitch, yaw - 0.02, pitch + 0.06, aim_distance_armor);
  /*根据目标转速动态调整跟踪方式*/
  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target_msg.v_yaw) > max_tracking_v_yaw_) //如果目标角速度大于最大追踪角速度（6.0）
      {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) //检测如果超过5次则追踪中心
      {
        state = TRACKING_CENTER;
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target_msg.v_yaw) < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      //如果目标角速度大于最大追踪角速度（6.0）则追踪目标中心
      yaw = -(double)(std::atan2(target_msg.position.y, target_msg.position.x));
      double aim_distance_center = std::sqrt(target_msg.position.x * target_msg.position.x + target_msg.position.y * target_msg.position.y);
      pitch = pitchSolve(aim_distance_armor, target_msg.position.z, v, 0.038);
      double fire_yaw = -(double)(std::atan2(aim_y, aim_x));
      double fire_pitch = pitchSolve(aim_distance_armor, aim_z, v, 0.038);
      target_msg.is_fire = firecontrol(gimbal_now_yaw, gimbal_now_pitch, fire_yaw, fire_pitch, aim_distance_armor);;
      break;
    }
  }
  /*给电控端发数据*/
  target_msg.position.x = -yaw; 
  target_msg.position.y = pitch; 
}

bool Trajectory::firecontrol(const double gimbal_yaw,
                             const double gimbal_pitch,
                             const double target_yaw,
                             const double target_pitch,
                             const double distance){
  // 计算当前角度和目标角度是否在可击打范围内
  double shooting_range_yaw =  std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch =  std::abs(atan2(shooting_range_h_ * 1.5, distance));
  //限制1度以上击打范围防止距离过大以至于无法发射,3m8以外角度会小于1度
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  //计算当前角度和目标角度是否在可击打范围内
  if (std::abs(gimbal_yaw - target_yaw) < shooting_range_yaw) {
    return true;
  }
  else
  {
    return false;
  }
}

void Trajectory::getarmorposition(
                                 auto_aim_interfaces::msg::Target & target_msg,
                                 tar_pos *tar_position,
                                 const double target_yaw,
                                 const double r1,
                                 const double r2,
                                 const double d_za,
                                 const size_t armors_num) {
  // 初始化半径以及高度差
  double r = 0., target_dz = 0.;
  // 开始逐个计算装甲板坐标
  for (size_t i = 0; i<armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = is_current_pair ? 0 : d_za;
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = 0;
    }
    tar_position[i].x = target_msg.position.x - r*std::cos(temp_yaw);
    tar_position[i].y = target_msg.position.y - r*std::sin(temp_yaw);
    tar_position[i].z = target_msg.position.z; //+ target_dz;
    tar_position[i].yaw = temp_yaw;
  }
}

int Trajectory::selectbestarmor(auto_aim_interfaces::msg::Target & target_msg,
                                const double target_yaw,
                                const double target_v_yaw,
                                const size_t armors_num){
  // 获得目标车辆中心和自身x轴方向夹角
  double alpha = std::atan2(target_msg.position.y, target_msg.position.x);
  // 目标车辆x轴方向和检测到的装甲板夹角
  double beta = target_yaw; 
  // 坐标系转换，从odom坐标系到装甲板坐标系
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // 相对旋转计算
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;
  // 提取相对旋转角度
  double decision_angle = -std::asin(R_center2armor(0, 1));
  //std::cout << "decision_angle:" << decision_angle << std::endl;
  // 跳转到下一块装甲板的角度阈值
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;
  // 如果目标角速度比较慢，则不需要角度阈值 这里设置为1.0rad/s
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }
  double temp_angle = decision_angle + M_PI / armors_num - theta;
  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  //std::cout << "selected_id:" << selected_id << std::endl;
  return selected_id;
}


double Trajectory::pitchSolve(double s, double z, double v, double k)
{
  double theta = std::atan2(z, s);
  //std::cout << "origin theta:" << theta << std::endl;
  //std::cout << "horizontal_distance:" << s << std::endl;
  //std::cout << "z:" << z << std::endl;
  ceres::Problem problem;
  problem.AddResidualBlock(
    new ceres::AutoDiffCostFunction<ResistanceFuncLinear, 1, 1>(
      new ResistanceFuncLinear(s - 0.135, z - 0.1 + 0.1  * (s - 2.1) , 23.9, 0.038)
    ),
    nullptr,
    &theta
  );

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  //std::cout << "optimized result:" << theta << std::endl;1111
  return -theta;
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
