#ifndef _RTL_PLANNER_H_
#define _RTL_PLANNER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <mars_quadrotor_msgs/TakeoffLand.h>
#include <vector>

namespace rtlplanner
{

class RTLPlanner
{
public:
  RTLPlanner();
  ~RTLPlanner();

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // ---------- FSM ----------
  enum RTLState
  {
    IDLE,             // 空闲：记录面包屑，等待触发
    RTL_STOPPING,     // 已发 mandatory_stop，等待 diff_planner 停稳
    RTL_EXECUTING,    // 返航中：逆序发送面包屑
    RTL_DONE          // 返航完成：降落或等待
  };

  RTLState state_;

  // ---------- 面包屑 ----------
  std::vector<Eigen::Vector3d> breadcrumbs_;   // 飞行时记录的航点
  Eigen::Vector3d last_record_pos_;            // 上次记录面包屑的位置
  double last_record_yaw_;                     // 上次记录面包屑时的航向
  bool home_recorded_;                         // 是否已记录Home点
  Eigen::Vector3d home_pos_;                   // 起飞点(Home)

  // ---------- 当前状态 ----------
  Eigen::Vector3d odom_pos_;
  Eigen::Vector3d odom_vel_;
  double odom_yaw_;

  // ---------- 返航执行 ----------
  int rtl_idx_;              // 当前正在飞向的面包屑索引(逆序)
  double arrival_threshold_; // 到达判定阈值
  bool rtl_active_;          // 返航是否激活
  ros::Time stop_sent_time_; // 发送 mandatory_stop 的时间
  ros::Time last_goal_sent_time_; // 上次发送 goal 的时间
  Eigen::Vector3d current_goal_;  // 当前正在飞向的目标(用于重发)

  // ---------- 参数 ----------
  double breadcrumb_dist_;     // 直道面包屑间距
  double heading_thresh_;      // 拐弯航向变化阈值(rad)
  double safe_hover_time_;     // 触发后悬停等待时间(s)
  bool auto_land_;             // 到达Home后是否自动降落

  // ---------- ROS 接口 ----------
  ros::NodeHandle node_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rtl_trigger_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher mandatory_stop_pub_;
  ros::Publisher takeoff_land_pub_;
  ros::Timer exec_timer_;

  // ---------- 回调 ----------
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void rtlTriggerCallback(const std_msgs::EmptyConstPtr &msg);
  void execCallback(const ros::TimerEvent &e);

  // ---------- 工具函数 ----------
  double normalizeAngle(double a);
  void recordBreadcrumb();
  void startRTL();
  void publishGoal(const Eigen::Vector3d &pos);
};

} // namespace rtlplanner

#endif
