#include <rtlplanner/rtl_planner.h>

namespace rtlplanner
{

RTLPlanner::RTLPlanner()
    : state_(IDLE),
      home_recorded_(false),
      rtl_idx_(0),
      rtl_active_(false)
{
  home_pos_ << 0.0, 0.0, 1.0;
  last_record_pos_ = home_pos_;
  last_record_yaw_ = 0.0;
  odom_pos_ = Eigen::Vector3d::Zero();
  odom_vel_ = Eigen::Vector3d::Zero();
  odom_yaw_ = 0.0;
}

RTLPlanner::~RTLPlanner() {}

void RTLPlanner::init(ros::NodeHandle &nh)
{
  node_ = nh;

  // ---------- 参数 ----------
  nh.param("rtl/breadcrumb_dist", breadcrumb_dist_, 6.0);
  nh.param("rtl/heading_thresh", heading_thresh_, 0.5);
  nh.param("rtl/arrival_threshold", arrival_threshold_, 0.5);
  nh.param("rtl/safe_hover_time", safe_hover_time_, 1.0);
  nh.param("rtl/auto_land", auto_land_, true);

  // ---------- 订阅 ----------
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &RTLPlanner::odomCallback, this);
  rtl_trigger_sub_ = nh.subscribe<std_msgs::Empty>("rtl_trigger", 1, &RTLPlanner::rtlTriggerCallback, this);

  // ---------- 发布 ----------
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal", 10);
  mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("mandatory_stop", 1);
  takeoff_land_pub_ = nh.advertise<mars_quadrotor_msgs::TakeoffLand>("takeoff_land", 1);

  // ---------- 定时器 ----------
  exec_timer_ = nh.createTimer(ros::Duration(0.05), &RTLPlanner::execCallback, this);

  ROS_INFO("[RTLPlanner] initialized. breadcrumb_dist=%.1f, heading_thresh=%.1f rad, arrival=%.1f",
           breadcrumb_dist_, heading_thresh_, arrival_threshold_);
}

// ==================== 回调 ====================

void RTLPlanner::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  // 从四元数提取航向角
  double w = msg->pose.pose.orientation.w;
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  odom_yaw_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  // Home点固定为 (0, 0, 1)
  if (!home_recorded_)
  {
    home_recorded_ = true;
    breadcrumbs_.push_back(home_pos_);
    ROS_INFO("[RTLPlanner] Home set to fixed point: (0.00, 0.00, 1.00)");
    return;
  }

  // 飞行中接近之前的面包屑 → 截断该点之后的面包屑（处理胡同掉头等场景）
  if (state_ == IDLE && breadcrumbs_.size() > 1)
  {
    for (size_t i = 0; i < breadcrumbs_.size() - 1; ++i)
    {
      double dist_to_bc = (odom_pos_ - breadcrumbs_[i]).norm();
      if (dist_to_bc < breadcrumb_dist_ * 0.5)
      {
        ROS_INFO("[RTLPlanner] Near breadcrumb %zu (dist=%.2f), truncating from %zu to %zu breadcrumbs.",
                 i, dist_to_bc, breadcrumbs_.size(), i + 1);
        breadcrumbs_.resize(i + 1);
        last_record_pos_ = odom_pos_;
        last_record_yaw_ = odom_yaw_;
        return;
      }
    }
  }

  // 只在IDLE状态记录面包屑
  if (state_ == IDLE)
    recordBreadcrumb();
}

void RTLPlanner::rtlTriggerCallback(const std_msgs::EmptyConstPtr &msg)
{
  if (state_ != IDLE)
  {
    ROS_WARN("[RTLPlanner] RTL already active, ignore trigger.");
    return;
  }
  if (breadcrumbs_.empty())
  {
    ROS_ERROR("[RTLPlanner] No breadcrumbs recorded, cannot RTL.");
    return;
  }

  ROS_INFO("[RTLPlanner] RTL triggered! %zu breadcrumbs recorded.", breadcrumbs_.size());
  startRTL();
}

// ==================== 定时器 ====================

void RTLPlanner::execCallback(const ros::TimerEvent &e)
{
  // RTL_STOPPING: 等待 diff_planner 退出 EMERGENCY_STOP 后再发 goal
  if (state_ == RTL_STOPPING)
  {
    double elapsed = (ros::Time::now() - stop_sent_time_).toSec();
    // 等待足够时间让 diff_planner 完成 EMERGENCY_STOP -> WAIT_TARGET 转换
    // diff_planner 需要 odom_vel < 0.1 才退出 EMERGENCY_STOP，然后清除 have_target_
    // 所以必须等它完全退出后再发 goal
    if (elapsed > safe_hover_time_ + 2.0)
    {
      state_ = RTL_EXECUTING;
      if (rtl_idx_ >= 0)
      {
        current_goal_ = breadcrumbs_[rtl_idx_];
        publishGoal(current_goal_);
        last_goal_sent_time_ = ros::Time::now();
        ROS_INFO("[RTLPlanner] RTL started. Flying to breadcrumb %d: (%.2f, %.2f, %.2f)",
                 rtl_idx_, current_goal_.x(), current_goal_.y(), current_goal_.z());
      }
    }
    return;
  }

  if (state_ != RTL_EXECUTING || !rtl_active_)
    return;

  // 定期重发当前 goal，防止 diff_planner 丢失目标
  if ((ros::Time::now() - last_goal_sent_time_).toSec() > 1.0)
  {
    publishGoal(current_goal_);
    last_goal_sent_time_ = ros::Time::now();
  }

  // 检查是否到达当前目标面包屑
  if (rtl_idx_ < 0)
  {
    // 所有面包屑走完，到达Home
    ROS_INFO("[RTLPlanner] Reached Home! RTL complete.");
    rtl_active_ = false;

    if (auto_land_)
    {
      mars_quadrotor_msgs::TakeoffLand land_msg;
      land_msg.takeoff_land_cmd = mars_quadrotor_msgs::TakeoffLand::LAND;
      takeoff_land_pub_.publish(land_msg);
      ROS_INFO("[RTLPlanner] Auto landing command sent.");
    }

    // 重置为 IDLE，允许用户再次起飞并触发返航
    breadcrumbs_.clear();
    breadcrumbs_.push_back(home_pos_);
    last_record_pos_ = home_pos_;
    last_record_yaw_ = 0.0;
    state_ = IDLE;
    ROS_INFO("[RTLPlanner] RTL reset to IDLE. Ready for next flight.");
    return;
  }

  Eigen::Vector3d target = breadcrumbs_[rtl_idx_];
  double dist = (target - odom_pos_).norm();

  if (dist < arrival_threshold_)
  {
    ROS_INFO("[RTLPlanner] Reached breadcrumb %d/%zu, moving to next.",
             rtl_idx_, breadcrumbs_.size() - 1);
    rtl_idx_--;

    if (rtl_idx_ >= 0)
    {
      current_goal_ = breadcrumbs_[rtl_idx_];
      publishGoal(current_goal_);
      last_goal_sent_time_ = ros::Time::now();
    }
    else
    {
      current_goal_ = home_pos_;
      publishGoal(home_pos_);  // 最后飞向Home
      last_goal_sent_time_ = ros::Time::now();
    }
  }
}

// ==================== 工具函数 ====================

double RTLPlanner::normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void RTLPlanner::recordBreadcrumb()
{
  double moved = (odom_pos_ - last_record_pos_).norm();
  double heading_diff = fabs(normalizeAngle(odom_yaw_ - last_record_yaw_));

  bool should_record = false;

  // 条件1：直道走够远 → 稀疏记录
  if (moved > breadcrumb_dist_)
    should_record = true;

  // 条件2：航向变化大 → 拐弯密集记录（保住转弯点）
  if (heading_diff > heading_thresh_)
    should_record = true;

  if (should_record)
  {
    breadcrumbs_.push_back(odom_pos_);
    last_record_pos_ = odom_pos_;
    last_record_yaw_ = odom_yaw_;
    ROS_INFO("[RTLPlanner] Breadcrumb %zu recorded: (%.2f, %.2f, %.2f) moved=%.1f heading_diff=%.1f",
             breadcrumbs_.size() - 1, odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
             moved, heading_diff);
  }
}

void RTLPlanner::startRTL()
{
  // 1. 发 mandatory_stop 让 diff_planner 停下来
  std_msgs::Empty stop_msg;
  mandatory_stop_pub_.publish(stop_msg);
  ROS_INFO("[RTLPlanner] Sent mandatory_stop to diff_planner.");

  // 2. 进入 RTL_STOPPING 状态，在 execCallback 中非阻塞等待
  stop_sent_time_ = ros::Time::now();
  state_ = RTL_STOPPING;
  rtl_active_ = true;

  // 3. 从最后一个面包屑开始逆序发送
  rtl_idx_ = (int)breadcrumbs_.size() - 1;
}

void RTLPlanner::publishGoal(const Eigen::Vector3d &pos)
{
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.header.frame_id = "world";
  goal_msg.pose.position.x = pos.x();
  goal_msg.pose.position.y = pos.y();
  goal_msg.pose.position.z = pos.z();
  goal_msg.pose.orientation.w = 1.0;
  goal_pub_.publish(goal_msg);
}

} // namespace rtlplanner

// ==================== main ====================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtl_planner");
  ros::NodeHandle nh("~");

  rtlplanner::RTLPlanner rtl;
  rtl.init(nh);

  ros::spin();
  return 0;
}
