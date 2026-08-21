#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <mars_quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

mars_quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

int yaw_mode = 0;  // 0=到点后旋转yaw, 1=途中保持yaw角飞行

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
double yaw_custom_;
double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;

bool receive_yaw_ = false;
ros::Time receive_yaw_time_(0);
Eigen::Vector3d yaw_pos_;       // hover position carried in yaw message
bool arrival_yaw_aligned_ = false;
ros::Time traj_finish_time_(0);  // when trajectory ended, for timed hover before releasing to AUTO_HOVER

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void yawCallback(const mars_quadrotor_msgs::PositionCommandPtr msg)
{
  receive_yaw_ = true;
  receive_yaw_time_ = ros::Time::now();
  yaw_custom_ = msg->yaw;
  yaw_pos_(0) = msg->position.x;
  yaw_pos_(1) = msg->position.y;
  yaw_pos_(2) = msg->position.z;
  arrival_yaw_aligned_ = false;  // new yaw command, re-trigger arrival alignment
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  arrival_yaw_aligned_ = false;  // new trajectory: allow arrival yaw alignment
  traj_finish_time_ = ros::Time(0);  // reset hover timer
  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt, bool use_custom_yaw=false)
{
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double xy_norm = sqrt(dir(0) * dir(0) + dir(1) * dir(1));
  double yaw_temp = xy_norm > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;
  if (use_custom_yaw && receive_yaw_ && yaw_custom_ > -100.0)
  {
    if (yaw_mode == 1)
    {
      // 途中旋转模式：需要检查超时，防止yaw指令中断后继续用旧值
      if ((ros::Time::now() - receive_yaw_time_).toSec() < 0.5)
        yaw_temp = yaw_custom_;
      else
        receive_yaw_ = false;
    }
    else
    {
      // 到点旋转模式：直接使用，无超时检查
      yaw_temp = yaw_custom_;
    }
  }

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = mars_quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
  {
    // Timed hover after trajectory ends: keep publishing for a few seconds
    // to stabilize yaw, then stop to allow AUTO_HOVER transition.
    // BUT: if a new yaw arrived AFTER the trajectory finished, it means
    // pre-alignment is active — skip the timed hover and fall through to the
    // yaw-rotation block below (yaw_pos_ was just updated, not stale).
    if (traj_finish_time_.toSec() > 0.01)
    {
      bool new_yaw_for_prealign = receive_yaw_ && (receive_yaw_time_ - traj_finish_time_).toSec() > 0;
      if (!new_yaw_for_prealign)
      {
        ros::Time now = ros::Time::now();
        if ((now - traj_finish_time_).toSec() < 1.0)
        {
          publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
        }
        return;
      }
      // new yaw for pre-alignment: fall through
    }

    // No trajectory active, but yaw command received: hover at position from yaw message
    // and smoothly rotate yaw. This supports pre-alignment from multipointplan.
    if (receive_yaw_ && yaw_custom_ > -100.0 && yaw_pos_.norm() > 0.01)
    {
      static ros::Time yaw_only_time_last = ros::Time::now();
      ros::Time now = ros::Time::now();
      double dt = (now - yaw_only_time_last).toSec();
      if (dt <= 0.0 || dt > 0.1) dt = 0.01;
      yaw_only_time_last = now;

      double d_yaw = yaw_custom_ - last_yaw_;
      if (d_yaw >= M_PI) d_yaw -= 2 * M_PI;
      if (d_yaw <= -M_PI) d_yaw += 2 * M_PI;

      const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
      const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
      double d_yaw_max;
      if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
        d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
      else
      {
        double t1 = (YDM - last_yawdot_) / YDDM;
        d_yaw_max = (t1 + dt) * (YDM - last_yawdot_) / 2.0;
      }

      if (fabs(d_yaw) > fabs(d_yaw_max))
        d_yaw = d_yaw_max;
      double yawdot = d_yaw / dt;

      double yaw = last_yaw_ + d_yaw;
      if (yaw > M_PI) yaw -= 2 * M_PI;
      if (yaw < -M_PI) yaw += 2 * M_PI;

      last_yaw_ = yaw;
      last_yawdot_ = yawdot;
      last_pos_ = yaw_pos_;

      publish_cmd(yaw_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), yaw, yawdot);
    }
    return;
  }

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    arrival_yaw_aligned_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  static bool finished = false;

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    // yaw_mode==0: use_custom_yaw=false, 面朝运动方向飞行
    // yaw_mode==1: use_custom_yaw=true,  保持目标yaw角飞行
    yaw_yawdot = calculate_yaw(t_cur, pos, 0.01, yaw_mode == 1);
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
    finished = false;
  }

  // --- Arrival yaw alignment: after trajectory finishes (or pre-alignment),   ---
  // --- rotate in place to waypoint yaw.                                       ---
  else if (t_cur >= traj_duration_ && receive_yaw_ && yaw_custom_ > -100.0 && !arrival_yaw_aligned_)
  {
    pos = last_pos_;
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double dt = (time_now - time_last).toSec();
    if (dt <= 0.0 || dt > 0.1) dt = 0.01;

    double d_yaw = yaw_custom_ - last_yaw_;
    if (d_yaw >= M_PI) d_yaw -= 2 * M_PI;
    if (d_yaw <= -M_PI) d_yaw += 2 * M_PI;

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
      d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
    else
    {
      double t1 = (YDM - last_yawdot_) / YDDM;
      d_yaw_max = (t1 + dt) * (YDM - last_yawdot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
      d_yaw = d_yaw_max;
    double yawdot = d_yaw / dt;

    double yaw = last_yaw_ + d_yaw;
    if (yaw > M_PI) yaw -= 2 * M_PI;
    if (yaw < -M_PI) yaw += 2 * M_PI;

    last_yaw_ = yaw;
    last_yawdot_ = yawdot;
    last_pos_ = pos;

    double yaw_err = fabs(yaw_custom_ - yaw);
    if (yaw_err > M_PI) yaw_err = 2 * M_PI - yaw_err;
    if (yaw_err < 0.05 && fabs(yawdot) < 0.05)  // ~3 deg threshold
    {
      arrival_yaw_aligned_ = true;
      ROS_INFO("[traj_server] Arrival yaw alignment complete, yaw=%.2f", yaw);
    }

    publish_cmd(pos, vel, acc, jer, yaw, yawdot);
  }
  // No custom yaw received: hover at final position with current yaw for a few
  // seconds, then release so px4ctrl can transition to AUTO_HOVER for landing.
  else if (t_cur >= traj_duration_)
  {
    if (traj_finish_time_.toSec() < 0.01)
      traj_finish_time_ = time_now;

    pos = last_pos_;
    vel.setZero();
    acc.setZero();
    jer.setZero();

    // Publish hover for 1 second to stabilize yaw, then stop
    if ((time_now - traj_finish_time_).toSec() < 1.0)
    {
      publish_cmd(pos, vel, acc, jer, last_yaw_, 0);
    }
    else
    {
      receive_traj_ = false;
    }
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber yaw_sub = nh.subscribe("/planning/yaw", 10, yawCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  pos_cmd_pub = nh.advertise<mars_quadrotor_msgs::PositionCommand>("position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/yaw_mode", yaw_mode, 0);  // 0=到点旋转, 1=途中保持yaw
  nh.param("traj_server/yaw_dot_max", YAW_DOT_MAX_PER_SEC, YAW_DOT_MAX_PER_SEC);
  nh.param("traj_server/yaw_dot_dot_max", YAW_DOT_DOT_MAX_PER_SEC, YAW_DOT_DOT_MAX_PER_SEC);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}