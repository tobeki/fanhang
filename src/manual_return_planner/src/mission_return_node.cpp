#include "manual_return_planner/mission_return_planner.h"

#include <geometry_msgs/PoseStamped.h>
#include <mars_quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace mrp = manual_return_planner;

namespace {

double unwrapYaw(double value, double reference) {
  while (value - reference > M_PI) value -= 2.0 * M_PI;
  while (value - reference < -M_PI) value += 2.0 * M_PI;
  return value;
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  return tf::createQuaternionMsgFromYaw(yaw);
}

}  // namespace

// Mission-mode simulation executor.  It deliberately has its own node,
// service namespace, topics, and planner instance; manual_return_node remains
// unchanged and is disabled by the Mission launch file.
class MissionReturnNode {
 public:
  MissionReturnNode() : nh_(), private_nh_("~") {
    private_nh_.param("world_frame", world_frame_, std::string("world"));
    private_nh_.param("map_topic", map_topic_,
                      std::string("/map_generator/global_cloud"));
    private_nh_.param("odom_topic", odom_topic_,
                      std::string("/quad_0/lidar_slam/odom"));
    private_nh_.param("return_command_topic", return_command_topic_,
                      std::string("/manual_return/return_pos_cmd"));
    private_nh_.param("controller_output_enabled", output_enabled_, true);
    private_nh_.param("return_speed", return_speed_, 0.5);
    private_nh_.param("home_tolerance", home_tolerance_, 0.30);
    private_nh_.param("takeoff_yaw", takeoff_yaw_, 0.0);
    private_nh_.param("home_x", home_x_, 0.0);
    private_nh_.param("home_y", home_y_, 0.0);
    private_nh_.param("home_z", home_z_, 1.0);

    loadMissionWaypoints();

    mission_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "/mission_return/mission_path", 1, true);
    return_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "/mission_return/return_path", 1, true);
    key_points_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/mission_return/key_points", 1, true);
    home_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/mission_return/home_marker", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/mission_return/status", 1, true);
    command_pub_ = nh_.advertise<mars_quadrotor_msgs::PositionCommand>(
        return_command_topic_, 50);

    map_sub_ = nh_.subscribe(map_topic_, 1, &MissionReturnNode::mapCallback,
                             this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20,
                              &MissionReturnNode::odomCallback, this);
    trigger_srv_ = nh_.advertiseService(
        "/mission_return/trigger", &MissionReturnNode::triggerCallback, this);
    reset_srv_ = nh_.advertiseService(
        "/mission_return/reset", &MissionReturnNode::resetCallback, this);
    gate_return_client_ = nh_.serviceClient<std_srvs::Trigger>(
        "/manual_return/gate/set_manual_return");
    gate_normal_client_ = nh_.serviceClient<std_srvs::Trigger>(
        "/manual_return/gate/set_normal");

    timer_ = nh_.createTimer(ros::Duration(0.05),
                              &MissionReturnNode::executeCallback, this);
    publishMissionVisualization();
    publishStatus("READY");
    ROS_INFO_STREAM("[MissionReturn] loaded " << mission_waypoints_.size()
                                               << " ordered waypoints; whole-map='"
                                               << map_topic_ << "'");
  }

 private:
  enum class State { READY, RETURNING, FINISHED, FAILED };

  void loadMissionWaypoints() {
    int count = 0;
    private_nh_.param("waypoint_num", count, 0);
    if (count < 1) {
      ROS_WARN("[MissionReturn] waypoint_num is zero; trigger will be rejected");
      return;
    }
    count = std::min(count, 10000);
    mrp::MissionWaypoint home;
    home.position << home_x_, home_y_, home_z_;
    home.yaw = takeoff_yaw_;
    home.is_key_wp = true;
    home.title = "Home";
    mission_waypoints_.push_back(home);
    for (int i = 0; i < count; ++i) {
      mrp::MissionWaypoint point;
      const std::string prefix = "waypoint" + std::to_string(i);
      private_nh_.param(prefix + "_x", point.position.x(), 0.0);
      private_nh_.param(prefix + "_y", point.position.y(), 0.0);
      private_nh_.param(prefix + "_z", point.position.z(), home_z_);
      private_nh_.param(prefix + "_yaw", point.yaw, takeoff_yaw_);
      point.is_key_wp = true;
      point.title = prefix;
      mission_waypoints_.push_back(point);
    }
  }

  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*message, *cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    map_cloud_ = cloud;
    map_frame_ = message->header.frame_id;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *message;
    have_odom_ = true;
  }

  bool triggerCallback(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& response) {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr map;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == State::RETURNING) {
        response.success = false;
        response.message = "Mission RTL is already running";
        return true;
      }
      map = map_cloud_;
    }
    if (mission_waypoints_.size() < 2) {
      response.success = false;
      response.message = "no Mission waypoints loaded";
      publishStatus("FAILED");
      return true;
    }
    if (!map || map->empty()) {
      response.success = false;
      response.message = "whole-map is not available yet";
      publishStatus("FAILED_NO_WHOLE_MAP");
      return true;
    }

    mrp::MissionReturnConfig config;
    config.safe_radius = 0.40;
    config.fixed_return_yaw = true;
    const mrp::MissionReturnResult planned =
        planner_.plan(mission_waypoints_, map, config);
    if (planned.status != mrp::MissionReturnStatus::SUCCESS ||
        planned.return_waypoints.size() < 2) {
      response.success = false;
      response.message = planned.message;
      publishStatus("FAILED_NO_SAFE_PATH");
      ROS_ERROR_STREAM("[MissionReturn] planning failed: " << planned.message);
      return true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_ = planned;
      cumulative_lengths_.clear();
      cumulative_lengths_.push_back(0.0);
      for (std::size_t i = 1; i < result_.return_waypoints.size(); ++i) {
        cumulative_lengths_.push_back(
            cumulative_lengths_.back() +
            (result_.return_waypoints[i].position -
             result_.return_waypoints[i - 1].position).norm());
      }
      total_length_ = cumulative_lengths_.back();
      start_time_ = ros::Time::now();
      state_ = State::RETURNING;
    }
    publishReturnVisualization();
    publishStatus("RETURNING");

    if (output_enabled_) {
      std_srvs::Trigger gate_request;
      if (!gate_return_client_.call(gate_request) ||
          !gate_request.response.success) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = State::FAILED;
        response.success = false;
        response.message = "failed to switch CommandGate to MANUAL_RETURN";
        publishStatus("FAILED_GATE");
        return true;
      }
    }
    response.success = true;
    response.message = "Mission RTL planned and CommandGate switched";
    ROS_INFO_STREAM("[MissionReturn] " << response.message
                                       << "; optimized points="
                                       << result_.optimized_points
                                       << ", map segments="
                                       << result_.map_checked_segments
                                       << ", safe radius=0.40 m");
    return true;
  }

  bool resetCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    if (output_enabled_) {
      std_srvs::Trigger request;
      if (!gate_normal_client_.call(request) || !request.response.success) {
        response.success = false;
        response.message = "failed to switch CommandGate to NORMAL";
        return true;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::READY;
    result_ = mrp::MissionReturnResult();
    publishStatus("READY");
    response.success = true;
    response.message = "Mission RTL reset";
    return true;
  }

  void executeCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::RETURNING && state_ != State::FINISHED) return;
    if (result_.return_waypoints.size() < 2) return;

    double s = total_length_;
    if (state_ == State::RETURNING) {
      s = std::min(total_length_,
                   std::max(0.0, (ros::Time::now() - start_time_).toSec()) *
                       std::max(0.05, return_speed_));
    }
    Eigen::Vector3d position = result_.return_waypoints.back().position;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double yaw = result_.return_waypoints.back().yaw;
    if (s < total_length_) {
      std::size_t segment = 0;
      while (segment + 1 < cumulative_lengths_.size() &&
             cumulative_lengths_[segment + 1] < s)
        ++segment;
      const double begin = cumulative_lengths_[segment];
      const double end = cumulative_lengths_[segment + 1];
      const double ratio = (end - begin) > 1e-9 ? (s - begin) / (end - begin) : 1.0;
      const auto& a = result_.return_waypoints[segment];
      const auto& b = result_.return_waypoints[segment + 1];
      position = a.position + ratio * (b.position - a.position);
      velocity = (b.position - a.position).normalized() * std::max(0.05, return_speed_);
      yaw = unwrapYaw(a.yaw + ratio * unwrapYaw(b.yaw, a.yaw), a.yaw);
    }
    publishCommand(position, velocity, yaw);
    if (state_ == State::RETURNING && s >= total_length_) {
      bool arrived = true;
      if (have_odom_) {
        const Eigen::Vector3d actual(latest_odom_.pose.pose.position.x,
                                     latest_odom_.pose.pose.position.y,
                                     latest_odom_.pose.pose.position.z);
        arrived = (actual - position).norm() <= home_tolerance_;
      }
      if (arrived) {
        state_ = State::FINISHED;
        publishStatus("HOME_HOLD_TAKEOFF_YAW");
        ROS_INFO("[MissionReturn] Home reached; holding takeoff yaw before landing");
      }
    }
  }

  void publishCommand(const Eigen::Vector3d& position,
                      const Eigen::Vector3d& velocity, double yaw) {
    if (!output_enabled_) return;
    mars_quadrotor_msgs::PositionCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = world_frame_;
    command.trajectory_id = 110000;
    command.trajectory_flag =
        mars_quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    command.position.x = position.x();
    command.position.y = position.y();
    command.position.z = position.z();
    command.velocity.x = velocity.x();
    command.velocity.y = velocity.y();
    command.velocity.z = velocity.z();
    command.acceleration.x = command.acceleration.y = command.acceleration.z = 0.0;
    command.jerk.x = command.jerk.y = command.jerk.z = 0.0;
    command.yaw = yaw;
    command.yaw_dot = 0.0;
    command_pub_.publish(command);
  }

  geometry_msgs::PoseStamped pose(const Eigen::Vector3d& position,
                                  double yaw) const {
    geometry_msgs::PoseStamped result;
    result.header.frame_id = world_frame_;
    result.header.stamp = ros::Time::now();
    result.pose.position.x = position.x();
    result.pose.position.y = position.y();
    result.pose.position.z = position.z();
    result.pose.orientation = yawQuaternion(yaw);
    return result;
  }

  void publishMissionVisualization() {
    nav_msgs::Path path;
    path.header.frame_id = world_frame_;
    for (const auto& point : mission_waypoints_)
      path.poses.push_back(pose(point.position, point.yaw));
    mission_path_pub_.publish(path);

    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "mission_return_key_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.18;
    marker.color.r = 1.0;
    marker.color.g = 0.65;
    marker.color.a = 1.0;
    for (const auto& point : mission_waypoints_) {
      geometry_msgs::Point p;
      p.x = point.position.x();
      p.y = point.position.y();
      p.z = point.position.z();
      marker.points.push_back(p);
    }
    key_points_pub_.publish(marker);

    visualization_msgs::Marker home;
    home.header.frame_id = world_frame_;
    home.header.stamp = ros::Time::now();
    home.ns = "mission_return_home";
    home.id = 0;
    home.type = visualization_msgs::Marker::ARROW;
    home.action = visualization_msgs::Marker::ADD;
    home.pose.position.x = home_x_;
    home.pose.position.y = home_y_;
    home.pose.position.z = home_z_;
    home.pose.orientation = yawQuaternion(takeoff_yaw_);
    home.scale.x = 0.6;
    home.scale.y = 0.08;
    home.scale.z = 0.08;
    home.color.g = 1.0;
    home.color.a = 1.0;
    home_pub_.publish(home);
  }

  void publishReturnVisualization() {
    nav_msgs::Path path;
    path.header.frame_id = world_frame_;
    for (const auto& point : result_.return_waypoints)
      path.poses.push_back(pose(point.position, point.yaw));
    return_path_pub_.publish(path);
  }

  void publishStatus(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    status_pub_.publish(message);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher mission_path_pub_, return_path_pub_, key_points_pub_, home_pub_;
  ros::Publisher status_pub_, command_pub_;
  ros::Subscriber map_sub_, odom_sub_;
  ros::ServiceServer trigger_srv_, reset_srv_;
  ros::ServiceClient gate_return_client_, gate_normal_client_;
  ros::Timer timer_;

  std::mutex mutex_;
  State state_ = State::READY;
  mrp::MissionReturnPlanner planner_;
  mrp::MissionReturnResult result_;
  std::vector<mrp::MissionWaypoint> mission_waypoints_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr map_cloud_;
  nav_msgs::Odometry latest_odom_;
  bool have_odom_ = false;
  std::string world_frame_, map_topic_, odom_topic_, return_command_topic_, map_frame_;
  bool output_enabled_ = true;
  double return_speed_ = 0.5;
  double home_tolerance_ = 0.30;
  double takeoff_yaw_ = 0.0;
  double home_x_ = 0.0, home_y_ = 0.0, home_z_ = 1.0;
  ros::Time start_time_;
  double total_length_ = 0.0;
  std::vector<double> cumulative_lengths_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_return_node");
  MissionReturnNode node;
  ros::spin();
  return 0;
}
