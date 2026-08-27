#include "manual_return_planner/manual_return_planner.h"
#include "manual_return_planner/trajectory_analysis/trajectory_analysis_manager.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mars_quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <ctime>
#include <vector>

namespace mrp = manual_return_planner;

namespace {

// Smallest signed angular difference in (-pi, pi].
double unwrapYawDelta(double target, double current) {
  double delta = target - current;
  while (delta > M_PI) delta -= 2.0 * M_PI;
  while (delta < -M_PI) delta += 2.0 * M_PI;
  return delta;
}

// Conservative trapezoidal velocity profile along the polyline arc length.
//
// The profile is TIME-driven: the reference speed is v = accel * t during the
// acceleration ramp, and the reference arc length is the integral of that
// speed.  This avoids the start deadlock of an arc-length-based profile where
// the allowed speed is zero at s = 0 (so s never advances and the vehicle
// never moves).  It accelerates from zero, cruises at return_cruise_speed, and
// decelerates to zero at Home; short paths use a symmetric triangular profile.
struct SpeedProfile {
  double total_length = 0.0;
  double cruise_speed = 0.0;
  double accel = 0.0;
  double accel_time = 0.0;      // duration of the acceleration ramp [s]
  double accel_distance = 0.0;  // distance covered during the ramp [m]
  double total_time = 0.0;      // total profile duration [s]
};

SpeedProfile makeSpeedProfile(double total_length, double cruise, double accel) {
  SpeedProfile profile;
  profile.total_length = total_length;
  profile.accel = std::max(0.01, accel);

  const double cruise_requested = std::max(0.01, cruise);
  const double ramp_distance =
      cruise_requested * cruise_requested / (2.0 * profile.accel);

  if (2.0 * ramp_distance >= total_length) {
    // Triangular profile: the path is too short to reach cruise speed.
    const double peak_speed = std::sqrt(profile.accel * total_length);
    profile.cruise_speed = peak_speed;
    profile.accel_time = peak_speed / profile.accel;
    profile.accel_distance = total_length / 2.0;
    profile.total_time = 2.0 * profile.accel_time;
  } else {
    // Trapezoidal profile: accelerate -> cruise -> decelerate.
    profile.cruise_speed = cruise_requested;
    profile.accel_time = cruise_requested / profile.accel;
    profile.accel_distance = ramp_distance;
    const double cruise_distance = total_length - 2.0 * ramp_distance;
    const double cruise_time = cruise_distance / profile.cruise_speed;
    profile.total_time = 2.0 * profile.accel_time + cruise_time;
  }
  return profile;
}

// Evaluate the profile at elapsed time t, returning reference arc length and
// speed.  v = accel * t is > 0 as soon as t > 0, so the reference advances.
void profileStateAt(double t, const SpeedProfile& p, double* s, double* v) {
  if (t <= 0.0) {
    *s = 0.0;
    *v = 0.0;
    return;
  }
  if (t >= p.total_time) {
    *s = p.total_length;
    *v = 0.0;
    return;
  }
  if (t < p.accel_time) {
    *v = p.accel * t;
    *s = 0.5 * p.accel * t * t;
  } else if (t < p.total_time - p.accel_time) {
    *v = p.cruise_speed;
    *s = p.accel_distance + p.cruise_speed * (t - p.accel_time);
  } else {
    const double remaining = p.total_time - t;
    *v = p.accel * remaining;
    *s = p.total_length - 0.5 * p.accel * remaining * remaining;
  }
}

enum class ReturnPhase {
  TAKEOVER = 0,
  ACCELERATION = 1,
  CRUISE = 2,
  TURN = 3,
  DECELERATION = 4,
  HOME_HOLD = 5
};

const char* phaseToString(ReturnPhase phase) {
  switch (phase) {
    case ReturnPhase::TAKEOVER: return "TAKEOVER";
    case ReturnPhase::ACCELERATION: return "ACCELERATION";
    case ReturnPhase::CRUISE: return "CRUISE";
    case ReturnPhase::TURN: return "TURN";
    case ReturnPhase::DECELERATION: return "DECELERATION";
    case ReturnPhase::HOME_HOLD: return "HOME_HOLD";
  }
  return "UNKNOWN";
}

double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  if (values.size() == 1) return values[0];
  std::sort(values.begin(), values.end());
  const double pos = p * (values.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
  if (lo == hi) return values[lo];
  const double frac = pos - static_cast<double>(lo);
  return values[lo] * (1.0 - frac) + values[hi] * frac;
}

struct TrackingStats {
  double mean = 0.0;
  double p50 = 0.0;
  double p90 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double max = 0.0;
};

TrackingStats computeStats(const std::vector<double>& values) {
  TrackingStats stats;
  if (values.empty()) return stats;
  double sum = 0.0;
  for (double v : values) sum += v;
  stats.mean = sum / values.size();
  stats.p50 = percentile(values, 0.50);
  stats.p90 = percentile(values, 0.90);
  stats.p95 = percentile(values, 0.95);
  stats.p99 = percentile(values, 0.99);
  stats.max = *std::max_element(values.begin(), values.end());
  return stats;
}

std::string makeRunId() {
  std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
  return std::string(buf);
}

}  // namespace

class ManualReturnNode {
 public:
  ManualReturnNode() : nh_(), private_nh_("~"), state_(RECORDING) {
    private_nh_.param("world_frame", world_frame_, std::string("world"));
    config_.world_frame = world_frame_;
    private_nh_.param("odom_topic", odom_topic_,
                      std::string("/quad_0/lidar_slam/odom"));
    private_nh_.param("map_topic", map_topic_,
                      std::string("/map_generator/global_cloud"));
    private_nh_.param("record_frequency", record_frequency_, 10.0);
    private_nh_.param("min_point_spacing", config_.min_point_spacing, 0.03);
    private_nh_.param("rdp_epsilon", config_.rdp_epsilon, 0.05);
    private_nh_.param("max_segment_length", config_.max_segment_length, 5.0);
    private_nh_.param("max_reasonable_speed", config_.max_reasonable_speed,
                      2.25);
    private_nh_.param("vertical_preserve_threshold",
                      config_.vertical_preserve_threshold, 0.05);
    private_nh_.param("corridor_radius", config_.corridor_radius, 0.5);
    private_nh_.param("corridor_check_step", config_.corridor_check_step, 0.2);
    private_nh_.param("min_length_ratio", config_.min_length_ratio, 0.85);
    private_nh_.param("vehicle_body_radius", config_.vehicle_body_radius, 0.39);
    private_nh_.param("localization_margin", config_.localization_margin, 0.05);
    private_nh_.param("tracking_margin", config_.tracking_margin, 0.05);
    private_nh_.param("map_margin", config_.map_margin, 0.05);
    private_nh_.param("extra_safety_margin", config_.extra_safety_margin, 0.05);
    private_nh_.param("collision_check_step", config_.collision_check_step,
                      0.1);
    private_nh_.param("safe_rdp/enabled", config_.safe_rdp.enabled, true);
    private_nh_.param("safe_rdp/uav_radius", config_.safe_rdp.uav_radius,
                      0.25);
    private_nh_.param("safe_rdp/tracking_margin",
                      config_.safe_rdp.tracking_margin, 0.15);
    private_nh_.param("safe_rdp/extra_margin", config_.safe_rdp.extra_margin,
                      0.10);
    private_nh_.param("safe_rdp/voxel_resolution",
                      config_.safe_rdp.voxel_resolution, 0.05);
    private_nh_.param("safe_rdp/collision_check_resolution",
                      config_.safe_rdp.collision_check_resolution, 0.05);
    private_nh_.param("trajectory_analysis/segment_angle_threshold_deg",
                      analysis_config_.segment_angle_threshold_deg, 30.0);
    private_nh_.param("trajectory_analysis/segment_speed_change_threshold",
                      analysis_config_.segment_speed_change_threshold, 0.5);
    private_nh_.param("trajectory_analysis/dwell_max_speed",
                      analysis_config_.dwell_max_speed, 0.1);
    private_nh_.param("trajectory_analysis/dwell_radius",
                      analysis_config_.dwell_radius, 0.15);
    private_nh_.param("trajectory_analysis/dwell_min_time",
                      analysis_config_.dwell_min_time, 2.0);
    private_nh_.param("trajectory_analysis/backtrack_epsilon_entry",
                      analysis_config_.backtrack_epsilon_entry, 0.5);
    private_nh_.param("trajectory_analysis/backtrack_min_spur_ratio",
                      analysis_config_.backtrack_min_spur_ratio, 3.0);
    private_nh_.param("trajectory_analysis/backtrack_min_spur_reach",
                      analysis_config_.backtrack_min_spur_reach, 0.5);
    private_nh_.param("trajectory_analysis/backtrack_apex_revisit_radius",
                      analysis_config_.backtrack_apex_revisit_radius, 0.5);
    private_nh_.param("trajectory_analysis/overlap_distance_threshold",
                      analysis_config_.overlap_distance_threshold, 1.2);
    private_nh_.param("trajectory_analysis/overlap_angle_threshold_deg",
                      analysis_config_.overlap_angle_threshold_deg, 30.0);
    private_nh_.param("trajectory_analysis/overlap_min_segment_length",
                      analysis_config_.overlap_min_segment_length, 0.5);
    private_nh_.param("trajectory_analysis/overlap_min_fraction",
                      analysis_config_.overlap_min_fraction, 0.5);
    private_nh_.param("return_cruise_speed", return_speed_, 0.5);
    config_.return_cruise_speed = return_speed_;
    private_nh_.param("max_return_acceleration", max_return_accel_, 1.5);
    config_.max_return_acceleration = max_return_accel_;
    private_nh_.param("home_position_tolerance", home_tolerance_, 0.3);
    config_.home_position_tolerance = home_tolerance_;
    private_nh_.param("landing_handoff_height",
                      config_.landing_handoff_height, 0.25);
    private_nh_.param("return_finish_speed_threshold",
                      return_finish_speed_threshold_, 0.15);
    private_nh_.param("control_takeover_delay", takeover_delay_, 1.5);
    private_nh_.param("command_frequency", command_frequency_, 100.0);
    private_nh_.param("turn_angle_threshold_deg", turn_angle_threshold_deg_,
                      20.0);
    private_nh_.param("turn_analysis_distance", turn_analysis_distance_, 0.5);
    private_nh_.param("yaw_mode", yaw_mode_, std::string("waypoint_hold"));
    private_nh_.param("yaw_transition_radius", yaw_transition_radius_, 0.2);
    private_nh_.param("enable_return_command_output", enable_command_output_,
                      false);
    private_nh_.param("return_command_topic", return_command_topic_,
                      std::string("/manual_return/return_pos_cmd"));
    private_nh_.param("gate_set_manual_return_service",
                      gate_set_manual_return_service_,
                      std::string("/manual_return/gate/set_manual_return"));
    private_nh_.param("gate_set_normal_service", gate_set_normal_service_,
                      std::string("/manual_return/gate/set_normal"));
    private_nh_.param("trajectory_csv", trajectory_csv_, std::string(""));
    private_nh_.param("output_dir", output_dir_,
                      std::string("/tmp/manual_return"));
    private_nh_.param("scenario", scenario_, std::string("unknown"));
    nh_.param("/manual_return/scenario", scenario_, scenario_);
    private_nh_.param("offline_csv_test", offline_csv_test_, false);
    config_.record_frequency = record_frequency_;
    private_nh_.param("pcd_path", pcd_path_, std::string(""));
    private_nh_.param("require_control_takeover", require_control_takeover_,
                      false);
    if (!output_dir_.empty()) {
      ::mkdir(output_dir_.c_str(), 0755);
    }
    run_output_dir_ = output_dir_;

    raw_pub_ = nh_.advertise<nav_msgs::Path>("/manual_return/raw_path", 1, true);
    history_pub_ =
        nh_.advertise<nav_msgs::Path>("/manual_return/history_path", 1, true);
    rdp_line_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/manual_return/return_path_line", 1, true);
    key_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/manual_return/key_points", 1, true);
    home_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/manual_return/home_marker", 1, true);
    safe_path_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/manual_return/safe_return_path", 1, true);
    yaw_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/manual_return/yaw_marker", 1, true);
    analysis_marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(
            "/manual_return/trajectory_analysis_marker", 1, true);
    preprocessed_pub_ =
        nh_.advertise<nav_msgs::Path>("/manual_return/preprocessed_path", 1,
                                      true);
    reversed_pub_ =
        nh_.advertise<nav_msgs::Path>("/manual_return/reversed_path", 1, true);
    rdp_pub_ = nh_.advertise<nav_msgs::Path>("/manual_return/rdp_path", 1, true);
    home_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/manual_return/home", 1,
                                                  true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/manual_return/map_cloud",
                                                       1, true);
    status_pub_ =
        nh_.advertise<std_msgs::String>("/manual_return/status", 1, true);
    // The executor no longer publishes to the controller topic.  It publishes
    // to the gate's return input; CommandGate is the sole publisher of
    // /quad_0/planning/pos_cmd.
    command_pub_ = nh_.advertise<mars_quadrotor_msgs::PositionCommand>(
        return_command_topic_, 50);
    mandatory_stop_pub_ =
        nh_.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 1, true);
    odom_sub_ = nh_.subscribe(odom_topic_, 50, &ManualReturnNode::odomCallback,
                              this);
    map_sub_ = nh_.subscribe(map_topic_, 1, &ManualReturnNode::mapCallback,
                             this);
    trigger_service_ = nh_.advertiseService(
        "/manual_return/trigger", &ManualReturnNode::triggerCallback, this);
    reset_service_ = nh_.advertiseService(
        "/manual_return/reset", &ManualReturnNode::resetCallback, this);
    gate_manual_client_ = nh_.serviceClient<std_srvs::Trigger>(
        gate_set_manual_return_service_);
    gate_normal_client_ =
        nh_.serviceClient<std_srvs::Trigger>(gate_set_normal_service_);

    record_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, record_frequency_)),
        &ManualReturnNode::recordTimer, this);
    execute_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, command_frequency_)),
        &ManualReturnNode::executeTimer, this);

    if (!pcd_path_.empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(
          new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *pcd) == 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_cloud_ = pcd;
        sensor_msgs::PointCloud2 map_message;
        pcl::toROSMsg(*pcd, map_message);
        map_message.header.frame_id = world_frame_;
        map_message.header.stamp = ros::Time::now();
        map_pub_.publish(map_message);
        ROS_INFO_STREAM("[ManualReturn] loaded PCD " << pcd_path_ << " ("
                        << pcd->size() << " points) in declared frame "
                        << world_frame_);
      } else {
        ROS_ERROR_STREAM("[ManualReturn] cannot load PCD: " << pcd_path_);
      }
    }

    if (!trajectory_csv_.empty()) {
      std::string error;
      if (mrp::CsvTrajectoryReader::read(trajectory_csv_, &history_, &error)) {
        publishRaw(history_);
        ROS_INFO_STREAM("[ManualReturn] loaded " << history_.size()
                        << " points from " << trajectory_csv_);
        if (offline_csv_test_) planFrozenTrajectory(history_);
      } else {
        ROS_ERROR_STREAM("[ManualReturn] " << error);
      }
    }
    ROS_INFO_STREAM("[ManualReturn] recording " << odom_topic_ << " at "
                    << record_frequency_ << " Hz in frame " << world_frame_);
    ROS_INFO_STREAM("[ManualReturn] return command output -> "
                    << return_command_topic_ << " (gated by CommandGate)");
    publishStatus("RECORDING");
  }

 private:
  enum State {
    RECORDING,
    PLANNING,
    WAITING_FOR_TAKEOVER,
    RETURNING,
    FINISHED,
    FAILED
  };

  struct TrackingSample {
    double timestamp = 0.0;
    Eigen::Vector3d reference = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual = Eigen::Vector3d::Zero();
    // V1.0.2 spatial decomposition fields.
    Eigen::Vector3d projection = Eigen::Vector3d::Zero();
    int matched_segment = -1;
    double reference_progress_s = 0.0;
    double actual_progress_s = 0.0;
    double along_track_error = 0.0;  // s_ref - s_actual (positive = lagging)
    double cross_track_3d = 0.0;
    double cross_track_xy = 0.0;
    double vertical_error = 0.0;
    ReturnPhase phase = ReturnPhase::CRUISE;
  };

  struct YawSample {
    double timestamp = 0.0;
    double current_x = 0.0, current_y = 0.0, current_z = 0.0;
    double current_yaw = 0.0;
    double target_yaw = 0.0;
    double yaw_error = 0.0;
    int segment_id = -1;
    std::string yaw_mode;
  };

  mrp::TrajectoryPoint odomToPoint(const nav_msgs::Odometry& odom) const {
    mrp::TrajectoryPoint point;
    // Use the monotonic ROS clock for every sample instead of the odometry
    // header stamp.  The odometry stamp's clock basis is unstable in this
    // simulator: it is sometimes zeroed and sometimes carries simulation time,
    // and the simulation clock itself jumps backwards (TF "jump back in time"),
    // which would make the recorded history's timestamps non-monotonic and trip
    // the strict order check in preprocessing.
    point.timestamp = ros::Time::now().toSec();
    point.position = Eigen::Vector3d(odom.pose.pose.position.x,
                                     odom.pose.pose.position.y,
                                     odom.pose.pose.position.z);
    point.velocity = Eigen::Vector3d(odom.twist.twist.linear.x,
                                     odom.twist.twist.linear.y,
                                     odom.twist.twist.linear.z);
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(point.roll, point.pitch, point.yaw);
    return point;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (message->header.frame_id != world_frame_) {
      frame_error_ = true;
      ROS_WARN_THROTTLE(
          2.0,
          "[ManualReturn] odometry frame '%s' does not match world_frame '%s'",
          message->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    latest_odom_ = *message;
    have_odom_ = true;
  }

  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (message->header.frame_id != world_frame_) {
      ROS_WARN_THROTTLE(
          2.0, "[ManualReturn] ignoring map in frame '%s'; expected '%s'",
          message->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*message, *cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    map_cloud_ = cloud;
    map_pub_.publish(*message);
  }

  void recordTimer(const ros::TimerEvent& event) {
    (void)event;
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != RECORDING || !have_odom_) return;
    mrp::TrajectoryPoint point = odomToPoint(latest_odom_);
    // Belt-and-suspenders: keep the history strictly increasing even if the
    // system clock returns the same instant for consecutive 10 Hz ticks.
    if (!history_.empty() && point.timestamp <= history_.back().timestamp) {
      point.timestamp = history_.back().timestamp + 1e-6;
    }
    history_.push_back(point);
    publishRaw(history_);
    history_pub_.publish(makePath(history_));
  }

  bool triggerCallback(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& response) {
    std::vector<mrp::TrajectoryPoint> frozen;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ != RECORDING) {
        response.success = false;
        response.message = "manual return is not in RECORDING state";
        return true;
      }
      if (frame_error_) {
        response.success = false;
        response.message = "odometry frame does not match world_frame";
        state_ = FAILED;
        publishStatus("FAILED");
        return true;
      }
      if (require_control_takeover_ && !enable_command_output_) {
        response.success = false;
        response.message =
            "control takeover is required but command output is disabled";
        state_ = FAILED;
        publishStatus("FAILED");
        return true;
      }
      // Step 1/3: capture the latest odometry as the trigger-time anchor.  The
      // 10 Hz history may lag the true trigger pose by up to ~100 ms.
      if (have_odom_) {
        trigger_position_ = Eigen::Vector3d(latest_odom_.pose.pose.position.x,
                                            latest_odom_.pose.pose.position.y,
                                            latest_odom_.pose.pose.position.z);
        trigger_velocity_ = Eigen::Vector3d(latest_odom_.twist.twist.linear.x,
                                            latest_odom_.twist.twist.linear.y,
                                            latest_odom_.twist.twist.linear.z);
        trigger_odom_valid_ = true;
      }
      state_ = PLANNING;
      publishStatus("PLANNING");
      frozen = history_;
    }

    // V1.0.2: give each run its own output subdirectory so successive runs do
    // not overwrite each other's tracking CSVs.
    run_id_ = makeRunId();
    run_output_dir_ = output_dir_.empty() ? "" : (output_dir_ + "/" + run_id_);
    if (!run_output_dir_.empty()) {
      ::mkdir(run_output_dir_.c_str(), 0755);
      ROS_INFO_STREAM("[ManualReturn] output dir: " << run_output_dir_);
    }

    // Step 3: force the trigger pose to be the final history point so that,
    // after reversal, it is the return start anchor.
    if (trigger_odom_valid_) {
      mrp::TrajectoryPoint anchor = odomToPoint(latest_odom_);
      // The odometry stamp can come from a different clock basis than the
      // recorded samples (zeroed vs sim-time headers), which would trip the
      // strict timestamp-order check in preprocessing.  The anchor's meaning
      // is purely spatial (the return start), so force a strictly increasing
      // timestamp just past the last recorded sample.
      if (!frozen.empty() && anchor.timestamp <= frozen.back().timestamp) {
        anchor.timestamp = frozen.back().timestamp + 1e-6;
      }
      frozen.push_back(anchor);
    }

    // V2.0: analyze the forward history (Home -> current).  Detection
    // only; the return path is never modified here.
    analyzeTrajectory(frozen);

    if (!planFrozenTrajectory(frozen)) {
      response.success = false;
      response.message = last_result_.message;
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = FAILED;
      publishStatus("FAILED");
      return true;
    }

    // Step 13: basic sanity checks before any control switch.
    if (last_result_.return_waypoints.size() < 2) {
      response.success = false;
      response.message = "return path has fewer than 2 waypoints";
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = FAILED;
      publishStatus("FAILED");
      return true;
    }
    if (trigger_odom_valid_ && !last_result_.return_waypoints.empty()) {
      return_start_error_ =
          (last_result_.return_waypoints.front().position - trigger_position_)
              .norm();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (enable_command_output_) {
        state_ = WAITING_FOR_TAKEOVER;
        takeover_start_ = ros::Time::now();
      } else {
        state_ = FINISHED;
      }
    }
    publishStatus(enable_command_output_ ? "WAITING_FOR_TAKEOVER" : "FINISHED");
    // Step 20: auxiliary stop signal.  It stops the planner from re-planning
    // but is NOT the control-source guarantee; CommandGate is.
    mandatory_stop_pub_.publish(std_msgs::Empty());

    const Eigen::Vector3d history_last =
        last_result_.original_points.empty()
            ? Eigen::Vector3d::Zero()
            : last_result_.original_points.back().position;
    const Eigen::Vector3d return_first =
        last_result_.return_waypoints.empty()
            ? Eigen::Vector3d::Zero()
            : last_result_.return_waypoints.front().position;
    ROS_INFO_STREAM("[ManualReturn] trigger position   : "
                    << trigger_position_.transpose());
    ROS_INFO_STREAM("[ManualReturn] history last point : "
                    << history_last.transpose());
    ROS_INFO_STREAM("[ManualReturn] return first wp    : "
                    << return_first.transpose());
    ROS_INFO_STREAM("[ManualReturn] return_start_error : " << return_start_error_
                    << " m");
    ROS_INFO_STREAM("[ManualReturn] trigger speed      : "
                    << trigger_velocity_.norm() << " m/s");

    response.success = true;
    response.message =
        std::string("planned ") + mrp::statusToString(last_result_.status);
    return true;
  }

  bool planFrozenTrajectory(
      const std::vector<mrp::TrajectoryPoint>& frozen) {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr map;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map = map_cloud_;
    }
    last_result_ = planner_.planManualReturn(frozen, map, config_);
    if (last_result_.status != mrp::ManualReturnStatus::SUCCESS_RDP &&
        last_result_.status !=
            mrp::ManualReturnStatus::SUCCESS_DENSE_BACKTRACK &&
        last_result_.status != mrp::ManualReturnStatus::SUCCESS) {
      ROS_ERROR_STREAM("[ManualReturn] planning failed: "
                       << last_result_.message);
      return false;
    }
    publishPaths(last_result_);
    writeOutputs(last_result_);
    ROS_INFO_STREAM("[ManualReturn]\n"
                    << "raw points             : " << last_result_.raw_point_num
                    << "\n"
                    << "preprocessed points    : "
                    << last_result_.preprocessed_point_num << "\n"
                    << "return waypoints       : " << last_result_.final_point_num
                    << "\n"
                    << "original length        : "
                    << last_result_.original_path_length << " m\n"
                    << "return path length     : "
                    << last_result_.return_path_length << " m\n"
                    << "compression ratio      : "
                    << 100.0 * last_result_.compression_ratio << " %\n"
                    << "max RDP deviation      : "
                    << last_result_.max_rdp_deviation << " m\n"
                    << "z-protected segments   : "
                    << last_result_.vertical_protected_segments << "\n"
                    << "z-restored points      : "
                    << last_result_.vertical_restored_points << "\n"
                    << "map shortcut candidates: "
                    << last_result_.shortcut_candidates << "\n"
                    << "map shortcuts accepted : "
                    << last_result_.shortcut_count << "\n"
                    << "map shortcuts rejected : "
                    << last_result_.unsafe_segments << "\n"
                    << "map-restored points    : "
                    << last_result_.map_restored_points << "\n"
                    << "planning time          : "
                    << last_result_.planning_time_ms << " ms\n"
                    << "status                 : "
                    << mrp::statusToString(last_result_.status));
    return true;
  }

  void initializeExecutor() {
    const std::vector<mrp::ReturnWaypoint>& waypoints =
        last_result_.return_waypoints;
    cumulative_lengths_.clear();
    cumulative_lengths_.push_back(0.0);
    double length = 0.0;
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      length +=
          (waypoints[i].position - waypoints[i - 1].position).norm();
      cumulative_lengths_.push_back(length);
    }
    speed_profile_ = makeSpeedProfile(length, return_speed_, max_return_accel_);
    current_s_ = 0.0;
    current_v_ = 0.0;
    return_start_time_ = ros::Time::now();
    last_exec_time_ = ros::Time::now();

    // V1.0.2: mark the arc-length of each sharp bend for TURN-phase diagnosis.
    turn_center_s_.clear();
    const double threshold_rad = turn_angle_threshold_deg_ * M_PI / 180.0;
    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
      const Eigen::Vector3d v1 =
          waypoints[i].position - waypoints[i - 1].position;
      const Eigen::Vector3d v2 =
          waypoints[i + 1].position - waypoints[i].position;
      const double n1 = v1.norm();
      const double n2 = v2.norm();
      if (n1 < 1e-9 || n2 < 1e-9) continue;
      double cosang = v1.dot(v2) / (n1 * n2);
      cosang = std::max(-1.0, std::min(1.0, cosang));
      const double angle = std::acos(cosang);
      if (angle > threshold_rad) {
        turn_center_s_.push_back(cumulative_lengths_[i]);
      }
    }
    last_projected_segment_ = 0;
  }

  // V1.0.4: compute the heading for the current position.
  //   tangent_interpolation : legacy behaviour (yaw sweeps across the whole
  //                           segment).
  //   waypoint_hold        : hold the segment heading during flight, then
  //                           transition to the next segment heading within
  //                           yaw_transition_radius of the waypoint.
  double computeReturnYaw(int segment, double alpha,
                          const Eigen::Vector3d& position,
                          const std::vector<mrp::ReturnWaypoint>& waypoints) const {
    if (waypoints.size() < 2) return 0.0;
    if (segment < 0) segment = 0;
    if (segment + 1 >= static_cast<int>(waypoints.size()))
      segment = static_cast<int>(waypoints.size()) - 2;
    const double yaw_cur = waypoints[segment].yaw;
    const double yaw_next = waypoints[segment + 1].yaw;

    if (yaw_mode_ != "waypoint_hold") {
      // Legacy tangent_interpolation: linear sweep across the whole segment.
      return yaw_cur + alpha * (yaw_next - yaw_cur);
    }

    // waypoint_hold: keep the segment heading, then turn near the waypoint.
    const double delta = unwrapYawDelta(yaw_next, yaw_cur);
    const Eigen::Vector3d wp = waypoints[segment + 1].position;
    const double dist_to_wp = (wp - position).norm();
    double transition = 0.0;
    if (yaw_transition_radius_ > 1e-9 && dist_to_wp < yaw_transition_radius_) {
      transition = 1.0 - dist_to_wp / yaw_transition_radius_;
      transition = std::max(0.0, std::min(1.0, transition));
    }
    return yaw_cur + transition * delta;
  }

  void locateSegment(double s, double speed, Eigen::Vector3d* position,
                     Eigen::Vector3d* velocity, double* yaw,
                     int* segment_out = nullptr) {
    const std::vector<mrp::ReturnWaypoint>& waypoints =
        last_result_.return_waypoints;
    std::size_t segment = 0;
    while (segment + 1 < cumulative_lengths_.size() &&
           cumulative_lengths_[segment + 1] < s) {
      ++segment;
    }
    if (segment + 1 >= waypoints.size()) segment = waypoints.size() - 2;
    const double segment_length =
        cumulative_lengths_[segment + 1] - cumulative_lengths_[segment];
    double alpha = segment_length <= 1e-9
                       ? 0.0
                       : (s - cumulative_lengths_[segment]) / segment_length;
    alpha = std::max(0.0, std::min(1.0, alpha));
    const Eigen::Vector3d a = waypoints[segment].position;
    const Eigen::Vector3d b = waypoints[segment + 1].position;
    *position = a + alpha * (b - a);
    Eigen::Vector3d direction = b - a;
    const double direction_length = direction.norm();
    if (direction_length > 1e-9) direction /= direction_length;
    *velocity = speed * direction;
    *yaw = computeReturnYaw(static_cast<int>(segment), alpha, *position,
                            waypoints);
    if (segment_out) *segment_out = static_cast<int>(segment);
  }

  // Project an actual position onto the planned polyline with monotonic path
  // progress: search forward from the last matched segment (a few segments
  // ahead), so a U-turn or self-crossing does not snap back to an earlier,
  // incorrect segment.
  void projectToPath(const Eigen::Vector3d& p, int* segment_out,
                     Eigen::Vector3d* projection_out, double* progress_s_out) {
    const std::vector<mrp::ReturnWaypoint>& waypoints =
        last_result_.return_waypoints;
    const int n = static_cast<int>(waypoints.size());
    if (n < 2) {
      *segment_out = 0;
      *projection_out = p;
      *progress_s_out = 0.0;
      return;
    }
    int best_seg = last_projected_segment_;
    if (best_seg < 0) best_seg = 0;
    if (best_seg > n - 2) best_seg = n - 2;
    const int max_seg = std::min(n - 2, best_seg + 3);
    double best_dist = std::numeric_limits<double>::infinity();
    double best_u = 0.0;
    Eigen::Vector3d best_q = waypoints[best_seg].position;
    for (int seg = best_seg; seg <= max_seg; ++seg) {
      const Eigen::Vector3d a = waypoints[seg].position;
      const Eigen::Vector3d b = waypoints[seg + 1].position;
      const Eigen::Vector3d v = b - a;
      const double vv = v.squaredNorm();
      double u = 0.0;
      if (vv > 1e-12) u = (p - a).dot(v) / vv;
      u = std::max(0.0, std::min(1.0, u));
      const Eigen::Vector3d q = a + u * v;
      const double d = (p - q).norm();
      if (d < best_dist) {
        best_dist = d;
        best_seg = seg;
        best_u = u;
        best_q = q;
      }
    }
    last_projected_segment_ = best_seg;
    *segment_out = best_seg;
    *projection_out = best_q;
    *progress_s_out =
        cumulative_lengths_[best_seg] +
        best_u * (cumulative_lengths_[best_seg + 1] - cumulative_lengths_[best_seg]);
  }

  bool isNearTurn(double s) const {
    for (double c : turn_center_s_) {
      if (std::abs(s - c) <= turn_analysis_distance_) return true;
    }
    return false;
  }

  ReturnPhase classifyPhase(double elapsed, double s) const {
    if (s >= speed_profile_.total_length - 1e-9) return ReturnPhase::HOME_HOLD;
    if (isNearTurn(s)) return ReturnPhase::TURN;
    if (elapsed < speed_profile_.accel_time) return ReturnPhase::ACCELERATION;
    if (elapsed < speed_profile_.total_time - speed_profile_.accel_time)
      return ReturnPhase::CRUISE;
    return ReturnPhase::DECELERATION;
  }

  void stepReturn() {
    const std::vector<mrp::ReturnWaypoint>& waypoints =
        last_result_.return_waypoints;
    if (waypoints.size() < 2) {
      state_ = FAILED;
      publishStatus("FAILED");
      return;
    }
    const ros::Time now = ros::Time::now();
    const double total_length = speed_profile_.total_length;

    // Time-driven reference: evaluate the trapezoidal profile at the elapsed
    // time instead of integrating speed against a position-dependent limit
    // (which deadlocks at the start where the allowed speed is zero).
    const double elapsed = (now - return_start_time_).toSec();
    double s = 0.0;
    double v = 0.0;
    profileStateAt(elapsed, speed_profile_, &s, &v);

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double yaw = 0.0;
    int segment_id = -1;
    if (s >= total_length - 1e-9) {
      position = waypoints.back().position;
      velocity = Eigen::Vector3d::Zero();
      yaw = waypoints.back().yaw;
      segment_id = static_cast<int>(waypoints.size()) - 1;
    } else {
      locateSegment(s, v, &position, &velocity, &yaw, &segment_id);
    }
    publishCommand(position, velocity, yaw);

    // V1.0.4: record the heading command and the actual heading.
    double actual_yaw = yaw;
    if (have_odom_) {
      tf::Quaternion quaternion;
      tf::quaternionMsgToTF(latest_odom_.pose.pose.orientation, quaternion);
      double roll = 0.0, pitch = 0.0;
      tf::Matrix3x3(quaternion).getRPY(roll, pitch, actual_yaw);
    }
    YawSample yaw_sample;
    yaw_sample.timestamp = now.toSec();
    yaw_sample.current_x = position.x();
    yaw_sample.current_y = position.y();
    yaw_sample.current_z = position.z();
    yaw_sample.current_yaw = actual_yaw;
    yaw_sample.target_yaw = yaw;
    yaw_sample.yaw_error = unwrapYawDelta(yaw, actual_yaw);
    yaw_sample.segment_id = segment_id;
    yaw_sample.yaw_mode = yaw_mode_;
    yaw_log_.push_back(yaw_sample);
    publishYawMarker(position, yaw);

    if (have_odom_) {
      Eigen::Vector3d actual(latest_odom_.pose.pose.position.x,
                             latest_odom_.pose.pose.position.y,
                             latest_odom_.pose.pose.position.z);
      TrackingSample sample;
      sample.timestamp = now.toSec();
      sample.reference = position;
      sample.actual = actual;
      // V1.0.2 spatial decomposition: project actual onto the planned polyline.
      projectToPath(actual, &sample.matched_segment, &sample.projection,
                    &sample.actual_progress_s);
      sample.reference_progress_s = s;
      sample.along_track_error = s - sample.actual_progress_s;
      const Eigen::Vector3d ct = actual - sample.projection;
      sample.cross_track_3d = ct.norm();
      sample.cross_track_xy = std::sqrt(ct.x() * ct.x() + ct.y() * ct.y());
      sample.vertical_error = std::abs(ct.z());
      sample.phase = classifyPhase(elapsed, s);
      tracking_log_.push_back(sample);
      executed_return_path_.push_back(odomToPoint(latest_odom_));
    }

    if (s >= total_length - 1e-9) {
      bool finished = false;
      if (have_odom_) {
        const Eigen::Vector3d actual(latest_odom_.pose.pose.position.x,
                                     latest_odom_.pose.pose.position.y,
                                     latest_odom_.pose.pose.position.z);
        const double home_distance = (actual - waypoints.back().position).norm();
        const double actual_speed = Eigen::Vector3d(
            latest_odom_.twist.twist.linear.x,
            latest_odom_.twist.twist.linear.y,
            latest_odom_.twist.twist.linear.z).norm();
        finished = home_distance <= home_tolerance_ &&
                   actual_speed <= return_finish_speed_threshold_;
      } else {
        finished = true;
      }
      if (finished) {
        state_ = FINISHED;
        publishStatus("FINISHED");
        writeTrackingLogs();
        writeYawLog();
        writeMetricsFile();
        ROS_INFO("[ManualReturn] Manual return completed.");
      }
    }
  }

  void publishHomeHold() {
    const std::vector<mrp::ReturnWaypoint>& waypoints =
        last_result_.return_waypoints;
    if (waypoints.empty()) return;
    publishCommand(waypoints.back().position, Eigen::Vector3d::Zero(),
                   waypoints.back().yaw);
  }

  void executeTimer(const ros::TimerEvent&) {
    bool do_takeover_switch = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == WAITING_FOR_TAKEOVER &&
          (ros::Time::now() - takeover_start_).toSec() >= takeover_delay_) {
        do_takeover_switch = true;
      }
    }
    if (do_takeover_switch) {
      if (enable_command_output_) {
        std_srvs::Trigger request;
        if (!gate_manual_client_.exists() || !gate_manual_client_.call(request) ||
            !request.response.success) {
          ROS_ERROR("[ManualReturn] failed to switch CommandGate to "
                    "MANUAL_RETURN");
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = FAILED;
          publishStatus("FAILED");
          return;
        }
      }
      std::lock_guard<std::mutex> lock(mutex_);
      initializeExecutor();
      state_ = RETURNING;
      publishStatus("RETURNING");
      ROS_INFO("[ManualReturn] control taken over by Manual Return.");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == RETURNING) {
      stepReturn();
    } else if (state_ == FINISHED) {
      // Step 22: keep holding Home; do NOT restore NORMAL automatically.
      publishHomeHold();
    }
  }

  bool resetCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == PLANNING || state_ == WAITING_FOR_TAKEOVER ||
          state_ == RETURNING) {
        response.success = false;
        response.message = "reset only allowed in FINISHED or FAILED state";
        return true;
      }
    }
    if (enable_command_output_) {
      std_srvs::Trigger request;
      if (gate_normal_client_.exists() && gate_normal_client_.call(request) &&
          request.response.success) {
        ROS_INFO("[ManualReturn] CommandGate reset to NORMAL");
      } else {
        ROS_WARN("[ManualReturn] failed to reset CommandGate to NORMAL");
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      history_.clear();
      tracking_log_.clear();
      executed_return_path_.clear();
      yaw_log_.clear();
      last_result_ = mrp::ReturnPlanResult();
      trigger_odom_valid_ = false;
      return_start_error_ = 0.0;
      current_s_ = 0.0;
      current_v_ = 0.0;
      state_ = RECORDING;
      publishStatus("RECORDING");
    }
    response.success = true;
    response.message = "manual return reset to RECORDING";
    return true;
  }

  void publishCommand(const Eigen::Vector3d& position,
                      const Eigen::Vector3d& velocity, double yaw) {
    if (!enable_command_output_) return;
    mars_quadrotor_msgs::PositionCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = world_frame_;
    command.trajectory_id = 100000;
    command.trajectory_flag =
        mars_quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    command.position.x = position.x();
    command.position.y = position.y();
    command.position.z = position.z();
    command.velocity.x = velocity.x();
    command.velocity.y = velocity.y();
    command.velocity.z = velocity.z();
    command.acceleration.x = 0.0;
    command.acceleration.y = 0.0;
    command.acceleration.z = 0.0;
    command.jerk.x = 0.0;
    command.jerk.y = 0.0;
    command.jerk.z = 0.0;
    command.yaw = yaw;
    command.yaw_dot = 0.0;
    command_pub_.publish(command);
  }

  void publishStatus(const std::string& state) {
    std_msgs::String message;
    message.data = state;
    status_pub_.publish(message);
  }

  void writeTrackingLogs() {
    if (run_output_dir_.empty()) return;

    std::vector<double> time_err, cross3d, cross_xy, vertical, along_abs;
    time_err.reserve(tracking_log_.size());
    cross3d.reserve(tracking_log_.size());
    cross_xy.reserve(tracking_log_.size());
    vertical.reserve(tracking_log_.size());
    along_abs.reserve(tracking_log_.size());

    // 1) Extended tracking log: keeps the original error_norm columns and
    //    appends the V1.0.2 spatial-decomposition columns.
    const std::string csv_path = run_output_dir_ + "/return_tracking_log.csv";
    std::ofstream output(csv_path.c_str());
    if (output.is_open()) {
      output << "timestamp,ref_x,ref_y,ref_z,actual_x,actual_y,actual_z,"
                "error_x,error_y,error_z,error_norm,"
                "matched_segment,projection_x,projection_y,projection_z,"
                "actual_progress_s,reference_progress_s,along_track_error,"
                "cross_track_3d,cross_track_xy,vertical_error,return_phase\n";
      output << std::setprecision(12);
      for (const TrackingSample& s : tracking_log_) {
        const Eigen::Vector3d error = s.actual - s.reference;
        const double norm = error.norm();
        time_err.push_back(norm);
        cross3d.push_back(s.cross_track_3d);
        cross_xy.push_back(s.cross_track_xy);
        vertical.push_back(s.vertical_error);
        along_abs.push_back(std::abs(s.along_track_error));
        output << s.timestamp << ',' << s.reference.x() << ','
               << s.reference.y() << ',' << s.reference.z() << ','
               << s.actual.x() << ',' << s.actual.y() << ',' << s.actual.z()
               << ',' << error.x() << ',' << error.y() << ',' << error.z()
               << ',' << norm << ',' << s.matched_segment << ','
               << s.projection.x() << ',' << s.projection.y() << ','
               << s.projection.z() << ',' << s.actual_progress_s << ','
               << s.reference_progress_s << ',' << s.along_track_error << ','
               << s.cross_track_3d << ',' << s.cross_track_xy << ','
               << s.vertical_error << ',' << phaseToString(s.phase) << '\n';
      }
      output.close();
    } else {
      ROS_WARN_STREAM("[ManualReturn] cannot write " << csv_path);
    }

    // 2) Compact analysis CSV under diagnostics/.
    const std::string diagnostics_dir = run_output_dir_ + "/diagnostics";
    ::mkdir(diagnostics_dir.c_str(), 0755);
    const std::string analysis_path =
        diagnostics_dir + "/return_tracking_analysis.csv";
    std::ofstream analysis(analysis_path.c_str());
    if (analysis.is_open()) {
      analysis << "timestamp,actual_progress_s,reference_progress_s,"
                  "along_track_error,cross_track_3d,cross_track_xy,"
                  "vertical_error,return_phase\n";
      analysis << std::setprecision(12);
      for (const TrackingSample& s : tracking_log_) {
        analysis << s.timestamp << ',' << s.actual_progress_s << ','
                 << s.reference_progress_s << ',' << s.along_track_error << ','
                 << s.cross_track_3d << ',' << s.cross_track_xy << ','
                 << s.vertical_error << ',' << phaseToString(s.phase) << '\n';
      }
      analysis.close();
    }

    const TrackingStats t = computeStats(time_err);
    const TrackingStats c3 = computeStats(cross3d);
    const TrackingStats cxy = computeStats(cross_xy);
    const TrackingStats vert = computeStats(vertical);
    const TrackingStats along = computeStats(along_abs);

    // 4) Worst-case cross-track location.
    auto worst = std::max_element(
        tracking_log_.begin(), tracking_log_.end(),
        [](const TrackingSample& a, const TrackingSample& b) {
          return a.cross_track_3d < b.cross_track_3d;
        });
    if (worst != tracking_log_.end()) {
      ROS_INFO_STREAM("[ManualReturn] worst cross-track: t=" << worst->timestamp
                      << " phase=" << phaseToString(worst->phase)
                      << " segment=" << worst->matched_segment
                      << " cross_track=" << worst->cross_track_3d << " m");
      ROS_INFO_STREAM("[ManualReturn]   actual=" << worst->actual.transpose()
                      << " projection=" << worst->projection.transpose());
      std::ofstream worst_file(
          (diagnostics_dir + "/worst_case_cross_track.txt").c_str());
      if (worst_file.is_open()) {
        worst_file << std::setprecision(6);
        worst_file << "timestamp=" << worst->timestamp << "\n";
        worst_file << "phase=" << phaseToString(worst->phase) << "\n";
        worst_file << "segment=" << worst->matched_segment << "\n";
        worst_file << "cross_track_3d=" << worst->cross_track_3d << "\n";
        worst_file << "actual=" << worst->actual.x() << ',' << worst->actual.y()
                   << ',' << worst->actual.z() << "\n";
        worst_file << "projection=" << worst->projection.x() << ','
                   << worst->projection.y() << ',' << worst->projection.z()
                   << "\n";
        worst_file.close();
      }
    }

    ROS_INFO_STREAM(
        "[ManualReturn] tracking decomposition (N=" << tracking_log_.size()
        << "):\n"
        << "  time-sync    : mean=" << t.mean << " P95=" << t.p95
        << " max=" << t.max << "\n"
        << "  cross-track3d: mean=" << c3.mean << " P95=" << c3.p95
        << " P99=" << c3.p99 << " max=" << c3.max << "\n"
        << "  cross-trackxy: mean=" << cxy.mean << " P95=" << cxy.p95
        << " max=" << cxy.max << "\n"
        << "  vertical     : mean=" << vert.mean << " P95=" << vert.p95
        << " max=" << vert.max << "\n"
        << "  along-track  : mean_abs=" << along.mean << " P95_abs=" << along.p95
        << " max_abs=" << along.max);

    std::string error;
    mrp::CsvTrajectoryReader::write(
        run_output_dir_ + "/executed_return_path.csv", executed_return_path_,
        &error);
  }

  void writeYawLog() {
    if (run_output_dir_.empty()) return;
    const std::string csv_path = run_output_dir_ + "/return_yaw_log.csv";
    std::ofstream output(csv_path.c_str());
    if (output.is_open()) {
      output << "timestamp,current_x,current_y,current_z,current_yaw,"
                "target_yaw,yaw_error,current_segment_id,yaw_mode\n";
      output << std::setprecision(12);
      for (const YawSample& s : yaw_log_) {
        output << s.timestamp << ',' << s.current_x << ',' << s.current_y
               << ',' << s.current_z << ',' << s.current_yaw << ','
               << s.target_yaw << ',' << s.yaw_error << ','
               << s.segment_id << ',' << s.yaw_mode << '\n';
      }
      output.close();
      ROS_INFO_STREAM("[ManualReturn] yaw log written to " << csv_path);
    } else {
      ROS_WARN_STREAM("[ManualReturn] cannot write " << csv_path);
    }
  }

  void publishYawMarker(const Eigen::Vector3d& position, double yaw) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = world_frame_;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "manual_return_yaw";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.position.x = position.x();
    arrow.pose.position.y = position.y();
    arrow.pose.position.z = position.z();
    arrow.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    arrow.scale.x = 0.6;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.15;
    arrow.color.r = 1.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;
    yaw_marker_pub_.publish(arrow);
  }

  void analyzeTrajectory(const std::vector<mrp::TrajectoryPoint>& forward) {
    analysis_result_ = analysis_manager_.analyze(forward, analysis_config_);
    publishAnalysisMarkers(forward);
    writeAnalysisFiles();
    ROS_INFO_STREAM("[ManualReturn] trajectory analysis: "
                    << analysis_result_.segments.size() << " segments, "
                    << analysis_result_.dwell_count << " dwell, "
                    << analysis_result_.backtrack_count << " backtrack, "
                    << analysis_result_.overlap_count << " overlap candidates");
  }

  void publishAnalysisMarkers(
      const std::vector<mrp::TrajectoryPoint>& forward) {
    visualization_msgs::MarkerArray array;
    int id = 0;
    for (const auto& seg : analysis_result_.segments) {
      visualization_msgs::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = "trajectory_analysis";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.08;
      m.color.a = 1.0;
      switch (seg.segment_type) {
        case mrp::SegmentType::DWELL:
          m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; break;  // yellow
        case mrp::SegmentType::BACKTRACK_CANDIDATE:
          m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; break;  // red
        case mrp::SegmentType::OVERLAP_CANDIDATE:
          m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; break;  // purple
        default:
          m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; break;  // blue
      }
      for (int k = seg.start_index; k <= seg.end_index; ++k) {
        if (k < 0 || k >= static_cast<int>(forward.size())) continue;
        geometry_msgs::Point p;
        p.x = forward[k].position.x();
        p.y = forward[k].position.y();
        p.z = forward[k].position.z();
        m.points.push_back(p);
      }
      array.markers.push_back(m);
    }

    // A dwell is a POINT in space (its spatial extent is only a few
    // millimetres), so a LINE_STRIP for it degenerates to something invisible
    // at map scale.  Publish an extra sphere at each dwell centroid, plus a
    // text label with the dwell duration, so hovering is actually visible.
    int sphere_id = 1000;
    for (const auto& c : analysis_result_.edit_candidates) {
      if (c.type != mrp::EditType::DWELL) continue;
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      int count = 0;
      for (int k = c.start_index; k <= c.end_index; ++k) {
        if (k < 0 || k >= static_cast<int>(forward.size())) continue;
        centroid += forward[k].position;
        ++count;
      }
      if (count == 0) continue;
      centroid /= static_cast<double>(count);

      visualization_msgs::Marker sphere;
      sphere.header.frame_id = world_frame_;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "trajectory_analysis_dwell";
      sphere.id = sphere_id++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = centroid.x();
      sphere.pose.position.y = centroid.y();
      sphere.pose.position.z = centroid.z();
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.45;
      sphere.color.r = 1.0;
      sphere.color.g = 1.0;
      sphere.color.b = 0.0;  // yellow
      sphere.color.a = 0.75;
      array.markers.push_back(sphere);

      visualization_msgs::Marker label;
      label.header = sphere.header;
      label.ns = "trajectory_analysis_dwell_label";
      label.id = sphere_id++;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose = sphere.pose;
      label.pose.position.z += 0.5;
      label.scale.z = 0.3;
      label.color.r = 1.0;
      label.color.g = 1.0;
      label.color.b = 0.0;
      label.color.a = 1.0;
      label.text = "DWELL " + c.metrics;
      array.markers.push_back(label);
    }

    analysis_marker_pub_.publish(array);
  }

  void writeAnalysisFiles() {
    if (run_output_dir_.empty()) return;
    // Per-segment analysis summary.
    {
      const std::string p = run_output_dir_ + "/trajectory_analysis.csv";
      std::ofstream out(p.c_str());
      if (out.is_open()) {
        out << "segment_id,start_index,end_index,type,length,duration,"
               "confidence,reason\n";
        out << std::setprecision(12);
        for (const auto& seg : analysis_result_.segments) {
          double conf = 0.0;
          std::string reason;
          for (const auto& c : analysis_result_.edit_candidates) {
            // Two rules must hold so the reported reason always explains the
            // reported type:
            //  1. fully-within -- identical to TrajectoryAnalysisManager's
            //     labelling rule, so a NORMAL segment that merely grazes a
            //     candidate is not given that candidate's reason;
            //  2. the candidate's implied label equals the segment's label --
            //     the manager labels by severity while confidence is
            //     independent, so the highest-confidence candidate is not
            //     necessarily the one that produced the label.
            if (c.start_index <= seg.start_index &&
                seg.end_index <= c.end_index &&
                mrp::segmentTypeForEdit(c.type) == seg.segment_type &&
                c.confidence > conf) {
              conf = c.confidence;
              reason = c.reason;
            }
          }
          out << seg.segment_id << ',' << seg.start_index << ',' << seg.end_index
              << ',' << mrp::segmentTypeToString(seg.segment_type) << ','
              << seg.length << ',' << seg.duration << ',' << conf << ','
              << reason << '\n';
        }
        out.close();
      }
    }
    // Edit candidates.
    {
      const std::string p = run_output_dir_ + "/trajectory_edit_candidates.csv";
      std::ofstream out(p.c_str());
      if (out.is_open()) {
        out << "candidate_id,type,start_index,end_index,confidence,reason,"
               "metrics\n";
        for (const auto& c : analysis_result_.edit_candidates) {
          out << c.candidate_id << ',' << mrp::editTypeToString(c.type) << ','
              << c.start_index << ',' << c.end_index << ',' << c.confidence << ','
              << c.reason << ',' << c.metrics << '\n';
        }
        out.close();
      }
    }
  }

  void writeMetricsFile() {
    if (run_output_dir_.empty()) return;

    mrp::ReturnMetrics metrics;
    metrics.scenario = scenario_;
    metrics.original_points = static_cast<int>(last_result_.raw_point_num);
    metrics.original_length_m = last_result_.original_path_length;
    metrics.simplified_points = static_cast<int>(last_result_.final_point_num);
    metrics.return_length_m = last_result_.return_path_length;
    metrics.planning_time_ms = last_result_.planning_time_ms;

    if (metrics.original_points > 0) {
      metrics.point_reduction_percent =
          100.0 * static_cast<double>(metrics.original_points -
                                      metrics.simplified_points) /
          static_cast<double>(metrics.original_points);
    }
    if (metrics.original_length_m > 0.0) {
      metrics.length_change_percent =
          100.0 * (metrics.return_length_m - metrics.original_length_m) /
          metrics.original_length_m;
    }

    // Path fidelity: deviation of the simplified return path from the history.
    std::vector<mrp::TrajectoryPoint> return_path;
    for (const mrp::ReturnWaypoint& w : last_result_.return_waypoints) {
      mrp::TrajectoryPoint p;
      p.position = w.position;
      p.yaw = w.yaw;
      return_path.push_back(p);
    }
    const auto deviation = mrp::ReturnMetricsAnalyzer::computePathDeviation(
        return_path, last_result_.preprocessed_points);
    metrics.max_deviation_m = deviation.max;
    metrics.mean_deviation_m = deviation.mean;
    metrics.p95_deviation_m = deviation.p95;

    // Safety: V1.1 Safe-RDP collision check (clearance_available=false
    // means no PCD map was present, so the check could not run).
    metrics.safe_rdp_enabled = last_result_.safe_rdp_enabled;
    metrics.clearance_available = last_result_.clearance_available;
    metrics.min_clearance_m = last_result_.min_clearance_m;
    metrics.unsafe_segments = last_result_.unsafe_segments;
    metrics.validated_segments = last_result_.validated_segments;
    metrics.collision_check_count = last_result_.collision_check_count;
    metrics.voxelized_cloud_size = last_result_.voxelized_cloud_size;
    metrics.safe_path_points = static_cast<int>(last_result_.safe_point_num);
    metrics.original_rdp_points =
        static_cast<int>(last_result_.original_rdp_point_num);
    metrics.shortcut_count = last_result_.shortcut_count;
    metrics.shortcut_candidates = last_result_.shortcut_candidates;
    metrics.map_restored_points =
        static_cast<int>(last_result_.map_restored_points);

    // Tracking (spatial decomposition).
    std::vector<double> cross3d, vertical, along_abs;
    cross3d.reserve(tracking_log_.size());
    vertical.reserve(tracking_log_.size());
    along_abs.reserve(tracking_log_.size());
    for (const TrackingSample& s : tracking_log_) {
      cross3d.push_back(s.cross_track_3d);
      vertical.push_back(s.vertical_error);
      along_abs.push_back(std::abs(s.along_track_error));
    }
    const TrackingStats c3 = computeStats(cross3d);
    const TrackingStats vert = computeStats(vertical);
    const TrackingStats along = computeStats(along_abs);
    metrics.cross_track_p95 = c3.p95;
    metrics.cross_track_max = c3.max;
    metrics.vertical_error_p95 = vert.p95;
    metrics.along_track_p95 = along.p95;

    // Completion.
    if (!last_result_.return_waypoints.empty() && !tracking_log_.empty()) {
      const Eigen::Vector3d home =
          last_result_.return_waypoints.back().position;
      metrics.final_home_error_m =
          (tracking_log_.back().actual - home).norm();
    }
    metrics.return_duration_s =
        (ros::Time::now() - return_start_time_).toSec();

    // Performance.
    metrics.pointcloud_size =
        map_cloud_ ? static_cast<int>(map_cloud_->size()) : 0;
    metrics.memory_usage_mb = "unknown";

    // return_metrics.csv
    const std::string csv_path = run_output_dir_ + "/return_metrics.csv";
    std::ofstream out(csv_path.c_str());
    if (out.is_open()) {
      out << std::setprecision(12);
      out << "run_id,scenario,original_points,original_length_m,simplified_points,"
             "point_reduction_percent,return_length_m,length_change_percent,"
             "max_deviation_m,mean_deviation_m,p95_deviation_m,"
             "min_clearance_m,clearance_available,unsafe_segments,"
             "validated_segments,collision_check_count,cross_track_p95,"
             "cross_track_max,along_track_p95,vertical_error_p95,"
             "final_home_error_m,return_duration_s,planning_time_ms,"
             "memory_usage_mb,pointcloud_size,voxelized_cloud_size,"
             "safe_rdp_enabled,safe_path_points,original_rdp_points,"
             "shortcut_count,shortcut_candidates,map_restored_points\n";
      out << run_id_ << ',' << metrics.scenario << ','
          << metrics.original_points << ','
          << metrics.original_length_m << ',' << metrics.simplified_points << ','
          << metrics.point_reduction_percent << ',' << metrics.return_length_m
          << ',' << metrics.length_change_percent << ','
          << metrics.max_deviation_m << ',' << metrics.mean_deviation_m << ','
          << metrics.p95_deviation_m << ',';
      if (metrics.clearance_available) {
        out << metrics.min_clearance_m;
      } else {
        out << "nan";
      }
      out << ',' << (metrics.clearance_available ? "true" : "false") << ','
          << metrics.unsafe_segments << ',' << metrics.validated_segments << ','
          << metrics.collision_check_count << ',' << metrics.cross_track_p95
          << ',' << metrics.cross_track_max << ',' << metrics.along_track_p95
          << ',' << metrics.vertical_error_p95 << ','
          << metrics.final_home_error_m << ',' << metrics.return_duration_s
          << ',' << metrics.planning_time_ms << ',' << metrics.memory_usage_mb
          << ',' << metrics.pointcloud_size << ','
          << metrics.voxelized_cloud_size << ','
          << (metrics.safe_rdp_enabled ? "true" : "false") << ','
          << metrics.safe_path_points << ','
          << metrics.original_rdp_points << ','
          << metrics.shortcut_count << ','
          << metrics.shortcut_candidates << ','
          << metrics.map_restored_points << '\n';
      out.close();
    } else {
      ROS_WARN_STREAM("[ManualReturn] cannot write " << csv_path);
    }

    // return_summary.txt (human readable)
    const std::string summary_path = run_output_dir_ + "/return_summary.txt";
    std::ofstream summary(summary_path.c_str());
    if (summary.is_open()) {
      summary << std::setprecision(6);
      summary << "====================\n";
      summary << "Manual Return Result\n";
      summary << "====================\n\n";
      summary << "Scenario: " << metrics.scenario << "\n\n";
      summary << "Input:\n";
      summary << "  original_points: " << metrics.original_points << "\n";
      summary << "  original_length: " << metrics.original_length_m << " m\n\n";
      summary << "Compression:\n";
      summary << "  simplified_points: " << metrics.simplified_points << "\n";
      summary << "  reduction: " << metrics.point_reduction_percent << " %\n";
      summary << "  return_length: " << metrics.return_length_m << " m\n";
      summary << "  length_change: " << metrics.length_change_percent
              << " %\n\n";
      summary << "Path:\n";
      summary << "  max deviation: " << metrics.max_deviation_m << " m\n";
      summary << "  mean deviation: " << metrics.mean_deviation_m << " m\n";
      summary << "  p95 deviation: " << metrics.p95_deviation_m << " m\n\n";
      summary << "Safety:\n";
      summary << "  clearance_available: "
              << (metrics.clearance_available ? "true" : "false") << "\n";
      summary << "  min clearance: "
              << (metrics.clearance_available
                      ? std::to_string(metrics.min_clearance_m)
                      : std::string("not_available"))
              << "\n";
      summary << "  unsafe segments: " << metrics.unsafe_segments << "\n";
      summary << "  validated segments: " << metrics.validated_segments
              << "\n\n";
      summary << "Tracking:\n";
      summary << "  cross track p95: " << metrics.cross_track_p95 << " m\n";
      summary << "  cross track max: " << metrics.cross_track_max << " m\n";
      summary << "  along track p95: " << metrics.along_track_p95 << " m\n";
      summary << "  vertical error p95: " << metrics.vertical_error_p95
              << " m\n\n";
      summary << "Mission:\n";
      summary << "  final home error: " << metrics.final_home_error_m << " m\n";
      summary << "  return duration: " << metrics.return_duration_s << " s\n\n";
      summary << "Performance:\n";
      summary << "  planning time: " << metrics.planning_time_ms << " ms\n";
      summary << "  memory usage: " << metrics.memory_usage_mb << " MB\n";
      summary << "  pointcloud size: " << metrics.pointcloud_size << "\n";
      summary << "  voxelized cloud size: " << metrics.voxelized_cloud_size
              << "\n";
      summary << "  safe_rdp_enabled: "
              << (metrics.safe_rdp_enabled ? "true" : "false") << "\n";
      summary << "  safe_path_points: " << metrics.safe_path_points << "\n";
      summary << "  original_rdp_points: " << metrics.original_rdp_points
              << "\n";
      summary << "  shortcut_count: " << metrics.shortcut_count << "\n";
      summary << "  shortcut_candidates: " << metrics.shortcut_candidates
              << "\n";
      summary << "  map_restored_points: " << metrics.map_restored_points
              << "\n";
      summary.close();
    }

    ROS_INFO_STREAM("[ManualReturn] metrics written to " << csv_path);
  }

  nav_msgs::Path makePath(
      const std::vector<mrp::TrajectoryPoint>& points) const {
    nav_msgs::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp = ros::Time::now();
    for (const mrp::TrajectoryPoint& point : points) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.header.stamp = ros::Time(point.timestamp);
      pose.pose.position.x = point.position.x();
      pose.pose.position.y = point.position.y();
      pose.pose.position.z = point.position.z();
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw);
      path.poses.push_back(pose);
    }
    return path;
  }

  void publishRaw(const std::vector<mrp::TrajectoryPoint>& points) {
    raw_pub_.publish(makePath(points));
  }

  void publishPaths(const mrp::ReturnPlanResult& result) {
    raw_pub_.publish(makePath(result.original_points));
    preprocessed_pub_.publish(makePath(result.preprocessed_points));
    reversed_pub_.publish(makePath(result.reversed_points));
    std::vector<mrp::TrajectoryPoint> rdp;
    for (const mrp::ReturnWaypoint& waypoint : result.return_waypoints) {
      mrp::TrajectoryPoint point;
      point.position = waypoint.position;
      point.yaw = waypoint.yaw;
      rdp.push_back(point);
    }
    rdp_pub_.publish(makePath(rdp));

    // Line marker for the simplified return polyline.
    visualization_msgs::Marker line;
    line.header.frame_id = world_frame_;
    line.header.stamp = ros::Time::now();
    line.ns = "manual_return";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.08;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;
    for (const mrp::ReturnWaypoint& waypoint : result.return_waypoints) {
      geometry_msgs::Point p;
      p.x = waypoint.position.x();
      p.y = waypoint.position.y();
      p.z = waypoint.position.z();
      line.points.push_back(p);
    }
    rdp_line_pub_.publish(line);

    // Safe-RDP path: green line (== RDP path in V1.1.0, which only
    // validates and never rewrites the compressed path).
    visualization_msgs::Marker safe_line;
    safe_line.header.frame_id = world_frame_;
    safe_line.header.stamp = ros::Time::now();
    safe_line.ns = "manual_return_safe";
    safe_line.id = 0;
    safe_line.type = visualization_msgs::Marker::LINE_STRIP;
    safe_line.action = visualization_msgs::Marker::ADD;
    safe_line.scale.x = 0.08;
    safe_line.color.r = 0.0;
    safe_line.color.g = 1.0;
    safe_line.color.b = 0.0;
    safe_line.color.a = 1.0;
    const std::vector<mrp::ReturnWaypoint>& safe_wps =
        result.safe_waypoints.empty() ? result.return_waypoints
                                      : result.safe_waypoints;
    for (const mrp::ReturnWaypoint& waypoint : safe_wps) {
      geometry_msgs::Point p;
      p.x = waypoint.position.x();
      p.y = waypoint.position.y();
      p.z = waypoint.position.z();
      safe_line.points.push_back(p);
    }
    safe_path_pub_.publish(safe_line);

    // Key points: one sphere per simplified waypoint.
    visualization_msgs::MarkerArray key_points;
    for (std::size_t i = 0; i < result.return_waypoints.size(); ++i) {
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = world_frame_;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "manual_return_key_points";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = result.return_waypoints[i].position.x();
      sphere.pose.position.y = result.return_waypoints[i].position.y();
      sphere.pose.position.z = result.return_waypoints[i].position.z();
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20;
      sphere.color.r = 1.0;
      sphere.color.g = 1.0;
      sphere.color.b = 0.0;
      sphere.color.a = 1.0;
      key_points.markers.push_back(sphere);
    }
    key_points_pub_.publish(key_points);

    // Home marker (both the legacy /manual_return/home and the new
    // /manual_return/home_marker).
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "manual_return";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.a = 1.0;
    if (!result.preprocessed_points.empty()) {
      marker.pose.position.x = result.preprocessed_points.front().position.x();
      marker.pose.position.y = result.preprocessed_points.front().position.y();
      marker.pose.position.z = result.preprocessed_points.front().position.z();
    }
    home_pub_.publish(marker);
    home_marker_pub_.publish(marker);
  }

  void writeOutputs(const mrp::ReturnPlanResult& result) {
    if (run_output_dir_.empty()) return;
    const std::string names[] = {"raw_path.csv", "preprocessed_path.csv",
                                 "reversed_path.csv", "return_path.csv"};
    const std::vector<std::vector<mrp::TrajectoryPoint>> paths = {
        result.original_points, result.preprocessed_points,
        result.reversed_points};
    for (std::size_t i = 0; i < paths.size(); ++i) {
      std::string error;
      mrp::CsvTrajectoryReader::write(run_output_dir_ + "/" + names[i], paths[i],
                                      &error);
    }
    std::vector<mrp::TrajectoryPoint> rdp;
    for (const mrp::ReturnWaypoint& waypoint : result.return_waypoints) {
      mrp::TrajectoryPoint point;
      point.position = waypoint.position;
      point.yaw = waypoint.yaw;
      rdp.push_back(point);
    }
    std::string error;
    mrp::CsvTrajectoryReader::write(run_output_dir_ + "/" + names[3], rdp, &error);
  }

  ros::NodeHandle nh_, private_nh_;
  std::string world_frame_, odom_topic_, map_topic_, return_command_topic_,
      trajectory_csv_, output_dir_, pcd_path_, gate_set_manual_return_service_,
      gate_set_normal_service_;
  double record_frequency_ = 10.0, return_speed_ = 0.5, home_tolerance_ = 0.3;
  double return_finish_speed_threshold_ = 0.15, takeover_delay_ = 1.5;
  double max_return_accel_ = 1.5, command_frequency_ = 100.0;
  bool enable_command_output_ = false, offline_csv_test_ = false,
       have_odom_ = false;
  bool require_control_takeover_ = false, frame_error_ = false;
  std::string scenario_ = "unknown";
  mrp::ManualReturnConfig config_;
  mrp::ManualReturnPlanner planner_;
  mrp::ReturnPlanResult last_result_;
  mrp::TrajectoryAnalysisConfig analysis_config_;
  mrp::TrajectoryAnalysisManager analysis_manager_;
  mrp::TrajectoryAnalysisResult analysis_result_;
  std::vector<mrp::TrajectoryPoint> history_;
  nav_msgs::Odometry latest_odom_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  State state_;
  bool trigger_odom_valid_ = false;
  Eigen::Vector3d trigger_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d trigger_velocity_ = Eigen::Vector3d::Zero();
  double return_start_error_ = 0.0;
  SpeedProfile speed_profile_;
  std::vector<double> cumulative_lengths_;
  double current_s_ = 0.0;
  double current_v_ = 0.0;
  double turn_angle_threshold_deg_ = 20.0;
  double turn_analysis_distance_ = 0.5;
  std::string yaw_mode_ = "waypoint_hold";
  double yaw_transition_radius_ = 0.2;
  std::vector<double> turn_center_s_;
  int last_projected_segment_ = 0;
  std::string run_output_dir_;
  std::string run_id_;
  ros::Time takeover_start_, return_start_time_, last_exec_time_;
  std::vector<TrackingSample> tracking_log_;
  std::vector<YawSample> yaw_log_;
  std::vector<mrp::TrajectoryPoint> executed_return_path_;
  std::mutex mutex_;
  ros::Publisher raw_pub_, preprocessed_pub_, reversed_pub_, rdp_pub_, home_pub_,
      map_pub_;
  ros::Publisher history_pub_, rdp_line_pub_, key_points_pub_, home_marker_pub_;
  ros::Publisher safe_path_pub_;
  ros::Publisher yaw_marker_pub_;
  ros::Publisher analysis_marker_pub_;
  ros::Publisher command_pub_, mandatory_stop_pub_, status_pub_;
  ros::Subscriber odom_sub_, map_sub_;
  ros::ServiceServer trigger_service_, reset_service_;
  ros::ServiceClient gate_manual_client_, gate_normal_client_;
  ros::Timer record_timer_, execute_timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_return_planner");
  ManualReturnNode node;
  ros::spin();
  return 0;
}
