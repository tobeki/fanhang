#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>
#include <vector>

namespace manual_return_planner {

struct TrajectoryPoint {
  double timestamp = 0.0;  // [s]
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // world frame [m]
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // world frame [m/s]
  double roll = 0.0;   // [rad]
  double pitch = 0.0;  // [rad]
  double yaw = 0.0;    // [rad]
};

struct ReturnWaypoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // world frame [m]
  double yaw = 0.0;  // [rad]
};

enum class ManualReturnStatus {
  SUCCESS,
  SUCCESS_RDP,
  SUCCESS_DENSE_BACKTRACK,
  INVALID_INPUT,
  TOO_FEW_POINTS,
  INVALID_TIMESTAMP,
  TRAJECTORY_DISCONTINUITY,
  FRAME_ERROR,
  EXECUTION_ERROR,
  NO_SAFE_PATH,
  INTERNAL_ERROR
};

// V1.1 Safe-RDP configuration.  All lengths are in metres.  The safety
// envelope is a sphere of radius R_safe = uav_radius + tracking_margin +
// extra_margin; a compressed segment must keep at least this clearance.
struct SafeRdpConfig {
  bool enabled = true;                       // master switch
  double uav_radius = 0.25;                  // [m] spherical body envelope
  double tracking_margin = 0.15;             // [m] measured tracking error
  double extra_margin = 0.10;                // [m] additional safety margin
  double voxel_resolution = 0.05;            // [m] VoxelGrid leaf size
  double collision_check_resolution = 0.05;  // [m] sampling step on segments

  double safeRadius() const {
    return uav_radius + tracking_margin + extra_margin;
  }
};

struct ManualReturnConfig {
  // Historical trajectory preprocessing and basic RDP [SI units].
  double min_point_spacing = 0.03;      // [m]
  double rdp_epsilon = 0.05;            // [m]
  double max_segment_length = 5.0;      // [m]
  double max_reasonable_speed = 2.25;   // [m/s], warning threshold
  // RDP may not skip measured samples across a larger z span.  This keeps
  // climbs/descents on the flown 3D trace instead of creating a new diagonal.
  double vertical_preserve_threshold = 0.05; // [m]

  // Safe-RDP V1.1 reservation.  These values are deliberately not used by
  // V1.0: a point cloud may veto a shortcut later, but must never be treated
  // as evidence that an unobserved area is free space.
  double corridor_radius = 0.5;         // [m]
  double corridor_check_step = 0.2;     // [m]
  double min_length_ratio = 0.85;       // direct/history ratio
  double vehicle_body_radius = 0.39;    // [m], mesh-derived conservative bound
  double localization_margin = 0.05;    // [m]
  double tracking_margin = 0.05;        // [m]
  double map_margin = 0.05;             // [m]
  double extra_safety_margin = 0.05;    // [m]
  double collision_check_step = 0.1;    // [m]

  // Execution/runtime reservation.  The ROS wrapper owns the actual timing.
  double return_cruise_speed = 0.5;    // [m/s]
  double max_return_acceleration = 1.5; // [m/s^2]
  double home_position_tolerance = 0.3; // [m]
  // Stop this far above the recorded arm/Home point, then hand control to the
  // landing state.  ENU uses positive z upward.
  double landing_handoff_height = 0.25; // [m]
  double record_frequency = 10.0;       // [Hz]
  std::string world_frame = "world";
    SafeRdpConfig safe_rdp;
};

struct ReturnPlanResult {
  ManualReturnStatus status = ManualReturnStatus::INVALID_INPUT;
  std::vector<TrajectoryPoint> original_points;
  std::vector<TrajectoryPoint> preprocessed_points;
  std::vector<TrajectoryPoint> reversed_points;
  std::vector<ReturnWaypoint> return_waypoints;
  std::size_t raw_point_num = 0;
  std::size_t preprocessed_point_num = 0;
  std::size_t final_point_num = 0;
  double original_path_length = 0.0;
  double return_path_length = 0.0;
  double compression_ratio = 0.0;
  double max_rdp_deviation = 0.0;
  std::size_t vertical_protected_segments = 0;
  std::size_t vertical_restored_points = 0;
  double planning_time_ms = 0.0;
    // Safe-RDP V1.1 outputs.
    std::vector<ReturnWaypoint> safe_waypoints;
    bool safe_rdp_enabled = false;
    int collision_check_count = 0;
    int unsafe_segments = 0;
    int validated_segments = 0;
    double min_clearance_m = 0.0;
    bool clearance_available = false;
    int voxelized_cloud_size = 0;
    std::size_t original_rdp_point_num = 0;
    std::size_t safe_point_num = 0;
    int shortcut_count = 0;  // map-accepted RDP shortcut segments
    int shortcut_candidates = 0;
    std::size_t map_restored_points = 0;
  std::string message;
};

class TrajectoryPreprocessor {
 public:
  /**
   * Validate ordered world-frame points, remove near duplicates, and report
   * suspicious single-step jumps without silently deleting flight history.
   */
  bool preprocess(const std::vector<TrajectoryPoint>& input,
                  const ManualReturnConfig& config,
                  std::vector<TrajectoryPoint>* output,
                  std::string* warning_message,
                  ManualReturnStatus* failure_status) const;

  bool detectPositionJump(const TrajectoryPoint& previous,
                          const TrajectoryPoint& current,
                          const ManualReturnConfig& config,
                          double* observed_speed) const;
};

class RdpSimplifier {
 public:
  /** Simplify an ordered 3D trajectory using point-to-segment distance [m]. */
  static std::vector<TrajectoryPoint> simplify(
      const std::vector<TrajectoryPoint>& points, double epsilon,
      double* max_deviation);

  static double pointToSegmentDistance(const Eigen::Vector3d& point,
                                       const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& end);
};

class CsvTrajectoryReader {
 public:
  static bool read(const std::string& path, std::vector<TrajectoryPoint>* points,
                   std::string* error_message);
  static bool write(const std::string& path,
                    const std::vector<TrajectoryPoint>& points,
                    std::string* error_message);
};

class ManualReturnPlanner {
 public:
  /**
   * Plan Manual Return only from the flown history. map_cloud is intentionally
   * not used for shortcut generation in V1; it is reserved for Safe-RDP.
   */
  ReturnPlanResult planManualReturn(
      const std::vector<TrajectoryPoint>& flown_trajectory,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
      const ManualReturnConfig& config) const;

 private:
  static double pathLength(const std::vector<TrajectoryPoint>& points);
  static double maxDeviation(const std::vector<TrajectoryPoint>& source,
                             const std::vector<TrajectoryPoint>& simplified);
  static std::vector<TrajectoryPoint> enforceMaxSegmentLength(
      const std::vector<TrajectoryPoint>& simplified,
      const std::vector<TrajectoryPoint>& history, double max_segment_length);
  static std::vector<TrajectoryPoint> preserveVerticalHistory(
      const std::vector<TrajectoryPoint>& simplified,
      const std::vector<TrajectoryPoint>& history,
      double vertical_preserve_threshold,
      std::size_t* protected_segments, std::size_t* restored_points);
  static std::vector<ReturnWaypoint> makeWaypoints(
      const std::vector<TrajectoryPoint>& reversed);
};

// Unified benchmark metrics for a single return run.  These fields are kept
// algorithm-agnostic (no "rdp" naming) so that different return algorithms
// (RDP, Safe-RDP, shortcut optimization, ...) can be compared fairly on the
// same set of numbers.
struct ReturnMetrics {
  // Experiment identification.
  std::string scenario = "unknown";

  // Input scale.
  int original_points = 0;
  double original_length_m = 0.0;

  // Compression effect.
  int simplified_points = 0;
  double point_reduction_percent = 0.0;
  double return_length_m = 0.0;
  double length_change_percent = 0.0;

  // Path fidelity: distance from the simplified return path back to the
  // original history (3D, not just XY).
  double max_deviation_m = 0.0;
  double mean_deviation_m = 0.0;
  double p95_deviation_m = 0.0;

  // Safety.  When no point-cloud collision check is available the flag is
  // false and min_clearance_m is reported as "not available" (never a sentinel
  // like -1).
  double min_clearance_m = 0.0;
  bool clearance_available = false;
  int unsafe_segments = 0;
  int validated_segments = 0;
  int collision_check_count = 0;

  // Tracking (spatial, not time-synchronized).
  double cross_track_p95 = 0.0;
  double cross_track_max = 0.0;
  double along_track_p95 = 0.0;
  double vertical_error_p95 = 0.0;

  // Completion.
  double final_home_error_m = 0.0;
  double return_duration_s = 0.0;

  // Performance.
  double planning_time_ms = 0.0;
  std::string memory_usage_mb = "unknown";
  int pointcloud_size = 0;
  int voxelized_cloud_size = 0;
    bool safe_rdp_enabled = false;
    int safe_path_points = 0;
    int original_rdp_points = 0;
    int shortcut_count = 0;
    int shortcut_candidates = 0;
    int map_restored_points = 0;
};

// Computes algorithm-agnostic quality metrics that do not depend on the
// specific simplification strategy, so results can be compared across return
// algorithms.
class ReturnMetricsAnalyzer {
 public:
  struct DeviationStats {
    double mean = 0.0;
    double p95 = 0.0;
    double max = 0.0;
  };

  // Distance from a point to a polyline (sequence of TrajectoryPoint).
  static double pointToPolylineDistance(
      const Eigen::Vector3d& point,
      const std::vector<TrajectoryPoint>& polyline);

  // Densely sample the simplified return path and measure each sample's
  // distance back to the original history polyline.  This captures how far the
  // simplified path strays from the actually-flown corridor.
  static DeviationStats computePathDeviation(
      const std::vector<TrajectoryPoint>& simplified_return_path,
      const std::vector<TrajectoryPoint>& history);
};

const char* statusToString(ManualReturnStatus status);

}  // namespace manual_return_planner
