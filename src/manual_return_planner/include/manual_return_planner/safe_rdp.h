#pragma once

#include "manual_return_planner/manual_return_planner.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace manual_return_planner {

// Result of validating a candidate return path against the PCD map.
struct SafeRdpResult {
  bool safe = false;                     // true iff every segment is clear
  std::vector<ReturnWaypoint> safe_path; // == input path (V1.1.0 never edits)
  // Map-aware fallback output.  Unsafe RDP shortcuts are replaced with the
  // measured history samples they would have skipped.
  std::vector<TrajectoryPoint> fallback_path;
  int collision_check_count = 0;         // number of sampled queries
  int unsafe_segments = 0;               // rejected shortcut candidates
  int validated_segments = 0;            // accepted shortcut candidates
  int shortcut_candidates = 0;
  std::size_t restored_history_points = 0;
  double min_clearance_m = 0.0;          // smallest obstacle distance seen
  bool clearance_available = false;      // false when no PCD was available
  int voxelized_cloud_size = 0;          // cloud size after VoxelGrid
  std::string message;
};

// PCD collision primitives plus the production original-route fallback.
// validate() remains a strict low-level checker; restoreUnsafeShortcuts() is
// used by the planner so a rejected compression is replaced locally instead
// of cancelling the complete return.
class SafeRdpPlanner {
 public:
  SafeRdpResult validate(
      const std::vector<ReturnWaypoint>& rdp_path,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
      const SafeRdpConfig& config) const;

  // Validate only the non-adjacent chords introduced by simplification.  A
  // chord that violates the PCD clearance is locally replaced by the original
  // measured samples; adjacent history segments are kept by provenance.
  SafeRdpResult restoreUnsafeShortcuts(
      const std::vector<TrajectoryPoint>& candidate_path,
      const std::vector<TrajectoryPoint>& reversed_history,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
      const SafeRdpConfig& config) const;
};

}  // namespace manual_return_planner
