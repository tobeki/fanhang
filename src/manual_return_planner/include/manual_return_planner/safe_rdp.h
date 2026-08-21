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
  int collision_check_count = 0;         // number of sampled queries
  int unsafe_segments = 0;               // segments closer than R_safe
  int validated_segments = 0;            // segments confirmed clear
  double min_clearance_m = 0.0;          // smallest obstacle distance seen
  bool clearance_available = false;      // false when no PCD was available
  int voxelized_cloud_size = 0;          // cloud size after VoxelGrid
  std::string message;
};

// V1.1.0 Safe-RDP: *validates* an already-compressed return path against the
// PCD map.  It does NOT detour, re-plan, shorten, or search for new free
// space.  If any sampled point is closer than R_safe to an obstacle, the path
// is rejected (safe == false) so the caller can fall back to NO_SAFE_PATH.
class SafeRdpPlanner {
 public:
  SafeRdpResult validate(
      const std::vector<ReturnWaypoint>& rdp_path,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
      const SafeRdpConfig& config) const;
};

}  // namespace manual_return_planner
