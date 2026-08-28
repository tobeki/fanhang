#pragma once

#include "manual_return_planner/safe_rdp.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace manual_return_planner {

struct MissionWaypoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  bool is_key_wp = false;
  int action = 0;
  std::string title;
};

enum class MissionReturnStatus {
  SUCCESS,
  TOO_FEW_POINTS,
  INVALID_INPUT,
  NO_SAFE_PATH
};

struct MissionReturnConfig {
  double rdp_epsilon = 0.05;
  double safe_radius = 0.40;
  double voxel_resolution = 0.05;
  double collision_check_resolution = 0.05;
  bool fixed_return_yaw = true;
};

struct MissionReturnResult {
  MissionReturnStatus status = MissionReturnStatus::INVALID_INPUT;
  std::vector<MissionWaypoint> return_waypoints;
  std::size_t input_points = 0;
  std::size_t optimized_points = 0;
  int map_checked_segments = 0;
  int rejected_segments = 0;
  bool original_chain_used = false;
  double min_clearance_m = 0.0;
  std::string message;
};

class MissionReturnPlanner {
 public:
  MissionReturnResult plan(
      const std::vector<MissionWaypoint>& mission_waypoints,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& whole_map,
      const MissionReturnConfig& config) const;
};

}  // namespace manual_return_planner
