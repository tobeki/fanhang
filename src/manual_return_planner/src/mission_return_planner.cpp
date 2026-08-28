#include "manual_return_planner/mission_return_planner.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace manual_return_planner {
namespace {

std::vector<std::size_t> simplifyIndices(
    const std::vector<MissionWaypoint>& points, double epsilon) {
  if (points.size() <= 2) return {0, points.empty() ? 0 : points.size() - 1};
  std::vector<bool> keep(points.size(), false);
  keep.front() = true;
  keep.back() = true;
  for (std::size_t i = 0; i < points.size(); ++i)
    if (points[i].is_key_wp) keep[i] = true;
  std::vector<std::pair<std::size_t, std::size_t>> stack{{0, points.size() - 1}};
  while (!stack.empty()) {
    const auto range = stack.back();
    stack.pop_back();
    if (range.second <= range.first + 1) continue;
    double largest = -1.0;
    std::size_t largest_index = range.first;
    for (std::size_t i = range.first + 1; i < range.second; ++i) {
      const double d = RdpSimplifier::pointToSegmentDistance(
          points[i].position, points[range.first].position,
          points[range.second].position);
      if (d > largest) {
        largest = d;
        largest_index = i;
      }
    }
    if (largest > epsilon) {
      keep[largest_index] = true;
      stack.emplace_back(range.first, largest_index);
      stack.emplace_back(largest_index, range.second);
    }
  }
  std::vector<std::size_t> result;
  for (std::size_t i = 0; i < keep.size(); ++i)
    if (keep[i]) result.push_back(i);
  return result;
}

std::vector<ReturnWaypoint> toReturnWaypoints(
    const std::vector<MissionWaypoint>& points,
    const std::vector<std::size_t>& indices, double yaw) {
  std::vector<ReturnWaypoint> result;
  result.reserve(indices.size());
  for (const std::size_t i : indices)
    result.push_back({points[i].position, yaw});
  return result;
}

std::vector<MissionWaypoint> selectMissionWaypoints(
    const std::vector<MissionWaypoint>& points,
    const std::vector<std::size_t>& indices, double yaw) {
  std::vector<MissionWaypoint> result;
  result.reserve(indices.size());
  for (const std::size_t i : indices) {
    MissionWaypoint point = points[i];
    point.yaw = yaw;
    result.push_back(point);
  }
  return result;
}

}  // namespace

MissionReturnResult MissionReturnPlanner::plan(
    const std::vector<MissionWaypoint>& mission_waypoints,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& whole_map,
    const MissionReturnConfig& config) const {
  MissionReturnResult result;
  result.input_points = mission_waypoints.size();
  if (mission_waypoints.size() < 2 || !whole_map || whole_map->empty()) {
    result.status = mission_waypoints.size() < 2
                        ? MissionReturnStatus::TOO_FEW_POINTS
                        : MissionReturnStatus::NO_SAFE_PATH;
    result.message = mission_waypoints.size() < 2
                         ? "mission RTL needs at least two waypoints"
                         : "mission RTL requires a non-empty whole-map";
    return result;
  }
  if (!std::isfinite(config.rdp_epsilon) || config.rdp_epsilon < 0.0 ||
      !std::isfinite(config.safe_radius) || config.safe_radius <= 0.0) {
    result.status = MissionReturnStatus::INVALID_INPUT;
    result.message = "invalid mission RTL configuration";
    return result;
  }
  for (const auto& point : mission_waypoints) {
    if (!point.position.allFinite() || !std::isfinite(point.yaw)) {
      result.status = MissionReturnStatus::INVALID_INPUT;
      result.message = "non-finite mission waypoint";
      return result;
    }
  }

  std::vector<MissionWaypoint> reversed(mission_waypoints.rbegin(),
                                        mission_waypoints.rend());
  const double landing_yaw = mission_waypoints.front().yaw;
  for (auto& point : reversed)
    point.yaw = config.fixed_return_yaw ? landing_yaw : point.yaw;
  const auto indices = simplifyIndices(reversed, config.rdp_epsilon);
  const auto candidate = toReturnWaypoints(reversed, indices, landing_yaw);
  const auto candidate_mission =
      selectMissionWaypoints(reversed, indices, landing_yaw);
  const auto original_indices = [&]() {
    std::vector<std::size_t> all(reversed.size());
    for (std::size_t i = 0; i < all.size(); ++i) all[i] = i;
    return all;
  }();
  const auto original = toReturnWaypoints(reversed, original_indices,
                                           landing_yaw);

  SafeRdpConfig safe_config;
  safe_config.uav_radius = 0.25;
  safe_config.tracking_margin = 0.0;
  safe_config.extra_margin = 0.0;
  safe_config.safe_rdp_radius = config.safe_radius;
  safe_config.voxel_resolution = config.voxel_resolution;
  safe_config.collision_check_resolution = config.collision_check_resolution;
  safe_config.trust_flown_history = false;
  SafeRdpPlanner checker;
  const auto checked = checker.validate(candidate, whole_map, safe_config);
  result.optimized_points = candidate.size();
  result.map_checked_segments = checked.validated_segments + checked.unsafe_segments;
  result.rejected_segments = checked.unsafe_segments;
  result.min_clearance_m = checked.min_clearance_m;
  if (checked.safe) {
    result.return_waypoints = candidate_mission;
    result.status = MissionReturnStatus::SUCCESS;
    result.message = "mission RTL optimized path passed whole-map Safe-RDP";
    return result;
  }

  const auto original_checked = checker.validate(original, whole_map, safe_config);
  if (!original_checked.safe) {
    result.status = MissionReturnStatus::NO_SAFE_PATH;
    result.message = "both optimized and original mission chains are unsafe";
    result.rejected_segments += original_checked.unsafe_segments;
    return result;
  }
  result.return_waypoints = reversed;
  result.original_chain_used = true;
  result.status = MissionReturnStatus::SUCCESS;
  result.message = "optimized shortcut rejected; original mission chain used";
  return result;
}

}  // namespace manual_return_planner
