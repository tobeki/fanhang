#include "manual_return_planner/safe_rdp.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace manual_return_planner {
namespace {

// Distance from a query point to its nearest neighbour in the kd-tree.
double nearestDistance(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                       const Eigen::Vector3d& p) {
  pcl::PointXYZ query(static_cast<float>(p.x()), static_cast<float>(p.y()),
                      static_cast<float>(p.z()));
  std::vector<int> index(1);
  std::vector<float> squared_distance(1);
  const int found = kdtree.nearestKSearch(query, 1, index, squared_distance);
  if (found <= 0 || squared_distance.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(static_cast<double>(squared_distance[0]));
}

bool sameHistoryPoint(const TrajectoryPoint& a, const TrajectoryPoint& b) {
  return a.timestamp == b.timestamp && a.position == b.position;
}

double historyDeviation(const std::vector<TrajectoryPoint>& history,
                        std::size_t begin, std::size_t end,
                        const Eigen::Vector3d& start,
                        const Eigen::Vector3d& finish) {
  if (begin >= history.size() || end >= history.size() || begin > end)
    return std::numeric_limits<double>::infinity();
  double maximum = 0.0;
  for (std::size_t i = begin; i <= end; ++i) {
    maximum = std::max(maximum,
                       RdpSimplifier::pointToSegmentDistance(
                           history[i].position, start, finish));
  }
  return maximum;
}

void appendHistoryFallback(std::vector<TrajectoryPoint>* output,
                           const std::vector<TrajectoryPoint>& history,
                           std::size_t begin, std::size_t end,
                           double spacing, double turn_threshold) {
  if (!output || history.empty() || begin >= history.size() ||
      end >= history.size() || begin > end) return;
  const double step = std::max(0.1, spacing);
  double accumulated = 0.0;
  std::size_t last_kept = begin;
  for (std::size_t i = begin + 1; i <= end; ++i) {
    accumulated += (history[i].position - history[i - 1].position).norm();
    bool keep = accumulated >= step || i == end;
    if (i > begin + 1 && i < end) {
      const Eigen::Vector3d incoming =
          history[i - 1].position - history[i - 2].position;
      const Eigen::Vector3d outgoing =
          history[i + 1].position - history[i].position;
      const double ni = incoming.norm();
      const double no = outgoing.norm();
      if (ni > 1e-6 && no > 1e-6) {
        double cosine = incoming.dot(outgoing) / (ni * no);
        cosine = std::max(-1.0, std::min(1.0, cosine));
        if (std::acos(cosine) >= turn_threshold) keep = true;
      }
    }
    if (keep) {
      output->push_back(history[i]);
      last_kept = i;
      accumulated = 0.0;
    }
  }
  if (last_kept != end) output->push_back(history[end]);
}

std::size_t findHistoryIndex(const std::vector<TrajectoryPoint>& history,
                             std::size_t begin,
                             const TrajectoryPoint& target) {
  for (std::size_t i = begin; i < history.size(); ++i) {
    if (sameHistoryPoint(history[i], target)) return i;
  }
  for (std::size_t i = begin; i < history.size(); ++i) {
    if (history[i].position == target.position) return i;
  }
  return history.size();
}

bool segmentIsClear(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                    const Eigen::Vector3d& start,
                    const Eigen::Vector3d& end,
                    const SafeRdpConfig& config,
                    int* collision_check_count, double* minimum_clearance) {
  const Eigen::Vector3d delta = end - start;
  const double length = delta.norm();
  const double step = std::max(1e-3, config.collision_check_resolution);
  const int samples = length <= 1e-9
                          ? 0
                          : std::max(2, static_cast<int>(
                                            std::ceil(length / step)) + 1);
  bool clear = true;
  for (int sample = 0; sample <= samples; ++sample) {
    const double ratio = samples == 0
                             ? 0.0
                             : static_cast<double>(sample) / samples;
    const double clearance = nearestDistance(kdtree, start + ratio * delta);
    if (collision_check_count) ++(*collision_check_count);
    if (minimum_clearance)
      *minimum_clearance = std::min(*minimum_clearance, clearance);
    if (clearance < config.safeRadius()) clear = false;
  }
  return clear;
}

}  // namespace

SafeRdpResult SafeRdpPlanner::validate(
    const std::vector<ReturnWaypoint>& rdp_path,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
    const SafeRdpConfig& config) const {
  SafeRdpResult result;
  result.safe_path = rdp_path;  // V1.1.0 validates only; never rewrites.
  result.clearance_available = false;

  if (rdp_path.size() < 2) {
    result.message = "safe-rdp: fewer than 2 waypoints";
    return result;
  }
  if (!map_cloud || map_cloud->empty()) {
    result.message = "safe-rdp: no PCD map; clearance cannot be verified";
    return result;
  }

  // Downsample the map so the kd-tree query stays tractable on a large global
  // cloud.  The voxel size trades resolution against speed.
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  const float leaf =
      static_cast<float>(std::max(1e-3, config.voxel_resolution));
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(map_cloud);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*voxelized);
  result.voxelized_cloud_size = static_cast<int>(voxelized->size());
  if (voxelized->empty()) {
    result.message = "safe-rdp: voxelized cloud is empty";
    return result;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(voxelized);

  const double safe_radius = config.safeRadius();
  const double step = std::max(1e-3, config.collision_check_resolution);
  double min_clearance = std::numeric_limits<double>::infinity();
  result.clearance_available = true;

  for (std::size_t i = 0; i + 1 < rdp_path.size(); ++i) {
    const Eigen::Vector3d a = rdp_path[i].position;
    const Eigen::Vector3d b = rdp_path[i + 1].position;
    const double segment_length = (b - a).norm();

    bool segment_unsafe = false;
    if (segment_length <= 1e-9) {
      // Degenerate segment: check the single point.
      ++result.collision_check_count;
      const double d = nearestDistance(kdtree, a);
      min_clearance = std::min(min_clearance, d);
      if (d < safe_radius) segment_unsafe = true;
    } else {
      const Eigen::Vector3d dir = (b - a) / segment_length;
      const int samples = std::max(
          2, static_cast<int>(std::ceil(segment_length / step)) + 1);
      for (int s = 0; s <= samples; ++s) {
        const double t = segment_length * static_cast<double>(s) /
                         static_cast<double>(samples);
        const Eigen::Vector3d p = a + t * dir;
        ++result.collision_check_count;
        const double d = nearestDistance(kdtree, p);
        min_clearance = std::min(min_clearance, d);
        if (d < safe_radius) segment_unsafe = true;
      }
    }
    if (segment_unsafe) {
      ++result.unsafe_segments;
    } else {
      ++result.validated_segments;
    }
  }

  result.min_clearance_m =
      std::isfinite(min_clearance) ? min_clearance : 0.0;
  result.safe = (result.unsafe_segments == 0);
  result.message = result.safe ? "safe-rdp: path validated"
                               : "safe-rdp: unsafe segment detected";
  return result;
}

SafeRdpResult SafeRdpPlanner::restoreUnsafeShortcuts(
    const std::vector<TrajectoryPoint>& candidate_path,
    const std::vector<TrajectoryPoint>& reversed_history,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
    const SafeRdpConfig& config) const {
  SafeRdpResult result;
  result.fallback_path = candidate_path;
  if (candidate_path.size() < 2 || reversed_history.size() < 2) {
    result.message = "safe-rdp fallback: fewer than 2 points";
    return result;
  }

  // No map must preserve the existing Reverse + RDP behaviour.  It is usable
  // but explicitly unverified, rather than being reported as map-safe.
  if (!map_cloud || map_cloud->empty()) {
    result.safe = true;
    result.message = "safe-rdp fallback: no PCD; candidate path unchanged";
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  const float leaf =
      static_cast<float>(std::max(1e-3, config.voxel_resolution));
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(map_cloud);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*voxelized);
  result.voxelized_cloud_size = static_cast<int>(voxelized->size());
  if (voxelized->empty()) {
    result.safe = true;
    result.message =
        "safe-rdp fallback: voxelized PCD empty; candidate path unchanged";
    return result;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(voxelized);
  result.fallback_path.clear();
  result.fallback_path.reserve(candidate_path.size());
  result.fallback_path.push_back(candidate_path.front());
  std::size_t history_index = 0;
  double minimum_clearance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 1; i < candidate_path.size(); ++i) {
    const std::size_t target_index = findHistoryIndex(
        reversed_history, history_index + 1, candidate_path[i]);
    if (target_index >= reversed_history.size()) {
      // This should be impossible because RDP copies history points.  Use the
      // same bounded fallback policy rather than replaying the entire 10 Hz
      // trace if a future planner introduces a synthetic point.
      result.fallback_path.clear();
      result.fallback_path.push_back(candidate_path.front());
      appendHistoryFallback(&result.fallback_path, reversed_history, 0,
                            reversed_history.size() - 1,
                            config.history_fallback_spacing,
                            20.0 * M_PI / 180.0);
      result.fallback_segments = 1;
      result.fallback_points = result.fallback_path.size() > 1
                                   ? result.fallback_path.size() - 1
                                   : 0;
      result.restored_history_points = result.fallback_points;
      result.safe = true;
      result.message =
          "safe-rdp fallback: candidate mapping failed; dense history used";
      return result;
    }

    if (target_index == history_index + 1) {
      result.fallback_path.push_back(candidate_path[i]);
      history_index = target_index;
      continue;
    }

    ++result.shortcut_candidates;
    const double deviation = historyDeviation(
        reversed_history, history_index, target_index,
        candidate_path[i - 1].position, candidate_path[i].position);
    if (config.trust_flown_history &&
        deviation <= config.provenance_deviation_threshold) {
      ++result.trusted_history_segments;
      result.fallback_path.push_back(candidate_path[i]);
      history_index = target_index;
      continue;
    }
    ++result.map_checked_segments;
    const bool clear = segmentIsClear(
        kdtree, candidate_path[i - 1].position, candidate_path[i].position,
        config, &result.collision_check_count, &minimum_clearance);
    if (clear) {
      ++result.validated_segments;
      result.fallback_path.push_back(candidate_path[i]);
    } else {
      ++result.unsafe_segments;
      ++result.fallback_segments;
      const std::size_t before = result.fallback_path.size();
      appendHistoryFallback(&result.fallback_path, reversed_history,
                            history_index, target_index,
                            config.history_fallback_spacing,
                            20.0 * M_PI / 180.0);
      const std::size_t added = result.fallback_path.size() - before;
      const std::size_t restored = added > 0 ? added - 1 : 0;
      result.restored_history_points += restored;
      result.fallback_points += restored;
    }
    history_index = target_index;
  }

  result.clearance_available = result.map_checked_segments > 0;
  result.min_clearance_m = std::isfinite(minimum_clearance)
                               ? minimum_clearance
                               : 0.0;
  result.safe = true;
  result.message = result.unsafe_segments == 0
                       ? "safe-rdp fallback: all shortcut candidates accepted"
                       : "safe-rdp fallback: unsafe shortcuts restored";
  return result;
}

}  // namespace manual_return_planner
