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

}  // namespace manual_return_planner
