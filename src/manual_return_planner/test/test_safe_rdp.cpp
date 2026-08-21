#include "manual_return_planner/safe_rdp.h"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace {
using manual_return_planner::ReturnWaypoint;
using manual_return_planner::SafeRdpConfig;
using manual_return_planner::SafeRdpPlanner;

ReturnWaypoint waypoint(double x, double y, double z = 0.0) {
  ReturnWaypoint w;
  w.position << x, y, z;
  return w;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makeCloud(
    const std::vector<Eigen::Vector3d>& pts) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const Eigen::Vector3d& p : pts) {
    cloud->push_back(
        pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()),
                      static_cast<float>(p.z())));
  }
  return cloud;
}

// Default config: safe_radius = 0.25 + 0.15 + 0.10 = 0.50 m.
SafeRdpConfig defaultConfig() { return SafeRdpConfig(); }

// A straight 1 m path along x, 10 waypoints (11 points, 10 segments).
std::vector<ReturnWaypoint> straightPath() {
  std::vector<ReturnWaypoint> path;
  for (int i = 0; i <= 10; ++i) path.push_back(waypoint(0.1 * i, 0.0));
  return path;
}
}  // namespace

// 1. No obstacle near the path -> safe.
TEST(SafeRdpPlanner, ClearPathPasses) {
  auto cloud = makeCloud({{5.0, 5.0, 0.0}});  // far away from the path
  SafeRdpPlanner planner;
  const auto r = planner.validate(straightPath(), cloud, defaultConfig());
  EXPECT_TRUE(r.safe);
  EXPECT_TRUE(r.clearance_available);
  EXPECT_EQ(r.unsafe_segments, 0);
  EXPECT_GT(r.collision_check_count, 0);
  EXPECT_GT(r.min_clearance_m, defaultConfig().safeRadius());
}

// 2. An obstacle sits directly on a segment -> unsafe.
TEST(SafeRdpPlanner, ObstacleOnSegmentFails) {
  auto cloud = makeCloud({{0.5, 0.0, 0.0}});  // directly on the path
  SafeRdpPlanner planner;
  const auto r = planner.validate(straightPath(), cloud, defaultConfig());
  EXPECT_FALSE(r.safe);
  EXPECT_GT(r.unsafe_segments, 0);
  EXPECT_LT(r.min_clearance_m, defaultConfig().safeRadius());
}

// 3. Obstacle further than safe_radius -> safe.
TEST(SafeRdpPlanner, ObstacleBeyondSafeRadiusPasses) {
  // safe_radius = 0.50 m; obstacle sits 0.60 m off the path.
  auto cloud = makeCloud({{0.5, 0.6, 0.0}});
  SafeRdpPlanner planner;
  const auto r = planner.validate(straightPath(), cloud, defaultConfig());
  EXPECT_TRUE(r.safe);
  EXPECT_GT(r.min_clearance_m, defaultConfig().safeRadius());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
